
import sqlite3
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

# We need to run the system and record a new bag, OR we can analyze the output if we record it.
# Since I can't interactively record, I will assume we want to analyze the COMPONENTS from the source bag
# to see if the INPUTS to the EKF are garbage.

# Actually, the best way is to Play Bag -> Run Nodes -> Record /odometry/* -> Analyze.
# I will create a script that subscribes to the live topics and computes metrics in real-time.

import rclpy
from rclpy.node import Node
import math

class OdomAnalyzer(Node):
    def __init__(self):
        super().__init__('odom_analyzer')

        self.ekf_sub = self.create_subscription(Odometry, '/odometry/ekf', self.ekf_cb, 10)
        self.icp_sub = self.create_subscription(Odometry, '/odometry/icp', self.icp_cb, 10)
        self.wheel_sub = self.create_subscription(Odometry, '/odometry/wheel', self.wheel_cb, 10)

        # Store (x, y, timestamp) tuples
        self.ekf_path = []
        self.icp_path = []
        self.wheel_path = []

        self.icp_jumps = []
        self.prev_icp = None

        # Track start time for 15-second window
        self.start_time = None
        self.time_limit = 15.0  # Only evaluate first 15 seconds

        self.get_logger().info("Analyzer Started. Waiting for data...")

    def ekf_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = t
        elapsed = t - self.start_time
        if elapsed <= self.time_limit:
            self.ekf_path.append((msg.pose.pose.position.x, msg.pose.pose.position.y, elapsed))

    def wheel_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = t
        elapsed = t - self.start_time
        if elapsed <= self.time_limit:
            self.wheel_path.append((msg.pose.pose.position.x, msg.pose.pose.position.y, elapsed))

    def icp_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = t
        elapsed = t - self.start_time

        if elapsed <= self.time_limit:
            self.icp_path.append((x, y, elapsed))

        if self.prev_icp is not None:
            dist = math.sqrt((x - self.prev_icp[0])**2 + (y - self.prev_icp[1])**2)
            if elapsed <= self.time_limit:
                self.icp_jumps.append(dist)
            if dist > 0.1: # 10cm jump in one frame?
                self.get_logger().warn(f"HUGE ICP JUMP: {dist:.4f}m")
        self.prev_icp = (x, y)

    def report(self):
        self.get_logger().info("-" * 40)
        self.get_logger().info(f"Evaluating first {self.time_limit}s (Wheel Odom = Ground Truth)")

        if len(self.wheel_path) < 2:
            self.get_logger().error("Not enough wheel odometry data!")
            score = 999.0
        elif len(self.ekf_path) < 2 or len(self.icp_path) < 2:
            self.get_logger().error("Not enough EKF/ICP data!")
            score = 999.0
        else:
            # === WHEEL ODOMETRY AS GROUND TRUTH (first 15s) ===

            # 1. Track ICP jumps (for logging, not penalty)
            max_jump = np.max(self.icp_jumps) if len(self.icp_jumps) > 0 else 0.0

            # 2. EKF vs Wheel endpoint error (how close to ground truth?)
            wx, wy = self.wheel_path[-1][0], self.wheel_path[-1][1]
            ex, ey = self.ekf_path[-1][0], self.ekf_path[-1][1]
            ekf_error = math.sqrt((ex - wx)**2 + (ey - wy)**2)

            # 3. ICP vs Wheel endpoint error
            ix, iy = self.icp_path[-1][0], self.icp_path[-1][1]
            icp_error = math.sqrt((ix - wx)**2 + (iy - wy)**2)

            # 4. Path length comparison (should match wheel)
            ekf_len = sum(math.sqrt((self.ekf_path[i][0]-self.ekf_path[i-1][0])**2 +
                                   (self.ekf_path[i][1]-self.ekf_path[i-1][1])**2)
                         for i in range(1, len(self.ekf_path)))

            icp_len = sum(math.sqrt((self.icp_path[i][0]-self.icp_path[i-1][0])**2 +
                                   (self.icp_path[i][1]-self.icp_path[i-1][1])**2)
                         for i in range(1, len(self.icp_path)))

            wheel_len = sum(math.sqrt((self.wheel_path[i][0]-self.wheel_path[i-1][0])**2 +
                                     (self.wheel_path[i][1]-self.wheel_path[i-1][1])**2)
                           for i in range(1, len(self.wheel_path)))

            ekf_len_error = abs(ekf_len - wheel_len)
            icp_len_error = abs(icp_len - wheel_len)

            # Final Score: Prioritize EKF accuracy (it's what we're tuning!)
            # EKF endpoint error (heavy), EKF path length error, ICP as sanity check
            score = (ekf_error * 100.0) + (ekf_len_error * 10.0) + (icp_error * 1.0)

            if max_jump > 0.05:
                self.get_logger().warn(f"Large ICP jump: {max_jump:.3f}m")

            self.get_logger().info(f"METRICS: EKF_err={ekf_error:.4f}m  ICP_err={icp_error:.4f}m  "
                                  f"EKF_len_err={ekf_len_error:.4f}m  MaxJump={max_jump:.4f}m")

            self.get_logger().info(f"SCORE: {score:.4f} (Lower is Better)")
        
        # Write to file
        with open("/tmp/odom_score.txt", "w") as f:
            f.write(f"{score}")

def main():
    rclpy.init()
    node = OdomAnalyzer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.report()  # ALWAYS report on exit
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
