#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os
import time
from datetime import datetime
import math

def euler_from_quaternion(q):
    """Convert quaternion to euler (roll, pitch, yaw)."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger')
        
        self.declare_parameter('log_dir', '/tmp/banana_logs')
        self.log_dir = self.get_parameter('log_dir').value
        os.makedirs(self.log_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = os.path.join(self.log_dir, f'odom_log_{timestamp}.csv')
        
        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['timestamp', 'source', 'x', 'y', 'theta', 'v', 'w'])
        
        self.get_logger().info(f'Logging data to {self.filename}')
        
        # Subscribe to all odometry sources
        self.create_subscription(Odometry, '/odometry/wheel', lambda m: self.callback(m, 'wheel'), 10)
        self.create_subscription(Odometry, '/odometry/ekf', lambda m: self.callback(m, 'ekf'), 10)
        self.create_subscription(Odometry, '/odometry/icp', lambda m: self.callback(m, 'icp'), 10)
        
    def callback(self, msg, source):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        _, _, theta = euler_from_quaternion(msg.pose.pose.orientation)
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        
        self.writer.writerow([f'{t:.6f}', source, f'{x:.4f}', f'{y:.4f}', f'{theta:.4f}', f'{v:.4f}', f'{w:.4f}'])

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
