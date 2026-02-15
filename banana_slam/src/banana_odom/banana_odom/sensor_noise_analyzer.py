#!/usr/bin/env python3
"""
Sensor Noise Analyzer - Collect IMU and wheel encoder data for noise characterization.

Subscribes to:
  /imu - IMU data (gyro_z for rotation)
  /joint_states - Wheel encoder data

Records data for specified duration and saves to CSV for offline analysis.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
import numpy as np
import csv
import os
from math import atan2, sqrt


class SensorNoiseAnalyzer(Node):
    """
    Collects sensor data for noise analysis.
    Records IMU gyro and wheel encoder data to CSV files.
    """

    def __init__(self):
        super().__init__('sensor_noise_analyzer')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('duration', 5.0),  # seconds to record
                ('wheel_radius', 0.033),  # meters
                ('wheel_separation', 0.16),  # meters
                ('output_dir', '/tmp'),
            ]
        )

        self.duration = self.get_parameter('duration').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.output_dir = self.get_parameter('output_dir').value

        # Data storage
        self.imu_data = []  # [(timestamp, gyro_z, accel_x, accel_y), ...]
        self.wheel_data = []  # [(timestamp, left_pos, right_pos, v_left, v_right, v_linear, v_angular), ...]

        # State
        self.start_time = None
        self.recording = False
        self.prev_joint_state = None
        self.prev_joint_time = None

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        self.get_logger().info('Sensor Noise Analyzer initialized')
        self.get_logger().info(f'  Recording duration: {self.duration} seconds')
        self.get_logger().info(f'  Output directory: {self.output_dir}')
        self.get_logger().info('Waiting for sensor data...')

    def imu_callback(self, msg):
        """Record IMU data"""
        if not self.recording:
            self.recording = True
            self.start_time = self.get_clock().now()
            self.get_logger().info('Started recording sensor data...')

        # Calculate elapsed time
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if elapsed > self.duration:
            if self.recording:
                self.finish_recording()
            return

        # Record timestamp and gyro_z (yaw rate)
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        gyro_z = msg.angular_velocity.z
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y

        self.imu_data.append((timestamp, gyro_z, accel_x, accel_y))

    def joint_states_callback(self, msg):
        """Record wheel encoder data and compute velocities"""
        if not self.recording:
            return

        # Calculate elapsed time
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.duration:
            return

        # Find wheel indices
        try:
            left_idx = msg.name.index('wheel_left_joint')
            right_idx = msg.name.index('wheel_right_joint')
        except ValueError:
            self.get_logger().warn('Could not find wheel joints in joint_states', throttle_duration_sec=5.0)
            return

        timestamp = self.get_clock().now().nanoseconds / 1e9
        left_pos = msg.position[left_idx]
        right_pos = msg.position[right_idx]

        # Compute velocities if we have previous data
        v_left = 0.0
        v_right = 0.0
        v_linear = 0.0
        v_angular = 0.0

        if self.prev_joint_state is not None and self.prev_joint_time is not None:
            dt = timestamp - self.prev_joint_time
            if dt > 0:
                # Compute wheel velocities (rad/s)
                left_vel_rad = (left_pos - self.prev_joint_state[0]) / dt
                right_vel_rad = (right_pos - self.prev_joint_state[1]) / dt

                # Convert to linear velocities (m/s)
                v_left = left_vel_rad * self.wheel_radius
                v_right = right_vel_rad * self.wheel_radius

                # Differential drive kinematics
                v_linear = (v_left + v_right) / 2.0
                v_angular = (v_right - v_left) / self.wheel_separation

        self.wheel_data.append((
            timestamp,
            left_pos,
            right_pos,
            v_left,
            v_right,
            v_linear,
            v_angular
        ))

        self.prev_joint_state = (left_pos, right_pos)
        self.prev_joint_time = timestamp

    def finish_recording(self):
        """Save data and shutdown"""
        self.recording = False

        self.get_logger().info(f'Recording complete! Collected:')
        self.get_logger().info(f'  IMU samples: {len(self.imu_data)}')
        self.get_logger().info(f'  Wheel samples: {len(self.wheel_data)}')

        # Save IMU data
        imu_file = os.path.join(self.output_dir, 'imu_noise.csv')
        with open(imu_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'gyro_z', 'accel_x', 'accel_y'])
            writer.writerows(self.imu_data)
        self.get_logger().info(f'Saved IMU data to: {imu_file}')

        # Save wheel data
        wheel_file = os.path.join(self.output_dir, 'wheel_noise.csv')
        with open(wheel_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'left_pos', 'right_pos', 'v_left', 'v_right', 'v_linear', 'v_angular'])
            writer.writerows(self.wheel_data)
        self.get_logger().info(f'Saved wheel data to: {wheel_file}')

        self.get_logger().info('Shutting down...')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SensorNoiseAnalyzer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.recording:
            node.finish_recording()
        node.destroy_node()


if __name__ == '__main__':
    main()
