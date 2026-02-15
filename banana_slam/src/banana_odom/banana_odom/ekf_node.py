#!/usr/bin/env python3
# Copyright (c) 2024
# Licensed under MIT

"""
EKF Odometry Node for FRA532 Lab 1.

This node fuses wheel odometry (from /joint_states) and IMU data (from /imu)
using an Extended Kalman Filter to produce a filtered odometry estimate.

Data Flow (from diagram):
- Wheel_Odom -> Noise Filter -> Wheel_Odom_CALIB -> Pose Estimation
- IMU_RAW -> Calibration (3s @ Startup) -> IMU_CALIB -> Pose Estimation
- Pose Estimation -> Transformation Integration -> Lidar Mapping
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
from math import sin, cos, atan2, pi

from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def euler_from_quaternion(q):
    """Convert quaternion to euler angles (roll, pitch, yaw)."""
    x, y, z, w = q.x, q.y, q.z, q.w

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """Convert euler angles to quaternion."""
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle


class EKFOdometryNode(Node):
    """
    Extended Kalman Filter Odometry Node.
    
    State vector: [x, y, theta]
    - x, y: position in odom frame
    - theta: orientation (yaw) in odom frame
    
    Prediction model: Differential drive kinematics from wheel odometry
    Update model: IMU gyroscope for theta correction
    """
    
    def __init__(self):
        super().__init__('ekf_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_radius', 0.033),
                ('wheel_separation', 0.160),
                ('process_noise_x', 0.00001),
                ('process_noise_y', 0.00001),
                ('process_noise_theta', 0.00001),
                ('imu_noise_theta', 0.000001),
                ('initial_cov_x', 0.01),
                ('initial_cov_y', 0.01),
                ('initial_cov_theta', 0.01),
                ('icp_noise_x', 0.00000001),
                ('icp_noise_y', 0.00000001),
                ('icp_noise_theta', 0.00000001),
                ('imu_calibration_duration', 3.0),
                ('odom_frame', 'odom'),
                ('base_frame', 'base_footprint'),
                ('joint_states_topic', '/joint_states'),
                ('imu_topic', '/imu'),
                ('icp_topic', '/odometry/icp'),
                ('odom_output_topic', '/odometry/ekf'),
            ]
        )

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.state = np.zeros(3)

        self.P = np.diag([
            self.get_parameter('initial_cov_x').value,
            self.get_parameter('initial_cov_y').value,
            self.get_parameter('initial_cov_theta').value
        ])

        self.Q = np.diag([
            self.get_parameter('process_noise_x').value,
            self.get_parameter('process_noise_y').value,
            self.get_parameter('process_noise_theta').value
        ])

        self.R_imu = np.array([[self.get_parameter('imu_noise_theta').value]])

        self.R_icp = np.diag([
            self.get_parameter('icp_noise_x').value,
            self.get_parameter('icp_noise_y').value,
            self.get_parameter('icp_noise_theta').value
        ])

        self.prev_left_pos = None
        self.prev_right_pos = None
        self.prev_time = None

        self.imu_calibration_duration = self.get_parameter('imu_calibration_duration').value
        self.imu_gyro_bias_z = 0.0
        self.imu_calibration_samples = []
        self.imu_calibrated = False
        self.initial_imu_yaw = None
        self.calibration_start_stamp = None

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter('odom_output_topic').value,
            10
        )

        self.wheel_odom_pub = self.create_publisher(
            Odometry,
            '/odometry/wheel',
            10
        )

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.joint_states_sub = self.create_subscription(
            JointState,
            self.get_parameter('joint_states_topic').value,
            self.joint_states_callback,
            sensor_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self.imu_callback,
            sensor_qos
        )

        self.icp_sub = self.create_subscription(
            Odometry,
            self.get_parameter('icp_topic').value,
            self.icp_callback,
            10
        )

        self.wheel_odom_state = np.zeros(3)
        
        self.get_logger().info('EKF Odometry Node initialized')
        self.get_logger().info(f'  Wheel radius: {self.wheel_radius} m')
        self.get_logger().info(f'  Wheel separation: {self.wheel_separation} m')
        self.get_logger().info(f'  IMU calibration duration: {self.imu_calibration_duration} s')
    
    def joint_states_callback(self, msg: JointState):
        """
        Process wheel joint states to compute wheel odometry.
        
        Turtlebot3 joint_states contains:
        - name: ['wheel_left_joint', 'wheel_right_joint']
        - position: [left_pos, right_pos] in radians
        - velocity: [left_vel, right_vel] in rad/s
        """
        current_time = self.get_clock().now()

        try:
            left_idx = msg.name.index('wheel_left_joint')
            right_idx = msg.name.index('wheel_right_joint')
        except ValueError:
            self.get_logger().warn(f'Could not find wheel joints in joint_states. Available: {msg.name}', throttle_duration_sec=5.0)
            return

        left_pos = msg.position[left_idx]
        right_pos = msg.position[right_idx]

        if self.prev_left_pos is None:
            self.prev_left_pos = left_pos
            self.prev_right_pos = right_pos
            self.prev_time = current_time
            return

        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            return

        delta_left = left_pos - self.prev_left_pos
        delta_right = right_pos - self.prev_right_pos

        delta_left_dist = delta_left * self.wheel_radius
        delta_right_dist = delta_right * self.wheel_radius

        delta_s = (delta_left_dist + delta_right_dist) / 2.0
        delta_theta = (delta_right_dist - delta_left_dist) / self.wheel_separation

        v = delta_s / dt
        w = delta_theta / dt

        theta = self.state[2]

        if abs(delta_theta) < 1e-6:
            delta_x = delta_s * cos(theta)
            delta_y = delta_s * sin(theta)
        else:
            delta_x = delta_s * cos(theta + delta_theta / 2.0)
            delta_y = delta_s * sin(theta + delta_theta / 2.0)

        self.state[0] += delta_x
        self.state[1] += delta_y
        self.state[2] += delta_theta
        self.state[2] = normalize_angle(self.state[2])

        F = np.array([
            [1.0, 0.0, -delta_s * sin(theta + delta_theta / 2.0)],
            [0.0, 1.0, delta_s * cos(theta + delta_theta / 2.0)],
            [0.0, 0.0, 1.0]
        ])

        self.P = F @ self.P @ F.T + self.Q

        if abs(delta_theta) < 1e-6:
            self.wheel_odom_state[0] += delta_s * cos(self.wheel_odom_state[2])
            self.wheel_odom_state[1] += delta_s * sin(self.wheel_odom_state[2])
        else:
            self.wheel_odom_state[0] += delta_s * cos(self.wheel_odom_state[2] + delta_theta / 2.0)
            self.wheel_odom_state[1] += delta_s * sin(self.wheel_odom_state[2] + delta_theta / 2.0)
        self.wheel_odom_state[2] += delta_theta
        self.wheel_odom_state[2] = normalize_angle(self.wheel_odom_state[2])

        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos
        self.prev_time = current_time

        self.publish_odometry(current_time, v, w)
        self.publish_wheel_odometry(current_time, v, w)
    
    def imu_callback(self, msg: Imu):
        """
        Process IMU data for EKF update step.

        - During calibration: collect gyro z samples to estimate bias
        - After calibration: use gyro z for theta correction
        """
        msg_stamp = Time.from_msg(msg.header.stamp)

        if not self.imu_calibrated:
            if self.calibration_start_stamp is None:
                self.calibration_start_stamp = msg_stamp
                self.get_logger().info(f'Starting IMU calibration ({self.imu_calibration_duration}s)...')

            elapsed = (msg_stamp - self.calibration_start_stamp).nanoseconds / 1e9

            self.imu_calibration_samples.append(msg.angular_velocity.z)

            if len(self.imu_calibration_samples) % 20 == 0:
                self.get_logger().info(f'  IMU calibration: {elapsed:.1f}/{self.imu_calibration_duration}s ({len(self.imu_calibration_samples)} samples)')

            if elapsed >= self.imu_calibration_duration:
                if len(self.imu_calibration_samples) > 0:
                    self.imu_gyro_bias_z = np.mean(self.imu_calibration_samples)
                self.imu_calibrated = True
                self.get_logger().info(f'IMU calibration complete. Gyro Z bias: {self.imu_gyro_bias_z:.6f} rad/s ({len(self.imu_calibration_samples)} samples)')
            return

        _, _, imu_yaw = euler_from_quaternion(msg.orientation)

        if self.initial_imu_yaw is None:
            self.initial_imu_yaw = imu_yaw

        measured_yaw = normalize_angle(imu_yaw - self.initial_imu_yaw)

        H = np.array([[0.0, 0.0, 1.0]])

        z = np.array([measured_yaw])
        z_pred = H @ self.state
        y = z - z_pred
        y[0] = normalize_angle(y[0])

        S = H @ self.P @ H.T + self.R_imu

        K = self.P @ H.T @ np.linalg.inv(S)

        self.state = self.state + (K @ y).flatten()
        self.state[2] = normalize_angle(self.state[2])

        I = np.eye(3)
        self.P = (I - K @ H) @ self.P
        
    def icp_callback(self, msg: Odometry):
        """Process ICP Odometry for EKF update step."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, icp_yaw = euler_from_quaternion(q)

        z = np.array([p.x, p.y, icp_yaw])

        H = np.eye(3)

        z_pred = H @ self.state
        y = z - z_pred
        y[2] = normalize_angle(y[2])

        S = H @ self.P @ H.T + self.R_icp

        K = self.P @ H.T @ np.linalg.inv(S)

        self.state = self.state + (K @ y).flatten()
        self.state[2] = normalize_angle(self.state[2])

        I = np.eye(3)
        self.P = (I - K @ H) @ self.P

    
    
    def publish_odometry(self, stamp: Time, v: float, w: float):
        """Publish EKF-fused odometry message and TF."""
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.state[0]
        odom.pose.pose.position.y = self.state[1]
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, self.state[2])

        odom.pose.covariance[0] = self.P[0, 0]
        odom.pose.covariance[7] = self.P[1, 1]
        odom.pose.covariance[35] = self.P[2, 2]

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.state[0]
        t.transform.translation.y = self.state[1]
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def publish_wheel_odometry(self, stamp: Time, v: float, w: float):
        """Publish raw wheel odometry for comparison."""
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = 'base_link_wheel'
        
        odom.pose.pose.position.x = self.wheel_odom_state[0]
        odom.pose.pose.position.y = self.wheel_odom_state[1]
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, self.wheel_odom_state[2])
        
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        
        self.wheel_odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = EKFOdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
