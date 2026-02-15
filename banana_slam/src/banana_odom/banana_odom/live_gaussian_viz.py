#!/usr/bin/env python3
"""
Real-time Gaussian Distribution Visualizer

Displays live Gaussian distribution plots of IMU and wheel odometry data
over a 3-second sliding window.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy import stats
from collections import deque
import threading


class LiveGaussianViz(Node):
    """
    Visualizes sensor noise as Gaussian distributions in real-time.
    """

    def __init__(self):
        super().__init__('live_gaussian_viz')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('window_duration', 15.0),  # seconds
                ('wheel_radius', 0.033),
                ('wheel_separation', 0.16),
            ]
        )

        self.window_duration = self.get_parameter('window_duration').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value

        # Data storage (3-second sliding window)
        self.imu_gyro_data = deque(maxlen=1000)
        self.wheel_angular_data = deque(maxlen=1000)
        self.timestamps = deque(maxlen=1000)

        # Previous joint state for velocity calculation
        self.prev_joint_state = None
        self.prev_joint_time = None

        # Lock for thread-safe data access
        self.data_lock = threading.Lock()

        # Recording state
        self.start_time = None
        self.recording_complete = False

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

        self.get_logger().info('Live Gaussian Visualizer initialized')
        self.get_logger().info(f'  Window duration: {self.window_duration}s')

    def imu_callback(self, msg):
        """Record IMU gyro_z data"""
        if self.recording_complete:
            return

        timestamp = self.get_clock().now().nanoseconds / 1e9
        gyro_z = msg.angular_velocity.z

        # Start recording timer on first message
        if self.start_time is None:
            self.start_time = timestamp
            self.get_logger().info('Started recording data...')

        # Check if we've collected enough data
        elapsed = timestamp - self.start_time
        if elapsed >= self.window_duration:
            if not self.recording_complete:
                self.recording_complete = True
                self.get_logger().info(f'Recording complete! Collected {self.window_duration}s of data.')
                self.get_logger().info(f'  IMU samples: {len(self.imu_gyro_data)}')
                self.get_logger().info(f'  Wheel samples: {len(self.wheel_angular_data)}')
            return

        with self.data_lock:
            self.imu_gyro_data.append(gyro_z)
            self.timestamps.append(timestamp)

            # Remove data older than window
            if len(self.timestamps) > 1:
                cutoff_time = timestamp - self.window_duration
                while self.timestamps and self.timestamps[0] < cutoff_time:
                    self.timestamps.popleft()
                    if self.imu_gyro_data:
                        self.imu_gyro_data.popleft()
                    if self.wheel_angular_data:
                        self.wheel_angular_data.popleft()

    def joint_states_callback(self, msg):
        """Compute and record wheel angular velocity"""
        if self.recording_complete:
            return

        try:
            left_idx = msg.name.index('wheel_left_joint')
            right_idx = msg.name.index('wheel_right_joint')
        except ValueError:
            return

        timestamp = self.get_clock().now().nanoseconds / 1e9
        left_pos = msg.position[left_idx]
        right_pos = msg.position[right_idx]

        if self.prev_joint_state is not None and self.prev_joint_time is not None:
            dt = timestamp - self.prev_joint_time
            if dt > 0:
                # Compute wheel velocities
                left_vel_rad = (left_pos - self.prev_joint_state[0]) / dt
                right_vel_rad = (right_pos - self.prev_joint_state[1]) / dt

                v_left = left_vel_rad * self.wheel_radius
                v_right = right_vel_rad * self.wheel_radius

                # Angular velocity from differential drive
                v_angular = (v_right - v_left) / self.wheel_separation

                with self.data_lock:
                    self.wheel_angular_data.append(v_angular)

        self.prev_joint_state = (left_pos, right_pos)
        self.prev_joint_time = timestamp

    def get_data_snapshot(self):
        """Get thread-safe snapshot of current data"""
        with self.data_lock:
            return (
                list(self.imu_gyro_data),
                list(self.wheel_angular_data)
            )


def main(args=None):
    rclpy.init(args=args)
    node = LiveGaussianViz()

    # Setup matplotlib
    plt.style.use('ggplot')  # Use widely available style
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Real-time Sensor Noise Gaussian Distribution (3s window)', fontsize=16)

    ax_imu_hist = axes[0, 0]
    ax_wheel_hist = axes[0, 1]
    ax_imu_gaussian = axes[1, 0]
    ax_wheel_gaussian = axes[1, 1]

    # Configure axes
    ax_imu_hist.set_title('IMU Gyro Z - Histogram')
    ax_imu_hist.set_xlabel('Angular Velocity (rad/s)')
    ax_imu_hist.set_ylabel('Frequency')

    ax_wheel_hist.set_title('Wheel Angular Velocity - Histogram')
    ax_wheel_hist.set_xlabel('Angular Velocity (rad/s)')
    ax_wheel_hist.set_ylabel('Frequency')

    ax_imu_gaussian.set_title('IMU Gyro Z - Gaussian Fit')
    ax_imu_gaussian.set_xlabel('Angular Velocity (rad/s)')
    ax_imu_gaussian.set_ylabel('Probability Density')

    ax_wheel_gaussian.set_title('Wheel Angular Velocity - Gaussian Fit')
    ax_wheel_gaussian.set_xlabel('Angular Velocity (rad/s)')
    ax_wheel_gaussian.set_ylabel('Probability Density')

    def update_plot(frame):
        """Update plot with latest data"""
        # Stop updating if recording is complete
        if node.recording_complete and frame > 0:
            # Keep showing the final plot
            return

        imu_data, wheel_data = node.get_data_snapshot()

        # Clear axes
        ax_imu_hist.clear()
        ax_wheel_hist.clear()
        ax_imu_gaussian.clear()
        ax_wheel_gaussian.clear()

        # Reset titles and labels
        ax_imu_hist.set_title('IMU Gyro Z - Histogram')
        ax_imu_hist.set_xlabel('Angular Velocity (rad/s)')
        ax_imu_hist.set_ylabel('Frequency')

        ax_wheel_hist.set_title('Wheel Angular Velocity - Histogram')
        ax_wheel_hist.set_xlabel('Angular Velocity (rad/s)')
        ax_wheel_hist.set_ylabel('Frequency')

        ax_imu_gaussian.set_title('IMU Gyro Z - Gaussian Fit')
        ax_imu_gaussian.set_xlabel('Angular Velocity (rad/s)')
        ax_imu_gaussian.set_ylabel('Probability Density')

        ax_wheel_gaussian.set_title('Wheel Angular Velocity - Gaussian Fit')
        ax_wheel_gaussian.set_xlabel('Angular Velocity (rad/s)')
        ax_wheel_gaussian.set_ylabel('Probability Density')

        # Process IMU data
        if len(imu_data) > 10:
            imu_array = np.array(imu_data)
            imu_mean = np.mean(imu_array)
            imu_std = np.std(imu_array)

            # Histogram
            ax_imu_hist.hist(imu_array, bins=30, alpha=0.7, color='blue', density=True)
            ax_imu_hist.axvline(imu_mean, color='red', linestyle='--', label=f'μ={imu_mean:.4f}')
            ax_imu_hist.axvline(imu_mean + imu_std, color='orange', linestyle=':', label=f'σ={imu_std:.4f}')
            ax_imu_hist.axvline(imu_mean - imu_std, color='orange', linestyle=':')
            ax_imu_hist.legend()
            ax_imu_hist.set_title(f'IMU Gyro Z (n={len(imu_data)})')

            # Gaussian fit
            x_range = np.linspace(imu_mean - 4*imu_std, imu_mean + 4*imu_std, 100)
            gaussian = stats.norm.pdf(x_range, imu_mean, imu_std)
            ax_imu_gaussian.plot(x_range, gaussian, 'b-', linewidth=2, label='Fitted Gaussian')
            ax_imu_gaussian.hist(imu_array, bins=30, alpha=0.5, color='blue', density=True, label='Data')
            ax_imu_gaussian.legend()
            ax_imu_gaussian.text(0.05, 0.95, f'μ = {imu_mean:.6f} rad/s\nσ = {imu_std:.6f} rad/s',
                                transform=ax_imu_gaussian.transAxes, verticalalignment='top',
                                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        # Process Wheel data
        if len(wheel_data) > 10:
            wheel_array = np.array(wheel_data)
            wheel_mean = np.mean(wheel_array)
            wheel_std = np.std(wheel_array)

            # Histogram
            ax_wheel_hist.hist(wheel_array, bins=30, alpha=0.7, color='green', density=True)
            ax_wheel_hist.axvline(wheel_mean, color='red', linestyle='--', label=f'μ={wheel_mean:.4f}')
            ax_wheel_hist.axvline(wheel_mean + wheel_std, color='orange', linestyle=':', label=f'σ={wheel_std:.4f}')
            ax_wheel_hist.axvline(wheel_mean - wheel_std, color='orange', linestyle=':')
            ax_wheel_hist.legend()
            ax_wheel_hist.set_title(f'Wheel Angular Velocity (n={len(wheel_data)})')

            # Gaussian fit
            x_range = np.linspace(wheel_mean - 4*wheel_std, wheel_mean + 4*wheel_std, 100)
            gaussian = stats.norm.pdf(x_range, wheel_mean, wheel_std)
            ax_wheel_gaussian.plot(x_range, gaussian, 'g-', linewidth=2, label='Fitted Gaussian')
            ax_wheel_gaussian.hist(wheel_array, bins=30, alpha=0.5, color='green', density=True, label='Data')
            ax_wheel_gaussian.legend()
            ax_wheel_gaussian.text(0.05, 0.95, f'μ = {wheel_mean:.6f} rad/s\nσ = {wheel_std:.6f} rad/s',
                                  transform=ax_wheel_gaussian.transAxes, verticalalignment='top',
                                  bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

            # Add comparison text
            if len(imu_data) > 10:
                imu_std = np.std(np.array(imu_data))
                ratio = wheel_std / imu_std if imu_std > 0 else 0
                fig.text(0.5, 0.02, f'Wheel/IMU noise ratio: {ratio:.2f}x',
                        ha='center', fontsize=12, bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.5))

        plt.tight_layout()

    # Animation
    ani = FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)

    # ROS spin in separate thread
    def spin_ros():
        rclpy.spin(node)

    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
