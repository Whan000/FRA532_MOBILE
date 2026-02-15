#!/usr/bin/env python3
"""
LIDAR Filter Node for FRA532 Lab 1.

Filters full LIDAR scan using empirically measured noise parameters.
Applies noise filtering to all range measurements for cleaner data.

Empirical parameters from bag analysis:
- Mean distance (90°): 1.029071450 m
- Std dev (noise): 0.003607501 m
- Distribution: Gaussian (p-value: 0.728)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
from sensor_msgs.msg import LaserScan

class LidarFilterNode(Node):
    """
    LIDAR filter node.

    Filters full LIDAR scan ranges to reduce noise.
    """

    def __init__(self):
        super().__init__('lidar_filter_node')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('mean_distance', 1.029),
                ('std_dev_distance', 0.00361),
                ('filter_type', 'median'),  # 'median', 'gaussian', 'outlier_reject'
                ('filter_window_size', 5),
                ('outlier_threshold', 3.0),  # Standard deviations
                ('scan_topic', '/scan'),
                ('output_topic', '/scan_filtered'),
            ]
        )

        self.mean_distance = self.get_parameter('mean_distance').value
        self.std_dev = self.get_parameter('std_dev_distance').value
        self.filter_type = self.get_parameter('filter_type').value
        self.window_size = self.get_parameter('filter_window_size').value
        self.outlier_threshold = self.get_parameter('outlier_threshold').value

        self.get_logger().info('LIDAR Filter Node initialized')
        self.get_logger().info(f'  Mean distance: {self.mean_distance:.6f} m')
        self.get_logger().info(f'  Noise std dev: {self.std_dev:.6f} m')
        self.get_logger().info(f'  Filter type: {self.filter_type}')

        # Circular buffer for filtering
        self.measurement_buffer = []
        self.max_buffer_size = self.window_size

        # QoS profile
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            sensor_qos
        )

        # Publishers
        self.filtered_pub = self.create_publisher(
            LaserScan,
            self.get_parameter('output_topic').value,
            10
        )

    def scan_callback(self, msg: LaserScan):
        """Process incoming LIDAR scan."""
        try:
            ranges = np.array(msg.ranges)

            # Apply filtering to all ranges
            filtered_ranges = self.filter_ranges(ranges, msg.range_min, msg.range_max)

            # Create filtered scan message
            filtered_msg = LaserScan()
            filtered_msg.header = msg.header
            filtered_msg.angle_min = msg.angle_min
            filtered_msg.angle_max = msg.angle_max
            filtered_msg.angle_increment = msg.angle_increment
            filtered_msg.time_increment = msg.time_increment
            filtered_msg.scan_time = msg.scan_time
            filtered_msg.range_min = msg.range_min
            filtered_msg.range_max = msg.range_max
            filtered_msg.ranges = filtered_ranges.tolist()
            filtered_msg.intensities = msg.intensities

            self.filtered_pub.publish(filtered_msg)

            # Log periodically
            if hasattr(self, 'log_counter'):
                self.log_counter += 1
            else:
                self.log_counter = 0

            if self.log_counter % 10 == 0:
                valid_mask = (ranges > msg.range_min) & (ranges < msg.range_max)
                valid_count = np.sum(valid_mask)
                self.get_logger().info(f'Published filtered scan ({valid_count} valid ranges)')

        except Exception as e:
            self.get_logger().error(f'Error processing scan: {e}')

    def filter_ranges(self, ranges, range_min, range_max):
        """Apply filtering to all ranges in the scan."""
        if self.filter_type == 'median':
            return self.median_filter_scan(ranges, range_min, range_max)
        elif self.filter_type == 'gaussian':
            return self.gaussian_filter_scan(ranges, range_min, range_max)
        elif self.filter_type == 'outlier_reject':
            return self.outlier_rejection_filter_scan(ranges, range_min, range_max)
        else:
            return ranges

    def median_filter_scan(self, ranges, range_min, range_max):
        """
        Apply median filter using sliding window across scan.
        Uses neighboring measurements to filter each range.
        """
        filtered = np.array(ranges, dtype=float)
        window_radius = self.window_size // 2

        for i in range(len(ranges)):
            # Get window around current index (with wraparound)
            start = max(0, i - window_radius)
            end = min(len(ranges), i + window_radius + 1)
            window = ranges[start:end]

            # Filter valid measurements in window
            valid = (window > range_min) & (window < range_max)
            if np.any(valid):
                filtered[i] = np.median(window[valid])

        return filtered

    def gaussian_filter_scan(self, ranges, range_min, range_max):
        """
        Apply Gaussian weighting to each measurement.
        Measurements closer to empirical mean get higher weight.
        """
        filtered = np.array(ranges, dtype=float)

        for i in range(len(ranges)):
            r = ranges[i]
            if range_min < r < range_max:
                # Apply Gaussian weight based on distance from mean
                weight = np.exp(-0.5 * ((r - self.mean_distance) / self.std_dev) ** 2)
                # Blend with original measurement
                filtered[i] = weight * r + (1 - weight) * self.mean_distance

        return filtered

    def outlier_rejection_filter_scan(self, ranges, range_min, range_max):
        """
        Reject outliers using sliding window statistics.
        Replace outliers with neighborhood mean.
        """
        filtered = np.array(ranges, dtype=float)
        window_radius = self.window_size // 2

        for i in range(len(ranges)):
            r = ranges[i]

            # Skip invalid measurements
            if not (range_min < r < range_max):
                continue

            # Get window around current index
            start = max(0, i - window_radius)
            end = min(len(ranges), i + window_radius + 1)
            window = ranges[start:end]

            # Filter valid measurements in window
            valid = (window > range_min) & (window < range_max)
            if np.sum(valid) > 1:
                mean = np.mean(window[valid])
                std = np.std(window[valid])

                # Check if current measurement is an outlier
                if np.abs(r - mean) > (self.outlier_threshold * std):
                    filtered[i] = mean
                else:
                    filtered[i] = r
            else:
                filtered[i] = r

        return filtered


def main(args=None):
    rclpy.init(args=args)
    node = LidarFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
