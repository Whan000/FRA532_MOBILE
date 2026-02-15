#!/usr/bin/env python3
# Copyright (c) 2024
# Licensed under MIT

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import numpy as np
from scipy.signal import medfilt
from statistics import median

class LidarFilterNode(Node):
    """
    Lidar Filter Node.
    
    Filters LaserScan data by range:
    - Sets ranges < min_range to NaN
    - Sets ranges > max_range to NaN
    Publishes filtered scan to /scan_filtered
    """
    
    def __init__(self):
        super().__init__('lidar_filter_node')
        

        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_range', 0.0),
                ('max_range', 4.0),
                ('min_intensity', 200.0),  # Higher threshold - stricter noise filtering
                ('max_intensity', 10000.0),  # Filter very high intensity (reflections)
                ('smoothing_window_size', 7),
                ('scan_topic', '/scan'),
                ('filtered_scan_topic', '/scan_filtered'),
            ]
        )


        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.min_intensity = self.get_parameter('min_intensity').value
        self.max_intensity = self.get_parameter('max_intensity').value
        self.smoothing_window_size = self.get_parameter('smoothing_window_size').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.output_topic = self.get_parameter('filtered_scan_topic').value
        

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            sensor_qos
        )
        
        self.scan_pub = self.create_publisher(
            LaserScan,
            self.output_topic,
            10
        )
        
        self.get_logger().info('Lidar Filter Node initialized')
        self.get_logger().info(f'  Range filter: [{self.min_range}, {self.max_range}] m')
        self.get_logger().info(f'  Intensity filter: [{self.min_intensity}, {self.max_intensity}]')
        self.get_logger().info(f'  Smoothing window: {self.smoothing_window_size}')
        self.get_logger().info(f'  Input: {self.scan_topic}')
        self.get_logger().info(f'  Output: {self.output_topic}')

    def median_deviation_filter(self, ranges, window_size=10, threshold_percent=20):
        """
        Advanced median deviation filter from ch-geo/lidar_noise_filtering.

        Divides scan into windows, computes median for each window,
        and removes points that deviate more than threshold_percent from median.
        """
        result = []
        scan_size = len(ranges)
        windows_num = scan_size // window_size
        mod = scan_size % window_size

        for i in range(windows_num):
            temp = ranges[window_size*i : window_size*(i+1)]

            valid_temp = [x for x in temp if np.isfinite(x) and x > 0]

            if len(valid_temp) > 0:
                med = median(valid_temp)


                filtered_temp = []
                for elem in temp:
                    if np.isfinite(elem) and elem > 0:
                        deviation_percent = abs((med - elem) / med * 100) if med > 0 else 0
                        if deviation_percent > threshold_percent:
                            filtered_temp.append(float('nan'))  # Mark as outlier
                        else:
                            filtered_temp.append(elem)
                    else:
                        filtered_temp.append(elem)

                result.extend(filtered_temp)
            else:
                result.extend(temp)


        if mod != 0:
            temp = ranges[-mod:]
            valid_temp = [x for x in temp if np.isfinite(x) and x > 0]

            if len(valid_temp) > 0:
                med = median(valid_temp)
                filtered_temp = []
                for elem in temp:
                    if np.isfinite(elem) and elem > 0:
                        deviation_percent = abs((med - elem) / med * 100) if med > 0 else 0
                        if deviation_percent > threshold_percent:
                            filtered_temp.append(float('nan'))
                        else:
                            filtered_temp.append(elem)
                    else:
                        filtered_temp.append(elem)
                result.extend(filtered_temp)
            else:
                result.extend(temp)

        return np.array(result)

    def scan_callback(self, msg: LaserScan):
        """Filter scan and republish."""
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        
        ranges = np.array(msg.ranges)
        
        # Keep valid ranges, set others to NaN
        # (Compatible with rviz and most nodes which ignore NaNs)
        valid_mask = (ranges >= self.min_range) & (ranges <= self.max_range)
        

        if msg.intensities:
            intensities = np.array(msg.intensities)
            valid_mask &= (intensities >= self.min_intensity) & (intensities <= self.max_intensity)

            # Debug logging (throttle to 1Hz)
            # Log min/max intensity and how many points kept
            if hasattr(self, 'last_log_time'):
                now = self.get_clock().now()
                if (now - self.last_log_time).nanoseconds > 1e9:
                    self.get_logger().info(
                        f'Intensity: min={np.min(intensities):.1f}, max={np.max(intensities):.1f}, '
                        f'filter=[{self.min_intensity}, {self.max_intensity}]. '
                        f'Keeping {np.sum(valid_mask)}/{len(ranges)} points'
                    )
                    self.last_log_time = now
            else:
                self.last_log_time = self.get_clock().now()
        

        filtered_ranges = ranges.copy()

        # --- Advanced Median Deviation Filter (ch-geo implementation) ---
        # ENABLED: Removes outliers based on median deviation
        filtered_ranges = self.median_deviation_filter(
            filtered_ranges,
            window_size=10,
            threshold_percent=20
        )

        # --- Additional Smoothing (Median Filter) ---
        if self.smoothing_window_size > 1:
            # Apply additional median smoothing after outlier removal
            kernel_size = self.smoothing_window_size if self.smoothing_window_size % 2 == 1 else self.smoothing_window_size + 1

            valid_for_smooth = np.isfinite(filtered_ranges)
            if np.sum(valid_for_smooth) > kernel_size:
                filtered_ranges[valid_for_smooth] = medfilt(
                    filtered_ranges[valid_for_smooth],
                    kernel_size=min(kernel_size, np.sum(valid_for_smooth))
                )


        filtered_ranges[~valid_mask] = float('nan')
        
        filtered_scan.ranges = filtered_ranges.tolist()
        

        if msg.intensities:
            filtered_scan.intensities = msg.intensities
            
        self.scan_pub.publish(filtered_scan)



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
