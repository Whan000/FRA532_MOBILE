#!/usr/bin/env python3
# Copyright (c) 2024
# Licensed under MIT

"""
Live Trajectory Plotter Node for FRA532 Lab 1.

This node subscribes to multiple odometry topics and plots the robot trajectories
in real-time using matplotlib with a live-updating window.
"""

import rclpy
import rclpy.time
from rclpy.node import Node
import numpy as np
from collections import deque
import threading
import os
from datetime import datetime

from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32
from tf2_ros import TransformListener, Buffer
from rclpy.duration import Duration

import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg for interactive display
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class TrajectoryPlotterNode(Node):
    """
    Node that subscribes to odometry topics and plots trajectories live.
    """
    
    def __init__(self):
        super().__init__('trajectory_plotter')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_odom_topic', '/odometry/wheel'),
                ('ekf_odom_topic', '/odometry/ekf'),
                ('icp_odom_topic', '/odometry/icp'),
                ('max_points', 10000),
                ('update_interval', 0.01),  # seconds (10ms = 100Hz for smooth animation)
                ('save_dir', '/tmp/trajectory_plots'),
            ]
        )
        

        self.max_points = self.get_parameter('max_points').value
        self.update_interval = self.get_parameter('update_interval').value
        self.save_dir = self.get_parameter('save_dir').value
        

        os.makedirs(self.save_dir, exist_ok=True)
        

        self.trajectories = {
            'wheel': {'x': deque(maxlen=self.max_points), 'y': deque(maxlen=self.max_points),
                      'color': '#FF5500', 'label': 'Wheel Odometry'},
            'ekf': {'x': deque(maxlen=self.max_points), 'y': deque(maxlen=self.max_points),
                    'color': '#00FF00', 'label': 'EKF Odometry'},
            'icp': {'x': deque(maxlen=self.max_points), 'y': deque(maxlen=self.max_points),
                    'color': '#00AAFF', 'label': 'ICP Odometry'},
            'slam': {'x': deque(maxlen=self.max_points), 'y': deque(maxlen=self.max_points),
                     'color': '#FF00FF', 'label': 'SLAM Toolbox'},
        }
        
        self.start_time = None
        self.sample_counts = {'wheel': 0, 'ekf': 0, 'icp': 0, 'slam': 0}
        self.path_lengths = {'wheel': 0.0, 'ekf': 0.0, 'icp': 0.0, 'slam': 0.0}
        self.icp_quality = 0.0  # ICP quality percentage (0-100%)


        self.last_save_time = None
        self.auto_save_interval = 5.0  # Save every 5 seconds


        self.lock = threading.Lock()


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.wheel_sub = self.create_subscription(
            Odometry,
            self.get_parameter('wheel_odom_topic').value,
            lambda msg: self.odom_callback(msg, 'wheel'),
            10
        )
        
        self.ekf_sub = self.create_subscription(
            Odometry,
            self.get_parameter('ekf_odom_topic').value,
            lambda msg: self.odom_callback(msg, 'ekf'),
            10
        )
        
        self.icp_sub = self.create_subscription(
            Odometry,
            self.get_parameter('icp_odom_topic').value,
            lambda msg: self.odom_callback(msg, 'icp'),
            10
        )


        self.slam_graph_sub = self.create_subscription(
            MarkerArray,
            '/slam_toolbox/graph_visualization',
            self.slam_graph_callback,
            10
        )


        self.icp_quality_sub = self.create_subscription(
            Float32,
            '/icp/quality',
            self.icp_quality_callback,
            10
        )

        self.get_logger().info('Trajectory Plotter Node initialized')
        self.get_logger().info(f'  Saving plots to: {self.save_dir}')
        
        # Setup matplotlib figure
        self.setup_plot()
    
    def setup_plot(self):
        """Setup the matplotlib figure for live plotting."""
        plt.style.use('dark_background')
        # Single full-size plot
        self.fig, self.ax = plt.subplots(figsize=(14, 10))

        self.fig.canvas.manager.set_window_title('FRA532 LAB1 - Trajectory Comparison')

        # Dark theme
        self.fig.patch.set_facecolor('#1e1e1e')
        self.ax.set_facecolor('#2d2d2d')


        self.lines = {}
        self.start_markers = {}
        self.end_markers = {}

        for name, traj in self.trajectories.items():
            line, = self.ax.plot([], [], color=traj['color'], linewidth=2.0,
                                 label=traj['label'], alpha=0.9)
            self.lines[name] = line


            start, = self.ax.plot([], [], 'o', color=traj['color'], markersize=10)
            self.start_markers[name] = start


            end, = self.ax.plot([], [], 's', color=traj['color'], markersize=8)
            self.end_markers[name] = end

        self.ax.set_xlabel('X Position (m)', color='white', fontsize=12)
        self.ax.set_ylabel('Y Position (m)', color='white', fontsize=12)
        self.ax.set_title('Robot Trajectory Comparison', color='white',
                         fontsize=14, fontweight='bold', pad=15)
        self.ax.tick_params(colors='white', labelsize=10)
        for spine in self.ax.spines.values():
            spine.set_color('white')
            spine.set_linewidth(1.5)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3, color='gray', linestyle='--', linewidth=0.5)
        self.ax.legend(loc='upper left', facecolor='#3d3d3d', edgecolor='white',
                      labelcolor='white', fontsize=10, framealpha=0.9)

        # Stats overlay (upper right corner)
        self.stats_text = self.ax.text(0.98, 0.98, '', transform=self.ax.transAxes,
                                       fontsize=9, verticalalignment='top',
                                       horizontalalignment='right',
                                       family='monospace',
                                       color='white',
                                       bbox=dict(boxstyle='round,pad=0.5',
                                                facecolor='#1e1e1e',
                                                edgecolor='white',
                                                alpha=0.85))
    
    def odom_callback(self, msg: Odometry, source: str):
        """Store odometry position and update metrics."""
        with self.lock:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y


            if len(self.trajectories[source]['x']) > 0:
                prev_x = self.trajectories[source]['x'][-1]
                prev_y = self.trajectories[source]['y'][-1]
                distance = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
                self.path_lengths[source] += distance

            self.trajectories[source]['x'].append(x)
            self.trajectories[source]['y'].append(y)
            self.sample_counts[source] += 1


            if self.start_time is None:
                self.start_time = self.get_clock().now()

    def slam_graph_callback(self, msg: MarkerArray):
        """
        Parse SLAM graph nodes from visualization markers.
        The graph_visualization topic contains the optimized pose graph nodes.
        """

        with self.lock:
            self.trajectories['slam']['x'].clear()
            self.trajectories['slam']['y'].clear()
            self.path_lengths['slam'] = 0.0

            # Extract pose from each marker in the array
            # slam_toolbox publishes markers with namespace 'slam_toolbox'
            prev_x, prev_y = None, None
            for marker in msg.markers:
                if marker.ns == 'slam_toolbox':  # Graph nodes
                    x = marker.pose.position.x
                    y = marker.pose.position.y
                    self.trajectories['slam']['x'].append(x)
                    self.trajectories['slam']['y'].append(y)

        
                    if prev_x is not None:
                        distance = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
                        self.path_lengths['slam'] += distance
                    prev_x, prev_y = x, y

            self.sample_counts['slam'] = len(self.trajectories['slam']['x'])

    def icp_quality_callback(self, msg: Float32):
        """Update ICP quality percentage from /icp/quality topic."""
        with self.lock:
            self.icp_quality = msg.data

    def update_plot(self, frame):
        """Update function for animation."""
        with self.lock:
            all_x = []
            all_y = []

    
            for name, traj in self.trajectories.items():
                if len(traj['x']) > 0:
                    x = list(traj['x'])
                    y = list(traj['y'])

        
                    self.lines[name].set_data(x, y)

        
                    self.start_markers[name].set_data([x[0]], [y[0]])
                    self.end_markers[name].set_data([x[-1]], [y[-1]])

                    all_x.extend(x)
                    all_y.extend(y)

    
            if all_x and all_y:
                margin = 0.5
                x_min, x_max = min(all_x) - margin, max(all_x) + margin
                y_min, y_max = min(all_y) - margin, max(all_y) + margin

        
                x_range = x_max - x_min
                y_range = y_max - y_min
                max_range = max(x_range, y_range)

                x_center = (x_min + x_max) / 2
                y_center = (y_min + y_max) / 2

                self.ax.set_xlim(x_center - max_range/2, x_center + max_range/2)
                self.ax.set_ylim(y_center - max_range/2, y_center + max_range/2)

    
            self.update_stats_overlay()

    
            current_time = self.get_clock().now()
            if self.last_save_time is None:
                self.last_save_time = current_time
            elif (current_time - self.last_save_time).nanoseconds / 1e9 > self.auto_save_interval:
                self.save_final_plot()
                self.last_save_time = current_time

        return (list(self.lines.values()) + list(self.start_markers.values()) +
                list(self.end_markers.values()))

    def update_stats_overlay(self):
        """Update the stats overlay text."""
        # Calculate elapsed time
        elapsed_str = 'N/A'
        if self.start_time is not None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            mins = int(elapsed // 60)
            secs = int(elapsed % 60)
            elapsed_str = f'{mins:02d}:{secs:02d}'

        # Build stats text
        stats_lines = [f'Runtime: {elapsed_str}']

        # ICP Quality
        stats_lines.append(f'ICP Quality: {self.icp_quality:.1f}%')
        stats_lines.append('')

        # Path distances
        stats_lines.append('Path Distance:')
        for name, label in [('wheel', 'Wheel'), ('ekf', 'EKF'), ('icp', 'ICP'), ('slam', 'SLAM')]:
            dist = self.path_lengths[name]
            if dist > 0:
                stats_lines.append(f'  {label}: {dist:.2f}m')

        self.stats_text.set_text('\n'.join(stats_lines))
    
    def save_final_plot(self):
        """Save final plot on shutdown."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(self.save_dir, f'trajectory_final_{timestamp}.png')
        self.fig.savefig(filepath, dpi=150, facecolor=self.fig.get_facecolor())
        self.get_logger().info(f'Saved final trajectory plot: {filepath}')
    
    def run(self):
        """Run the node with live animation."""
        # Create animation
        self.ani = FuncAnimation(
            self.fig, 
            self.update_plot, 
            interval=int(self.update_interval * 1000),  # Convert to ms
            blit=False,
            cache_frame_data=False
        )
        
        # Start ROS spinning in a separate thread
        spin_thread = threading.Thread(target=self.spin_ros, daemon=True)
        spin_thread.start()
        
        # Show plot (blocking)
        plt.show()
        
        # Save when window is closed
        self.save_final_plot()
    
    def spin_ros(self):
        """Spin ROS in background thread."""
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotterNode()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.save_final_plot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
