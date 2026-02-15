#!/usr/bin/env python3
# Copyright (c) 2024
# Licensed under MIT

"""
Launch file for SLAM Toolbox (Part 3 of Lab).

This launch file runs slam_toolbox with the filtered LiDAR scans
and EKF odometry to perform full SLAM with loop closure.

Usage:
    ros2 launch banana_odom slam_launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('banana_odom')

    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (for bag playback)'
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            pkg_share,
            'config',
            'slam_toolbox',
            'mapper_params_online_async.yaml'
        ),
        description='Path to slam_toolbox parameters file'
    )

    # SLAM Toolbox Node (Async Mapping)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/scan', '/scan_filtered'),  # Use filtered LiDAR data
            ('/odom', '/odometry/ekf'),   # Use EKF odometry
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_file_arg,
        slam_toolbox_node,
    ])
