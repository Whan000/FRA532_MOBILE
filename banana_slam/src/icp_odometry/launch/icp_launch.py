#!/usr/bin/env python3
# Copyright (c) 2024
# Licensed under MIT

"""Launch file for ICP Odometry node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('icp_odometry')
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (for bag playback)'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'icp_params.yaml'),
        description='Path to parameters file'
    )
    
    # Simple ICP Odometry Node (pZ style - pure scan-to-scan)
    icp_node = Node(
        package='icp_odometry',
        executable='simple_icp_node',  # Use simple implementation
        name='icp_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/odometry/ekf', '/odometry/ekf'),
        ]
    )
    
    # Lidar Filter Node
    lidar_filter_node = Node(
        package='icp_odometry',
        executable='lidar_filter_node',
        name='lidar_filter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        lidar_filter_node,
        icp_node,
    ])
