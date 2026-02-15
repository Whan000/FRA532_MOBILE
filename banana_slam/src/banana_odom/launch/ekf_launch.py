#!/usr/bin/env python3
# Copyright (c) 2024
# Licensed under MIT

"""Launch file for EKF Odometry node."""

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
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'ekf_params.yaml'),
        description='Path to parameters file'
    )
    
    # EKF Odometry Node
    ekf_node = Node(
        package='banana_odom',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/joint_states', '/joint_states'),
            ('/imu', '/imu'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        ekf_node,
    ])
