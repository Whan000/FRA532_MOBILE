#!/usr/bin/env python3
# Copyright (c) 2024
# Licensed under MIT

"""
Complete SLAM pipeline launch file (Parts 1, 2, and 3).

This launches:
- Part 1: EKF Odometry Fusion (wheel + IMU)
- Part 2: ICP Odometry Refinement (LiDAR scan matching)
- Part 3: Full SLAM with slam_toolbox (SLAM with loop closure)

Usage:
    # With RViz visualization
    ros2 launch banana_odom full_slam_launch.py use_sim_time:=true

    # Start bag separately, then launch
    ros2 bag play dataset.db3 --clock &
    sleep 2
    ros2 launch banana_odom full_slam_launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directories
    banana_odom_share = get_package_share_directory('banana_odom')

    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (for bag playback)'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(banana_odom_share, 'rviz', 'slam_viz.rviz'),
        description='Path to RViz config file'
    )

    # Static TF publishers (same as before)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    static_tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_footprint_base',
        arguments=['--x', '0', '--y', '0', '--z', '0.010',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    static_tf_base_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_scan',
        arguments=['--x', '0', '--y', '0', '--z', '0.172',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'base_scan'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=['--x', '0', '--y', '0', '--z', '0.068',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Include EKF launch (Part 1)
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('banana_odom'),
                'launch',
                'ekf_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Include ICP launch (Part 2)
    icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('icp_odometry'),
                'launch',
                'icp_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Include SLAM Toolbox launch (Part 3)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('banana_odom'),
                'launch',
                'slam_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Trajectory Plotter node
    plotter_node = Node(
        package='banana_odom',
        executable='trajectory_plotter',
        name='trajectory_plotter',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'save_dir': '/tmp/trajectory_plots'},
            {'update_interval': 0.01},  # 100Hz update rate
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_config_arg,
        # Static TF publishers
        static_tf_map_odom,
        static_tf_footprint_base,
        static_tf_base_scan,
        static_tf_imu,
        # Launch includes
        ekf_launch,
        icp_launch,
        slam_launch,
        rviz_node,
        plotter_node,  # Added trajectory plotter!
    ])
