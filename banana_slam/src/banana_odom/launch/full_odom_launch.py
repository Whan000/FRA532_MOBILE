#!/usr/bin/env python3
# Copyright (c) 2024
# Licensed under MIT

"""Combined launch file for EKF + ICP Odometry pipeline with RViz and Plotter.

Usage:
    # With bag file (recommended - handles sim_time correctly):
    ros2 launch banana_odom full_odom_launch.py bag_file:=/path/to/bag.db3

    # Without bag file (you must start bag with --clock separately FIRST):
    ros2 bag play /path/to/bag.db3 --clock &
    sleep 2
    ros2 launch banana_odom full_odom_launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
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
        default_value=os.path.join(banana_odom_share, 'rviz', 'odom_viz.rviz'),
        description='Path to RViz config file'
    )

    save_dir_arg = DeclareLaunchArgument(
        'save_dir',
        default_value='/tmp/trajectory_plots',
        description='Directory to save trajectory plots'
    )

    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='',
        description='Path to bag file to play. If empty, no bag is played.'
    )

    bag_rate_arg = DeclareLaunchArgument(
        'bag_rate',
        default_value='1.0',
        description='Bag playback rate'
    )

    # TF Tree:
    # map -> odom -> base_footprint -> base_link -> base_scan
    #                                            -> imu_link
    #                                            -> wheel_left_link
    #                                            -> wheel_right_link

    # Static TF: map -> odom (identity - no loop closure yet)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Static TF: base_footprint -> base_link
    static_tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_footprint_base',
        arguments=['--x', '0', '--y', '0', '--z', '0.010',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Static TF: base_link -> base_scan
    static_tf_base_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_scan',
        arguments=['--x', '0', '--y', '0', '--z', '0.172',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'base_scan'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Static TF: base_link -> imu_link
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=['--x', '0', '--y', '0', '--z', '0.068',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Static TF: base_link -> wheel_left_link
    static_tf_wheel_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_wheel_left',
        arguments=['--x', '0', '--y', '0.080', '--z', '0.033',
                   '--roll', '-1.5708', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'wheel_left_link'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Static TF: base_link -> wheel_right_link
    static_tf_wheel_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_wheel_right',
        arguments=['--x', '0', '--y', '-0.080', '--z', '0.033',
                   '--roll', '-1.5708', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'wheel_right_link'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Include EKF launch
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

    # Include ICP launch
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
            {'save_dir': LaunchConfiguration('save_dir')},
            {'update_interval': 2.0},
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_config_arg,
        save_dir_arg,
        bag_file_arg,
        bag_rate_arg,
        # Static TF publishers
        static_tf_map_odom,
        static_tf_footprint_base,
        static_tf_base_scan,
        static_tf_imu,
        static_tf_wheel_left,
        static_tf_wheel_right,
        # Nodes
        ekf_launch,
        icp_launch,
        rviz_node,
        plotter_node,
    ])
