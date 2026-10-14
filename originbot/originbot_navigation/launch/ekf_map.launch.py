#!/usr/bin/env python3
# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0 (the "License");

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('originbot_navigation')
    ekf_map_file = os.path.join(pkg_share, 'config', 'ekf_map.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ekf_map_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_map_node',
        output='screen',
        parameters=[ekf_map_file, {'use_sim_time': use_sim_time}],
    )

    map_to_odom_node = Node(
        package='originbot_navigation',
        executable='map_to_odom',
        name='map_to_odom',
        output='screen',
        parameters=[{'map_pose_topic': '/odometry/filtered'}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        ekf_map_node,
        map_to_odom_node,
    ])
