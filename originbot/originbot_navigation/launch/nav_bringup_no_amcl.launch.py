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
    originbot_navigation_dir = get_package_share_directory('originbot_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration(
        'map', default=os.path.join(originbot_navigation_dir, 'maps', 'my_map.yaml'))
    nav2_param_path = LaunchConfiguration(
        'params_file',
        default=os.path.join(originbot_navigation_dir, 'param', 'originbot_nav2.yaml'))

    lifecycle_nodes = [
        'map_server',
        'planner_server',
        'controller_server',
        'smoother_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_param_path, {
                'yaml_filename': map_yaml_path,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_param_path, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'map', default_value=map_yaml_path,
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'params_file', default_value=nav2_param_path,
            description='Full path to Nav2 param file'),
        *nodes,
    ])
