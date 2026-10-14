#!/usr/bin/env python3
# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0 (the "License");
#
# slam_toolbox 在线异步建图（替代 cartographer.launch.py）。
#
# 发布行为与 cartographer 一致：
#   - 发布 map -> odom 的 TF（transform_publish_period=0.02）
#   - 发布 /map 栅格地图（use_map_saver=true，latched）
# 因此 delivery_server 的「捕获 map->odom → 停止建图 → AMCL 初始位姿」交接流程无需改动。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('originbot_navigation')
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_params_file,
            description='Full path to slam_toolbox params yaml'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
