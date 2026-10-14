#!/usr/bin/python3

# Copyright (c) 2024, www.guyuehome.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def _ekf_enabled():
    """EKF 融合开关（Web 端参数配置页 ekf.enabled）。

    启用时底层 robot.launch.py 会让底盘停发 odom→base_footprint TF（交给
    robot_localization），这里如果仍默认传 pub_odom=true 会把动态默认值覆盖掉，
    因此本文件的默认值必须与 robot.launch.py 保持同一份逻辑。
    """
    try:
        path = os.path.expanduser(
            '~/.originbot_delivery/config/generated/ekf.json')
        with open(path, 'r', encoding='utf-8') as handle:
            return bool(json.load(handle).get('enabled', False))
    except (OSError, ValueError):
        return False


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_lidar', default_value='true'),
        launch.actions.DeclareLaunchArgument(name='use_camera', default_value='true'),
        launch.actions.DeclareLaunchArgument(name='use_imu', default_value='true'),
        launch.actions.DeclareLaunchArgument(
            name='pub_odom',
            # EKF 启用时默认 false（由 ekf_node 发布 TF），否则 true（底盘自发布）
            default_value='false' if _ekf_enabled() else 'true'),
        # 底盘速度标定系数：默认留空串表示"未指定"，交由底层 robot.launch.py
        # 回落到 Web 端参数配置页生成的 chassis.json。
        # 这里若给非空默认值，会把底层的 chassis.json 值覆盖掉，
        # 导致 Web 页改的标定参数不生效（pub_odom 曾踩过同样的坑）。
        launch.actions.DeclareLaunchArgument(
            name='correct_factor_vx', default_value='',
            description='线速度标定系数；留空则用 Web 端 chassis.json'),
        launch.actions.DeclareLaunchArgument(
            name='correct_factor_vth', default_value='',
            description='角速度标定系数；留空则用 Web 端 chassis.json'),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('originbot_base'),
                             'launch/robot.launch.py')),
                launch_arguments={
                'use_imu': launch.substitutions.LaunchConfiguration('use_imu'),
                'pub_odom': launch.substitutions.LaunchConfiguration('pub_odom'),
                'correct_factor_vx': launch.substitutions.LaunchConfiguration(
                    'correct_factor_vx'),
                'correct_factor_vth': launch.substitutions.LaunchConfiguration(
                    'correct_factor_vth'),
                }.items(),
                ),

        # 将原来的vp100.launch.py替换为stl06n.launch.py
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ldlidar'),
                             'launch', 'stl06n.launch.py')),
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('use_lidar'))),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('originbot_bringup'),
                             'launch','camera.launch.py')),
                condition=launch.conditions.IfCondition(
                    launch.substitutions.LaunchConfiguration('use_camera')))
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
