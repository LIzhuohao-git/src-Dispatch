#!/usr/bin/python3

# Copyright (c) 2022, www.guyuehome.com
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
import launch_ros
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _ekf_settings():
    """读取 Web 端参数配置页生成的 EKF 设置。

    返回 (是否启用, 参数文件路径)。ekf.json 由 robot_params 生成，含 enabled
    开关；未生成时视为不启用（默认纯轮式里程计）。
    """
    generated_dir = os.path.expanduser('~/.originbot_delivery/config/generated')
    enabled = False
    try:
        with open(os.path.join(generated_dir, 'ekf.json'), 'r', encoding='utf-8') as handle:
            enabled = bool(json.load(handle).get('enabled', False))
    except (OSError, ValueError):
        pass
    params_file = os.path.join(generated_dir, 'ekf.yaml')
    if not os.path.isfile(params_file):
        pkg_share = launch_ros.substitutions.FindPackageShare(
            package='originbot_navigation').find('originbot_navigation')
        params_file = os.path.join(pkg_share, 'config/ekf.yaml')
    return enabled, params_file


def generate_launch_description():

    ld =  launch.LaunchDescription()
    ekf_enabled, robot_localization_file_path = _ekf_settings()
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                            description='Flag to disable use_sim_time'))

    if not ekf_enabled:
        # Web 端参数配置页关闭了 EKF：不启动任何节点。
        # 此时 odom→base_footprint TF 由底盘(originbot_base pub_odom)发布。
        ld.add_action(launch.actions.LogInfo(
            msg='EKF 融合未启用（参数配置页 ekf.enabled=false），跳过 robot_localization'))
        return ld

    # Start robot localization using an Extended Kalman filter
    # publish_tf=true：启用时 EKF 接管 odom→base_footprint TF，
    # 同时底盘会以 pub_odom=false 停发该 TF，避免两个节点争抢。
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path,
        {'use_sim_time': use_sim_time, 'publish_tf': True}])

    ld.add_action(robot_localization_node)
    return ld
