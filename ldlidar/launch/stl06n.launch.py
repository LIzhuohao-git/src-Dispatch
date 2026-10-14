#!/usr/bin/env python3
from pathlib import Path

import json
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

'''
Parameter Description:
---
- Set laser scan directon:
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': True}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
- Serial port communication mode or network port communication mode configuration
  --enable serial communication mode:
  {'enable_serial_or_network_communication': True}
  --enable network communication mode:
  {'enable_serial_or_network_communication': False}

- Web 端参数配置页覆盖：~/.originbot_delivery/config/generated/lidar.json
  保存参数时生成，此处启动时读取；文件不存在则使用下方出厂默认值。

安装位姿（mount_*）是本文件发布的 base_link -> laser_link 静态 TF 的唯一来源：
雷达挪位置/装歪只改 Web 端参数配置页（或直接改 lidar.json）即可，无需改代码。
'''

# 出厂默认参数（可在 Web 端参数配置页修改）
LIDAR_DEFAULTS = {
    'laser_scan_dir': True,          # False:雷达倒装  True:雷达正装
    'enable_angle_crop_func': False,  # 关闭角度裁剪
    'angle_crop_min': 95.0,
    'angle_crop_max': 265.0,
    'min_range': 0.1,
    'max_range': 20.0,
    # base_link -> laser_link 安装位姿。
    # 注意：历代注释写的是"前方 5cm，上方 15cm"，但代码一直按 (0.15, 0, 0.08)
    # 发布 TF——注释与代码矛盾，以代码实际生效值为准，物理真值请用钢尺实测。
    # 用 tools/lidar_crop_check.py 可辅助核对。
    'mount_x': 0.15,
    'mount_y': 0.0,
    'mount_z': 0.08,
    'mount_roll': 0.0,
    'mount_pitch': 0.0,
    'mount_yaw': 0.0,
}


def _load_overrides():
    """读取 Web 端参数配置页生成的雷达参数覆盖文件。

    键名与 ldlidar 驱动声明的 ROS 参数名保持一致。早期版本 Web 端写出的
    是 scan_dir / enable_angle_crop（与驱动参数名不符，导致这两项被静默
    忽略），这里做一次别名映射，兼容尚未重新保存参数的旧配置。
    """
    overrides_path = (
        Path.home() / '.originbot_delivery' / 'config' / 'generated' / 'lidar.json'
    )
    params = dict(LIDAR_DEFAULTS)
    try:
        with open(overrides_path, 'r', encoding='utf-8') as handle:
            stored = json.load(handle)
    except (OSError, ValueError):
        return params
    for key in params:
        if key in stored:
            params[key] = stored[key]
    # 兼容旧键名
    for new_key, old_key in (('laser_scan_dir', 'scan_dir'),
                             ('enable_angle_crop_func', 'enable_angle_crop')):
        if new_key not in stored and old_key in stored:
            params[new_key] = stored[old_key]
    return params


def _setup(context):
    params = _load_overrides()
    ldlidar_node = Node(
        package='ldlidar',
        executable='ldlidar',
        name='STL06N',
        output='screen',
        parameters=[
            {
                'product_name': 'LDLiDAR_STL06N',
                'topic_name': 'scan',
                'frame_id': 'laser_link',  # 与TF中的child_frame_id保持一致
                'enable_serial_or_network_communication': True,
                'port_name': '/dev/ttyACM0',
                'port_baudrate': 230400,
                'server_ip': '192.168.1.200',
                'server_port': '2368',
                'laser_scan_dir': params['laser_scan_dir'],
                'enable_angle_crop_func': params['enable_angle_crop_func'],
                'angle_crop_min': float(params['angle_crop_min']),
                'angle_crop_max': float(params['angle_crop_max']),
                'min_range': float(params['min_range']),
                'max_range': float(params['max_range']),
                'measure_point_freq': 5000
            }
        ]
    )

    # 安装位姿用 params.get 取，兼容旧版 lidar.json（缺键时回落到出厂默认值）
    def mount(key):
        return float(params.get(key, LIDAR_DEFAULTS[key]))

    # base_link to laser_link tf node
    # 安装位姿统一由 Web 端参数配置页生成的 lidar.json 提供（见文件头说明）
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_tf',
        arguments=[
            str(mount('mount_x')), str(mount('mount_y')), str(mount('mount_z')),
            str(mount('mount_roll')), str(mount('mount_pitch')), str(mount('mount_yaw')),
            'base_link', 'laser_link'
        ]
    )

    return [ldlidar_node, base_link_to_laser_tf_node]


def generate_launch_description():
    # Define LaunchDescription variable
    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=_setup))

    return ld
