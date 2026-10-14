#!/usr/bin/env python3
# 轨道路网导航节点启动文件。
# 由 delivery_server 作为子系统 "route" 管理（随导航栈一起启停），
# 也可单独调试：ros2 launch originbot_delivery route.launch.py
#
# 参数来源（优先级从高到低）：
#   1. launch 显式传参（调试用，如 max_vel_x:=0.3）
#   2. Web 端参数配置页生成的 ~/.originbot_delivery/config/generated/route.json
#   3. 本文件内置默认值（与 route_follower 节点内声明一致）
import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 与 route_follower.py 节点内 declare_parameter 的默认值保持一致
ROUTE_PARAM_DEFAULTS = {
    "control_rate": 20.0,
    "max_vel_x": 0.22,
    "max_vel_theta": 0.8,
    "v_accel": 0.5,
    "w_accel": 2.0,
    "lookahead": 0.35,
    "rotate_threshold": 0.9,
    "goal_xy_tolerance": 0.10,
    "goal_yaw_tolerance": 0.10,
    "align_yaw_speed": 0.6,
    "end_decel": 0.5,
    "scan_half_fov_deg": 30.0,
    "stop_distance": 0.45,
    "slow_distance": 0.80,
    "resume_clear_sec": 1.5,
    "scan_stale_sec": 3.0,
    "on_ramp_max": 0.60,
    "nearest_node_max": 1.50,
}


def _load_overrides():
    """读取 Web 端参数配置页生成的 route.json；缺失/损坏时静默用默认值。"""
    params = dict(ROUTE_PARAM_DEFAULTS)
    path = Path.home() / ".originbot_delivery" / "config" / "generated" / "route.json"
    try:
        with open(path, "r", encoding="utf-8") as handle:
            stored = json.load(handle)
    except (OSError, ValueError):
        return params
    for key in params:
        if key in stored:
            try:
                params[key] = float(stored[key])
            except (TypeError, ValueError):
                pass
    return params


def _setup(context):
    params = _load_overrides()
    # launch 显式传参优先于 route.json（与 robot.launch.py 的标定系数同套路）
    for key in params:
        text = ""
        try:
            text = context.launch_configurations.get(key, "") or ""
        except AttributeError:
            pass
        if isinstance(text, str) and text.strip():
            try:
                params[key] = float(text)
            except ValueError:
                print(f"[route.launch] {key}='{text}' 不是合法数字，忽略")
    node = Node(
        package="originbot_delivery",
        executable="route_follower",
        name="route_follower",
        output="screen",
        parameters=[{
            "routes_file": LaunchConfiguration("routes_file"),
            "map_frame": "map",
            "base_frame": "base_footprint",
            **params,
        }],
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "routes_file",
            default_value="",
            description="路网 JSON 路径；留空用 ~/.originbot_delivery/config/routes.json"),
        OpaqueFunction(function=_setup),
    ])
