import json
import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 相机安装位姿默认值；Web 端参数配置页保存后生成的
# ~/.originbot_delivery/config/generated/camera.json 会在此处启动时覆盖。
CAMERA_DEFAULTS = {"x": 0.12, "y": 0.0, "z": 0.25, "yaw": 0.0}


def _load_camera_overrides():
    try:
        overrides_path = (
            Path.home() / ".originbot_delivery" / "config" / "generated" / "camera.json"
        )
        with open(overrides_path, "r", encoding="utf-8") as handle:
            stored = json.load(handle)
    except (OSError, ValueError):
        return dict(CAMERA_DEFAULTS)
    params = dict(CAMERA_DEFAULTS)
    for key in params:
        if key in stored:
            try:
                params[key] = float(stored[key])
            except (TypeError, ValueError):
                pass
    return params


def _setup(context):
    base_frame = LaunchConfiguration("base_frame").perform(context)
    camera_frame = LaunchConfiguration("camera_frame").perform(context)
    image_topic = LaunchConfiguration("image_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    marker_file = LaunchConfiguration("marker_file").perform(context)
    overrides = _load_camera_overrides()
    # launch 参数留空时使用 Web 端参数配置页生成的覆盖值（无覆盖文件则用出厂默认）
    camera_x = LaunchConfiguration("camera_x").perform(context) or str(overrides["x"])
    camera_y = LaunchConfiguration("camera_y").perform(context) or str(overrides["y"])
    camera_z = LaunchConfiguration("camera_z").perform(context) or str(overrides["z"])
    camera_yaw_deg = LaunchConfiguration("camera_yaw").perform(context) or str(overrides["yaw"])
    # camera_yaw: 相机绕竖直轴的安装偏转角（0 = 光轴朝正前方）。
    # child 帧是 OpenCV 光学系（x 右、y 下、z 朝前），必须叠加标准光学旋转
    # roll=-90°/yaw=-90° 把它从 base 系（x 前、y 左、z 上）转过去；
    # 否则 PnP 的前向距离（光系 z）会被当成高度，定位/登记全部畸变。
    camera_yaw = math.radians(float(camera_yaw_deg))
    optical_yaw = camera_yaw - math.pi / 2.0

    static_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_tf",
        arguments=[
            "--x", camera_x,
            "--y", camera_y,
            "--z", camera_z,
            "--roll", "-1.5708",
            "--pitch", "0.0",
            "--yaw", f"{optical_yaw:.6f}",
            "--frame-id", base_frame,
            "--child-frame-id", camera_frame,
        ],
    )

    aruco_node = Node(
        package="originbot_delivery",
        executable="aruco_localization",
        name="aruco_localization",
        output="screen",
        parameters=[
            {
                "base_frame": base_frame,
                "camera_frame": camera_frame,
                "image_topic": image_topic,
                "camera_info_topic": camera_info_topic,
                "marker_file": marker_file,
            }
        ],
    )
    return [static_camera_tf, aruco_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("camera_frame", default_value="camera_optical"),
        DeclareLaunchArgument("image_topic", default_value="/image"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera_info_latched"),
        DeclareLaunchArgument("marker_file", default_value=""),
        DeclareLaunchArgument("camera_x", default_value=""),
        DeclareLaunchArgument("camera_y", default_value=""),
        DeclareLaunchArgument("camera_z", default_value=""),
        DeclareLaunchArgument("camera_yaw", default_value=""),
        OpaqueFunction(function=_setup),
    ])
