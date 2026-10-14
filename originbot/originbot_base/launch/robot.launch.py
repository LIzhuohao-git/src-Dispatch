import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# --------------------- base_link -> imu_link 默认安装位姿 ---------------------
# imu安装在小车x：-5cm，y：+10cm的位置，并且顺时针方向旋转了90°，然后上下翻转了安装
# Web 端参数配置页可修改：~/.originbot_delivery/config/generated/imu.json
IMU_DEFAULTS = {
    "x": -0.05, "y": 0.10, "z": 0.05,
    "roll": 3.1416, "pitch": 0.0, "yaw": 1.5708,
}

# --------------------- 底盘速度标定默认值 ---------------------
# 与 originbot_delivery/originbot_delivery/robot_params.py 的 chassis 组保持一致。
# Web 端参数配置页可修改：~/.originbot_delivery/config/generated/chassis.json
CHASSIS_DEFAULTS = {
    "correct_factor_vx": 1.00715,
    "correct_factor_vth": 1.016434,
}


def _load_imu_overrides():
    overrides_path = (
        Path.home() / ".originbot_delivery" / "config" / "generated" / "imu.json"
    )
    params = dict(IMU_DEFAULTS)
    try:
        with open(overrides_path, "r", encoding="utf-8") as handle:
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


def _load_chassis_overrides():
    """读取 Web 端参数配置页生成的底盘速度标定值。"""
    overrides_path = (
        Path.home() / ".originbot_delivery" / "config" / "generated" / "chassis.json"
    )
    params = dict(CHASSIS_DEFAULTS)
    try:
        with open(overrides_path, "r", encoding="utf-8") as handle:
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


def _ekf_enabled():
    """EKF 融合开关（Web 端参数配置页 ekf.enabled，未生成时默认关闭）。"""
    try:
        path = (Path.home() / ".originbot_delivery" / "config" / "generated"
                / "ekf.json")
        with open(path, "r", encoding="utf-8") as handle:
            return bool(json.load(handle).get("enabled", False))
    except (OSError, ValueError):
        return False


def _resolve_factor(context, name, fallback):
    """解析标定系数：显式传入的 launch 参数 > chassis.json > 内置默认值。

    优先级设计的原因：delivery_server 用 `ros2 launch originbot_bringup
    originbot.launch.py` 启动底盘，不会传标定参数；而 Web 端参数配置页保存后
    生成 chassis.json。若让 DeclareLaunchArgument 带非空默认值，它会覆盖
    chassis.json，Web 页改的参数就永远不生效——这正是老版本的老问题。
    因此这里默认值留空串，空串即视为"未指定"，回落到 chassis.json。
    """
    try:
        text = context.launch_configurations.get(name, "")
    except AttributeError:
        text = ""
    if isinstance(text, str) and text.strip():
        try:
            return float(text)
        except ValueError:
            print(f"[robot.launch] {name}='{text}' 不是合法数字，回落到 {fallback}")
    return fallback


def _setup(context):
    imu_pose = _load_imu_overrides()
    chassis = _load_chassis_overrides()
    correct_factor_vx = _resolve_factor(
        context, 'correct_factor_vx', chassis['correct_factor_vx'])
    correct_factor_vth = _resolve_factor(
        context, 'correct_factor_vth', chassis['correct_factor_vth'])

    port_name_arg = LaunchConfiguration('port_name')
    auto_stop_on_arg = LaunchConfiguration('auto_stop_on')
    use_imu_arg = LaunchConfiguration('use_imu')
    pub_odom_arg = LaunchConfiguration('pub_odom')

    # --------------------- base node ---------------------
    originbot_base_node = Node(
        package='originbot_base',
        executable='originbot_base',
        output='screen',
        parameters=[{
            'port_name': port_name_arg,
            'correct_factor_vx': correct_factor_vx,
            'correct_factor_vth': correct_factor_vth,
            'auto_stop_on': auto_stop_on_arg,
            'use_imu': use_imu_arg,
            'pub_odom': pub_odom_arg,
        }]
    )

    # --------------------- base_footprint -> base_link ---------------------
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_basefoot_to_baselink',
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.13325",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "base_footprint", "--child-frame-id", "base_link"
        ]
    )

    # --------------------- base_link -> imu_link ---------------------
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu_to_base',
        arguments=[
            "--x", str(imu_pose["x"]), "--y", str(imu_pose["y"]), "--z", str(imu_pose["z"]),
            "--roll", str(imu_pose["roll"]), "--pitch", str(imu_pose["pitch"]),
            "--yaw", str(imu_pose["yaw"]),
            "--frame-id", "base_link", "--child-frame-id", "imu_link"
        ]
    )

    return [
        originbot_base_node,
        base_footprint_tf,
        imu_tf,
    ]


def generate_launch_description():
    port_name_arg = DeclareLaunchArgument('port_name', default_value='ttyS1',
                                          description='usb bus name, e.g. ttyS1')

    correct_factor_vx_arg = DeclareLaunchArgument(
        'correct_factor_vx', default_value='',
        description='线速度标定系数；留空则读取 Web 端生成的 chassis.json')

    correct_factor_vth_arg = DeclareLaunchArgument(
        'correct_factor_vth', default_value='',
        description='角速度标定系数；留空则读取 Web 端生成的 chassis.json')

    auto_stop_on_arg = DeclareLaunchArgument('auto_stop_on', default_value='true',
                                             description='auto stop if no cmd received, true or true')

    use_imu_arg = DeclareLaunchArgument('use_imu', default_value='true',
                                        description='if has imu sensor to drive')

    pub_odom_arg = DeclareLaunchArgument(
        'pub_odom',
        # EKF 启用时由 robot_localization 发布 odom→base_footprint TF，
        # 底盘必须停发，否则两个节点争抢同一 TF 会导致定位抖动
        default_value='false' if _ekf_enabled() else 'true',
        description='publish odom to base_footprint tf (auto-disabled when EKF enabled)')

    ld = LaunchDescription([
        port_name_arg,
        correct_factor_vx_arg,
        correct_factor_vth_arg,
        auto_stop_on_arg,
        use_imu_arg,
        pub_odom_arg,
    ])
    ld.add_action(OpaqueFunction(function=_setup))
    return ld
