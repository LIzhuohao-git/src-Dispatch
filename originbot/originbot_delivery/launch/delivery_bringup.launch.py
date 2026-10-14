import subprocess
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _ensure_single_instance():
    result = subprocess.run(
        ["pgrep", "-f", "lib/originbot_delivery/delivery_server"],
        capture_output=True,
        text=True,
    )
    if result.returncode == 0:
        print(
            "检测到已有 delivery_server 在运行，请勿重复启动。"
            "如需重启，请先停止当前 delivery_bringup 再启动。",
            file=sys.stderr,
        )
        sys.exit(1)


def generate_launch_description():
    _ensure_single_instance()
    web_port = LaunchConfiguration("web_port", default="8080")
    rosbridge_port = LaunchConfiguration("rosbridge_port", default="9090")

    delivery_server = Node(
        package="originbot_delivery",
        executable="delivery_server",
        name="delivery_server",
        output="screen",
    )

    web_server = Node(
        package="originbot_delivery",
        executable="web_server",
        name="delivery_web_server",
        output="screen",
        parameters=[{"port": web_port}],
    )

    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "address": "0.0.0.0",
                "port": rosbridge_port,
                "retry_startup_delay": 2.0,
                "fragment_timeout": 600,
                "delay_between_messages": 0,
                "max_message_size": 10000000,
            }
        ],
    )

    rosapi = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("web_port", default_value="8080"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        delivery_server,
        web_server,
        rosbridge,
        rosapi,
    ])
