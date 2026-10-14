from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_path = os.path.join(
        get_package_share_directory("originbot_delivery"),
        "launch",
        "delivery_bringup.launch.py",
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup_path))
    ])
