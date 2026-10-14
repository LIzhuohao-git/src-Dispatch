from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_file = LaunchConfiguration("map_file", default="")

    return LaunchDescription([
        DeclareLaunchArgument("map_file", default_value=""),
        Node(
            package="originbot_delivery",
            executable="map_viewer",
            name="map_viewer",
            output="screen",
            parameters=[{"map_file": map_file}],
        ),
    ])


if __name__ == "__main__":
    generate_launch_description()
