import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("web_bridge"), "config", "web_bridge.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("params_file", default_value=default_params),
        Node(
            package="web_bridge",
            executable="web_bridge",
            namespace=LaunchConfiguration("namespace"),
            parameters=[LaunchConfiguration("params_file")],
            output="screen",
        ),
    ])
