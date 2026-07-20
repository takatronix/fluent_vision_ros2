import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("fv_visual_condition_detector")
    default_config = os.path.join(pkg_share, "config", "default.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("namespace", default_value="/fv"),
        Node(
            package="fv_visual_condition_detector",
            executable="fv_visual_condition_detector_node",
            name="fv_visual_condition_detector",
            namespace=LaunchConfiguration("namespace"),
            parameters=[LaunchConfiguration("config")],
            output="screen",
        ),
    ])
