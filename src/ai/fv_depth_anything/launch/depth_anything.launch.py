from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("fv_depth_anything")
    default_config = os.path.join(pkg_share, "config", "default.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("namespace", default_value="/fv"),
        Node(
            package="fv_depth_anything",
            executable="fv_depth_anything_node",
            name="fv_depth_anything",
            namespace=LaunchConfiguration("namespace"),
            parameters=[LaunchConfiguration("config")],
            output="screen",
        ),
    ])
