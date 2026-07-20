from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("fv_soundboard")
    return LaunchDescription([
        DeclareLaunchArgument(
            "registry_file", default_value=f"{share}/config/events.yaml"
        ),
        DeclareLaunchArgument("sounds_dir", default_value=f"{share}/sounds"),
        Node(
            package="fv_soundboard",
            executable="fv_soundboard_node",
            name="fv_soundboard",
            output="screen",
            parameters=[{
                "registry_file": LaunchConfiguration("registry_file"),
                "sounds_dir": LaunchConfiguration("sounds_dir"),
            }],
        ),
    ])
