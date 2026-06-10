"""Launch fv_episode_recorder standalone."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="/data/datasets",
        description="Episode storage root (container path).",
    )
    port_arg = DeclareLaunchArgument(
        "port", default_value="8083",
        description="aiohttp REST API port.",
    )

    return LaunchDescription([
        output_dir_arg,
        port_arg,
        Node(
            package="fv_episode_recorder",
            executable="recorder_node",
            name="fv_episode_recorder",
            output="screen",
            parameters=[{
                "output_dir": LaunchConfiguration("output_dir"),
                "port": LaunchConfiguration("port"),
            }],
        ),
    ])
