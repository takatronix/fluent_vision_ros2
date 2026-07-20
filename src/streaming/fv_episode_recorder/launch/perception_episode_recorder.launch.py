"""Perception, anomaly annotation, and lexical search nodes."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("fv_episode_recorder")
    default_config = str(package_share + "/config/perception.yaml")
    config = LaunchConfiguration("config")
    output_dir = LaunchConfiguration("output_dir")
    database_path = PathJoinSubstitution([output_dir, "annotations.db"])
    default_output_dir = os.environ.get(
        "FV_EPISODE_OUTPUT_DIR",
        str(Path.home() / ".aspa" / "episodes"),
    )

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("output_dir", default_value=default_output_dir),
        Node(
            package="fv_episode_recorder",
            executable="episode_recorder",
            name="episode_recorder",
            parameters=[config, {"database_path": database_path}],
            output="screen",
        ),
        Node(
            package="fv_episode_recorder",
            executable="video_anomaly_detector",
            name="video_anomaly_detector",
            parameters=[config],
            output="screen",
        ),
        Node(
            package="fv_episode_recorder",
            executable="moss_realtime_adapter",
            name="moss_realtime_adapter",
            parameters=[config, {"database_path": database_path}],
            output="screen",
        ),
        Node(
            package="fv_episode_recorder",
            executable="episode_search",
            name="episode_search",
            parameters=[config, {"database_path": database_path}],
            output="screen",
        ),
    ])
