#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("fv_insta360_x3")
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [pkg_share, "config", "default_config.yaml"]
                ),
                description="Path to config file",
            ),
            Node(
                package="fv_insta360_x3",
                executable="fv_insta360_x3_node",
                name="fv_insta360_x3",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="fv_insta360_x3",
                executable="fv_insta360_x3_stitcher_node",
                name="fv_insta360_x3_stitcher",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
