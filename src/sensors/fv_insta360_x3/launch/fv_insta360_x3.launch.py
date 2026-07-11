#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_node(context):
    node_name = LaunchConfiguration("node_name").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    device_path = LaunchConfiguration("device_path").perform(context).strip()
    width = LaunchConfiguration("width").perform(context).strip()
    height = LaunchConfiguration("height").perform(context).strip()
    fps = LaunchConfiguration("fps").perform(context).strip()

    overrides = {}
    if device_path:
        overrides["device.path"] = device_path
    if width:
        overrides["camera.width"] = int(width)
    if height:
        overrides["camera.height"] = int(height)
    if fps:
        overrides["camera.fps"] = int(fps)

    parameters = [config_file]
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package="fv_insta360_x3",
            executable="fv_insta360_x3_node",
            name=node_name,
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    pkg_share = FindPackageShare("fv_insta360_x3")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "node_name",
                default_value="fv_insta360_x3",
                description="Node name",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [pkg_share, "config", "default_config.yaml"]
                ),
                description="Path to config file",
            ),
            DeclareLaunchArgument(
                "device_path",
                default_value="",
                description="Optional override for device.path",
            ),
            DeclareLaunchArgument(
                "width",
                default_value="",
                description="Optional override for camera.width",
            ),
            DeclareLaunchArgument(
                "height",
                default_value="",
                description="Optional override for camera.height",
            ),
            DeclareLaunchArgument(
                "fps",
                default_value="",
                description="Optional override for camera.fps",
            ),
            OpaqueFunction(function=_build_node),
        ]
    )
