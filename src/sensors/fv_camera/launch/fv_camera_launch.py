#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_node(context):
    node_name = LaunchConfiguration('node_name').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    sel_method = LaunchConfiguration('camera_selection_method').perform(context).strip()
    dev_name = LaunchConfiguration('camera_device_name').perform(context).strip()

    parameters = [config_file]
    overrides = {}
    if sel_method:
        overrides['camera_selection.selection_method'] = sel_method
    if dev_name:
        overrides['camera_selection.device_name'] = dev_name
    if overrides:
        parameters.append(overrides)

    return [Node(
        package='fv_camera',
        executable='fv_camera_node',
        name=node_name,
        output='screen',
        parameters=parameters,
    )]


def generate_launch_description():
    pkg_share = FindPackageShare('fv_camera')

    return LaunchDescription([
        DeclareLaunchArgument(
            'node_name',
            default_value='fv_camera',
            description='Node name',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'default_config.yaml']),
            description='Path to config file',
        ),
        DeclareLaunchArgument(
            'camera_selection_method',
            default_value='',
            description='Optional override for camera_selection.selection_method',
        ),
        DeclareLaunchArgument(
            'camera_device_name',
            default_value='',
            description='Optional override for camera_selection.device_name (/dev/videoX)',
        ),
        OpaqueFunction(function=_build_node),
    ])
