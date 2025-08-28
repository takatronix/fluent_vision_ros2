#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('fv_camera')
    
    # Launch arguments
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='fv_camera',
        description='Node name'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'default_config.yaml']),
        description='Path to config file'
    )
    
    # Get launch configuration
    node_name = LaunchConfiguration('node_name')
    config_file = LaunchConfiguration('config_file')
    
    # FV Camera Node
    fv_camera_node = Node(
        package='fv_camera',
        executable='fv_camera_node',
        name=node_name,
        output='screen',
        parameters=[config_file]
    )
    
    return LaunchDescription([
        node_name_arg,
        config_file_arg,
        fv_camera_node,
    ]) 