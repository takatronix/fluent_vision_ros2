#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('fv_realsense')
    
    # Launch arguments
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='fv_realsense',
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
    
    # FV Depth Camera Node
    fv_realsense_node = Node(
        package='fv_realsense',
        executable='fv_realsense_node',
        name=node_name,
        output='screen',
        parameters=[config_file],
        # Force separate process to avoid device conflicts
        emulate_tty=True,
        # The node exits on purpose (watchdog stall / camera missing at
        # boot) and relies on this respawn as its recovery path — an
        # in-process librealsense reset corrupts the heap, so a fresh
        # process is the only reliable reset.
        respawn=True,
        respawn_delay=3.0,
    )
    
    return LaunchDescription([
        node_name_arg,
        config_file_arg,
        fv_realsense_node,
    ]) 