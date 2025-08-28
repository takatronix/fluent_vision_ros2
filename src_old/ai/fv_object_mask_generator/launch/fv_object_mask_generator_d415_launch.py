#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('fv_object_mask_generator')
    config_file = os.path.join(pkg_dir, 'config', 'd415_mask_generator_config.yaml')
    
    # D415 specific configuration
    mask_generator_node = Node(
        package='fv_object_mask_generator',
        executable='fv_object_mask_generator_node',
        name='fv_object_mask_generator_d415',
        output='screen',
        parameters=[config_file]
    )
    
    return LaunchDescription([
        mask_generator_node,
    ])