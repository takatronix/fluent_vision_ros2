#!/usr/bin/env python3
"""
効率的なポイントクラウド処理ノードのlaunchファイル
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch description を生成"""
    
    # Launch引数の宣言
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='d415',
        description='Camera name (d415 or d405)'
    )
    
    smoothing_factor_arg = DeclareLaunchArgument(
        'smoothing_factor',
        default_value='0.3',
        description='Smoothing factor for bounding box animation (0.0-1.0)'
    )
    
    animation_speed_arg = DeclareLaunchArgument(
        'animation_speed',
        default_value='0.1',
        description='Animation speed for visual effects'
    )
    
    # D415用ノード
    d415_node = Node(
        package='fv_aspara_analyzer',
        executable='efficient_pointcloud_processor.py',
        name='efficient_pointcloud_d415',
        parameters=[{
            'camera_name': 'd415',
            'smoothing_factor': LaunchConfiguration('smoothing_factor'),
            'animation_speed': LaunchConfiguration('animation_speed'),
            'min_depth': 0.1,
            'max_depth': 2.0,
        }],
        output='screen'
    )
    
    # D405用ノード
    d405_node = Node(
        package='fv_aspara_analyzer',
        executable='efficient_pointcloud_processor.py',
        name='efficient_pointcloud_d405',
        parameters=[{
            'camera_name': 'd405',
            'smoothing_factor': LaunchConfiguration('smoothing_factor'),
            'animation_speed': LaunchConfiguration('animation_speed'),
            'min_depth': 0.1,
            'max_depth': 2.0,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        camera_name_arg,
        smoothing_factor_arg,
        animation_speed_arg,
        d415_node,
        d405_node,
    ])