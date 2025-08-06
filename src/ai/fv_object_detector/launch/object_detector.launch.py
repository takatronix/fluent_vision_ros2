#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_cpp.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('fv_object_detector')
    
    # Launch arguments
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='fv_object_detector_node',
        description='Node name'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='default_config.yaml',
        description='Configuration file name'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/color/image_raw',
        description='Input image topic'
    )
    
    output_image_topic_arg = DeclareLaunchArgument(
        'output_image_topic',
        default_value='/object_detection/annotated_image',
        description='Output annotated image topic'
    )
    
    output_detections_topic_arg = DeclareLaunchArgument(
        'output_detections_topic',
        default_value='/object_detection/detections',
        description='Output detections topic'
    )
    
    # Node
    object_detector_node = Node(
        package='fv_object_detector',
        executable='fv_object_detector_node',
        name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_dir, 'config', LaunchConfiguration('config_file')]),
            {
                'input_image_topic': LaunchConfiguration('input_topic'),
                'output_image_topic': LaunchConfiguration('output_image_topic'),
                'output_detections_topic': LaunchConfiguration('output_detections_topic'),
            }
        ],
        remappings=[
            ('input_image', LaunchConfiguration('input_topic')),
            ('output_image', LaunchConfiguration('output_image_topic')),
            ('output_detections', LaunchConfiguration('output_detections_topic')),
        ]
    )
    
    return LaunchDescription([
        node_name_arg,
        config_file_arg,
        input_topic_arg,
        output_image_topic_arg,
        output_detections_topic_arg,
        object_detector_node,
    ]) 