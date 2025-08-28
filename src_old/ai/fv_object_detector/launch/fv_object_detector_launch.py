#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('fv_object_detector')
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Launch arguments
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='fv_object_detector',
        description='Node name for the object detector'
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
    
    # Object Detector Node
    object_detector_node = Node(
        package='fv_object_detector',
        executable='fv_object_detector_node',
        name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'input_image_topic': LaunchConfiguration('input_topic'),
                'output_image_topic': LaunchConfiguration('output_image_topic'),
                'output_detections_topic': LaunchConfiguration('output_detections_topic'),
                'ros_params_file': LaunchConfiguration('config_file'),
            }
        ],
        remappings=[
            ('input_image', LaunchConfiguration('input_topic')),
            ('output_image', LaunchConfiguration('output_image_topic')),
            ('output_detections', LaunchConfiguration('output_detections_topic')),
        ]
    )
    
    # Log info
    log_info = LogInfo(msg="🚀 Starting FV Object Detector...")
    
    return LaunchDescription([
        node_name_arg,
        config_file_arg,
        input_topic_arg,
        output_image_topic_arg,
        output_detections_topic_arg,
        log_info,
        object_detector_node,
    ]) 