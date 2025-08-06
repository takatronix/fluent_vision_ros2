#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('fv_object_mask_generator')
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Launch arguments
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='fv_object_mask_generator',
        description='Node name for the object mask generator'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='config/default_config.yaml',
        description='Path to config file'
    )
    
    input_image_topic_arg = DeclareLaunchArgument(
        'input_image_topic',
        default_value='/camera/color/image_raw',
        description='Input image topic'
    )
    
    output_segmentation_mask_topic_arg = DeclareLaunchArgument(
        'output_segmentation_mask_topic',
        default_value='/segmentation_mask/image',
        description='Output segmentation mask topic'
    )
    
    output_colored_mask_topic_arg = DeclareLaunchArgument(
        'output_colored_mask_topic',
        default_value='/segmentation_mask/colored',
        description='Output colored mask topic'
    )
    
    # Object Mask Generator Node
    mask_generator_node = Node(
        package='fv_object_mask_generator',
        executable='fv_object_mask_generator_node',
        name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'input_image_topic': LaunchConfiguration('input_image_topic'),
                'output_segmentation_mask_topic': LaunchConfiguration('output_segmentation_mask_topic'),
                'output_colored_mask_topic': LaunchConfiguration('output_colored_mask_topic'),
                'ros_params_file': LaunchConfiguration('config_file'),
            }
        ],
        remappings=[
            ('input_image', LaunchConfiguration('input_image_topic')),
            ('output_segmentation_mask', LaunchConfiguration('output_segmentation_mask_topic')),
            ('output_colored_mask', LaunchConfiguration('output_colored_mask_topic')),
        ]
    )
    
    # Log configuration
    log_info = LogInfo(
        msg=['Launching FV Object Mask Generator with config: ', LaunchConfiguration('config_file')]
    )
    
    return LaunchDescription([
        # Arguments
        node_name_arg,
        config_file_arg,
        input_image_topic_arg,
        output_segmentation_mask_topic_arg,
        output_colored_mask_topic_arg,
        # Actions
        log_info,
        mask_generator_node,
    ])