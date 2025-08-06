#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('fv_rtmp_server'),
            'config',
            'fv_rtmp_server.yaml'
        ]),
        description='Path to the RTMP bridge configuration file'
    )
    
    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value='1935',
        description='RTMP server port'
    )
    
    target_width_arg = DeclareLaunchArgument(
        'target_width',
        default_value='1280',
        description='Target image width'
    )
    
    target_height_arg = DeclareLaunchArgument(
        'target_height',
        default_value='720',
        description='Target image height'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='/fv_rtmp_server/camera/image_raw',
        description='Output topic name'
    )
    
    low_latency_arg = DeclareLaunchArgument(
        'low_latency',
        default_value='true',
        description='Enable low latency mode'
    )
    
    compressed_arg = DeclareLaunchArgument(
        'compressed',
        default_value='false',
        description='Publish compressed images'
    )

    # FV RTMP Server Node
    fv_rtmp_server_node = Node(
        package='fv_rtmp_server',
        executable='fv_rtmp_server_node',
        name='fv_rtmp_server_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'server.port': LaunchConfiguration('server_port'),
                'output.target_width': LaunchConfiguration('target_width'),
                'output.target_height': LaunchConfiguration('target_height'),
                'output.topic_name': LaunchConfiguration('topic_name'),
                'processing.enable_low_latency': LaunchConfiguration('low_latency'),
                'output.compressed': LaunchConfiguration('compressed'),
            }
        ],
        emulate_tty=True
    )

    return LaunchDescription([
        config_file_arg,
        server_port_arg,
        target_width_arg,
        target_height_arg,
        topic_name_arg,
        low_latency_arg,
        compressed_arg,
        fv_rtmp_server_node
    ]) 