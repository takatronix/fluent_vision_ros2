#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('fv_recorder')
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    default_config_file = os.path.join(pkg_dir, 'config', 'recorder_config.yaml')
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='設定YAMLファイルのパス'
    )
    recording_directory_arg = DeclareLaunchArgument(
        'recording_directory',
        default_value='/recordings',
        description='録画保存ディレクトリ'
    )
    
    segment_duration_arg = DeclareLaunchArgument(
        'segment_duration',
        default_value='300',
        description='セグメント時間（秒）'
    )
    
    retention_days_arg = DeclareLaunchArgument(
        'retention_days',
        default_value='7',
        description='保持日数'
    )
    
    # 録画ディレクトリの作成
    create_dir_cmd = ExecuteProcess(
        cmd=['mkdir', '-p', LaunchConfiguration('recording_directory')],
        output='screen'
    )
    
    # 録画ノード
    recorder_node = Node(
        package='fv_recorder',
        executable='fv_recorder_node',
        name='fv_recorder_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'recording.output_directory': LaunchConfiguration('recording_directory'),
                'recording.segment_duration': LaunchConfiguration('segment_duration'),
                'recording.retention_days': LaunchConfiguration('retention_days'),
            }
        ],
        remappings=[
            ('/fv_recorder/status', '/fv/recorder/status'),
        ]
    )
    
    # 再生ノード
    player_node = Node(
        package='fv_recorder',
        executable='fv_player_node',
        name='fv_player_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'playback.recording_directory': LaunchConfiguration('recording_directory'),
            }
        ],
        remappings=[
            ('/fv_player/status', '/fv/player/status'),
        ]
    )
    
    # Log info
    log_info = LogInfo(msg="🚀 Starting FV Recorder for D415 and D405...")
    
    return LaunchDescription([
        config_file_arg,
        recording_directory_arg,
        segment_duration_arg,
        retention_days_arg,
        log_info,
        create_dir_cmd,
        recorder_node,
        player_node,
    ]) 