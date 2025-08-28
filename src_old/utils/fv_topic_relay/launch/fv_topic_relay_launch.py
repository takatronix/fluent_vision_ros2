#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 設定ファイルのパス
    config = os.path.join(
        get_package_share_directory('fv_topic_relay'),
        'config',
        'topic_relay.yaml'
    )
    
    # ノードの設定
    topic_relay_node = Node(
        package='fv_topic_relay',
        executable='fv_topic_relay_node',
        name='fv_topic_relay',
        output='screen',
        parameters=[config]
    )
    
    return LaunchDescription([
        topic_relay_node
    ])