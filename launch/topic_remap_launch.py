#!/usr/bin/env python3
"""
トピックリマップ用のlaunchファイル
/fv/* -> /vision_ai/* へのリマップを行う
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # リマップの定義
    topic_remappings = [
        # D415カメラ
        ('/vision_ai/d415/depth/colormap', '/fv/d415/depth/colormap'),
        ('/vision_ai/d415/object_detection/annotated_image', '/fv/d415/object_detection/annotated_image'),
        ('/vision_ai/d415/object_detection/annotated_image_mouse_left', '/fv/d415/object_detection/annotated_image_mouse_left'),
        ('/vision_ai/d415/segmentation_mask/colored', '/fv/d415/segmentation_mask/colored'),
        ('/vision_ai/d415/segmentation_mask/colored_mouse_left', '/fv/d415/segmentation_mask/colored_mouse_left'),
        
        # D405カメラ（必要に応じて追加）
        # ('/vision_ai/d405/depth/colormap', '/fv/d405/depth/colormap'),
    ]
    
    # 各ノードを起動（リマップ付き）
    # 例: vision_aiが購読する側のノード
    vision_ai_node = Node(
        package='your_vision_ai_package',  # 実際のパッケージ名に変更
        executable='vision_ai_node',        # 実際の実行ファイル名に変更
        name='vision_ai',
        remappings=topic_remappings,
        output='screen'
    )
    
    return LaunchDescription([
        vision_ai_node,
        # 他のノードも同様に追加
    ])