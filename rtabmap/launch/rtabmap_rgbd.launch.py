#!/usr/bin/env python3
"""
RTABMap RGB-Dカメラ用 Launch ファイル
D415（遠距離）とD405（近距離）カメラを使用したSLAM設定
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch引数の定義
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='indoor_mapping',
        description='Config file to use (indoor_mapping, outdoor_mapping, localization, performance)'
    )
    
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Set to true for localization mode'
    )
    
    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value='$(find-pkg-share rtabmap)/databases/rtabmap.db',
        description='Path to RTABMap database'
    )
    
    rtabmapviz_arg = DeclareLaunchArgument(
        'rtabmapviz',
        default_value='false',
        description='Launch RTABMapviz for visualization'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    # 設定ファイルのパス
    rtabmap_config_dir = '/home/aspara/seedbox-r1/fluent_vision_ros2/rtabmap/configs'
    
    # === TF発行ノード（重要：座標系構築） ===
    # Base Link to Camera Link（D415メインカメラ）
    tf_base_to_d415 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_d415_tf',
        arguments=['0.1', '0.0', '0.3', '0', '0', '0', 'base_link', 'd415_link'],
        output='screen'
    )
    
    # Base Link to D405（近距離カメラ）
    tf_base_to_d405 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_d405_tf',
        arguments=['0.05', '0.0', '0.1', '0', '0', '0', 'base_link', 'd405_link'],
        output='screen'
    )
    
    # Base Link to LiDAR
    tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0.0', '0.0', '0.5', '0', '0', '0', 'base_link', 'livox_frame'],
        output='screen'
    )
    
    # Odom to Base Link（初期位置）
    tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )

    # === RTABMap メインノード ===
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            # 基本設定
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
            
            # 入力トピック設定
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': True,
            'subscribe_odom': True,
            
            # トピック名設定
            'rgb_topic': '/fv/d415/color/image_raw',
            'depth_topic': '/fv/d415/depth/image_rect_raw',
            'camera_info_topic': '/fv/d415/color/camera_info',
            'scan_topic': '/livox/lidar',
            'odom_topic': '/odom',
            
            # 同期設定
            'approx_sync': True,
            'queue_size': 30,
            
            # データベース設定
            'database_path': LaunchConfiguration('database_path'),
            
            # モード設定
            'Mem/IncrementalMemory': LaunchConfiguration('localization') == 'false',
            
            # Wait for transform
            'wait_for_transform': 0.2,
        }],
        remappings=[
            ('rgb/image', '/fv/d415/color/image_raw'),
            ('depth/image', '/fv/d415/depth/image_rect_raw'),
            ('rgb/camera_info', '/fv/d415/color/camera_info'),
            ('scan', '/livox/lidar'),
            ('odom', '/odom'),
        ],
        arguments=['-d'],  # ログ出力
    )

    # === RGB-D オドメトリノード ===
    rgbd_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'guess_frame_id': 'base_link',
            'wait_for_transform': 0.2,
            
            # オドメトリ設定
            'Odom/Strategy': 0,  # Frame-to-Map
            'Odom/GuessMotion': True,
            'Vis/MinInliers': 12,
            'Vis/InlierDistance': 0.02,
        }],
        remappings=[
            ('rgb/image', '/fv/d415/color/image_raw'),
            ('depth/image', '/fv/d415/depth/image_rect_raw'),
            ('rgb/camera_info', '/fv/d415/color/camera_info'),
        ]
    )

    # === RTABMapViz（可視化ツール） ===
    rtabmapviz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmapviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rtabmapviz')),
        parameters=[{
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': True,
            'subscribe_odom_info': True,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'wait_for_transform': 0.2,
        }],
        remappings=[
            ('rgb/image', '/fv/d415/color/image_raw'),
            ('depth/image', '/fv/d415/depth/image_rect_raw'),
            ('rgb/camera_info', '/fv/d415/color/camera_info'),
            ('scan', '/livox/lidar'),
            ('odom', '/odom'),
        ]
    )

    # === RViz設定 ===
    rviz_config = os.path.join(
        get_package_share_directory('rtabmap_viz'),
        'config', 'rgbd.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    # LaunchDescriptionの構築
    return LaunchDescription([
        # Launch引数
        config_arg,
        localization_arg,
        database_path_arg,
        rtabmapviz_arg,
        rviz_arg,
        
        # TF発行ノード（座標系構築）
        tf_base_to_d415,
        tf_base_to_d405,
        tf_base_to_lidar,
        tf_odom_to_base,
        
        # メインノード
        rgbd_odometry_node,
        rtabmap_node,
        
        # 可視化ノード
        rtabmapviz_node,
        rviz_node,
    ])