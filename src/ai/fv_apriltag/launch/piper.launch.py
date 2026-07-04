"""Launch fv_apriltag for piper_single_teleop profile.

One detector instance on the D405 color stream, plus the cube estimator.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('fv_apriltag')
    cubes_cfg = os.path.join(share, 'config', 'cubes_piper.yaml')

    d405_detector = Node(
        package='fv_apriltag',
        executable='apriltag_node',
        name='apriltag_node',
        namespace='d405_apriltag',
        output='screen',
        parameters=[{
            # OFF by default: tag detection on the 30fps color stream costs
            # ~0.5 core and is only needed for cube-bundle calibration /
            # AprilTag work. Toggle at runtime via
            # `ros2 param set /d405_apriltag/apriltag_node enabled true`
            # or the perception MCP set_apriltag_enabled - the node stays
            # up, so switching on is instant.
            'enabled': False,
            'family': 'tag36h11',
            'nthreads': 2,
            'quad_decimate': 2.0,
            'quad_sigma': 0.0,
            'refine_edges': 1,
            'decode_sharpening': 0.25,
            'default_tag_size': 0.050,
            'min_decision_margin': 50.0,
            'publish_annotated': True,
            'tag_frame_prefix': 'd405_tag',
            'publish_tf': True,
            # Depth for per-tag accurate distance overlay (mm-level).
            'depth_topic': '/d405_depth/image_rect_raw',
            'depth_info_topic': '/d405_depth/camera_info',
        }],
        remappings=[
            ('image', '/d405_color/image_raw'),
            ('camera_info', '/d405_color/camera_info'),
        ],
    )

    cube_estimator = Node(
        package='fv_apriltag',
        executable='cube_estimator_node',
        name='cube_estimator',
        output='screen',
        parameters=[{'config_file': cubes_cfg}],
    )

    return LaunchDescription([d405_detector, cube_estimator])
