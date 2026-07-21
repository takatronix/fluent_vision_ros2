"""Launch fv_apriltag for daihen_vc4l_teleop profile.

Two detector instances (D405 arm_camera + C920 top_camera) plus the
cube estimator. Depth refine is only available on the D405 stream.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('fv_apriltag')
    cubes_cfg = os.path.join(share, 'config', 'cubes_daihen.yaml')

    arm_d405 = Node(
        package='fv_apriltag',
        executable='apriltag_node',
        name='apriltag_node',
        namespace='arm_camera_apriltag',
        output='screen',
        parameters=[{
            'family': 'tag36h11',
            'nthreads': 2,
            'quad_decimate': 2.0,
            # This legacy cube profile is a controlled homogeneous 40mm
            # black-edge environment. Without this explicit override, IDs
            # 21-38 correctly mean 150mm field tags in the active registry.
            'tag_size': 0.040,
            'min_decision_margin': 50.0,
            'publish_annotated': True,
            'tag_frame_prefix': 'arm_tag',
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

    side_c920 = Node(
        package='fv_apriltag',
        executable='apriltag_node',
        name='apriltag_node',
        namespace='top_camera_apriltag',
        output='screen',
        parameters=[{
            'family': 'tag36h11',
            'nthreads': 2,
            'quad_decimate': 2.0,
            # Same legacy cube-kit contract as the arm camera above.
            'tag_size': 0.040,
            'min_decision_margin': 50.0,
            'publish_annotated': True,
            'tag_frame_prefix': 'side_tag',
            'publish_tf': True,
        }],
        remappings=[
            ('image', '/top_camera/image_raw'),
            ('camera_info', '/top_camera/camera_info'),
        ],
    )

    cube_estimator = Node(
        package='fv_apriltag',
        executable='cube_estimator_node',
        name='cube_estimator',
        output='screen',
        parameters=[{'config_file': cubes_cfg}],
    )

    return LaunchDescription([arm_d405, side_c920, cube_estimator])
