"""Launch fv_apriltag for the daihen teleop profiles.

Two detector instances (D405 arm_camera + top_camera) plus the cube
estimator. Depth refine is only available on the D405 stream.

Launch arguments (defaults reproduce the historical cube-kit setup, so
existing profiles are unchanged):

  backend            auto | pupil | opencv   detector backend (see
                     fv_apriltag/detector_backends.py). auto = pupil when
                     importable, else OpenCV ArUco; the node logs the pick.
  tag_size           homogeneous black-edge override [m]. 0.040 = the legacy
                     40 mm cube kit (ids 1-38 have no registry size). Pass 0
                     to resolve sizes per id from config/tag_registry.yaml
                     (e.g. the 200 band container boxes = 0.060).
  quad_decimate      AprilTag quad decimation for both cameras. 2.0 is cheap;
                     use 1.0 when tags are small in the image (60 mm tag at
                     0.5 m on a 640x480 D405 is ~45 px).
  top_camera         start the top_camera instance (false when that camera
                     cannot see the tags, e.g. a straight-down overhead view
                     of boxes tagged on their vertical faces).
  cube_estimator     start the cube bundle estimator (config/cubes_daihen.yaml).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('fv_apriltag')
    cubes_cfg = os.path.join(share, 'config', 'cubes_daihen.yaml')

    backend_arg = DeclareLaunchArgument(
        'backend', default_value='auto', choices=['auto', 'pupil', 'opencv'],
        description='Detector backend: auto (pupil-apriltags if importable, '
                    'else OpenCV ArUco), pupil, opencv.')
    tag_size_arg = DeclareLaunchArgument(
        'tag_size', default_value='0.040',
        description='Homogeneous black-edge size [m] for every id (legacy '
                    '40 mm cube kit). 0 = per-id sizes from tag_registry.yaml.')
    quad_decimate_arg = DeclareLaunchArgument(
        'quad_decimate', default_value='2.0',
        description='AprilTag quad decimation (1.0 = full resolution).')
    top_camera_arg = DeclareLaunchArgument(
        'top_camera', default_value='true',
        description='Start the top_camera detector instance.')
    cube_estimator_arg = DeclareLaunchArgument(
        'cube_estimator', default_value='true',
        description='Start the cube bundle estimator.')

    backend = LaunchConfiguration('backend')
    tag_size = ParameterValue(LaunchConfiguration('tag_size'), value_type=float)
    quad_decimate = ParameterValue(LaunchConfiguration('quad_decimate'), value_type=float)

    arm_d405 = Node(
        package='fv_apriltag',
        executable='apriltag_node',
        name='apriltag_node',
        namespace='arm_camera_apriltag',
        output='screen',
        parameters=[{
            'family': 'tag36h11',
            'backend': backend,
            'nthreads': 2,
            'quad_decimate': quad_decimate,
            # With the default 0.040 this is the legacy homogeneous 40 mm
            # cube-kit override (ids 21-38 would otherwise mean 150 mm field
            # tags in the active registry). 0 switches to per-id sizes.
            'tag_size': tag_size,
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

    side_top = Node(
        package='fv_apriltag',
        executable='apriltag_node',
        name='apriltag_node',
        namespace='top_camera_apriltag',
        output='screen',
        parameters=[{
            'family': 'tag36h11',
            'backend': backend,
            'nthreads': 2,
            'quad_decimate': quad_decimate,
            # Same size contract as the arm camera above.
            'tag_size': tag_size,
            'min_decision_margin': 50.0,
            'publish_annotated': True,
            'tag_frame_prefix': 'side_tag',
            'publish_tf': True,
        }],
        remappings=[
            ('image', '/top_camera/image_raw'),
            ('camera_info', '/top_camera/camera_info'),
        ],
        condition=IfCondition(LaunchConfiguration('top_camera')),
    )

    cube_estimator = Node(
        package='fv_apriltag',
        executable='cube_estimator_node',
        name='cube_estimator',
        output='screen',
        parameters=[{'config_file': cubes_cfg}],
        condition=IfCondition(LaunchConfiguration('cube_estimator')),
    )

    return LaunchDescription([
        backend_arg, tag_size_arg, quad_decimate_arg, top_camera_arg,
        cube_estimator_arg,
        arm_d405, side_top, cube_estimator,
    ])
