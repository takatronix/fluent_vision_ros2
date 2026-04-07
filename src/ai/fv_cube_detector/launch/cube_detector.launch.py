from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=''),
        Node(
            package='fv_cube_detector',
            executable='fv_cube_detector_node',
            name='cube_detector_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')] if LaunchConfiguration('params_file') else [],
        )
    ])
