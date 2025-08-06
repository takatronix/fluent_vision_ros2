from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('fv_aspara_analyzer')
    
    # Configuration file path (in launch directory)
    launch_dir = '/home/aspara/seedbox-r1/fluent_vision_ros2/launch'
    config_file = os.path.join(launch_dir, 'fv_aspara_analyzer_d415.yaml')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Full path to the ROS2 parameters file to use'
        ),
        
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'
        ),

        # FV Aspara Analyzer Node
        Node(
            package='fv_aspara_analyzer',
            executable='fv_aspara_analyzer_node',
            name='fv_aspara_analyzer_d415',
            parameters=[LaunchConfiguration('config_file')],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen',
            emulate_tty=True,
        ),
    ])