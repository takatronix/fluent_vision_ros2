from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = (
        get_package_share_directory("fv_video_event_describer")
        + "/config/default.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument(
            "recorder_profile",
            default_value=EnvironmentVariable("ASPA_PROFILE", default_value="aspa_dev"),
        ),
        Node(
            package="fv_video_event_describer",
            executable="video_event_describer",
            name="video_event_describer",
            parameters=[
                LaunchConfiguration("config"),
                {"recorder_profile": LaunchConfiguration("recorder_profile")},
            ],
            output="screen",
        ),
    ])
