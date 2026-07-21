"""Visual anomaly detector and MOSS realtime adapter."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("fv_episode_recorder")
    default_config = str(package_share + "/config/perception.yaml")
    config = LaunchConfiguration("config")
    model_path = LaunchConfiguration("model_path")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument(
            "model_path",
            default_value=EnvironmentVariable(
                "FV_VISUAL_CONDITION_MODEL_PATH",
                default_value="/home/aspa/.aspa/models/visual-condition-multihead.onnx",
            ),
        ),
        Node(
            package="fv_episode_recorder",
            executable="video_anomaly_detector",
            name="video_anomaly_detector",
            parameters=[config, {"model_path": model_path}],
            output="screen",
        ),
        Node(
            package="fv_episode_recorder",
            executable="moss_realtime_adapter",
            name="moss_realtime_adapter",
            parameters=[config],
            output="screen",
        ),
    ])
