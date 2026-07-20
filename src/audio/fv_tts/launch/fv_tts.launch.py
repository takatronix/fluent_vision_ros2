import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("fv_tts")
    return LaunchDescription([
        DeclareLaunchArgument(
            "onnxruntime_filename",
            default_value=EnvironmentVariable("VOICEVOX_ONNXRUNTIME_PATH", default_value=""),
        ),
        DeclareLaunchArgument(
            "open_jtalk_dict_dir",
            default_value=EnvironmentVariable("VOICEVOX_OPEN_JTALK_DICT_DIR", default_value=""),
        ),
        DeclareLaunchArgument(
            "voice_model_path",
            default_value=EnvironmentVariable("VOICEVOX_VOICE_MODEL_PATH", default_value=""),
        ),
        DeclareLaunchArgument("acceleration_mode", default_value="auto"),
        DeclareLaunchArgument("cpu_num_threads", default_value="0"),
        DeclareLaunchArgument("style_id", default_value="3"),
        Node(
            package="fv_tts",
            executable="fv_tts_node",
            name="fv_tts",
            output="screen",
            parameters=[
                os.path.join(share, "config", "default.yaml"),
                {
                    "onnxruntime_filename": LaunchConfiguration("onnxruntime_filename"),
                    "open_jtalk_dict_dir": LaunchConfiguration("open_jtalk_dict_dir"),
                    "voice_model_path": LaunchConfiguration("voice_model_path"),
                    "acceleration_mode": LaunchConfiguration("acceleration_mode"),
                    "cpu_num_threads": ParameterValue(
                        LaunchConfiguration("cpu_num_threads"), value_type=int
                    ),
                    "style_id": ParameterValue(
                        LaunchConfiguration("style_id"), value_type=int
                    ),
                },
            ],
        ),
    ])
