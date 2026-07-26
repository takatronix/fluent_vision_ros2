import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.events import Shutdown
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
        DeclareLaunchArgument("style_id", default_value="30"),
        DeclareLaunchArgument("synthesis_timeout_seconds", default_value="60.0"),
        # 合成結果の再利用。既定は無効で、置き場所は配備側が決める。
        DeclareLaunchArgument("cache_directory", default_value=""),
        DeclareLaunchArgument("cache_memory_entries", default_value="64"),
        DeclareLaunchArgument("cache_disk_budget_mb", default_value="256"),
        # 起動後に先に合成しておく文の一覧 (1行1発話、'#' 始まりはコメント)
        DeclareLaunchArgument("cache_warmup_file", default_value=""),
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
                    "synthesis_timeout_seconds": ParameterValue(
                        LaunchConfiguration("synthesis_timeout_seconds"),
                        value_type=float,
                    ),
                    "cache_directory": LaunchConfiguration("cache_directory"),
                    "cache_memory_entries": ParameterValue(
                        LaunchConfiguration("cache_memory_entries"), value_type=int
                    ),
                    "cache_disk_budget_mb": ParameterValue(
                        LaunchConfiguration("cache_disk_budget_mb"), value_type=int
                    ),
                    "cache_warmup_file": LaunchConfiguration("cache_warmup_file"),
                },
            ],
            on_exit=EmitEvent(
                event=Shutdown(reason="VOICEVOX TTS exited")
            ),
        ),
    ])
