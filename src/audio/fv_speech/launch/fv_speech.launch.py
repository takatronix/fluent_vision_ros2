import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _default_runtime_manifest():
    data_home = os.environ.get("XDG_DATA_HOME", "")
    if not os.path.isabs(data_home):
        data_home = os.path.join(os.path.expanduser("~"), ".local", "share")
    return os.path.join(
        data_home, "aspa-navigation", "parakeet-cuda", "runtime.json"
    )


def _speech_nodes(context, *, share):
    config = LaunchConfiguration("config")
    runtime_manifest = LaunchConfiguration("parakeet_runtime_manifest").perform(context)
    runtime_env = {
        "ASPA_PARAKEET_RUNTIME_MANIFEST": runtime_manifest,
        "CUDA_VISIBLE_DEVICES": LaunchConfiguration("cuda_visible_devices").perform(
            context
        ),
    }
    silero_model = os.path.join(share, "models", "silero_vad_16k_op15.onnx")
    return [
        Node(
            package="fv_speech",
            executable="fv_speech_runtime_exec",
            arguments=["parakeet_asr"],
            parameters=[config],
            additional_env=runtime_env,
            output="screen",
            on_exit=EmitEvent(
                event=Shutdown(reason="Parakeet CUDA ASR exited")
            ),
        ),
        Node(
            package="fv_speech",
            executable="fv_speech_runtime_exec",
            arguments=["silero_vad"],
            parameters=[config, {"model_path": silero_model}],
            additional_env=runtime_env,
            output="screen",
            on_exit=EmitEvent(
                event=Shutdown(reason="Silero VAD exited")
            ),
        ),
        Node(
            package="fv_speech",
            executable="turn_detector",
            parameters=[config],
            output="screen",
            on_exit=EmitEvent(
                event=Shutdown(reason="Turn detector exited")
            ),
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory("fv_speech")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=os.path.join(share, "config", "fv_speech.yaml"),
        ),
        DeclareLaunchArgument(
            "parakeet_runtime_manifest",
            default_value=EnvironmentVariable(
                "ASPA_PARAKEET_RUNTIME_MANIFEST",
                default_value=_default_runtime_manifest(),
            ),
        ),
        DeclareLaunchArgument(
            "cuda_visible_devices",
            default_value=EnvironmentVariable(
                "ASPA_PARAKEET_CUDA_DEVICE", default_value="0"
            ),
        ),
        OpaqueFunction(function=_speech_nodes, kwargs={"share": share}),
    ])
