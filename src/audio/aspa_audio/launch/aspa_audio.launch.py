import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("aspa_audio")
    capture_device = LaunchConfiguration("capture_device")
    capture_chunk_frames = LaunchConfiguration("capture_chunk_frames")
    capture_queue_chunks = LaunchConfiguration("capture_queue_chunks")
    output_device = LaunchConfiguration("output_device")
    output_rate = LaunchConfiguration("output_rate")
    output_channels = LaunchConfiguration("output_channels")
    output_buffer_ms = LaunchConfiguration("output_buffer_ms")
    aec_stream_delay_ms = LaunchConfiguration("aec_stream_delay_ms")

    return LaunchDescription([
        DeclareLaunchArgument("capture_device", default_value="default"),
        DeclareLaunchArgument("capture_chunk_frames", default_value="160"),
        DeclareLaunchArgument("capture_queue_chunks", default_value="64"),
        DeclareLaunchArgument("output_device", default_value="default"),
        DeclareLaunchArgument("output_rate", default_value="48000"),
        DeclareLaunchArgument("output_channels", default_value="2"),
        DeclareLaunchArgument("output_buffer_ms", default_value="80"),
        DeclareLaunchArgument("aec_stream_delay_ms", default_value="40"),
        Node(
            package="aspa_audio",
            executable="audio_capture",
            output="screen",
            additional_env={
                "ASPA_AUDIO_CAPTURE_DEVICE": capture_device,
                "ASPA_AUDIO_CAPTURE_CHUNK_FRAMES": capture_chunk_frames,
                "ASPA_AUDIO_CAPTURE_QUEUE_CHUNKS": capture_queue_chunks,
            },
            on_exit=EmitEvent(
                event=Shutdown(reason="ASPA audio capture exited")
            ),
        ),
        Node(
            package="aspa_audio",
            executable="audio_aec",
            output="screen",
            additional_env={
                "ASPA_AUDIO_AEC_STREAM_DELAY_MS": aec_stream_delay_ms,
                "ASPA_AUDIO_AEC_FAR_SAMPLE_RATE": output_rate,
                "ASPA_AUDIO_AEC_FAR_CHANNELS": output_channels,
            },
            on_exit=EmitEvent(
                event=Shutdown(reason="ASPA WebRTC AEC exited")
            ),
        ),
        Node(
            package="aspa_audio",
            executable="playback_controller",
            parameters=[
                os.path.join(share, "config", "aspa_audio.yaml"),
                {
                    "output_sample_rate_hz": ParameterValue(output_rate, value_type=int),
                    "output_channels": ParameterValue(output_channels, value_type=int),
                },
            ],
            output="screen",
            on_exit=EmitEvent(
                event=Shutdown(reason="ASPA playback controller exited")
            ),
        ),
        Node(
            package="aspa_audio",
            executable="audio_output",
            output="screen",
            additional_env={
                "ASPA_AUDIO_OUTPUT_DEVICE": output_device,
                "ASPA_AUDIO_OUTPUT_RATE": output_rate,
                "ASPA_AUDIO_OUTPUT_CHANNELS": output_channels,
                "ASPA_AUDIO_OUTPUT_BUFFER_MS": output_buffer_ms,
            },
            on_exit=EmitEvent(
                event=Shutdown(reason="ASPA audio output exited")
            ),
        ),
    ])
