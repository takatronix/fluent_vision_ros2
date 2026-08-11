import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("fv_audio")
    capture_device = LaunchConfiguration("capture_device")
    capture_chunk_frames = LaunchConfiguration("capture_chunk_frames")
    capture_queue_chunks = LaunchConfiguration("capture_queue_chunks")
    capture_prebuffer_chunks = LaunchConfiguration("capture_prebuffer_chunks")
    output_device = LaunchConfiguration("output_device")
    output_rate = LaunchConfiguration("output_rate")
    output_channels = LaunchConfiguration("output_channels")
    output_buffer_ms = LaunchConfiguration("output_buffer_ms")
    tts_timing_receipt_path = LaunchConfiguration("tts_timing_receipt_path")
    tts_timing_receipt_limit = LaunchConfiguration("tts_timing_receipt_limit")
    tts_timing_timeout_seconds = LaunchConfiguration(
        "tts_timing_timeout_seconds")
    aec_stream_delay_ms = LaunchConfiguration("aec_stream_delay_ms")

    capture = Node(
        package="fv_audio",
        executable="audio_capture",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        additional_env={
            "FV_AUDIO_CAPTURE_DEVICE": capture_device,
            "FV_AUDIO_CAPTURE_CHUNK_FRAMES": capture_chunk_frames,
            "FV_AUDIO_CAPTURE_QUEUE_CHUNKS": capture_queue_chunks,
            "FV_AUDIO_CAPTURE_PREBUFFER_CHUNKS": capture_prebuffer_chunks,
        },
    )
    aec = Node(
        package="fv_audio",
        executable="audio_aec",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        additional_env={
            "FV_AUDIO_AEC_STREAM_DELAY_MS": aec_stream_delay_ms,
            "FV_AUDIO_AEC_FAR_SAMPLE_RATE": output_rate,
            "FV_AUDIO_AEC_FAR_CHANNELS": output_channels,
        },
    )
    timing_collector = Node(
        package="fv_audio",
        executable="tts_timing_collector",
        output="screen",
        respawn=False,
        on_exit=[
            Shutdown(
                reason=(
                    "tts_timing_collector exited; the audio launch group "
                    "cannot continue without timing evidence"
                ),
            ),
        ],
        additional_env={
            "FV_AUDIO_TTS_TIMING_RECEIPT_PATH": tts_timing_receipt_path,
            "FV_AUDIO_TTS_TIMING_RECEIPT_LIMIT": tts_timing_receipt_limit,
            "FV_AUDIO_TTS_TIMING_TIMEOUT_SECONDS": tts_timing_timeout_seconds,
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument("capture_device", default_value="default"),
        DeclareLaunchArgument("capture_chunk_frames", default_value="160"),
        DeclareLaunchArgument("capture_queue_chunks", default_value="64"),
        DeclareLaunchArgument("capture_prebuffer_chunks", default_value="4"),
        DeclareLaunchArgument("output_device", default_value="default"),
        DeclareLaunchArgument("output_rate", default_value="48000"),
        DeclareLaunchArgument("output_channels", default_value="2"),
        DeclareLaunchArgument("output_buffer_ms", default_value="80"),
        DeclareLaunchArgument(
            "tts_timing_receipt_path",
            default_value=os.path.expanduser(
                "~/.aspa/tts-timing-receipts.json"),
        ),
        DeclareLaunchArgument("tts_timing_receipt_limit", default_value="256"),
        DeclareLaunchArgument("tts_timing_timeout_seconds", default_value="180"),
        DeclareLaunchArgument("aec_stream_delay_ms", default_value="40"),
        Node(
            package="fv_audio",
            executable="audio_device_registry",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ),
        timing_collector,
        Node(
            package="fv_audio",
            executable="playback_controller",
            parameters=[
                os.path.join(share, "config", "fv_audio.yaml"),
                {
                    "output_sample_rate_hz": ParameterValue(output_rate, value_type=int),
                    "output_channels": ParameterValue(output_channels, value_type=int),
                },
            ],
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="fv_audio",
            executable="audio_output",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            additional_env={
                "FV_AUDIO_OUTPUT_DEVICE": output_device,
                "FV_AUDIO_OUTPUT_RATE": output_rate,
                "FV_AUDIO_OUTPUT_CHANNELS": output_channels,
                "FV_AUDIO_OUTPUT_BUFFER_MS": output_buffer_ms,
            },
        ),
        # The hardware output callback is the AEC far-end clock. Give it time
        # to start publishing render reference before microphone PCM arrives.
        TimerAction(period=2.0, actions=[capture, aec]),
    ])
