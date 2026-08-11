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
            "native_library",
            default_value=EnvironmentVariable(
                "MAGPIE_TTS_RT_NATIVE_LIBRARY", default_value=""
            ),
        ),
        DeclareLaunchArgument(
            "bundle_path",
            default_value=EnvironmentVariable("MAGPIE_TTS_RT_BUNDLE", default_value=""),
        ),
        DeclareLaunchArgument(
            "manifest_sha256",
            default_value=EnvironmentVariable(
                "MAGPIE_TTS_RT_MANIFEST_SHA256", default_value=""
            ),
        ),
        DeclareLaunchArgument("cuda_device_index", default_value="0"),
        DeclareLaunchArgument(
            "frontend_python",
            default_value=EnvironmentVariable(
                "MAGPIE_TTS_RT_FRONTEND_PYTHON", default_value=""
            ),
        ),
        DeclareLaunchArgument(
            "frontend_server",
            default_value=EnvironmentVariable(
                "MAGPIE_TTS_RT_FRONTEND_SERVER", default_value=""
            ),
        ),
        DeclareLaunchArgument(
            "frontend_lock",
            default_value=EnvironmentVariable(
                "MAGPIE_TTS_RT_FRONTEND_LOCK", default_value=""
            ),
        ),
        DeclareLaunchArgument(
            "frontend_contract",
            default_value=EnvironmentVariable(
                "MAGPIE_TTS_RT_FRONTEND_CONTRACT", default_value=""
            ),
        ),
        DeclareLaunchArgument("frontend_timeout_seconds", default_value="2.0"),
        DeclareLaunchArgument("request_progress_timeout_seconds", default_value="30.0"),
        DeclareLaunchArgument("cancellation_timeout_seconds", default_value="2.0"),
        DeclareLaunchArgument("startup_timeout_seconds", default_value="300.0"),
        DeclareLaunchArgument(
            "timing_collector_discovery_timeout_seconds", default_value="10.0"
        ),
        DeclareLaunchArgument("scheduler_watchdog_seconds", default_value="5.0"),
        Node(
            package="fv_tts",
            executable="fv_tts_node",
            name="fv_tts",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                os.path.join(share, "config", "default.yaml"),
                {
                    "native_library": LaunchConfiguration("native_library"),
                    "bundle_path": LaunchConfiguration("bundle_path"),
                    "manifest_sha256": LaunchConfiguration("manifest_sha256"),
                    "cuda_device_index": ParameterValue(
                        LaunchConfiguration("cuda_device_index"), value_type=int
                    ),
                    "frontend_python": LaunchConfiguration("frontend_python"),
                    "frontend_server": LaunchConfiguration("frontend_server"),
                    "frontend_lock": LaunchConfiguration("frontend_lock"),
                    "frontend_contract": LaunchConfiguration("frontend_contract"),
                    "frontend_timeout_seconds": ParameterValue(
                        LaunchConfiguration("frontend_timeout_seconds"),
                        value_type=float,
                    ),
                    "request_progress_timeout_seconds": ParameterValue(
                        LaunchConfiguration("request_progress_timeout_seconds"),
                        value_type=float,
                    ),
                    "cancellation_timeout_seconds": ParameterValue(
                        LaunchConfiguration("cancellation_timeout_seconds"),
                        value_type=float,
                    ),
                    "startup_timeout_seconds": ParameterValue(
                        LaunchConfiguration("startup_timeout_seconds"),
                        value_type=float,
                    ),
                    "timing_collector_discovery_timeout_seconds": ParameterValue(
                        LaunchConfiguration(
                            "timing_collector_discovery_timeout_seconds"
                        ),
                        value_type=float,
                    ),
                    "scheduler_watchdog_seconds": ParameterValue(
                        LaunchConfiguration("scheduler_watchdog_seconds"),
                        value_type=float,
                    ),
                },
            ],
        ),
    ])
