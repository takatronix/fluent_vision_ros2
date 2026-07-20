"""Launch the official MOSS runtime as a standalone process, not a ROS node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():
    executable = PathJoinSubstitution([
        FindPackagePrefix("fv_episode_recorder"),
        "lib",
        "fv_episode_recorder",
        "moss_vl_realtime_runtime",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("repo", default_value="/opt/MOSS-VL"),
        DeclareLaunchArgument(
            "revision",
            default_value="5cf6753156df2c8e7f38e36b55d28687e1df425e",
        ),
        DeclareLaunchArgument(
            "checkpoint",
            default_value="OpenMOSS-Team/MOSS-VL-Realtime",
        ),
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="18081"),
        DeclareLaunchArgument("attention_backend", default_value="flash_attention_2"),
        DeclareLaunchArgument("allow_download", default_value="false"),
        ExecuteProcess(
            cmd=[executable],
            additional_env={
                "MOSS_VL_REPO": LaunchConfiguration("repo"),
                "MOSS_VL_REVISION": LaunchConfiguration("revision"),
                "MOSS_VL_CHECKPOINT": LaunchConfiguration("checkpoint"),
                "MOSS_VL_HOST": LaunchConfiguration("host"),
                "MOSS_VL_PORT": LaunchConfiguration("port"),
                "MOSS_VL_ATTENTION_BACKEND": LaunchConfiguration("attention_backend"),
                "MOSS_VL_ALLOW_DOWNLOAD": LaunchConfiguration("allow_download"),
            },
            output="screen",
        ),
    ])
