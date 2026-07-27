import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _default_runtime_manifest():
    data_home = os.environ.get("XDG_DATA_HOME", "")
    if not os.path.isabs(data_home):
        data_home = os.path.join(os.path.expanduser("~"), ".local", "share")
    return os.path.join(
        data_home, "aspa-navigation", "vosk-kws", "runtime.json"
    )

def _default_library_path():
    data_home = os.environ.get("XDG_DATA_HOME", "")
    if not os.path.isabs(data_home):
        data_home = os.path.join(os.path.expanduser("~"), ".local", "share")
    library_dir = os.path.join(data_home, "aspa-navigation", "vosk-kws", "lib")
    existing = os.environ.get("LD_LIBRARY_PATH", "")
    return os.pathsep.join(part for part in (library_dir, existing) if part)


def generate_launch_description():
    share = get_package_share_directory("fv_kws")
    return LaunchDescription([
        DeclareLaunchArgument(
            "runtime_manifest",
            default_value=EnvironmentVariable(
                "ASPA_VOSK_KWS_RUNTIME_MANIFEST",
                default_value=_default_runtime_manifest(),
            ),
        ),
        DeclareLaunchArgument(
            "library_path",
            default_value=EnvironmentVariable(
                "ASPA_VOSK_KWS_LIBRARY_PATH",
                default_value=_default_library_path(),
            ),
        ),
        DeclareLaunchArgument(
            "kws_config",
            default_value=os.path.join(share, "config", "fv_kws.yaml"),
        ),
        Node(
            package="fv_kws",
            executable="fv_kws",
            parameters=[
                LaunchConfiguration("kws_config"),
                {"runtime_manifest": LaunchConfiguration("runtime_manifest")},
            ],
            additional_env={
                "LD_LIBRARY_PATH": LaunchConfiguration("library_path"),
            },
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
