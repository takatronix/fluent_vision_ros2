from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("fv_policy_runner")
    default_params = PathJoinSubstitution([pkg_share, "config", "default.yaml"])

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
    )

    node = Node(
        package="fv_policy_runner",
        executable="fv_policy_runner_node",
        name="fv_policy_runner",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_arg, node])
