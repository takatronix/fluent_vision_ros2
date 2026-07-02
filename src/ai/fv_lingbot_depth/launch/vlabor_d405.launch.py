from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("fv_lingbot_depth")
    node_name_arg = DeclareLaunchArgument(
        "node_name",
        default_value="fv_lingbot_depth",
        description="Node name",
    )
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "vlabor_d405.yaml"]),
        description="Path to YAML config file",
    )

    node = Node(
        package="fv_lingbot_depth",
        executable="fv_lingbot_depth_node",
        name=LaunchConfiguration("node_name"),
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
        emulate_tty=True,
    )

    return LaunchDescription([node_name_arg, config_file_arg, node])
