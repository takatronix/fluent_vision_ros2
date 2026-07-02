from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # No DeclareLaunchArgument here on purpose: generic names like
    # `node_name` / `config_file` inherit values from the parent launch
    # context when included by the vlabor profile launcher (observed:
    # node_name resolved to "d405" and collided with the RealSense node).
    pkg_share = FindPackageShare("fv_lingbot_depth")
    node = Node(
        package="fv_lingbot_depth",
        executable="fv_lingbot_depth_node",
        name="fv_lingbot_depth",
        output="screen",
        parameters=[PathJoinSubstitution([pkg_share, "config", "vlabor_d405.yaml"])],
        emulate_tty=True,
    )
    return LaunchDescription([node])
