from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="fv_foundationpose_bridge",
            executable="fv_foundationpose_bridge_node",
            name="fv_foundationpose_bridge",
            output="screen",
        ),
    ])
