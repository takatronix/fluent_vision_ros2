"""Persist raw anomaly episode intervals for later semantic annotation."""

from __future__ import annotations

import rclpy
from fv_episode_msgs.msg import EnvironmentChange
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .annotation_store import AnnotationStore, default_episode_root


class EpisodeRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("episode_recorder")
        default_root = default_episode_root()
        self.declare_parameter("database_path", str(default_root / "annotations.db"))
        self.declare_parameter("environment_change_topic", "/environment/change")

        database_path = str(self.get_parameter("database_path").value)
        change_topic = str(self.get_parameter("environment_change_topic").value)
        self.annotation_store = AnnotationStore(database_path)
        self.change_sub = self.create_subscription(
            EnvironmentChange,
            change_topic,
            self._on_environment_change,
            10,
        )
        self.get_logger().info(
            f"episode annotation recorder ready: topic={change_topic} database={database_path}"
        )

    def _on_environment_change(self, msg: EnvironmentChange) -> None:
        if not msg.episode_id:
            return
        try:
            if msg.state == "started":
                self.annotation_store.start_episode(msg.episode_id)
            elif msg.state == "ended":
                self.annotation_store.end_episode(msg.episode_id)
            else:
                self.get_logger().warning(f"ignored invalid environment state: {msg.state}")
        except Exception as exc:
            self.get_logger().error(f"anomaly episode persistence failed: {exc}")

    def destroy_node(self):
        self.annotation_store.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EpisodeRecorderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
