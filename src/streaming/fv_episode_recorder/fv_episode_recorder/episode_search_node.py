"""ROS 2 lexical search service over MOSS annotations."""

from __future__ import annotations

import rclpy
from fv_episode_msgs.srv import EpisodeSearch
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .annotation_store import AnnotationStore, default_episode_root


class EpisodeSearchNode(Node):
    def __init__(self) -> None:
        super().__init__("episode_search")
        default_root = default_episode_root()
        self.declare_parameter("database_path", str(default_root / "annotations.db"))
        self.declare_parameter("service_name", "/episode/search")
        self.declare_parameter("default_limit", 5)

        database_path = str(self.get_parameter("database_path").value)
        service_name = str(self.get_parameter("service_name").value)
        self.default_limit = max(1, int(self.get_parameter("default_limit").value))
        self.annotation_store = AnnotationStore(database_path)
        self.search_service = self.create_service(
            EpisodeSearch,
            service_name,
            self._search,
        )
        self.get_logger().info(
            f"episode lexical search ready: service={service_name} database={database_path}"
        )

    def _search(self, request: EpisodeSearch.Request, response: EpisodeSearch.Response):
        limit = int(request.limit) if request.limit else self.default_limit
        try:
            response.texts = self.annotation_store.search(request.query, limit)
        except Exception as exc:
            self.get_logger().error(f"episode search failed: {exc}")
            response.texts = []
        return response

    def destroy_node(self):
        self.annotation_store.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EpisodeSearchNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
