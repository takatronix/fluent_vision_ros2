"""Canonical anomaly episode publisher for the dialogue architecture."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException

from fv_visual_condition_detector_py.visual_condition_detector_node import (
    VisualConditionDetectorNode,
)


class VideoAnomalyDetectorNode(VisualConditionDetectorNode):
    def __init__(self) -> None:
        super().__init__(
            node_name="video_anomaly_detector",
            publish_legacy=False,
            publish_environment=True,
            require_model=True,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VideoAnomalyDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
