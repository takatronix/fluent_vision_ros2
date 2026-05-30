"""FluentVision Episode Recorder — ROS2 node + aiohttp REST API.

Phase 1 Step 1 walking skeleton:
  - rclpy Node (currently no subscriptions; just provides ROS lifecycle + logger)
  - aiohttp server on port 8083 with minimal /api/v1/episodes endpoints
  - filesystem-backed EpisodeStore (output_dir from ROS param)

Future steps add: bag recording, mp4 segment writer, frames sidecar,
markers, retention, replay, /vlabor/events publisher, etc.
"""

from __future__ import annotations

import asyncio
import logging
import os
import sys
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

try:
    from aiohttp import web
except ImportError:
    print("[fv_episode_recorder] ERROR: aiohttp not installed. "
          "Install with: pip3 install aiohttp", file=sys.stderr)
    raise

from .api_server import build_app
from .episode_store import EpisodeStore


LOG = logging.getLogger("fv_episode_recorder")


class FVEpisodeRecorderNode(Node):
    def __init__(self):
        super().__init__("fv_episode_recorder")

        # parameters
        default_output = os.environ.get("FV_EPISODE_OUTPUT_DIR", "/data/datasets")
        self.declare_parameter("output_dir", default_output)
        self.declare_parameter("port", 8083)
        self.declare_parameter("host", "0.0.0.0")

        self.output_dir = Path(self.get_parameter("output_dir").value)
        self.port = int(self.get_parameter("port").value)
        self.host = str(self.get_parameter("host").value)

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.store = EpisodeStore(self.output_dir)

        self.get_logger().info(
            f"fv_episode_recorder ready: output_dir={self.output_dir} api=http://{self.host}:{self.port}/api/v1"
        )


def main(args=None):
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    rclpy.init(args=args)
    node = FVEpisodeRecorderNode()

    # ROS2 spin in background thread
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # aiohttp loop in main thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    app = build_app(node.store)

    runner = web.AppRunner(app)
    loop.run_until_complete(runner.setup())
    site = web.TCPSite(runner, node.host, node.port)
    loop.run_until_complete(site.start())

    node.get_logger().info(f"aiohttp listening on http://{node.host}:{node.port}")

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        node.get_logger().info("shutdown requested")
    finally:
        loop.run_until_complete(runner.cleanup())
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
