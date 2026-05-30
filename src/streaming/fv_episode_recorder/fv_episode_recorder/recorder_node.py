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

from .active_lock import ActiveLock
from .api_server import build_app
from .bag_recorder import BagRecorder
from .camera_writer import CameraWriterPool
from .episode_store import EpisodeStore
from .topic_discovery import (
    discover_cameras,
    discover_episode_recorder_config,
    discover_topics,
    load_profile_yaml,
)


LOG = logging.getLogger("fv_episode_recorder")


class FVEpisodeRecorderNode(Node):
    def __init__(self):
        super().__init__("fv_episode_recorder")

        # parameters
        default_output = os.environ.get("FV_EPISODE_OUTPUT_DIR", "/data/datasets")
        default_profiles = self._resolve_default_profiles_dir()
        self.declare_parameter("output_dir", default_output)
        self.declare_parameter("profiles_dir", default_profiles)
        self.declare_parameter("port", 8083)
        self.declare_parameter("host", "0.0.0.0")

        self.output_dir = Path(self.get_parameter("output_dir").value)
        self.profiles_dir = Path(self.get_parameter("profiles_dir").value)
        self.port = int(self.get_parameter("port").value)
        self.host = str(self.get_parameter("host").value)

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.store = EpisodeStore(self.output_dir)
        self.bag_recorder = BagRecorder(max_bag_size_mb=1024)
        self.camera_pool = CameraWriterPool(node=self)
        self.active_lock = ActiveLock(self.output_dir)
        self.profile_cache: dict = {}  # profile_name -> parsed yaml dict
        self._orphan_payload: dict = {}

        # Step 6: detect orphan from previous run (recorder crash mid-record)
        orphan = self.active_lock.detect_orphan()
        if orphan and orphan.get("status") == "orphan":
            self._orphan_payload = orphan
            self.get_logger().warning(
                f"orphan episode detected: id={orphan.get('episode_id')} "
                f"dir={orphan.get('episode_dir')} — call POST /api/v1/episodes/{orphan.get('episode_id')}/recover"
            )
            # Mark the meta as failed if the dir is still there
            ep_dir = Path(orphan.get("episode_dir", ""))
            meta_path = ep_dir / "meta.json"
            if meta_path.exists():
                try:
                    import json
                    with meta_path.open() as f:
                        data = json.load(f)
                    if data.get("state") in ("recording", "finalizing"):
                        data["state"] = "failed"
                        data["outcome"] = "abort"
                        tmp = meta_path.with_suffix(".json.tmp")
                        with tmp.open("w") as f:
                            json.dump(data, f, ensure_ascii=False, indent=2)
                        tmp.replace(meta_path)
                except Exception as exc:
                    self.get_logger().warning(f"orphan meta rewrite failed: {exc}")
            # The lock is left in place until operator calls /recover so the
            # event remains visible across multiple restarts.
        elif orphan and orphan.get("status") == "another_alive":
            self.get_logger().error("another recorder instance appears alive — exiting")
            import sys
            sys.exit(2)

    def get_profile(self, profile_name: str) -> dict:
        """Lazy-load + cache profile yaml."""
        if profile_name in self.profile_cache:
            return self.profile_cache[profile_name]
        loaded = load_profile_yaml(profile_name, self.profiles_dir)
        self.profile_cache[profile_name] = loaded or {}
        return self.profile_cache[profile_name]

    @staticmethod
    def _resolve_default_profiles_dir() -> str:
        """Find profiles dir, preferring ament-installed vlabor_launch, then
        the ~/.vlabor/profiles convention, then env override."""
        env = os.environ.get("FV_EPISODE_PROFILES_DIR")
        if env:
            return env
        try:
            from ament_index_python.packages import get_package_share_directory
            share = Path(get_package_share_directory("vlabor_launch")) / "config" / "profiles"
            if share.exists():
                return str(share)
        except Exception:
            pass
        for fallback in ("/vlabor/profiles",
                         str(Path.home() / ".vlabor" / "profiles")):
            if Path(fallback).exists():
                return fallback
        return "/vlabor/profiles"  # final fallback (may not exist)

        self.get_logger().info(
            f"fv_episode_recorder ready: output_dir={self.output_dir} "
            f"profiles_dir={self.profiles_dir} api=http://{self.host}:{self.port}/api/v1"
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
    app = build_app(node.store, node.bag_recorder, node.camera_pool,
                    active_lock=node.active_lock,
                    get_profile=node.get_profile)

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
