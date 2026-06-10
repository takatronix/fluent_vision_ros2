"""Track teleop mux source so each episode is labeled VLA vs teleop.

Subscribes to every `*/teleop_mux/status` topic on the graph (auto-discovers
on a 2s timer so new arms / profiles light up without restart). Caches the
last JSON payload per topic. The api_server snapshots this dict at episode
start + stop so the play modal / Episodes list can render a badge without
decoding the bag.
"""

from __future__ import annotations

import json
import threading
from typing import Optional

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


_MUX_TOPIC_SUFFIX = "/teleop_mux/status"


class MuxTracker:
    """Watches mux status topics and serves snapshots on demand."""

    def __init__(self, node: Node):
        self._node = node
        self._lock = threading.Lock()
        self._latest: dict[str, dict] = {}      # topic -> parsed JSON dict
        self._subs: dict[str, object] = {}       # topic -> Subscription
        # Mux publishers use LATCHED/TRANSIENT_LOCAL so late subscribers
        # get the last value. Match that to avoid missing the initial state.
        self._qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Periodically scan for new mux topics (e.g. after `restart` the
        # publishers come up later than us). 2s polling is fine for a control-
        # plane discovery.
        self._timer = self._node.create_timer(2.0, self._discover)

    def _discover(self) -> None:
        try:
            topics = self._node.get_topic_names_and_types()
        except Exception:
            return
        for name, types in topics:
            if not name.endswith(_MUX_TOPIC_SUFFIX):
                continue
            if "std_msgs/msg/String" not in types:
                continue
            with self._lock:
                if name in self._subs:
                    continue
                self._subs[name] = self._node.create_subscription(
                    String, name,
                    lambda msg, t=name: self._on_msg(t, msg),
                    self._qos,
                )
                self._node.get_logger().info(f"mux_tracker: subscribed to {name}")

    def _on_msg(self, topic: str, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(data, dict):
            return
        with self._lock:
            self._latest[topic] = data

    def snapshot(self) -> dict[str, dict]:
        """Return a copy of the latest mux state for every known topic."""
        with self._lock:
            return {k: dict(v) for k, v in self._latest.items()}

    def summarize(self, snapshot: Optional[dict[str, dict]] = None) -> dict[str, str]:
        """Compact label per topic, e.g. {"/follower_arm/teleop_mux/status":
        "ai:pi0"}. Suitable for a one-line UI badge."""
        snap = snapshot if snapshot is not None else self.snapshot()
        out: dict[str, str] = {}
        for topic, d in snap.items():
            src = (d.get("source") or "?").strip() or "?"
            if src in ("ai", "ai_route"):
                ctl = (d.get("ai_controller") or "ai").strip() or "ai"
                route = (d.get("ai_route_key") or "").strip()
                out[topic] = f"VLA: {ctl}" + (f"/{route}" if route else "")
            elif src == "leader":
                out[topic] = "teleop"
            elif src == "none":
                out[topic] = "停止"
            else:
                out[topic] = src
        return out
