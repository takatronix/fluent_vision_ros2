"""Event Bus → episode marker bridge (Event Bus Contract v1).

The "進化成長していけるAI" principle: one situation-event stream feeds three
consumers — sound, speech, and *learning data*. fv_soundboard normalizes raw
events and re-publishes the record marker on ``/fv/event/active``
(std_msgs/String). This bridge is the *recording* consumer: every event that
arrives while an episode is active is written as a point-in-time marker
(``kind="event"``) so it becomes a semantic annotation on the episode
timeline — the input a VLA / RL trainer later reads to segment and label a run.

Concrete target (asparagus harvest cycle):

    detect → approach → grasp → harvest/cut → place → success | failure

Each of those events lands as an ordered marker (markers carry an absolute UTC
``started_at`` so they align to the bag / joint / video timelines). "phase"
events are tagged ``phase``; cycle boundaries + outcomes are tagged
``boundary``; success/failure additionally carry a marker ``outcome`` so a
segmenter can split the episode into labelled sub-tasks.

Payload forms accepted on ``/fv/event/active`` (both handled — forward
compatible):

  * ``"detect"``                              — plain record-marker name.
  * ``{"event":"detect","variant":"grade_A","confidence":0.9}`` — structured.
    Extra keys (variant / grade / class / object_id / confidence / ...) are
    stored as marker ``attributes`` and rolled up onto the episode at stop, so
    the *detected asparagus type/grade* is linked to the episode. NOTE:
    fv_soundboard currently forwards only the bare marker name (variant is
    stripped at normalization); for the structured payload to reach here the
    producer must publish JSON on ``/fv/event/active`` (see README / report).

Lifecycle note: this bridge does NOT start/stop recording. Episode
start/stop stays owned by the REST / batch control path (which runs disk
preflight, mux snapshot, bag + camera bring-up). Auto-segmentation
(auto_start → start, place/success/failure → stop) is a documented follow-up
that must go through that same path; see the design notes in the report.
"""

from __future__ import annotations

import json
from typing import Optional

from rclpy.node import Node
from std_msgs.msg import String


# Ordered harvest-cycle phases. Membership → marker tagged "phase" so a
# segmenter knows these define the sub-task sequence inside one episode.
# Configurable via the `event_phase_events` ROS param (aspa vs vlabor differ).
DEFAULT_PHASE_EVENTS = (
    "detect", "approach", "grasp", "harvest", "cut", "place",
)

# Cycle / session boundaries → tagged "boundary".
DEFAULT_BOUNDARY_EVENTS = (
    "auto_start", "auto_stop", "nav_start", "goal_reached",
    "success", "failure", "stop",
)

# Terminal events → carry a marker outcome ("success" | "abort").
# failure/abort map to "abort" to match EpisodeMarker.msg outcome vocabulary.
DEFAULT_OUTCOME_MAP = {
    "success": "success",
    "failure": "abort",
    "abort": "abort",
}

# Structured-payload keys that are NOT the event name — kept as attributes.
_RESERVED_KEYS = {"event", "record", "name", "say", "speak"}


class EventMarkerBridge:
    """Subscribes to the Event Bus and writes episode markers.

    Reads ``store.active`` (the single source of truth for the in-flight
    episode, owned by the API thread) and appends to the thread-safe
    ``MarkerManager``. Cross-thread safe: store.active is read-only here and
    marker_manager guards its own state.
    """

    def __init__(
        self,
        node: Node,
        store,
        marker_manager,
        *,
        topic: str = "/fv/event/active",
        kind: str = "event",
        phase_events=DEFAULT_PHASE_EVENTS,
        boundary_events=DEFAULT_BOUNDARY_EVENTS,
        outcome_map=None,
        enabled: bool = True,
    ):
        self._node = node
        self._store = store
        self._mm = marker_manager
        self._kind = kind
        self._phase = set(phase_events or ())
        self._boundary = set(boundary_events or ())
        self._outcome_map = dict(outcome_map if outcome_map is not None
                                 else DEFAULT_OUTCOME_MAP)
        self._topic = topic
        self._sub = None
        if enabled:
            # Default QoS (RELIABLE, KEEP_LAST 10) matches fv_soundboard's
            # publisher — events are discrete + must not be dropped.
            self._sub = node.create_subscription(String, topic, self._on_event, 10)
            node.get_logger().info(
                f"event bridge: subscribed to {topic} → episode markers "
                f"(phase={sorted(self._phase)}, boundary={sorted(self._boundary)})"
            )

    # ------------------------------------------------------------------

    @staticmethod
    def _parse(raw: str):
        """Return (event_name, attributes[]). Accepts a bare marker name or a
        JSON object; unknown extra JSON keys become {key, value} attributes."""
        raw = (raw or "").strip()
        if not raw:
            return None, []
        if raw[0] != "{":
            return raw, []
        try:
            obj = json.loads(raw)
        except (ValueError, TypeError):
            return None, []
        if not isinstance(obj, dict):
            return None, []
        name = (obj.get("event") or obj.get("record") or obj.get("name") or "").strip()
        if not name:
            return None, []
        attrs = [
            {"key": str(k), "value": v}
            for k, v in obj.items()
            if k not in _RESERVED_KEYS
        ]
        return name, attrs

    def _on_event(self, msg: String) -> None:
        name, attrs = self._parse(msg.data)
        if not name:
            return
        active = self._store.active
        if active is None:
            # Events fire continuously (nav, harvest); with nothing recording
            # there is no timeline to annotate. Not an error.
            self._node.get_logger().debug(
                f"event '{name}' ignored: no active episode")
            return

        tags = ["event_bus"]
        if name in self._phase:
            tags.append("phase")
        if name in self._boundary:
            tags.append("boundary")
        outcome = self._outcome_map.get(name)

        try:
            m = self._mm.start(
                active.episode_id,
                task_description=name,
                kind=self._kind,
                tags=tags,
                outcome=outcome,
                attributes=attrs,
            )
        except ValueError as exc:
            self._node.get_logger().warning(f"event marker rejected ({name}): {exc}")
            return
        extra = f" outcome={outcome}" if outcome else ""
        extra += f" attrs={len(attrs)}" if attrs else ""
        self._node.get_logger().info(
            f"episode {active.episode_id[-8:]}: marker '{name}'"
            f" [{','.join(tags)}]{extra} ({m.marker_id[-8:]})")
