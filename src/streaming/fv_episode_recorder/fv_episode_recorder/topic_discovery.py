"""Profile-driven topic discovery (Codex #5 対応).

Reads a profile yaml (vlabor_launch/config/profiles/*.yaml) and derives the set
of topics to record + the camera list, so recorder does not hardcode topic
names that differ per profile (Piper VR=pos_cmd, AI=joint_cmd,
daihen=joint_position_command, aspa-navigation=different again).

Output (matches meta.json.recorded_topics schema):
  [{"topic": str, "role": "state"|"command"|"mux"|"tf"|"annotation"|"other",
    "qos": "reliable"|"best_effort"|"sensor_data"|"default",
    "stamp_source": "message_header"|"rosbag_recv"|"system"}, ...]

Cameras: [{"name": str, "topic": str, "codec": str?}, ...]

Phase 1 Step 4: best-effort. Phase 4+ may extend with role detection per arm.
"""

from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Any, Optional

import yaml

LOG = logging.getLogger("fv_episode_recorder.discovery")

# Topics every episode should include regardless of profile.
ALWAYS_INCLUDE: list[dict[str, str]] = [
    {"topic": "/tf",          "role": "tf",         "qos": "reliable",         "stamp_source": "message_header"},
    {"topic": "/tf_static",   "role": "tf",         "qos": "transient_local",  "stamp_source": "message_header"},
    {"topic": "/episode/markers", "role": "annotation", "qos": "reliable",     "stamp_source": "message_header"},
]


def _read_profile_inner(profile_name: str, profiles_dir: Path) -> Optional[dict[str, Any]]:
    """Raw single-file load (no extends resolution)."""
    for candidate in (profiles_dir / f"{profile_name}.yaml", profiles_dir / f"{profile_name}.yml"):
        if candidate.exists():
            try:
                with candidate.open() as f:
                    raw = yaml.safe_load(f) or {}
                if isinstance(raw, dict) and isinstance(raw.get("profile"), dict):
                    return raw["profile"]
                return raw
            except yaml.YAMLError as exc:
                LOG.error("profile yaml parse error %s: %s", candidate, exc)
                return None
    return None


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """Deep merge mimicking vlabor profile_loader: dicts merge recursively,
    lists are replaced wholesale (vlabor convention)."""
    out = dict(base)
    for k, v in override.items():
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_profile_yaml(
    profile_name: str,
    profiles_dir: Path | str,
    _visited: Optional[set[str]] = None,
) -> Optional[dict[str, Any]]:
    """Load profile yaml by name + resolve `extends:` chain.

    Returns the inner `profile:` block (vlabor convention) with parent merged
    in. Lists are replaced wholesale on conflict; dicts merge recursively.
    """
    profiles_dir = Path(profiles_dir)
    if _visited is None:
        _visited = set()
    if profile_name in _visited:
        LOG.error("profile extends cycle detected at %s", profile_name)
        return None
    _visited.add(profile_name)

    inner = _read_profile_inner(profile_name, profiles_dir)
    if inner is None:
        LOG.warning("profile yaml not found: %s in %s", profile_name, profiles_dir)
        return None

    parent_name = inner.get("extends")
    if isinstance(parent_name, str) and parent_name:
        parent = load_profile_yaml(parent_name, profiles_dir, _visited)
        if parent:
            inner = _deep_merge(parent, inner)
    return inner


def discover_topics(profile: dict[str, Any]) -> list[dict[str, Any]]:
    """Walk profile yaml and emit a deduped, role-tagged topic list."""
    discovered: list[dict[str, Any]] = []
    seen: set[str] = set()

    def add(topic: str, role: str, qos: str = "default",
            stamp_source: str = "rosbag_recv", msg_type: Optional[str] = None):
        if not topic or topic in seen:
            return
        seen.add(topic)
        entry = {"topic": topic, "role": role, "qos": qos, "stamp_source": stamp_source}
        if msg_type:
            entry["msg_type"] = msg_type
        discovered.append(entry)

    # episode_recorder block has highest priority
    er = profile.get("episode_recorder") or {}
    for t in er.get("record_topics_override") or []:
        if isinstance(t, str):
            add(t, role="override", qos="default")
        elif isinstance(t, dict) and "topic" in t:
            add(t["topic"], role=t.get("role", "override"),
                qos=t.get("qos", "default"),
                stamp_source=t.get("stamp_source", "rosbag_recv"))
    if er.get("record_topics_override"):
        # If profile says "use exactly this list", skip arm/teleop/lerobot
        # discovery — but STILL append depth cameras (kind=depth) since they
        # live under the cameras: block, and ALWAYS_INCLUDE (tf/markers).
        # Depth is recorded via the compressedDepth image_transport plugin
        # (lossless rvl/png ~5-10× compression) — recorder spawns a republisher
        # so we record the <topic>_compressed/compressedDepth topic, not raw.
        for c in er.get("cameras") or []:
            if not isinstance(c, dict) or (c.get("kind") or "").lower() != "depth":
                continue
            t = c.get("topic")
            if t:
                compressed = f"{t.rstrip('/')}_compressed/compressedDepth"
                add(compressed, role="camera_depth_compressed", qos="best_effort",
                    stamp_source="message_header",
                    msg_type="sensor_msgs/msg/CompressedImage")
        return discovered + [d for d in ALWAYS_INCLUDE if d["topic"] not in seen]

    # arm streams (Piper / daihen / SO101)
    for stream in profile.get("arm_streams") or []:
        rx = stream.get("rx", {}) or {}
        tx = stream.get("tx", {}) or {}
        # rx = state
        js = rx.get("joint_state") if isinstance(rx, dict) else None
        if isinstance(js, dict):
            add(js.get("topic"), role="state", qos=js.get("qos", "reliable"),
                stamp_source="message_header", msg_type=js.get("type"))
        elif isinstance(js, str):
            add(js, role="state", qos="reliable", stamp_source="message_header")
        # tx = commands (varies per profile)
        for tx_key in ("joint_cmd", "pos_cmd", "gripper_cmd"):
            entry = tx.get(tx_key) if isinstance(tx, dict) else None
            if isinstance(entry, dict):
                add(entry.get("topic"), role="command", qos=entry.get("qos", "reliable"),
                    msg_type=entry.get("type"))
            elif isinstance(entry, str):
                add(entry, role="command", qos="reliable")

    # teleop mux status
    teleop = profile.get("teleop") or {}
    mux = teleop.get("mux") or {}
    status_topic = mux.get("status_topic") or mux.get("status")
    if status_topic:
        add(status_topic, role="mux", qos="reliable")

    # lerobot block may carry arm_streams legacy (Piper teleop yaml uses this)
    lerobot = profile.get("lerobot") or {}
    for stream in lerobot.get("arm_streams") or []:
        for side in ("follower", "leader"):
            side_block = stream.get(side) if isinstance(stream, dict) else None
            if not isinstance(side_block, dict):
                continue
            rx = side_block.get("rx") or {}
            tx = side_block.get("tx") or {}
            js = rx.get("joint_state")
            if isinstance(js, dict):
                add(js.get("topic"), role="state",
                    qos=js.get("qos", "reliable"), stamp_source="message_header",
                    msg_type=js.get("type"))
            elif isinstance(js, str):
                add(js, role="state", qos="reliable", stamp_source="message_header")
            for tx_key in ("joint_cmd", "pos_cmd", "gripper_cmd"):
                ent = tx.get(tx_key)
                if isinstance(ent, dict):
                    add(ent.get("topic"), role="command",
                        qos=ent.get("qos", "reliable"), msg_type=ent.get("type"))
                elif isinstance(ent, str):
                    add(ent, role="command", qos="reliable")

    # Depth cameras → bag, recorded as CompressedImage via the
    # compressed_depth_image_transport plugin (lossless rvl/png 5-10×). The
    # recorder spawns `ros2 run image_transport republish` per depth camera
    # at episode start to feed the compressed topic that we record here.
    for c in er.get("cameras") or []:
        if not isinstance(c, dict) or (c.get("kind") or "").lower() != "depth":
            continue
        t = c.get("topic")
        if not t:
            continue
        compressed = f"{t.rstrip('/')}_compressed/compressedDepth"
        add(compressed, role="camera_depth_compressed", qos="best_effort",
            stamp_source="message_header",
            msg_type="sensor_msgs/msg/CompressedImage")

    # always include tf, tf_static, /episode/markers
    for d in ALWAYS_INCLUDE:
        if d["topic"] not in seen:
            discovered.append(d)
            seen.add(d["topic"])
    return discovered


def discover_cameras(profile: dict[str, Any]) -> list[dict[str, Any]]:
    """Return camera list from profile.episode_recorder.cameras (explicit only)."""
    er = profile.get("episode_recorder") or {}
    cams = er.get("cameras") or []
    out: list[dict[str, Any]] = []
    for c in cams:
        if not isinstance(c, dict):
            continue
        if "name" not in c or "topic" not in c:
            continue
        entry = {
            "name": str(c["name"]),
            "topic": str(c["topic"]),
            "codec": str(c.get("codec", "mp4v")),
        }
        # Preserve kind so CameraWriterPool dispatches color → mp4 vs depth →
        # 16-bit PNG sequence. Dropping this field was why arm_depth recorded
        # 0 frames (the color writer was instantiated for an Image topic).
        if c.get("kind"):
            entry["kind"] = str(c["kind"])
        out.append(entry)
    return out


def discover_episode_recorder_config(profile: dict[str, Any]) -> dict[str, Any]:
    """Extract episode_recorder block (Phase 1: subset used; full schema later)."""
    er = profile.get("episode_recorder") or {}
    return {
        "enabled": bool(er.get("enabled", False)),
        "output_dir": er.get("output_dir"),
        "fps": int(er.get("fps", 30)),
        "auto_pin": er.get("auto_pin") or {},
        "retention": er.get("retention") or {},
        "segment": er.get("segment") or {},
    }
