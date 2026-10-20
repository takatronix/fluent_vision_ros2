"""Camera calibration helpers (pure python, no rclpy import at module load).

- ``camera_info_to_dict``: sensor_msgs/CameraInfo (rclpy or rosbags message,
  or anything with the same attributes) -> plain JSON-able dict. Used by
  camera_writer to store the first CameraInfo per camera into meta.json
  cameras[] (``camera_info`` key), and by lerobot_materializer to copy it into
  the dataset's per-episode sidecar.
- ``read_tf_static``: read /tf_static out of a rosbag2 directory (sqlite3 or
  mcap) via the pure-python ``rosbags`` reader. Returns None when the bag or
  ``rosbags`` is unavailable — callers must treat calibration as optional.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Optional


def _field(msg: Any, *names: str, default: Any = None) -> Any:
    # ROS 2 uses lower-case k/d/p/r, ROS 1 (and some bridges) upper-case.
    for n in names:
        if hasattr(msg, n):
            return getattr(msg, n)
    return default


def _floats(seq: Any) -> list[float]:
    if seq is None:
        return []
    return [float(x) for x in list(seq)]


def camera_info_to_dict(msg: Any) -> dict[str, Any]:
    """sensor_msgs/CameraInfo -> {frame_id, width, height, distortion_model, K, D, R, P}.

    K (3x3), R (3x3), P (3x4) are row-major flat lists, D is the distortion
    vector (length depends on distortion_model)."""
    header = _field(msg, "header")
    frame_id = str(_field(header, "frame_id", default="")) if header is not None else ""
    return {
        "frame_id": frame_id,
        "width": int(_field(msg, "width", default=0) or 0),
        "height": int(_field(msg, "height", default=0) or 0),
        "distortion_model": str(_field(msg, "distortion_model", default="") or ""),
        "K": _floats(_field(msg, "k", "K")),
        "D": _floats(_field(msg, "d", "D")),
        "R": _floats(_field(msg, "r", "R")),
        "P": _floats(_field(msg, "p", "P")),
    }


def read_tf_static(bag_dir: Path, topic: str = "/tf_static") -> Optional[list[dict[str, Any]]]:
    """Static transforms recorded in the bag, deduplicated per child frame.

    Returns ``[{"parent", "child", "xyz": [x,y,z], "quat_xyzw": [x,y,z,w]}, ...]``
    (sorted by parent/child), ``[]`` when the bag has the topic but no
    transforms, or ``None`` when the bag / topic / ``rosbags`` is unavailable.
    Never raises.
    """
    bag_dir = Path(bag_dir)
    if not (bag_dir / "metadata.yaml").exists():
        return None
    try:
        from rosbags.rosbag2 import Reader
        from rosbags.typesys import Stores, get_typestore
    except ImportError:
        return None
    try:
        typestore = get_typestore(Stores.ROS2_HUMBLE)
        by_child: dict[str, dict[str, Any]] = {}
        with Reader(str(bag_dir)) as reader:
            conns = [c for c in reader.connections if c.topic == topic]
            if not conns:
                return None
            for conn, _recv_ns, data in reader.messages(connections=conns):
                msg = typestore.deserialize_cdr(data, conn.msgtype)
                for tf in getattr(msg, "transforms", []) or []:
                    t = tf.transform.translation
                    q = tf.transform.rotation
                    child = str(tf.child_frame_id)
                    by_child[child] = {
                        "parent": str(tf.header.frame_id),
                        "child": child,
                        "xyz": [float(t.x), float(t.y), float(t.z)],
                        "quat_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
                    }
        return sorted(by_child.values(), key=lambda d: (d["parent"], d["child"]))
    except Exception:
        return None
