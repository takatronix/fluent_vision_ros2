"""Materialize fv_episode_recorder episodes into a LeRobot v3.0 dataset.

This module reads one or more episode directories produced by
``fv_episode_recorder`` (``meta.json`` + a rosbag2 sqlite3 ``bag/`` + per-camera
``videos/<cam>/0000.mp4`` with a ``frames.parquet`` sidecar) and writes them as a
single LeRobot **v3.0** dataset using the vendored ``lerobot`` package
(``LeRobotDataset.create`` -> ``add_frame`` -> ``save_episode`` -> ``finalize``).

CLI::

    python3 -m fv_episode_recorder.lerobot_materializer <episode_dir>... \
        --out <dataset_root> --repo-id <id> [--fps 30] \
        [--camera-map fv=lerobot,...] [--state-topic ...] [--action-topic ...] \
        [--task <override>] [--windows windows.json] [--include-depth] \
        [--outcome success|abort|discard]... [--tag X]...

Design notes
------------
* ``observation.state`` and ``action`` are float32 vectors of shape (7,):
  ``[joint1..joint6, gripper]``. Values are scaled by ``180/pi`` for ALL seven
  channels (the "degrees" convention, matching ``session_recorder.py``; the
  gripper is intentionally emitted as a synthetic "degree" value).
* Joint names on the wire are matched by alias/suffix (tolerating ``_joint``
  aliases and namespaces), mirroring
  ``percus_ai.vlabor.lerobot_streams.ArmStream.joint_values_in_profile_order``.
* Frames are resampled onto a uniform ``k/fps`` grid anchored at the first frame
  of the primary camera, then aligned the same way ``session_recorder.py`` does:
  the primary camera's nearest frame defines the pairing stamp, and state /
  action / secondary cameras are all sampled at-or-before that same stamp
  (zero-order hold). A tick whose nearest primary frame is farther than
  ``max(0.2s, 2/fps)`` is skipped, mirroring the recorder's staleness gate.
* The tick range is clipped to the overlap of the camera span and the recorded
  state/action spans. Without the clip, every episode ends with a band where the
  action stream has already stopped (episode judgement, bag teardown) but the
  cameras keep rolling — the last command would be zero-order-held under moving
  video for ~2-3s per episode, which is exactly the kind of frame/action
  misalignment the teleop recorder never writes.
* Depth cameras never go through LeRobot's 8-bit video encoder. By default
  they are skipped (as in v1). With ``include_depth=True`` / ``--include-depth``
  the ``depth_hevc10`` recordings are copied byte-for-byte (no re-encode, no
  re-quantisation) into ``depth/<key>/<source_episode_id>/`` and registered in
  ``meta/info.json["fv_depth"]`` with the exact frame range and timestamp
  offset per LeRobot episode; ``load_depth_frames()`` decodes them.
* ``windows`` (``--windows windows.json``) cut source episodes into time
  windows; each window becomes ONE LeRobot episode (e.g. one harvest attempt).
  Without windows: one LeRobot episode per episode dir (unchanged).
* ``meta/fv_episodes.jsonl`` carries one line per LeRobot ``episode_index``
  with the source episode metadata (outcome, tags, window, window ``extra``
  such as attempt_id/result/reason) and calibration (camera_info, /tf_static).
* lerobot / cv2 are imported lazily so the pure helpers (window selection,
  sidecar building, info.json patching, depth copy) are testable without them.
* Episodes are materialized one at a time (decode -> write -> release) so the
  peak RSS stays at ~one episode regardless of the episode count.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Environment must be configured *before* importing lerobot so that video
# encoding picks H.264 / yuv420p (matches session_recorder.py:32-34).
# ---------------------------------------------------------------------------
os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("LEROBOT_VIDEO_VCODEC", "h264")
os.environ.setdefault("LEROBOT_VIDEO_PIX_FMT", "yuv420p")

import pandas as pd  # noqa: E402

RAD_TO_DEG = 180.0 / math.pi

# Target joint order for the 7-DoF state/action vector.
JOINT_ORDER: Tuple[str, ...] = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "gripper",
)

# LeRobot feature channel names (prefixed to disambiguate the follower arm).
STATE_ACTION_NAMES: List[str] = [
    "follower_arm_joint1",
    "follower_arm_joint2",
    "follower_arm_joint3",
    "follower_arm_joint4",
    "follower_arm_joint5",
    "follower_arm_joint6",
    "follower_arm_gripper",
]

# Camera "kind" values that never become LeRobot image/video features
# (depth streams). depth_hevc10 can be exported as a sidecar (include_depth).
DEPTH_KINDS = {"depth_bag", "depth_hevc10", "depth_png_seq"}
DEPTH_EXPORT_KINDS = {"depth_hevc10"}
DEPTH_KEY_PREFIX = "observation.depth."
FV_EPISODES_FILE = "fv_episodes.jsonl"
OUTCOMES = ("success", "abort", "discard")

DEFAULT_CAMERA_MAP = {
    "top_camera": "top_camera",
    "arm_color": "arm_camera_1",
}

DEFAULT_STATE_TOPIC = "/follower_arm/joint_states_single"
DEFAULT_ACTION_TOPIC = "/follower_arm/joint_cmd"


# ===========================================================================
# Joint name alias resolution (mirrors ArmStream._joint_name_aliases)
# ===========================================================================
def _joint_aliases(name: object) -> Tuple[str, ...]:
    base = str(name or "").strip()
    if not base:
        return ()
    aliases: List[str] = [base]
    tail = base.split("/")[-1]
    if tail and tail not in aliases:
        aliases.append(tail)
    if tail.endswith("_joint"):
        plain = tail[: -len("_joint")]
        if plain and plain not in aliases:
            aliases.append(plain)
    elif tail:
        suffixed = f"{tail}_joint"
        if suffixed not in aliases:
            aliases.append(suffixed)
    return tuple(aliases)


def _resolve_vector(
    names: Sequence[object],
    values: Sequence[float],
    carry: np.ndarray,
) -> np.ndarray:
    """Resolve a JointState (names, positions) into the 7-DoF profile order.

    Any joint not present in the message keeps its previous ("carried") value,
    which handles messages that omit the gripper channel.
    """
    out = carry.astype(np.float64, copy=True)
    if not names or not values or len(names) != len(values):
        return out
    index_by_alias: Dict[str, int] = {}
    for idx, nm in enumerate(names):
        for alias in _joint_aliases(nm):
            index_by_alias.setdefault(alias, idx)
    for j, joint in enumerate(JOINT_ORDER):
        for alias in _joint_aliases(joint):
            if alias in index_by_alias:
                out[j] = float(values[index_by_alias[alias]])
                break
    return out


# ===========================================================================
# Bag reading
# ===========================================================================
def _read_joint_series_rosbags(
    bag_dir: Path, topic: str, stamp_source: str
) -> List[Tuple[int, np.ndarray]]:
    """Read a JointState topic via the pure-python ``rosbags`` reader.

    Returns a time-sorted list of ``(stamp_ns, vec7)`` where vec7 is the raw
    (unscaled) ROS value in profile order, with zero-order carry for any joint
    missing from an individual message.
    """
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    raw: List[Tuple[int, List[str], List[float]]] = []
    with Reader(str(bag_dir)) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            return []
        for conn, recv_ns, data in reader.messages(connections=conns):
            if conn.msgtype != "sensor_msgs/msg/JointState":
                continue
            msg = typestore.deserialize_cdr(data, conn.msgtype)
            if stamp_source == "header":
                hdr = msg.header.stamp
                stamp_ns = int(hdr.sec) * 1_000_000_000 + int(hdr.nanosec)
                if stamp_ns <= 0:
                    stamp_ns = int(recv_ns)
            else:
                stamp_ns = int(recv_ns)
            raw.append((stamp_ns, list(msg.name), [float(x) for x in msg.position]))

    return _fold_carry(raw)


def _read_joint_series_manual(
    bag_dir: Path, topic: str, stamp_source: str
) -> List[Tuple[int, np.ndarray]]:
    """Fallback: read JointState directly from sqlite3 with a minimal CDR parser.

    Only used when ``rosbags`` is unavailable. Parses the ROS2 CDR encoding:
    4-byte encapsulation header, then std_msgs/Header (sec int32, nanosec
    uint32, frame_id string), then string[] name / float64[] position /
    velocity / effort, honouring 4-byte and 8-byte alignment.
    """
    import sqlite3

    db_files = sorted(bag_dir.glob("*.db3"))
    if not db_files:
        return []

    raw: List[Tuple[int, List[str], List[float]]] = []
    for db in db_files:
        con = sqlite3.connect(str(db))
        try:
            cur = con.cursor()
            row = cur.execute("SELECT id FROM topics WHERE name = ?", (topic,)).fetchone()
            if row is None:
                continue
            topic_id = row[0]
            for recv_ns, blob in cur.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
                (topic_id,),
            ):
                try:
                    sec, nanosec, names, positions = _parse_jointstate_cdr(bytes(blob))
                except Exception:
                    continue
                if stamp_source == "header":
                    stamp_ns = sec * 1_000_000_000 + nanosec
                    if stamp_ns <= 0:
                        stamp_ns = int(recv_ns)
                else:
                    stamp_ns = int(recv_ns)
                raw.append((stamp_ns, names, positions))
        finally:
            con.close()

    return _fold_carry(raw)


def _fold_carry(
    raw: List[Tuple[int, List[str], List[float]]]
) -> List[Tuple[int, np.ndarray]]:
    """Sort raw messages by stamp and apply zero-order carry per joint."""
    raw.sort(key=lambda r: r[0])
    series: List[Tuple[int, np.ndarray]] = []
    carry = np.zeros(len(JOINT_ORDER), dtype=np.float64)
    for stamp_ns, names, positions in raw:
        carry = _resolve_vector(names, positions, carry)
        series.append((stamp_ns, carry.copy()))
    return series


class _CDRReader:
    """Minimal little-endian CDR reader with alignment relative to stream start."""

    def __init__(self, buf: bytes):
        # 4-byte encapsulation header; CDR alignment is relative to the byte
        # following it (the start of the CDR stream).
        self.buf = buf
        self.base = 4
        self.pos = 4

    def _align(self, size: int) -> None:
        rel = self.pos - self.base
        pad = (-rel) % size
        self.pos += pad

    def _read(self, fmt: str, size: int):
        import struct

        self._align(size)
        val = struct.unpack_from(fmt, self.buf, self.pos)[0]
        self.pos += size
        return val

    def int32(self) -> int:
        return self._read("<i", 4)

    def uint32(self) -> int:
        return self._read("<I", 4)

    def float64(self) -> float:
        return self._read("<d", 8)

    def string(self) -> str:
        n = self.uint32()
        raw = self.buf[self.pos : self.pos + n]
        self.pos += n
        return raw.split(b"\x00", 1)[0].decode("utf-8", "replace")

    def float64_array(self) -> List[float]:
        n = self.uint32()
        return [self.float64() for _ in range(n)]

    def string_array(self) -> List[str]:
        n = self.uint32()
        return [self.string() for _ in range(n)]


def _parse_jointstate_cdr(blob: bytes) -> Tuple[int, int, List[str], List[float]]:
    r = _CDRReader(blob)
    sec = r.int32()
    nanosec = r.uint32()
    _frame_id = r.string()
    names = r.string_array()
    positions = r.float64_array()
    return sec, nanosec, names, positions


def read_joint_series(
    bag_dir: Path, topic: str, stamp_source: str
) -> List[Tuple[int, np.ndarray]]:
    try:
        import rosbags  # noqa: F401

        return _read_joint_series_rosbags(bag_dir, topic, stamp_source)
    except ImportError:
        print(
            "  [warn] 'rosbags' not available; using manual CDR parser fallback",
            file=sys.stderr,
        )
        return _read_joint_series_manual(bag_dir, topic, stamp_source)


# ===========================================================================
# Camera decoding
# ===========================================================================
def load_camera_frames(
    cam_video_dir: Path, sidecar: Path
) -> Tuple[np.ndarray, List[np.ndarray]]:
    """Decode a camera's frames into an ordered list of RGB uint8 arrays.

    Row ``i`` of ``frames.parquet`` corresponds to element ``i`` of the returned
    list. Returns ``(ros_stamp_ns array, images)``.
    """
    import cv2  # lazy: the pure helpers / tests must not need OpenCV

    df = pd.read_parquet(sidecar)
    df = df.sort_values("frame_index").reset_index(drop=True)
    stamps = df["ros_stamp_ns"].to_numpy(dtype=np.int64)

    # Decode each segment video once, sequentially. Frame N read from a segment
    # corresponds to segment_local_frame == N.
    seg_cache: Dict[str, List[np.ndarray]] = {}
    for seg_file in df["segment_file"].unique():
        cap = cv2.VideoCapture(str(cam_video_dir / str(seg_file)))
        frames: List[np.ndarray] = []
        while True:
            ok, bgr = cap.read()
            if not ok:
                break
            frames.append(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        cap.release()
        seg_cache[str(seg_file)] = frames

    images: List[np.ndarray] = []
    for _, row in df.iterrows():
        seg = str(row["segment_file"])
        local = int(row["segment_local_frame"])
        frames = seg_cache.get(seg, [])
        if 0 <= local < len(frames):
            images.append(frames[local])
        elif images:
            images.append(images[-1])  # tolerate a missing decode
        else:
            images.append(np.zeros((1, 1, 3), dtype=np.uint8))
    return stamps, images


# ===========================================================================
# Episode model
# ===========================================================================
class CameraSource:
    def __init__(self, fv_name: str, lerobot_key: str, stamps: np.ndarray, images: List[np.ndarray]):
        self.fv_name = fv_name
        self.lerobot_key = lerobot_key
        self.stamps = stamps
        self.images = images
        self.height = images[0].shape[0] if images else 0
        self.width = images[0].shape[1] if images else 0
        self.reused = 0
        self.used = 0


def _sample_zoh(series: List[Tuple[int, np.ndarray]], tick_ns: int) -> np.ndarray:
    """Zero-order hold: latest sample at-or-before tick; else first; else zeros."""
    if not series:
        return np.zeros(len(JOINT_ORDER), dtype=np.float64)
    stamps = [s for s, _ in series]
    idx = np.searchsorted(stamps, tick_ns, side="right") - 1
    if idx < 0:
        idx = 0
    return series[idx][1]


def _nearest_index(cam: CameraSource, tick_ns: int) -> int:
    """Index of the camera frame whose stamp is nearest to ``tick_ns``."""
    return int(np.argmin(np.abs(cam.stamps - tick_ns)))


def _floor_index(cam: CameraSource, stamp_ns: int) -> int:
    """Index of the latest frame at-or-before ``stamp_ns`` (clamped to 0).

    Secondary cameras use this so every modality of a frame refers to the same
    pairing stamp, matching session_recorder's floor selection. The clamp only
    fires for ticks before the camera's first frame (episode head).
    """
    idx = int(np.searchsorted(cam.stamps, stamp_ns, side="right")) - 1
    return max(idx, 0)


def _pick_image(
    cam: CameraSource, idx: int, pair_stamp_ns: int, slack_ns: int
) -> np.ndarray:
    """Fetch frame ``idx``; count it as reused when farther than ``slack_ns``.

    The slack differs by selection mode: nearest (primary) is normally within
    half a frame period, floor (secondaries) within a full period — only
    distances beyond that indicate an actual source gap (frozen image).
    """
    dist_ns = abs(int(cam.stamps[idx]) - pair_stamp_ns)
    if dist_ns > slack_ns:
        cam.reused += 1
    cam.used += 1
    return cam.images[idx]


def _select_cameras(
    episode_dir: Path, meta: dict, camera_map: Dict[str, str]
) -> Tuple[List[Tuple[str, str, Path, Path]], List[str]]:
    """meta.json からデコード対象の (color) カメラを選別する。デコードはしない。

    実ロード (Episode.load) と materialize() の事前検証の両方がこの関数を使う。
    判定を 1 箇所に集約しているのは、事前検証と実ロードで判定がずれると
    「検証は通ったのに書き込み途中でカメラ不一致になる」ため。二重実装禁止。

    末尾のソートにより EP 間でカメラの「順序だけ」が異なるケースは
    camera set mismatch にならず黙って canonical 順に正規化される (意図的:
    recorder 設定順に依存しない出力が目的)。集合の差 (欠落/余剰/別名) は
    引き続き検出される。

    Returns ``(selected, skipped)`` — selected の各要素は
    ``(fv_name, lerobot_key, cam_video_dir, sidecar_path)``、skipped は表示用ラベル。
    """
    selected: List[Tuple[str, str, Path, Path]] = []
    skipped: List[str] = []
    for cam in meta.get("cameras", []):
        name = cam.get("name")
        kind = cam.get("kind")
        if kind in DEPTH_KINDS:
            skipped.append(f"{name} (depth:{kind})")
            continue
        video_dir = cam.get("video_dir")
        sidecar = cam.get("sidecar_file")
        frame_count = int(cam.get("frame_count") or 0)
        if not video_dir or not sidecar or frame_count <= 0:
            skipped.append(f"{name} (no frames)")
            continue
        sidecar_path = episode_dir / "videos" / sidecar
        cam_dir = episode_dir / "videos" / str(video_dir)
        if not sidecar_path.exists():
            skipped.append(f"{name} (missing sidecar)")
            continue
        lerobot_key = camera_map.get(str(name), str(name))
        selected.append((str(name), lerobot_key, cam_dir, sidecar_path))
    # 視覚 feature の並びを teleop 収録 (session_recorder) と同一に固定する:
    # arm 系カメラが先頭、次に top、その他は名前順。並びは LeRobot の
    # info.json features 順 = ポリシーの視覚スロット割当になるため、録画元
    # (meta.json cameras の記載順 = recorder 設定順) 任せにすると teleop
    # データセットとスロットが入れ替わる (2026-08-20 W40 監査: datagen
    # データセットだけ top が第1スロットになり学習条件のパリティが崩れた)。
    # 「順=スロット」の根拠 (lerobot 側): 学習時 config.input_features は
    # データセット features 順から充填され (policies/factory.py の
    # dataset_to_policy_features)、pi05 は config.image_features の反復順で
    # 画像トークンを連結する (policies/pi05/modeling_pi05.py の
    # _preprocess_images / embed_prefix)。位置=スロットで名前引きは無い。
    # daihen-physical-ai 側の同一規則:
    # percus_ai/vlabor/lerobot_streams.py の camera_slot_order。変更時は両方を揃えること。
    def _slot_order(item: Tuple[str, str, Path, Path]) -> Tuple[int, str]:
        key = item[1]
        if key.startswith("arm"):
            return (0, key)
        if key.startswith("top"):
            return (1, key)
        return (2, key)

    selected.sort(key=_slot_order)
    return selected, skipped


class Episode:
    def __init__(self, episode_dir: Path):
        self.dir = episode_dir
        self.meta = json.loads((episode_dir / "meta.json").read_text())
        self.task = str(self.meta.get("task_description") or "").strip()
        self.cameras: List[CameraSource] = []
        self.primary: Optional[CameraSource] = None
        self.state_series: List[Tuple[int, np.ndarray]] = []
        self.action_series: List[Tuple[int, np.ndarray]] = []

    def load(self, camera_map: Dict[str, str], state_topic: str, action_topic: str) -> None:
        # --- cameras ---
        selected, skipped = _select_cameras(self.dir, self.meta, camera_map)
        for fv_name, lerobot_key, cam_dir, sidecar_path in selected:
            stamps, images = load_camera_frames(cam_dir, sidecar_path)
            self.cameras.append(CameraSource(fv_name, lerobot_key, stamps, images))
        self.skipped_cameras = skipped

        if not self.cameras:
            raise RuntimeError(f"episode {self.dir} has no usable (color) cameras")

        # primary: prefer fv 'top_camera' if usable, else first usable camera
        self.primary = next((c for c in self.cameras if c.fv_name == "top_camera"), self.cameras[0])

        # --- state / action series (raw ROS values, then scaled to degrees) ---
        # Both series use header stamps (publish time), the same domain
        # session_recorder aligns on. The bag receive stamp adds transport +
        # sqlite-write latency (measured 0.3-2.3ms) and mixes domains between
        # state and action for no benefit. _fold_carry sorts by stamp, so
        # slight header-order inversions on the merged topic are tolerated.
        bag_dir = self.dir / str(self.meta.get("bag_path", "bag/")).rstrip("/")
        self.state_series = self._scale(read_joint_series(bag_dir, state_topic, "header"))
        self.action_series = self._scale(read_joint_series(bag_dir, action_topic, "header"))

    @staticmethod
    def _scale(series: List[Tuple[int, np.ndarray]]) -> List[Tuple[int, np.ndarray]]:
        return [(s, v * RAD_TO_DEG) for s, v in series]

    def timeline(self, fps: int) -> List[int]:
        stamps = self.primary.stamps
        if len(stamps) == 0:
            return []
        t0, t_end = int(stamps[0]), int(stamps[-1])
        n = int(math.floor((t_end - t0) / 1e9 * fps)) + 1
        return [t0 + int(round(k / fps * 1e9)) for k in range(max(n, 1))]

    def aligned_span(self) -> Tuple[int, int]:
        """Overlap of camera / state / action recordings, in stamp ns.

        The bag subscribes late (~0.5s) and stops publishing actions before the
        cameras stop (episode judgement runs while recording); ticks outside
        this overlap would be filled by extrapolated zero-order hold — a frozen
        action under moving video at the tail, a future-value backfill at the
        head. The teleop recorder never emits either, so neither do we.
        """
        if not self.state_series or not self.action_series:
            raise RuntimeError(f"episode {self.dir} has an empty state or action series")
        start = max(
            int(self.primary.stamps[0]),
            self.state_series[0][0],
            self.action_series[0][0],
        )
        end = min(
            int(self.primary.stamps[-1]),
            self.state_series[-1][0],
            self.action_series[-1][0],
        )
        if end <= start:
            raise RuntimeError(
                f"episode {self.dir}: camera/state/action spans do not overlap"
            )
        return start, end

    def release(self) -> None:
        """デコード済み画像と関節系列を解放する。

        materialize() が save_episode 直後に呼ぶ。次のエピソードのデコードが
        始まる前に確実に手放すことで、ピーク RSS を常に「1 エピソード分」に
        保つ。全エピソード保持だと RSS がエピソード数に比例して伸び、実測で
        60EP (~40k フレーム、カメラ 2 本 640x480) が 65GB に達して OOM kill
        された — この解放を外してはいけない。
        """
        for cam in self.cameras:
            cam.images = []
            cam.stamps = np.empty(0, dtype=np.int64)
        self.state_series = []
        self.action_series = []


# ===========================================================================
# Episode selection / time windows / sidecars (pure — no lerobot, no cv2)
# ===========================================================================
def parse_started_at(value: Any) -> Optional[float]:
    """meta.json ``started_at`` (ISO-8601, ``...Z``) -> unix seconds, or None."""
    if value is None or value == "":
        return None
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value).strip()
    if text.endswith("Z"):
        text = text[:-1] + "+00:00"  # python 3.10 fromisoformat has no 'Z'
    try:
        dt = datetime.fromisoformat(text)
    except ValueError:
        return None
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt.timestamp()


def _opt_float(value: Any, what: str) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        raise ValueError(f"window {what} must be a number, got {value!r}") from None


def normalize_windows(raw: Any, base_dir: Optional[Path] = None) -> List[dict]:
    """Validate a windows list (the parsed windows.json).

    Accepts a list or ``{"windows": [...]}``. Each entry::

        {"episode_dir": "<path>",              # required
         "start_s": 12.0, "end_s": 48.5,       # seconds from meta.started_at
         # or "start_unix": ..., "end_unix": ...  (absolute unix seconds)
         "task": "harvest asparagus",          # optional, overrides the task
         "extra": {"attempt_id": ..., ...}}    # optional, copied to fv_episodes.jsonl

    Either bound may be omitted (= open towards that end). ``*_unix`` wins over
    ``*_s`` when both are given. Relative ``episode_dir`` resolves against
    ``base_dir`` (the windows.json directory) when that path exists, else cwd.
    Returns normalized dicts with an added ``index`` (position in the input).
    """
    if isinstance(raw, dict) and "windows" in raw:
        raw = raw["windows"]
    if not isinstance(raw, list):
        raise ValueError("windows must be a JSON list (or {\"windows\": [...]})")
    out: List[dict] = []
    for i, w in enumerate(raw):
        if not isinstance(w, dict) or not w.get("episode_dir"):
            raise ValueError(f"window #{i}: 'episode_dir' is required")
        ep = Path(str(w["episode_dir"])).expanduser()
        if not ep.is_absolute():
            if base_dir is not None and (base_dir / ep).exists():
                ep = base_dir / ep
            ep = ep.resolve()
        extra = w.get("extra") or {}
        if not isinstance(extra, dict):
            raise ValueError(f"window #{i}: 'extra' must be an object")
        norm = {
            "index": i,
            "episode_dir": ep,
            "start_s": _opt_float(w.get("start_s"), "start_s"),
            "end_s": _opt_float(w.get("end_s"), "end_s"),
            "start_unix": _opt_float(w.get("start_unix"), "start_unix"),
            "end_unix": _opt_float(w.get("end_unix"), "end_unix"),
            "task": (str(w["task"]).strip() or None) if w.get("task") else None,
            "extra": dict(extra),
        }
        out.append(norm)
    return out


def load_windows(path: Path) -> List[dict]:
    path = Path(path)
    return normalize_windows(json.loads(path.read_text()), base_dir=path.resolve().parent)


def window_bounds_ns(window: dict, meta: dict) -> Tuple[Optional[int], Optional[int]]:
    """Absolute [start, end] of a window in ns (None = unbounded).

    Relative bounds (start_s/end_s) are offsets from meta ``started_at`` —
    the same origin as the recorder's trim_start_s/trim_end_s. Camera/joint
    stamps are header stamps on the same wall clock."""
    origin: Optional[float] = None

    def _bound(unix_key: str, rel_key: str) -> Optional[int]:
        nonlocal origin
        if window.get(unix_key) is not None:
            return int(round(float(window[unix_key]) * 1e9))
        if window.get(rel_key) is not None:
            if origin is None:
                origin = parse_started_at(meta.get("started_at"))
                if origin is None:
                    raise ValueError(
                        f"window #{window.get('index')}: relative {rel_key} needs "
                        f"meta.started_at (episode {meta.get('episode_id')})")
            return int(round((origin + float(window[rel_key])) * 1e9))
        return None

    start = _bound("start_unix", "start_s")
    end = _bound("end_unix", "end_s")
    if start is not None and end is not None and end <= start:
        raise ValueError(f"window #{window.get('index')}: end must be after start")
    return start, end


def episode_matches(meta: dict, outcomes: Optional[Sequence[str]],
                    tags: Optional[Sequence[str]]) -> bool:
    """--outcome: any-of (meta.outcome in outcomes). --tag: all-of
    (every requested tag present in meta.tags). Empty filter = pass."""
    if outcomes:
        if str(meta.get("outcome") or "") not in set(outcomes):
            return False
    if tags:
        have = {str(t) for t in (meta.get("tags") or [])}
        if not set(str(t) for t in tags) <= have:
            return False
    return True


def plan_units(
    episode_dirs: Sequence[Path],
    windows: Optional[Sequence[dict]] = None,
    outcomes: Optional[Sequence[str]] = None,
    tags: Optional[Sequence[str]] = None,
) -> List[dict]:
    """Decide which LeRobot episodes to write.

    Returns a list of ``{"episode_dir": Path, "meta": dict, "window": dict|None}``
    — one element per future LeRobot episode, in episode_index order.
    Without windows: one unit per episode dir (input order). With windows: one
    unit per window, grouped by episode dir in first-appearance order (so a
    source episode is decoded once even if its windows are interleaved),
    windows of the same dir keep their input order. Outcome/tag filters apply
    to the source episode's meta.json in both modes.
    """
    meta_cache: Dict[Path, dict] = {}

    def _meta(d: Path) -> dict:
        if d not in meta_cache:
            meta_cache[d] = json.loads((d / "meta.json").read_text())
        return meta_cache[d]

    units: List[dict] = []
    if windows is None:
        for d in episode_dirs:
            d = Path(d)
            meta = _meta(d)
            if not episode_matches(meta, outcomes, tags):
                print(f"[filter] skip {d.name} (outcome={meta.get('outcome')} tags={meta.get('tags')})")
                continue
            units.append({"episode_dir": d, "meta": meta, "window": None})
        return units

    order: List[Path] = []
    grouped: Dict[Path, List[dict]] = {}
    for w in windows:
        d = Path(w["episode_dir"])
        if d not in grouped:
            grouped[d] = []
            order.append(d)
        grouped[d].append(w)
    for d in order:
        meta = _meta(d)
        if not episode_matches(meta, outcomes, tags):
            print(f"[filter] skip {d.name}: {len(grouped[d])} window(s) "
                  f"(outcome={meta.get('outcome')} tags={meta.get('tags')})")
            continue
        for w in grouped[d]:
            units.append({"episode_dir": d, "meta": meta, "window": w})
    return units


def select_ticks(
    all_ticks: Sequence[int],
    span_start: int,
    span_end: int,
    window_ns: Optional[Tuple[Optional[int], Optional[int]]] = None,
) -> List[int]:
    """Ticks inside the aligned camera/state/action span AND the window."""
    lo, hi = span_start, span_end
    if window_ns is not None:
        if window_ns[0] is not None:
            lo = max(lo, int(window_ns[0]))
        if window_ns[1] is not None:
            hi = min(hi, int(window_ns[1]))
    return [t for t in all_ticks if lo <= t <= hi]


def collect_calibration(ep_dir: Path, meta: dict, camera_map: Dict[str, str]) -> Optional[dict]:
    """camera_info from meta.json cameras[] + /tf_static from the bag.

    ``{"cameras": {fv_name: {"key", "kind", "camera_info"}}, "tf_static": [...]|None}``
    or None when neither is available. Never raises."""
    from .calibration import read_tf_static

    cams: Dict[str, dict] = {}
    for cam in meta.get("cameras", []) or []:
        info = cam.get("camera_info")
        if not info:
            continue
        name = str(cam.get("name"))
        kind = cam.get("kind") or "color"
        prefix = DEPTH_KEY_PREFIX if kind in DEPTH_KINDS else "observation.images."
        cams[name] = {
            "key": prefix + camera_map.get(name, name),
            "kind": kind,
            "camera_info": info,
        }
    try:
        bag_dir = ep_dir / str(meta.get("bag_path", "bag/")).rstrip("/")
        tf_static = read_tf_static(bag_dir)
    except Exception:
        tf_static = None
    if not cams and tf_static is None:
        return None
    return {"cameras": cams, "tf_static": tf_static}


def build_fv_episode_record(
    episode_index: int,
    ep_dir: Path,
    meta: dict,
    window: Optional[dict],
    *,
    task: str,
    n_frames: int,
    fps: int,
    t0_stamp_ns: int,
    t_end_stamp_ns: int,
    calibration: Optional[dict] = None,
    depth_keys: Optional[List[str]] = None,
) -> dict:
    """One meta/fv_episodes.jsonl line (see README for the key list)."""
    win = None
    if window is not None:
        win = {
            "index": window.get("index"),
            "start_s": window.get("start_s"),
            "end_s": window.get("end_s"),
            "start_unix": window.get("start_unix"),
            "end_unix": window.get("end_unix"),
        }
    started_unix = parse_started_at(meta.get("started_at"))
    return {
        "episode_index": int(episode_index),
        "source_episode_id": meta.get("episode_id"),
        "episode_dir": ep_dir.name,
        "episode_path": str(ep_dir),
        "profile": meta.get("profile"),
        "robot_id": meta.get("robot_id"),
        "operator": meta.get("operator"),
        "parent_session_id": meta.get("parent_session_id"),
        "task": task,
        "task_description": meta.get("task_description"),
        "started_at": meta.get("started_at"),
        "outcome": meta.get("outcome"),
        "tags": list(meta.get("tags") or []),
        "pinned": bool(meta.get("pinned", False)),
        "detected_variants": list(meta.get("detected_variants") or []),
        "window": win,
        "extra": dict((window or {}).get("extra") or {}),
        "n_frames": int(n_frames),
        "fps": int(fps),
        # LeRobot timestamp t of this episode <-> header stamp t0 + t (ns)
        "t0_stamp_ns": int(t0_stamp_ns),
        "t_end_stamp_ns": int(t_end_stamp_ns),
        "t0_offset_s": (round(t0_stamp_ns / 1e9 - started_unix, 6)
                        if started_unix is not None else None),
        "calibration": calibration,
        "depth_keys": list(depth_keys or []),
    }


def write_fv_episodes(out_root: Path, records: Sequence[dict]) -> Path:
    path = Path(out_root) / "meta" / FV_EPISODES_FILE
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        for rec in sorted(records, key=lambda r: r["episode_index"]):
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    return path


# ---------------------------------------------------------------------------
# Depth sidecar (depth_hevc10 copied as-is, never re-encoded)
# ---------------------------------------------------------------------------
def depth_sources(ep_dir: Path, meta: dict, camera_map: Dict[str, str]) -> List[dict]:
    """depth_hevc10 cameras with frames + video + depth_meta.json on disk."""
    out: List[dict] = []
    for cam in meta.get("cameras", []) or []:
        if cam.get("kind") not in DEPTH_EXPORT_KINDS:
            continue
        name = str(cam.get("name"))
        video_dir = cam.get("video_dir")
        if not video_dir or int(cam.get("frame_count") or 0) <= 0:
            print(f"  skip depth: {name} (no frames)")
            continue
        cam_dir = ep_dir / "videos" / str(video_dir)
        segs = cam.get("segments") or [{"file": "0000.mp4"}]
        video = cam_dir / str(segs[0].get("file") or "0000.mp4")
        meta_file = cam_dir / "depth_meta.json"
        sidecar = ep_dir / "videos" / str(cam.get("sidecar_file") or f"{name}/frames.parquet")
        missing = [p.name for p in (video, meta_file, sidecar) if not p.exists()]
        if missing:
            print(f"  skip depth: {name} (missing {missing})")
            continue
        out.append({
            "fv_name": name,
            "key": DEPTH_KEY_PREFIX + camera_map.get(name, name),
            "video": video,
            "meta_file": meta_file,
            "sidecar": sidecar,
        })
    return out


def _read_depth_sidecar(sidecar: Path) -> Tuple[np.ndarray, np.ndarray]:
    """(ros_stamp_ns, segment_local_frame) in decode order."""
    df = pd.read_parquet(sidecar, columns=["frame_index", "segment_local_frame", "ros_stamp_ns"])
    df = df.sort_values("frame_index").reset_index(drop=True)
    return (df["ros_stamp_ns"].to_numpy(dtype=np.int64),
            df["segment_local_frame"].to_numpy(dtype=np.int64))


def depth_frame_range(stamps_ns: np.ndarray, t0_ns: int, t_end_ns: int) -> Tuple[int, int]:
    """[first, last] row (inclusive) covering a LeRobot episode [t0, t_end].

    Floor selection on both ends — the same at-or-before rule secondary
    colour cameras use — so depth frame ``first`` is what was current at
    LeRobot timestamp 0. Clamped to the recording (head before the first
    depth frame → 0)."""
    n = len(stamps_ns)
    if n == 0:
        return 0, -1
    first = max(int(np.searchsorted(stamps_ns, t0_ns, side="right")) - 1, 0)
    last = max(int(np.searchsorted(stamps_ns, t_end_ns, side="right")) - 1, first)
    return first, min(last, n - 1)


def frame_stamps_rel(episode_index: int) -> str:
    return f"depth/frame_stamps/episode_{int(episode_index):06d}.json"


def plan_depth_entry(src: dict, episode_index: int, source_id: str,
                     t0_ns: int, t_end_ns: int,
                     frame_stamps_ns: Optional[Sequence[int]] = None) -> dict:
    """Per-episode depth index entry (+ private ``_copy`` / ``_write`` lists for the copy step).

    frame_stamps_ns: source stamp (ns) of every emitted LeRobot frame, in
    frame_index order. Stale ticks are skipped, so the LeRobot clock
    (frame_index / fps) is not the source clock minus t0; load_depth_frames
    maps depth stamps through this list so depth lines up with RGB/actions."""
    from . import depth10

    stamps, local = _read_depth_sidecar(src["sidecar"])
    first, last = depth_frame_range(stamps, t0_ns, t_end_ns)
    dmeta = json.loads(src["meta_file"].read_text())
    rel_dir = f"depth/{src['key']}/{source_id}"
    actual_pf = depth10.probe_pix_fmt(src["video"])
    bit_ok = depth10.is_10bit_pix_fmt(actual_pf) if actual_pf else None
    if actual_pf and not bit_ok:
        print(
            f"  [WARN] depth {src['fv_name']} ({src['video']}): stream pix_fmt is "
            f"{actual_pf} (8-bit) but depth_meta says {dmeta.get('pix_fmt')} — the "
            f"10-bit inverse-depth codes were truncated to ~255 levels by the "
            f"encoder; decoded depth is coarse. Exported as-is, flagged bit_depth_ok=false.",
            file=sys.stderr,
        )
    entry = {
        "episode_index": int(episode_index),
        "source_episode_id": source_id,
        "file": f"{rel_dir}/{src['video'].name}",
        "meta_file": f"{rel_dir}/{src['meta_file'].name}",
        "frames_file": f"{rel_dir}/frames.parquet",
        # decode-order frame numbers (= segment_local_frame) inclusive
        "frame_from": int(local[first]) if last >= first else 0,
        "frame_to": int(local[last]) if last >= first else -1,
        "n_frames": int(last - first + 1) if last >= first else 0,
        "t0_stamp_ns": int(t0_ns),
        "from_stamp_ns": int(stamps[first]) if last >= first else None,
        "to_stamp_ns": int(stamps[last]) if last >= first else None,
        # LeRobot timestamp of depth frame_from (negative = slightly before t0)
        "first_frame_timestamp_s": (round((int(stamps[first]) - t0_ns) / 1e9, 6)
                                    if last >= first else None),
        "d_min_counts": dmeta.get("d_min_counts"),
        "d_max_counts": dmeta.get("d_max_counts"),
        "depth_scale_mm_per_count": dmeta.get("depth_scale_mm_per_count"),
        "actual_pix_fmt": actual_pf,
        "bit_depth_ok": bit_ok,
        "_key": src["key"],
        "_fv_name": src["fv_name"],
        "_meta": dmeta,
        "_write": ([(frame_stamps_rel(episode_index),
                     json.dumps([int(x) for x in frame_stamps_ns]).encode("utf-8"))]
                   if frame_stamps_ns else []),
        "_copy": [
            (src["video"], f"{rel_dir}/{src['video'].name}"),
            (src["meta_file"], f"{rel_dir}/{src['meta_file'].name}"),
            (src["sidecar"], f"{rel_dir}/frames.parquet"),
        ],
    }
    if frame_stamps_ns:
        entry["frame_stamps_file"] = frame_stamps_rel(episode_index)
    return entry


def copy_depth_files(out_root: Path, entries: Sequence[dict]) -> int:
    """Copy each source depth file once (byte-identical to ``ffmpeg -c copy``
    of the whole file, and needs no ffmpeg). Returns bytes copied."""
    done: set = set()
    total = 0
    for e in entries:
        for src, rel in e.get("_copy", []):
            if rel in done:
                continue
            dst = Path(out_root) / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
            total += dst.stat().st_size
            done.add(rel)
        for rel, data in e.get("_write", []):
            if rel in done:
                continue
            dst = Path(out_root) / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            dst.write_bytes(data)
            total += len(data)
            done.add(rel)
    return total


def build_fv_depth_block(entries: Sequence[dict]) -> dict:
    """``meta/info.json["fv_depth"]`` from planned depth entries."""
    keys: Dict[str, dict] = {}
    for e in sorted(entries, key=lambda x: (x["_key"], x["episode_index"])):
        key = e["_key"]
        m = e["_meta"]
        if key not in keys:
            keys[key] = {
                "source_camera": e["_fv_name"],
                "path_template": f"depth/{key}/{{source_episode_id}}/0000.mp4",
                "meta_template": f"depth/{key}/{{source_episode_id}}/depth_meta.json",
                "frames_template": f"depth/{key}/{{source_episode_id}}/frames.parquet",
                "format": m.get("format"),
                "codec": "hevc",
                "encoder": m.get("codec"),
                "pix_fmt": m.get("pix_fmt", "yuv420p10le"),
                "quantisation": "inverse_depth_10bit",
                "d_min_counts": m.get("d_min_counts"),
                "d_max_counts": m.get("d_max_counts"),
                "depth_scale_mm_per_count": m.get("depth_scale_mm_per_count"),
                "luma_levels": m.get("luma_levels"),
                "invalid_luma": m.get("invalid_luma"),
                "far_handling": m.get("far_handling"),
                "width": m.get("width"),
                "height": m.get("height"),
                "fps": m.get("fps"),
                "decode": "fv_episode_recorder.depth10.decode_video",
                "loader": "fv_episode_recorder.lerobot_materializer.load_depth_frames",
                "actual_pix_fmt": e.get("actual_pix_fmt"),
                "bit_depth_ok": e.get("bit_depth_ok"),
                "episodes": {},
            }
        k = keys[key]
        # ranges may differ between source episodes (profile change): the
        # per-episode entry + its depth_meta.json are authoritative.
        for fld in ("d_min_counts", "d_max_counts", "depth_scale_mm_per_count"):
            if k[fld] != m.get(fld):
                k[fld] = "per_episode"
        if e.get("actual_pix_fmt") and k.get("actual_pix_fmt") != e.get("actual_pix_fmt"):
            k["actual_pix_fmt"] = "mixed"
        if e.get("bit_depth_ok") is False:
            k["bit_depth_ok"] = False
        elif e.get("bit_depth_ok") is True and k["bit_depth_ok"] is None:
            k["bit_depth_ok"] = True
        k["episodes"][str(e["episode_index"])] = {
            kk: vv for kk, vv in e.items() if not kk.startswith("_")
        }
    return {"version": 1, "keys": keys}


def patch_info_json(out_root: Path, fv_depth: dict) -> Path:
    """Add the top-level ``fv_depth`` block to meta/info.json (after finalize —
    LeRobot rewrites info.json on save_episode/finalize). Unknown top-level
    keys are ignored by LeRobot's loader."""
    path = Path(out_root) / "meta" / "info.json"
    info = json.loads(path.read_text()) if path.exists() else {}
    info["fv_depth"] = fv_depth
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(info, indent=4, ensure_ascii=False))
    return path


def load_depth_frames(
    dataset_root: Path, episode_index: int, key: str, unit: str = "mm"
) -> Iterator[Tuple[float, np.ndarray]]:
    """Yield ``(timestamp_s, depth)`` for one LeRobot episode's depth key.

    ``timestamp_s`` is on the LeRobot episode clock (0 = the episode's first
    frame; may be slightly negative for the first depth frame, which is the
    one current at t=0). ``unit``: "mm" → uint16 millimetres (rounded; 0 =
    invalid), "counts" → uint16 raw sensor counts (D405: 0.1 mm/count, keeps
    sub-mm resolution), "mm_f32" → float32 millimetres. Needs ffmpeg.
    """
    from . import depth10

    root = Path(dataset_root)
    info = json.loads((root / "meta" / "info.json").read_text())
    try:
        ent = info["fv_depth"]["keys"][key]["episodes"][str(int(episode_index))]
    except KeyError:
        raise KeyError(f"no depth '{key}' for episode {episode_index} in {root}") from None
    dmeta = json.loads((root / ent["meta_file"]).read_text())
    scale = float(dmeta.get("depth_scale_mm_per_count") or 1.0)
    stamps, local = _read_depth_sidecar(root / ent["frames_file"])
    stamp_by_local = dict(zip(local.tolist(), stamps.tolist()))
    t0 = int(ent["t0_stamp_ns"])
    lo, hi = int(ent["frame_from"]), int(ent["frame_to"])
    # LeRobot clock = frame_index / fps. With the per-frame source stamps the
    # depth stamp is placed on that clock (gaps from skipped stale ticks are
    # compressed the same way as RGB/actions); older exports: source - t0.
    pair = None
    if ent.get("frame_stamps_file") and (root / ent["frame_stamps_file"]).exists():
        pair = np.asarray(json.loads((root / ent["frame_stamps_file"]).read_text()), dtype=np.float64)
        fps = float(info.get("fps") or 0) or None
        if pair.size < 2 or fps is None:
            pair = None
    for i, counts in enumerate(depth10.decode_video(root / ent["file"], dmeta)):
        if i < lo:
            continue
        if i > hi:
            break
        st = int(stamp_by_local.get(i, t0))
        if pair is not None:
            idx = np.arange(pair.size, dtype=np.float64)
            if pair[0] <= st <= pair[-1]:
                ts = float(np.interp(st, pair, idx)) / fps
            elif st < pair[0]:
                ts = (st - pair[0]) / 1e9
            else:
                ts = (pair.size - 1) / fps + (st - pair[-1]) / 1e9
        else:
            ts = (st - t0) / 1e9
        if unit == "counts":
            yield ts, counts
        elif unit == "mm_f32":
            yield ts, counts.astype(np.float32) * np.float32(scale)
        elif unit == "mm":
            if scale == 1.0:
                yield ts, counts
            else:
                yield ts, np.rint(counts.astype(np.float32) * scale).astype(np.uint16)
        else:
            raise ValueError(f"unit must be mm|counts|mm_f32, got {unit!r}")


# ===========================================================================
# Materialization
# ===========================================================================
def build_features(cameras: List[CameraSource], use_videos: bool) -> dict:
    features: dict = {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(STATE_ACTION_NAMES),),
            "names": list(STATE_ACTION_NAMES),
        },
        "action": {
            "dtype": "float32",
            "shape": (len(STATE_ACTION_NAMES),),
            "names": list(STATE_ACTION_NAMES),
        },
    }
    for cam in cameras:
        features[f"observation.images.{cam.lerobot_key}"] = {
            "dtype": "video" if use_videos else "image",
            "shape": (cam.height, cam.width, 3),
            "names": ["height", "width", "channels"],
        }
    return features


def parse_camera_map(spec: Optional[str]) -> Dict[str, str]:
    mapping = dict(DEFAULT_CAMERA_MAP)
    if spec:
        for pair in spec.split(","):
            pair = pair.strip()
            if not pair:
                continue
            if "=" not in pair:
                raise ValueError(f"invalid --camera-map entry '{pair}', expected fv=lerobot")
            fv, lr = pair.split("=", 1)
            mapping[fv.strip()] = lr.strip()
    return mapping


def materialize(
    episode_dirs: List[Path],
    out_root: Path,
    repo_id: str,
    fps: int,
    camera_map: Dict[str, str],
    state_topic: str,
    action_topic: str,
    task_override: Optional[str],
    image_writer_threads: int = 0,
    video_file_size_mb: Optional[int] = None,
    *,
    windows: Optional[Sequence[dict]] = None,
    include_depth: bool = False,
    outcomes: Optional[Sequence[str]] = None,
    tags: Optional[Sequence[str]] = None,
) -> dict:
    """image_writer_threads / video_file_size_mb は変換時間のチューニング
    (既定は従来どおり = 同期書き出し・動画 1 ファイル):

    - image_writer_threads > 0: add_frame のフレーム画像書き出しを別スレッドへ
      逃がし、tick 整列ループと重ねる (save_episode が合流を待つので結果は同じ)
    - video_file_size_mb: 出力動画をこのサイズで新ファイルへ切り替える。
      LeRobot は「新エピソードの動画を直前のファイルへ連結」するたびに直前
      ファイルの全フレームを検証デコードするため、1 ファイルに詰め続けると
      変換時間がエピソード数の 2 乗で伸びる (45 エピソードで変換時間の約 4 割)。
      v3.0 形式は複数ファイルが正規の形 (meta/episodes が chunk/file index と
      from/to_timestamp を持つ)

    追加オプション (いずれも既定は従来動作):

    - windows: normalize_windows() 形式の時間窓リスト。窓 1 つ = LeRobot
      エピソード 1 つ (収穫の試み単位など)。None なら従来どおり 1 ディレクトリ
      = 1 エピソードで、episode_dirs を使う (windows 指定時 episode_dirs は無視)
    - include_depth: depth_hevc10 カメラを再エンコードせずに depth/ へコピーし
      meta/info.json の fv_depth に登録する (load_depth_frames で読める)
    - outcomes / tags: 入力エピソードの絞り込み (outcome は any-of、tag は all-of)

    meta/fv_episodes.jsonl (episode_index ごとの由来・窓・extra・calibration)
    は常に書く。
    """
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    units = plan_units(episode_dirs, windows=windows, outcomes=outcomes, tags=tags)
    if not units:
        raise RuntimeError("no episodes to materialize (all filtered out?)")

    # --- 事前検証: meta.json のみ・動画はデコードしない ---
    # かつては全エピソードをここで一括ロード (全フレームデコード) してから
    # カメラ集合を検証していたが、それだとピーク RSS がエピソード数に比例して
    # 伸びる (実測: 60EP/~40k フレームで 65GB → OOM kill)。カメラ集合の検証は
    # meta.json だけで済むので、書き込み開始前にデコード無しで先に済ませる。
    canonical_keys: Optional[List[str]] = None
    seen_dirs: set = set()
    for unit in units:
        ep_dir = unit["episode_dir"]
        if ep_dir in seen_dirs:
            continue
        seen_dirs.add(ep_dir)
        selected, _ = _select_cameras(ep_dir, unit["meta"], camera_map)
        keys = [lerobot_key for _, lerobot_key, _, _ in selected]
        if not keys:
            raise RuntimeError(f"episode {ep_dir} has no usable (color) cameras")
        if canonical_keys is None:
            canonical_keys = keys  # 集合の基準は先頭エピソード (従来通り)
        elif keys != canonical_keys:
            raise RuntimeError(
                f"camera set mismatch: {ep_dir} has {keys}, expected {canonical_keys}"
            )
        if windows is not None:
            # 相対窓の started_at 欠落などは書き込み開始前に落とす
            for u in units:
                if u["episode_dir"] == ep_dir:
                    window_bounds_ns(u["window"], u["meta"])

    if out_root.exists() and any(out_root.iterdir()):
        raise RuntimeError(f"output root {out_root} already exists and is not empty")

    # --- 1 エピソードずつ: デコード → 書き込み → 即解放 ---
    # ピーク RSS を「1 エピソード分 + 一定バッファ」に保つための構造。
    # dataset の生成は先頭エピソードのロード後 (features に画像の実寸が必要)。
    # [load] / [episode] の行フォーマットは datagen / sim_eval の
    # parse_materializer_progress が読むので変えないこと。ストリーミング化で
    # 両者の出現順が交互になるが、あちらは行頭前綴りを数えるだけなので無害。
    # 窓モードでは同じソースの窓が連続する (plan_units がまとめる) ので、
    # ソース 1 本につきデコードは 1 回、解放は最後の窓の後。
    dataset = None
    total_frames = 0
    episode_count = 0
    total_skipped_stale = 0
    total_head_trim_s = 0.0
    total_tail_trim_s = 0.0
    per_camera_reuse: Dict[str, Dict[str, int]] = {}
    fv_records: List[dict] = []
    depth_entries: List[dict] = []
    skipped_windows: List[dict] = []
    ep: Optional[Episode] = None
    calibration: Optional[dict] = None
    depth_srcs: List[dict] = []
    for i_unit, unit in enumerate(units):
        ep_dir = unit["episode_dir"]
        window = unit["window"]
        if ep is None or ep.dir != ep_dir:
            print(f"[load] {ep_dir}")
            ep = Episode(ep_dir)
            ep.load(camera_map, state_topic, action_topic)
            for s in ep.skipped_cameras:
                print(f"  skip camera: {s}")
            print(
                f"  cameras: {[(c.fv_name, c.lerobot_key, c.width, c.height) for c in ep.cameras]}"
                f"  state_msgs={len(ep.state_series)} action_msgs={len(ep.action_series)}"
            )
            for cam in ep.cameras:
                acc = per_camera_reuse.setdefault(
                    cam.lerobot_key, {"used": 0, "reused": 0, "src_frames": 0})
                acc["src_frames"] += len(cam.stamps)
            calibration = collect_calibration(ep_dir, ep.meta, camera_map)
            depth_srcs = depth_sources(ep_dir, ep.meta, camera_map) if include_depth else []

        if dataset is None:
            features = build_features(ep.cameras, use_videos=True)
            print(f"[features] {json.dumps({k: {kk: (list(vv) if isinstance(vv, tuple) else vv) for kk, vv in v.items()} for k, v in features.items()})}")
            dataset = LeRobotDataset.create(
                repo_id=repo_id,
                fps=fps,
                features=features,
                root=str(out_root),
                robot_type="piper",
                use_videos=True,
                image_writer_threads=max(0, int(image_writer_threads)),
            )
            if video_file_size_mb is not None:
                dataset.meta.update_chunk_settings(
                    video_files_size_in_mb=int(video_file_size_mb))

        # 優先度: 窓の task > --task > meta.task_description
        task = ((window or {}).get("task") or task_override or ep.task or "recording")
        span_start, span_end = ep.aligned_span()
        all_ticks = ep.timeline(fps)
        window_ns = window_bounds_ns(window, ep.meta) if window is not None else None
        ticks = select_ticks(all_ticks, span_start, span_end, window_ns)
        head_trim_s = (span_start - all_ticks[0]) / 1e9 if all_ticks else 0.0
        tail_trim_s = (all_ticks[-1] - span_end) / 1e9 if all_ticks else 0.0
        used_before = {c.lerobot_key: (c.used, c.reused) for c in ep.cameras}

        # Alignment mirrors session_recorder: the primary camera's frame stamp
        # is the pairing stamp for the whole tuple, and a tick with no primary
        # frame within the staleness bound is skipped instead of padded.
        stale_ns = int(max(0.2, 2.0 / fps) * 1e9)
        written = 0
        skipped_stale = 0
        first_pair_ns: Optional[int] = None
        last_pair_ns: Optional[int] = None
        pair_stamps: List[int] = []
        skipped_window = 0
        w_lo = window_ns[0] if window_ns is not None else None
        w_hi = window_ns[1] if window_ns is not None else None
        for tick_ns in ticks:
            p_idx = _nearest_index(ep.primary, tick_ns)
            pair_ns = int(ep.primary.stamps[p_idx])
            if abs(pair_ns - tick_ns) > stale_ns:
                skipped_stale += 1
                continue
            # the tick is inside the window but the nearest primary frame may
            # not be (it could belong to the neighbouring harvest attempt)
            if (w_lo is not None and pair_ns < int(w_lo)) or (w_hi is not None and pair_ns > int(w_hi)):
                skipped_window += 1
                continue
            state = _sample_zoh(ep.state_series, pair_ns).astype(np.float32)
            action = _sample_zoh(ep.action_series, pair_ns).astype(np.float32)
            frame: dict = {
                "observation.state": state,
                "action": action,
                "task": task,
            }
            for cam in ep.cameras:
                if cam is ep.primary:
                    idx, slack_ns = p_idx, int(0.5 / fps * 1e9)
                else:
                    idx, slack_ns = _floor_index(cam, pair_ns), int(1.0 / fps * 1e9)
                img = _pick_image(cam, idx, pair_ns, slack_ns)
                frame[f"observation.images.{cam.lerobot_key}"] = np.ascontiguousarray(img)
            dataset.add_frame(frame)
            written += 1
            if first_pair_ns is None:
                first_pair_ns = pair_ns
            last_pair_ns = pair_ns
            pair_stamps.append(pair_ns)
        is_last_of_source = (i_unit + 1 == len(units)
                             or units[i_unit + 1]["episode_dir"] != ep_dir)
        if written == 0:
            if window is None:
                raise RuntimeError(
                    f"episode {ep.dir}: no frames survived alignment "
                    f"(ticks={len(ticks)}, stale_skipped={skipped_stale})"
                )
            # 窓が録画/関節の重なり区間の外 (または短すぎ) — その窓だけ捨てる。
            # エピソード番号は詰めたまま (save_episode していない)。
            print(f"  [warn] window #{window.get('index')} on {ep.dir.name}: no frames "
                  f"in span (ticks={len(ticks)}, stale_skipped={skipped_stale}) — skipped",
                  file=sys.stderr)
            skipped_windows.append({"index": window.get("index"),
                                    "episode_dir": str(ep_dir),
                                    "extra": window.get("extra")})
            if is_last_of_source:
                ep.release()
            continue
        dataset.save_episode()
        episode_index = episode_count
        total_frames += written
        total_skipped_stale += skipped_stale
        total_head_trim_s += max(head_trim_s, 0.0)
        total_tail_trim_s += max(tail_trim_s, 0.0)
        episode_count += 1
        for cam in ep.cameras:
            acc = per_camera_reuse[cam.lerobot_key]
            u0, r0 = used_before.get(cam.lerobot_key, (0, 0))
            acc["used"] += cam.used - u0
            acc["reused"] += cam.reused - r0

        source_id = str(ep.meta.get("episode_id") or ep_dir.name)
        ep_depth_keys: List[str] = []
        for src in depth_srcs:
            entry = plan_depth_entry(src, episode_index, source_id,
                                     int(first_pair_ns), int(last_pair_ns),
                                     frame_stamps_ns=pair_stamps)
            depth_entries.append(entry)
            ep_depth_keys.append(src["key"])
        fv_records.append(build_fv_episode_record(
            episode_index, ep_dir, ep.meta, window,
            task=task, n_frames=written, fps=fps,
            t0_stamp_ns=int(first_pair_ns), t_end_stamp_ns=int(last_pair_ns),
            calibration=calibration, depth_keys=ep_depth_keys,
        ))
        win_label = f" window=#{window.get('index')}" if window is not None else ""
        print(
            f"[episode] {ep.dir.name}: {written} frames "
            f"(trim head {head_trim_s:.2f}s / tail {tail_trim_s:.2f}s, "
            f"stale_skipped={skipped_stale}, window_skipped={skipped_window}), task='{task}'{win_label}"
        )
        # save_episode 済みの画像はもう使わない。次のデコードが始まる前に解放
        if is_last_of_source:
            ep.release()

    if dataset is None:
        # CLI 経由は argparse nargs="+" で 1 件以上が保証されるが、
        # materialize() をライブラリとして空リストで呼ぶ経路を塞ぐ
        # (assert は python -O で消える)。
        raise RuntimeError("no episodes to materialize")
    if episode_count == 0:
        raise RuntimeError("no episodes to materialize (every window was empty)")
    dataset.finalize()

    # --- finalize 後: LeRobot が info.json を書き終えてから sidecar を足す ---
    fv_episodes_path = write_fv_episodes(out_root, fv_records)
    depth_summary: Dict[str, Any] = {}
    if include_depth and depth_entries:
        copied = copy_depth_files(out_root, depth_entries)
        block = build_fv_depth_block(depth_entries)
        patch_info_json(out_root, block)
        depth_summary = {
            "keys": sorted(block["keys"]),
            "entries": len(depth_entries),
            "bytes_copied": copied,
            "bit_depth_ok": {k: v.get("bit_depth_ok") for k, v in block["keys"].items()},
        }
    elif include_depth:
        print("  [warn] --include-depth: no depth_hevc10 recordings found", file=sys.stderr)

    return {
        "episodes": episode_count,
        "total_frames": total_frames,
        "skipped_stale": total_skipped_stale,
        "head_trim_s": round(total_head_trim_s, 3),
        "tail_trim_s": round(total_tail_trim_s, 3),
        "per_camera": per_camera_reuse,
        "root": str(out_root),
        "skipped_windows": skipped_windows,
        "fv_episodes": str(fv_episodes_path),
        "depth": depth_summary,
    }


def _dir_size_bytes(path: Path) -> int:
    return sum(f.stat().st_size for f in path.rglob("*") if f.is_file())


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python3 -m fv_episode_recorder.lerobot_materializer",
        description="Materialize fv_episode_recorder episodes into a LeRobot v3.0 dataset.",
    )
    parser.add_argument("episode_dirs", nargs="*", type=Path,
                        help="episode directories (omit when --windows is given)")
    parser.add_argument("--out", required=True, type=Path, help="output dataset root")
    parser.add_argument("--repo-id", required=True, help="lerobot repo id, e.g. user/name")
    parser.add_argument("--fps", type=int, default=30, help="dataset fps (default 30)")
    parser.add_argument("--camera-map", default=None, help="fv=lerobot,... camera name remap")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC)
    parser.add_argument("--action-topic", default=DEFAULT_ACTION_TOPIC)
    parser.add_argument("--task", default=None, help="override task string for all episodes")
    parser.add_argument(
        "--image-writer-threads", type=int, default=0,
        help="threads for asynchronous frame-image writes (default 0 = synchronous)")
    parser.add_argument(
        "--video-file-size-mb", type=int, default=None,
        help="rotate output video files at this size; default keeps the LeRobot "
             "default (one growing file, conversion time grows quadratically)")
    parser.add_argument(
        "--windows", type=Path, default=None,
        help="windows.json: list of {episode_dir, start_s/end_s | start_unix/end_unix, "
             "task?, extra?}; each window becomes one LeRobot episode")
    parser.add_argument(
        "--include-depth", action="store_true",
        help="copy depth_hevc10 recordings (no re-encode) into depth/ and register "
             "them in meta/info.json fv_depth")
    parser.add_argument(
        "--outcome", action="append", choices=OUTCOMES, default=None,
        help="only include episode dirs with this outcome (repeatable, any-of)")
    parser.add_argument(
        "--tag", action="append", default=None,
        help="only include episode dirs carrying this tag (repeatable, all-of)")
    args = parser.parse_args(argv)

    if args.windows is None and not args.episode_dirs:
        parser.error("give episode directories or --windows")
    if args.windows is not None and args.episode_dirs:
        parser.error("--windows names its episode dirs itself; drop the positional dirs")
    windows = load_windows(args.windows) if args.windows is not None else None

    camera_map = parse_camera_map(args.camera_map)
    out_root = args.out.resolve()

    summary = materialize(
        episode_dirs=[d.resolve() for d in args.episode_dirs],
        out_root=out_root,
        repo_id=args.repo_id,
        fps=args.fps,
        camera_map=camera_map,
        state_topic=args.state_topic,
        action_topic=args.action_topic,
        task_override=args.task,
        image_writer_threads=args.image_writer_threads,
        video_file_size_mb=args.video_file_size_mb,
        windows=windows,
        include_depth=args.include_depth,
        outcomes=args.outcome,
        tags=args.tag,
    )

    info_path = out_root / "meta" / "info.json"
    size_bytes = _dir_size_bytes(out_root)
    print("\n==================== SUMMARY ====================")
    print(f"episodes         : {summary['episodes']}")
    print(f"total frames     : {summary['total_frames']}")
    print(f"dataset fps      : {args.fps}")
    print(
        f"alignment        : stale_skipped={summary['skipped_stale']} "
        f"head_trim={summary['head_trim_s']}s tail_trim={summary['tail_trim_s']}s"
    )
    print("per-camera stats :")
    for key, acc in summary["per_camera"].items():
        print(
            f"  {key}: used={acc['used']} reused={acc['reused']} "
            f"source_frames={acc['src_frames']}"
        )
    print(f"output root      : {summary['root']}")
    print(f"output size      : {size_bytes / 1e6:.2f} MB")
    print(f"info.json        : {info_path}  (exists={info_path.exists()})")
    print(f"fv_episodes      : {summary['fv_episodes']}")
    if summary["skipped_windows"]:
        print(f"skipped windows  : {[w['index'] for w in summary['skipped_windows']]}")
    if summary["depth"]:
        print(f"depth            : {summary['depth']}")
    print("================================================")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
