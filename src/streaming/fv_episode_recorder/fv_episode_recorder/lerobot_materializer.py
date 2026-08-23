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
        [--task <override>]

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
* Depth cameras are skipped entirely in v1.
* Episodes are materialized one at a time (decode -> write -> release) so the
  peak RSS stays at ~one episode regardless of the episode count.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Environment must be configured *before* importing lerobot so that video
# encoding picks H.264 / yuv420p (matches session_recorder.py:32-34).
# ---------------------------------------------------------------------------
os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("LEROBOT_VIDEO_VCODEC", "h264")
os.environ.setdefault("LEROBOT_VIDEO_PIX_FMT", "yuv420p")

import cv2  # noqa: E402
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

# Camera "kind" values that we skip in v1 (depth streams).
DEPTH_KINDS = {"depth_bag", "depth_hevc10", "depth_png_seq"}

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
) -> dict:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    # --- 事前検証: meta.json のみ・動画はデコードしない ---
    # かつては全エピソードをここで一括ロード (全フレームデコード) してから
    # カメラ集合を検証していたが、それだとピーク RSS がエピソード数に比例して
    # 伸びる (実測: 60EP/~40k フレームで 65GB → OOM kill)。カメラ集合の検証は
    # meta.json だけで済むので、書き込み開始前にデコード無しで先に済ませる。
    canonical_keys: Optional[List[str]] = None
    for ep_dir in episode_dirs:
        meta = json.loads((ep_dir / "meta.json").read_text())
        selected, _ = _select_cameras(ep_dir, meta, camera_map)
        keys = [lerobot_key for _, lerobot_key, _, _ in selected]
        if not keys:
            raise RuntimeError(f"episode {ep_dir} has no usable (color) cameras")
        if canonical_keys is None:
            canonical_keys = keys  # 集合の基準は先頭エピソード (従来通り)
        elif keys != canonical_keys:
            raise RuntimeError(
                f"camera set mismatch: {ep_dir} has {keys}, expected {canonical_keys}"
            )

    if out_root.exists() and any(out_root.iterdir()):
        raise RuntimeError(f"output root {out_root} already exists and is not empty")

    # --- 1 エピソードずつ: デコード → 書き込み → 即解放 ---
    # ピーク RSS を「1 エピソード分 + 一定バッファ」に保つための構造。
    # dataset の生成は先頭エピソードのロード後 (features に画像の実寸が必要)。
    # [load] / [episode] の行フォーマットは datagen / sim_eval の
    # parse_materializer_progress が読むので変えないこと。ストリーミング化で
    # 両者の出現順が交互になるが、あちらは行頭前綴りを数えるだけなので無害。
    dataset = None
    total_frames = 0
    episode_count = 0
    total_skipped_stale = 0
    total_head_trim_s = 0.0
    total_tail_trim_s = 0.0
    per_camera_reuse: Dict[str, Dict[str, int]] = {}
    for ep_dir in episode_dirs:
        print(f"[load] {ep_dir}")
        ep = Episode(ep_dir)
        ep.load(camera_map, state_topic, action_topic)
        for s in ep.skipped_cameras:
            print(f"  skip camera: {s}")
        print(
            f"  cameras: {[(c.fv_name, c.lerobot_key, c.width, c.height) for c in ep.cameras]}"
            f"  state_msgs={len(ep.state_series)} action_msgs={len(ep.action_series)}"
        )

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
            )

        task = task_override or ep.task or "recording"
        span_start, span_end = ep.aligned_span()
        all_ticks = ep.timeline(fps)
        ticks = [t for t in all_ticks if span_start <= t <= span_end]
        head_trim_s = (span_start - all_ticks[0]) / 1e9 if all_ticks else 0.0
        tail_trim_s = (all_ticks[-1] - span_end) / 1e9 if all_ticks else 0.0

        # Alignment mirrors session_recorder: the primary camera's frame stamp
        # is the pairing stamp for the whole tuple, and a tick with no primary
        # frame within the staleness bound is skipped instead of padded.
        stale_ns = int(max(0.2, 2.0 / fps) * 1e9)
        written = 0
        skipped_stale = 0
        for tick_ns in ticks:
            p_idx = _nearest_index(ep.primary, tick_ns)
            pair_ns = int(ep.primary.stamps[p_idx])
            if abs(pair_ns - tick_ns) > stale_ns:
                skipped_stale += 1
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
        if written == 0:
            raise RuntimeError(
                f"episode {ep.dir}: no frames survived alignment "
                f"(ticks={len(ticks)}, stale_skipped={skipped_stale})"
            )
        dataset.save_episode()
        total_frames += written
        total_skipped_stale += skipped_stale
        total_head_trim_s += max(head_trim_s, 0.0)
        total_tail_trim_s += max(tail_trim_s, 0.0)
        episode_count += 1
        for cam in ep.cameras:
            acc = per_camera_reuse.setdefault(cam.lerobot_key, {"used": 0, "reused": 0, "src_frames": 0})
            acc["used"] += cam.used
            acc["reused"] += cam.reused
            acc["src_frames"] += len(cam.stamps)
        print(
            f"[episode] {ep.dir.name}: {written} frames "
            f"(trim head {head_trim_s:.2f}s / tail {tail_trim_s:.2f}s, "
            f"stale_skipped={skipped_stale}), task='{task}'"
        )
        # save_episode 済みの画像はもう使わない。次のデコードが始まる前に解放
        ep.release()

    if dataset is None:
        # CLI 経由は argparse nargs="+" で 1 件以上が保証されるが、
        # materialize() をライブラリとして空リストで呼ぶ経路を塞ぐ
        # (assert は python -O で消える)。
        raise RuntimeError("no episodes to materialize")
    dataset.finalize()

    return {
        "episodes": episode_count,
        "total_frames": total_frames,
        "skipped_stale": total_skipped_stale,
        "head_trim_s": round(total_head_trim_s, 3),
        "tail_trim_s": round(total_tail_trim_s, 3),
        "per_camera": per_camera_reuse,
        "root": str(out_root),
    }


def _dir_size_bytes(path: Path) -> int:
    return sum(f.stat().st_size for f in path.rglob("*") if f.is_file())


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python3 -m fv_episode_recorder.lerobot_materializer",
        description="Materialize fv_episode_recorder episodes into a LeRobot v3.0 dataset.",
    )
    parser.add_argument("episode_dirs", nargs="+", type=Path, help="episode directories")
    parser.add_argument("--out", required=True, type=Path, help="output dataset root")
    parser.add_argument("--repo-id", required=True, help="lerobot repo id, e.g. user/name")
    parser.add_argument("--fps", type=int, default=30, help="dataset fps (default 30)")
    parser.add_argument("--camera-map", default=None, help="fv=lerobot,... camera name remap")
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC)
    parser.add_argument("--action-topic", default=DEFAULT_ACTION_TOPIC)
    parser.add_argument("--task", default=None, help="override task string for all episodes")
    args = parser.parse_args(argv)

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
    print("================================================")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
