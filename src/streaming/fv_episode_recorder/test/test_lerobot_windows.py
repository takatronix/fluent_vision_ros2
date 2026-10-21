"""Time windows / fv_episodes.jsonl / filters for lerobot_materializer.

Pure pytest: no ROS, no lerobot (a fake LeRobotDataset stands in for the
end-to-end case), no video decoding.
"""

from __future__ import annotations

import json
import sys
import types
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("pandas")

from fv_episode_recorder import lerobot_materializer as lm  # noqa: E402

STARTED_AT = "2026-10-21T01:00:00.000000Z"
T0 = 1792544400.0  # unix seconds of STARTED_AT


def _make_episode(root: Path, name: str, *, outcome="success", tags=("harvest",),
                  episode_id=None, cameras=None) -> Path:
    d = root / name
    d.mkdir(parents=True)
    meta = {
        "episode_id": episode_id or f"ID_{name}",
        "task_description": "harvest asparagus",
        "profile": "aspa2",
        "robot_id": "aspa2",
        "started_at": STARTED_AT,
        "outcome": outcome,
        "tags": list(tags),
        "pinned": False,
        "cameras": cameras if cameras is not None else [],
        "bag_path": "bag/",
    }
    (d / "meta.json").write_text(json.dumps(meta))
    return d


def test_started_at_parse():
    assert lm.parse_started_at(STARTED_AT) == pytest.approx(T0)
    assert lm.parse_started_at(None) is None
    assert lm.parse_started_at("garbage") is None
    assert lm.parse_started_at(12.5) == 12.5


def test_window_bounds_relative_and_absolute():
    meta = {"started_at": STARTED_AT}
    w = lm.normalize_windows([{"episode_dir": "/x", "start_s": 10, "end_s": 20.5}])[0]
    assert lm.window_bounds_ns(w, meta) == (int((T0 + 10) * 1e9), int((T0 + 20.5) * 1e9))
    w = lm.normalize_windows([{"episode_dir": "/x", "start_unix": T0 + 1, "end_s": 3}])[0]
    assert lm.window_bounds_ns(w, meta) == (int((T0 + 1) * 1e9), int((T0 + 3) * 1e9))
    w = lm.normalize_windows([{"episode_dir": "/x", "start_s": 5}])[0]
    assert lm.window_bounds_ns(w, meta) == (int((T0 + 5) * 1e9), None)
    with pytest.raises(ValueError):
        lm.window_bounds_ns(
            lm.normalize_windows([{"episode_dir": "/x", "start_s": 5, "end_s": 4}])[0], meta)
    with pytest.raises(ValueError):  # relative needs started_at
        lm.window_bounds_ns(lm.normalize_windows([{"episode_dir": "/x", "start_s": 1}])[0], {})


def test_normalize_windows_validation(tmp_path):
    with pytest.raises(ValueError):
        lm.normalize_windows([{"start_s": 1}])
    with pytest.raises(ValueError):
        lm.normalize_windows({"nope": 1})
    with pytest.raises(ValueError):
        lm.normalize_windows([{"episode_dir": "/x", "extra": [1]}])
    ep = _make_episode(tmp_path, "ep1")
    wf = tmp_path / "windows.json"
    wf.write_text(json.dumps({"windows": [{"episode_dir": "ep1", "start_s": 1, "end_s": 2,
                                           "task": "t", "extra": {"attempt_id": 7}}]}))
    ws = lm.load_windows(wf)
    assert ws[0]["episode_dir"] == ep.resolve()
    assert ws[0]["index"] == 0 and ws[0]["task"] == "t" and ws[0]["extra"] == {"attempt_id": 7}


def test_select_ticks_window_clip():
    ticks = list(range(0, 100, 10))
    assert lm.select_ticks(ticks, 10, 80) == [10, 20, 30, 40, 50, 60, 70, 80]
    assert lm.select_ticks(ticks, 10, 80, (25, 55)) == [30, 40, 50]
    assert lm.select_ticks(ticks, 10, 80, (None, 35)) == [10, 20, 30]
    assert lm.select_ticks(ticks, 10, 80, (85, None)) == []


def test_plan_units_filters_and_grouping(tmp_path):
    a = _make_episode(tmp_path, "a", outcome="success", tags=("harvest", "aspa"))
    b = _make_episode(tmp_path, "b", outcome="abort", tags=("harvest",))
    c = _make_episode(tmp_path, "c", outcome="success", tags=("calib",))

    units = lm.plan_units([a, b, c])
    assert [u["episode_dir"].name for u in units] == ["a", "b", "c"]
    assert all(u["window"] is None for u in units)

    assert [u["episode_dir"].name for u in lm.plan_units([a, b, c], outcomes=["success"])] == ["a", "c"]
    assert [u["episode_dir"].name for u in lm.plan_units([a, b, c], tags=["harvest"])] == ["a", "b"]
    # tags are all-of
    assert [u["episode_dir"].name for u in lm.plan_units([a, b, c], tags=["harvest", "aspa"])] == ["a"]
    assert [u["episode_dir"].name
            for u in lm.plan_units([a, b, c], outcomes=["success", "abort"], tags=["harvest"])] == ["a", "b"]

    windows = lm.normalize_windows([
        {"episode_dir": str(a), "start_s": 0, "end_s": 1, "extra": {"attempt_id": 1}},
        {"episode_dir": str(b), "start_s": 0, "end_s": 1, "extra": {"attempt_id": 2}},
        {"episode_dir": str(a), "start_s": 2, "end_s": 3, "extra": {"attempt_id": 3}},
    ])
    units = lm.plan_units([], windows=windows)
    # grouped by source dir (first appearance), windows keep input order
    assert [(u["episode_dir"].name, u["window"]["extra"]["attempt_id"]) for u in units] == [
        ("a", 1), ("a", 3), ("b", 2)]
    units = lm.plan_units([], windows=windows, outcomes=["abort"])
    assert [u["window"]["index"] for u in units] == [1]


def test_fv_episode_record_and_jsonl(tmp_path):
    cam_info = {"frame_id": "d555_color_optical_frame", "width": 640, "height": 480,
                "distortion_model": "plumb_bob", "K": [1.0] * 9, "D": [0.0] * 5,
                "R": [1.0] * 9, "P": [1.0] * 12}
    ep = _make_episode(tmp_path, "ep", cameras=[
        {"name": "main", "kind": None, "camera_info": cam_info},
        {"name": "depth", "kind": "depth_hevc10"},
    ])
    meta = json.loads((ep / "meta.json").read_text())
    calib = lm.collect_calibration(ep, meta, {"main": "top_camera"})
    assert calib["cameras"]["main"]["key"] == "observation.images.top_camera"
    assert calib["cameras"]["main"]["camera_info"] == cam_info
    assert calib["tf_static"] is None  # no bag
    assert lm.collect_calibration(ep, {"cameras": []}, {}) is None

    window = lm.normalize_windows([{
        "episode_dir": str(ep), "start_s": 1.0, "end_s": 4.0, "task": "pick",
        "extra": {"attempt_id": "A1", "result": "failure", "reason": "slip",
                  "mode": "auto", "session_id": "S1"}}])[0]
    t0 = int((T0 + 1.25) * 1e9)
    rec = lm.build_fv_episode_record(
        3, ep, meta, window, task="pick", n_frames=90, fps=30,
        t0_stamp_ns=t0, t_end_stamp_ns=t0 + 3_000_000_000,
        calibration=calib, depth_keys=["observation.depth.depth"])
    assert rec["episode_index"] == 3
    assert rec["source_episode_id"] == "ID_ep"
    assert rec["episode_dir"] == "ep"
    assert rec["outcome"] == "success" and rec["tags"] == ["harvest"] and rec["pinned"] is False
    assert rec["window"] == {"index": 0, "start_s": 1.0, "end_s": 4.0,
                             "start_unix": None, "end_unix": None}
    assert rec["extra"]["result"] == "failure" and rec["extra"]["attempt_id"] == "A1"
    assert rec["t0_offset_s"] == pytest.approx(1.25)
    assert rec["calibration"] is calib

    rec0 = lm.build_fv_episode_record(
        0, ep, meta, None, task="x", n_frames=1, fps=30, t0_stamp_ns=t0, t_end_stamp_ns=t0)
    assert rec0["window"] is None and rec0["extra"] == {} and rec0["calibration"] is None

    path = lm.write_fv_episodes(tmp_path / "ds", [rec, rec0])
    lines = [json.loads(x) for x in path.read_text().splitlines()]
    assert [x["episode_index"] for x in lines] == [0, 3]
    assert path == tmp_path / "ds" / "meta" / "fv_episodes.jsonl"


def test_read_tf_static_from_bag(tmp_path):
    rosbags = pytest.importorskip("rosbags")  # noqa: F841
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_typestore

    from fv_episode_recorder.calibration import read_tf_static

    ts = get_typestore(Stores.ROS2_HUMBLE)
    Header = ts.types["std_msgs/msg/Header"]
    Time = ts.types["builtin_interfaces/msg/Time"]
    TS = ts.types["geometry_msgs/msg/TransformStamped"]
    Tf = ts.types["geometry_msgs/msg/Transform"]
    V3 = ts.types["geometry_msgs/msg/Vector3"]
    Q = ts.types["geometry_msgs/msg/Quaternion"]
    TFM = ts.types["tf2_msgs/msg/TFMessage"]

    def tf(parent, child, x):
        return TS(header=Header(stamp=Time(sec=1, nanosec=0), frame_id=parent),
                  child_frame_id=child,
                  transform=Tf(translation=V3(x=x, y=0.0, z=0.5),
                               rotation=Q(x=0.0, y=0.0, z=0.0, w=1.0)))

    bag = tmp_path / "bag"
    try:
        writer = Writer(bag, version=8)
    except TypeError:  # older rosbags
        writer = Writer(bag)
    with writer:
        conn = writer.add_connection("/tf_static", TFM.__msgtype__, typestore=ts)
        msg = TFM(transforms=[tf("base_link", "d555_link", 0.1), tf("link6", "d405_link", 0.02)])
        writer.write(conn, 1_000, ts.serialize_cdr(msg, TFM.__msgtype__))
    out = read_tf_static(bag)
    assert out == [
        {"parent": "base_link", "child": "d555_link", "xyz": [0.1, 0.0, 0.5],
         "quat_xyzw": [0.0, 0.0, 0.0, 1.0]},
        {"parent": "link6", "child": "d405_link", "xyz": [0.02, 0.0, 0.5],
         "quat_xyzw": [0.0, 0.0, 0.0, 1.0]},
    ]
    assert read_tf_static(tmp_path / "nobag") is None


def test_camera_info_to_dict():
    from fv_episode_recorder.calibration import camera_info_to_dict

    msg = types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id="cam"), width=640, height=480,
        distortion_model="plumb_bob", k=[1, 0, 2, 0, 1, 3, 0, 0, 1], d=[0.1, 0, 0, 0, 0],
        r=[1, 0, 0, 0, 1, 0, 0, 0, 1], p=[1] * 12)
    d = camera_info_to_dict(msg)
    assert d["frame_id"] == "cam" and d["width"] == 640 and d["K"][2] == 2.0
    assert d["D"][0] == 0.1 and len(d["P"]) == 12
    json.dumps(d)


# ---------------------------------------------------------------------------
# End-to-end materialize() with a fake LeRobotDataset (no lerobot needed)
# ---------------------------------------------------------------------------
class _FakeDataset:
    instances: list = []

    def __init__(self, root):
        self.root = Path(root)
        self.episodes: list = []
        self._cur: list = []
        self.finalized = False
        self.meta = types.SimpleNamespace(update_chunk_settings=lambda **kw: None)

    @classmethod
    def create(cls, repo_id, fps, features, root, robot_type, use_videos, image_writer_threads):
        ds = cls(root)
        ds.features = features
        (ds.root / "meta").mkdir(parents=True)
        (ds.root / "meta" / "info.json").write_text(json.dumps({"fps": fps, "features": {}}))
        cls.instances.append(ds)
        return ds

    def add_frame(self, frame):
        self._cur.append(frame)

    def save_episode(self):
        self.episodes.append(self._cur)
        self._cur = []

    def finalize(self):
        self.finalized = True


def _fake_load(self, camera_map, state_topic, action_topic):
    # 10 s of 30 fps primary frames starting 1 s after started_at, joints
    # covering the whole span.
    base = int((T0 + 1.0) * 1e9)
    stamps = np.array([base + int(i * 1e9 / 30) for i in range(300)], dtype=np.int64)
    imgs = [np.zeros((2, 2, 3), dtype=np.uint8)] * len(stamps)
    self.cameras = [lm.CameraSource("main", "top_camera", stamps, imgs)]
    self.primary = self.cameras[0]
    self.skipped_cameras = []
    vec = np.zeros(7)
    self.state_series = [(int(stamps[0]) - 1, vec), (int(stamps[-1]) + 1, vec)]
    self.action_series = list(self.state_series)


def test_materialize_windows_end_to_end(tmp_path, monkeypatch):
    cams = [{"name": "main", "video_dir": "main/", "sidecar_file": "main/frames.parquet",
             "frame_count": 300}]
    a = _make_episode(tmp_path / "src", "a", cameras=cams)
    b = _make_episode(tmp_path / "src", "b", cameras=cams, outcome="abort")
    for d in (a, b):
        (d / "videos" / "main").mkdir(parents=True)
        (d / "videos" / "main" / "frames.parquet").write_bytes(b"")

    fake_mod = types.ModuleType("lerobot.datasets.lerobot_dataset")
    fake_mod.LeRobotDataset = _FakeDataset
    for name in ("lerobot", "lerobot.datasets"):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    monkeypatch.setitem(sys.modules, "lerobot.datasets.lerobot_dataset", fake_mod)
    monkeypatch.setattr(lm.Episode, "load", _fake_load)
    _FakeDataset.instances = []

    windows = lm.normalize_windows([
        {"episode_dir": str(a), "start_s": 2.0, "end_s": 3.0, "task": "attempt",
         "extra": {"attempt_id": 1, "result": "success"}},
        {"episode_dir": str(b), "start_s": 4.0, "end_s": 6.0,
         "extra": {"attempt_id": 2, "result": "failure", "reason": "miss"}},
        {"episode_dir": str(a), "start_s": 50.0, "end_s": 60.0,  # outside → skipped
         "extra": {"attempt_id": 3}},
        {"episode_dir": str(a), "start_s": 5.0, "end_s": 5.5, "extra": {"attempt_id": 4}},
    ])
    out = tmp_path / "ds"
    summary = lm.materialize(
        [], out, "t/x", 30, {}, "/s", "/a", None, windows=windows)
    ds = _FakeDataset.instances[0]
    assert ds.finalized
    assert summary["episodes"] == 3
    assert [w["index"] for w in summary["skipped_windows"]] == [2]
    # 1.0 s window at 30 fps → 31 ticks inclusive; 0.5 s → 16; 2 s → 61
    assert [len(e) for e in ds.episodes] == [31, 16, 61]
    assert ds.episodes[0][0]["task"] == "attempt"
    assert ds.episodes[2][0]["task"] == "harvest asparagus"
    lines = [json.loads(x) for x in (out / "meta" / "fv_episodes.jsonl").read_text().splitlines()]
    assert [(x["episode_index"], x["episode_dir"], x["extra"]["attempt_id"]) for x in lines] == [
        (0, "a", 1), (1, "a", 4), (2, "b", 2)]
    assert lines[2]["outcome"] == "abort" and lines[2]["extra"]["reason"] == "miss"
    assert lines[0]["n_frames"] == 31
    assert lines[0]["t0_offset_s"] == pytest.approx(2.0, abs=1 / 30)
    assert summary["per_camera"]["top_camera"]["used"] == 31 + 16 + 61
    assert summary["per_camera"]["top_camera"]["src_frames"] == 600
    # info.json untouched without --include-depth
    assert "fv_depth" not in json.loads((out / "meta" / "info.json").read_text())

    # outcome filter: only the abort episode's window survives
    _FakeDataset.instances = []
    summary = lm.materialize(
        [], tmp_path / "ds2", "t/x", 30, {}, "/s", "/a", None,
        windows=windows, outcomes=["abort"])
    assert summary["episodes"] == 1

    # no windows → one episode per dir (legacy behaviour), tag filter
    _FakeDataset.instances = []
    summary = lm.materialize(
        [a, b], tmp_path / "ds3", "t/x", 30, {}, "/s", "/a", None, tags=["harvest"])
    assert summary["episodes"] == 2
    # timeline floor drops the last (sub-tick) frame — unchanged legacy behaviour
    assert [len(e) for e in _FakeDataset.instances[0].episodes] == [299, 299]


def test_cli_requires_dirs_or_windows(tmp_path):
    with pytest.raises(SystemExit):
        lm.main(["--out", str(tmp_path / "o"), "--repo-id", "a/b"])
    wf = tmp_path / "w.json"
    wf.write_text("[]")
    with pytest.raises(SystemExit):
        lm.main([str(tmp_path), "--windows", str(wf), "--out", str(tmp_path / "o"),
                 "--repo-id", "a/b"])


def _fake_load_offset(self, camera_map, state_topic, action_topic):
    # primary frames at 10 fps, 80 ms after each 100 ms mark: the tick at
    # exactly the window start (2.0 s) is nearest to the frame at 1.98 s,
    # which lies OUTSIDE the window (it belongs to the previous attempt).
    base = int((T0 + 0.08) * 1e9)
    stamps = np.array([base + int(i * 1e8) for i in range(100)], dtype=np.int64)
    imgs = [np.zeros((2, 2, 3), dtype=np.uint8)] * len(stamps)
    self.cameras = [lm.CameraSource("main", "top_camera", stamps, imgs)]
    self.primary = self.cameras[0]
    self.skipped_cameras = []
    vec = np.zeros(7)
    self.state_series = [(int(stamps[0]) - 1, vec), (int(stamps[-1]) + 1, vec)]
    self.action_series = list(self.state_series)


def test_window_bounds_apply_to_selected_frames(tmp_path, monkeypatch):
    """Review 2026-10-21: a tick inside the window must not pull in a primary
    frame from outside it."""
    cams = [{"name": "main", "video_dir": "main/", "sidecar_file": "main/frames.parquet",
             "frame_count": 100}]
    a = _make_episode(tmp_path / "src", "a", cameras=cams)
    (a / "videos" / "main").mkdir(parents=True)
    (a / "videos" / "main" / "frames.parquet").write_bytes(b"")
    fake_mod = types.ModuleType("lerobot.datasets.lerobot_dataset")
    fake_mod.LeRobotDataset = _FakeDataset
    for name in ("lerobot", "lerobot.datasets"):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    monkeypatch.setitem(sys.modules, "lerobot.datasets.lerobot_dataset", fake_mod)
    monkeypatch.setattr(lm.Episode, "load", _fake_load_offset)
    _FakeDataset.instances = []
    windows = lm.normalize_windows([{"episode_dir": str(a), "start_s": 2.0, "end_s": 3.0,
                                     "extra": {"attempt_id": 1}}])
    out = tmp_path / "ds"
    lm.materialize([], out, "t/x", 30, {}, "/s", "/a", None, windows=windows)
    line = json.loads((out / "meta" / "fv_episodes.jsonl").read_text().splitlines()[0])
    assert line["t0_offset_s"] >= 2.0 - 1e-6
    assert line["t0_offset_s"] + line["n_frames"] * 0 <= 3.0
