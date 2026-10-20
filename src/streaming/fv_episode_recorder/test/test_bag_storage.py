"""rosbag2 storage selection (sqlite3 default / mcap + preset) and storage
detection for the rosbag2_py readers. Pure pytest, no ROS."""

from __future__ import annotations

from pathlib import Path

import pytest

from fv_episode_recorder.bag_recorder import (
    BagRecorder,
    build_record_command,
    detect_bag_storage,
)


def test_default_command_is_unchanged(tmp_path):
    cmd = build_record_command(tmp_path / "bag", ["/a", "/b"])
    # exactly the historical argv
    assert cmd == ["ros2", "bag", "record", "-o", str(tmp_path / "bag"), "-s", "sqlite3",
                   "-b", str(1024 * 1024 * 1024), "/a", "/b"]


def test_mcap_with_preset(tmp_path):
    cmd = build_record_command(tmp_path / "bag", ["/a"], storage="mcap",
                               max_bag_size_mb=512, storage_preset="zstd_fast")
    assert cmd == ["ros2", "bag", "record", "-o", str(tmp_path / "bag"), "-s", "mcap",
                   "--storage-preset-profile", "zstd_fast",
                   "-b", str(512 * 1024 * 1024), "/a"]
    assert "--storage-preset-profile" not in build_record_command(tmp_path, ["/a"], storage="MCAP")


def test_preset_ignored_for_sqlite3_and_bad_storage(tmp_path):
    cmd = build_record_command(tmp_path, ["/a"], storage="sqlite3", storage_preset="zstd_fast")
    assert "--storage-preset-profile" not in cmd and cmd[cmd.index("-s") + 1] == "sqlite3"
    with pytest.raises(ValueError):
        build_record_command(tmp_path, ["/a"], storage="rosbag1")


def test_recorder_start_uses_override(tmp_path, monkeypatch):
    seen = {}

    class _P:
        pid = 1

        def poll(self):
            return None

    def fake_popen(cmd, **kw):
        seen["cmd"] = cmd
        return _P()

    monkeypatch.setattr("fv_episode_recorder.bag_recorder.subprocess.Popen", fake_popen)
    rec = BagRecorder()
    rec.start(tmp_path / "bag", ["/t"], storage="mcap", storage_preset="zstd_fast")
    assert seen["cmd"][seen["cmd"].index("-s") + 1] == "mcap"
    rec2 = BagRecorder()
    rec2.start(tmp_path / "bag2", ["/t"])
    assert seen["cmd"][seen["cmd"].index("-s") + 1] == "sqlite3"


def _metadata(bag: Path, storage: str) -> None:
    bag.mkdir(parents=True, exist_ok=True)
    (bag / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n"
        "  version: 5\n"
        f"  storage_identifier: {storage}\n"
        "  duration:\n    nanoseconds: 1\n")


def test_detect_storage_from_metadata(tmp_path):
    pytest.importorskip("yaml")
    _metadata(tmp_path / "m", "mcap")
    assert detect_bag_storage(tmp_path / "m") == "mcap"
    _metadata(tmp_path / "s", "sqlite3")
    assert detect_bag_storage(tmp_path / "s") == "sqlite3"


def test_detect_storage_fallbacks(tmp_path):
    rec = tmp_path / "recording"
    rec.mkdir()
    (rec / "recording_0.mcap").write_bytes(b"")
    assert detect_bag_storage(rec) == "mcap"  # no metadata.yaml yet
    old = tmp_path / "old"
    old.mkdir()
    (old / "old_0.db3").write_bytes(b"")
    assert detect_bag_storage(old) == "sqlite3"
    assert detect_bag_storage(tmp_path / "missing") == "sqlite3"
    bad = tmp_path / "bad"
    bad.mkdir()
    (bad / "metadata.yaml").write_text(": : :\n\t-")
    (bad / "x.mcap").write_bytes(b"")
    assert detect_bag_storage(bad) == "mcap"


def test_profile_bag_options():
    pytest.importorskip("yaml")
    from fv_episode_recorder.topic_discovery import discover_bag_options, discover_cameras

    assert discover_bag_options({}) == {"storage": None, "storage_preset": None}
    prof = {"episode_recorder": {"bag_storage": "MCAP", "bag_storage_preset": "zstd_fast",
                                 "cameras": [{"name": "main", "topic": "/c",
                                              "camera_info_topic": "/c/info"}]}}
    assert discover_bag_options(prof) == {"storage": "mcap", "storage_preset": "zstd_fast"}
    assert discover_cameras(prof)[0]["camera_info_topic"] == "/c/info"
