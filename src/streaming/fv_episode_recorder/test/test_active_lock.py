"""ActiveLock pid-reuse regression tests (pure pytest, no ROS runtime needed)."""

from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import pytest

from fv_episode_recorder import active_lock
from fv_episode_recorder.active_lock import ActiveLock


def _write_lock(output_dir: Path, *, pid: int, uuid: str) -> None:
    payload = {
        "episode_id": "01TESTEPISODE",
        "episode_dir": str(output_dir / "episodes" / "01TESTEPISODE"),
        "started_at": 1_000_000.0,
        "recorder_pid": pid,
        "recorder_uuid": uuid,
    }
    (output_dir / ".active_episode.lock").write_text(json.dumps(payload))


def test_detect_orphan_treats_reused_pid_as_orphan(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Lock pid alive but owned by an unrelated process -> orphan, not another_alive."""
    _write_lock(tmp_path, pid=os.getpid(), uuid="other-instance")
    monkeypatch.setattr(active_lock, "_pid_is_recorder", lambda pid: False)

    lock = ActiveLock(tmp_path)
    result = lock.detect_orphan()

    assert result is not None
    assert result["status"] == "orphan"


def test_detect_orphan_still_refuses_when_real_recorder_alive(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_lock(tmp_path, pid=os.getpid(), uuid="other-instance")
    monkeypatch.setattr(active_lock, "_pid_is_recorder", lambda pid: True)

    lock = ActiveLock(tmp_path)
    result = lock.detect_orphan()

    assert result is not None
    assert result["status"] == "another_alive"


def test_detect_orphan_dead_pid_is_orphan(tmp_path: Path) -> None:
    # PID 2^22 is far beyond default pid_max on the target systems.
    _write_lock(tmp_path, pid=4_194_304, uuid="other-instance")

    lock = ActiveLock(tmp_path)
    result = lock.detect_orphan()

    assert result is not None
    assert result["status"] == "orphan"


def test_pid_is_recorder_reads_cmdline() -> None:
    # 自プロセス (pytest) を使うと、テストパスに fv_episode_recorder が
    # 含まれて cmdline 部分一致が偽陽性になり得る (レビュー指摘)。
    # cmdline に recorder 語を含まないことが確実な子プロセスで検証する。
    proc = subprocess.Popen(["sleep", "30"])
    try:
        assert active_lock._pid_is_recorder(proc.pid) is False
    finally:
        proc.kill()
        proc.wait()


def test_pid_is_recorder_without_proc_is_not_recorder(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # /proc が読めない場合は「録画ノードではない」= orphan として上書きされ、
    # 次の録画開始で回復する (マージ済み実装 #9 の挙動に合わせる)
    def raise_oserror(_self: Path) -> bytes:
        raise OSError("no /proc")

    monkeypatch.setattr(active_lock.Path, "read_bytes", raise_oserror)
    assert active_lock._pid_is_recorder(1) is False
