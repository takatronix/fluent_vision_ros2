"""ActiveLock pid-reuse regression tests (pure pytest, no ROS runtime needed)."""

from __future__ import annotations

import json
import os
import subprocess
import time
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
    # pid_max (対象機で 4194304 = 2^22) と同値。カーネルは pid_max-1
    # までしか払い出さないため、この pid が生きていることはない。
    _write_lock(tmp_path, pid=4_194_304, uuid="other-instance")

    lock = ActiveLock(tmp_path)
    result = lock.detect_orphan()

    assert result is not None
    assert result["status"] == "orphan"


def _wait_cmdline(pid: int, timeout: float = 2.0) -> bytes:
    """fork→exec 直後は /proc/<pid>/cmdline が空のことがある (実測 ~45%)。
    実際に argv が見えるまで短くポーリングする。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        data = Path(f"/proc/{pid}/cmdline").read_bytes()
        if data:
            return data
        time.sleep(0.01)
    raise AssertionError(f"cmdline for pid {pid} stayed empty")


def test_pid_is_recorder_reads_cmdline() -> None:
    # 自プロセス (pytest) を使うと、テストパスに fv_episode_recorder が
    # 含まれて cmdline 部分一致が偽陽性になり得る (レビュー指摘)。
    # cmdline に recorder 語を含まないことが確実な子プロセスで検証する。
    proc = subprocess.Popen(["sleep", "30"])
    try:
        assert b"recorder" not in _wait_cmdline(proc.pid)
        assert active_lock._pid_is_recorder(proc.pid) is False
    finally:
        proc.kill()
        proc.wait()


def test_pid_is_recorder_detects_real_recorder_name(tmp_path: Path) -> None:
    # 陽性ケース: 照合ロジック/照合語が壊れたら (常に False・語の変更)
    # ここが落ちる。実運用の cmdline は
    # /usr/bin/python3 .../lib/fv_episode_recorder/recorder_node --ros-args
    # の形なので、argv0 に recorder_node を含む実プロセスで検証する。
    exe = tmp_path / "recorder_node"
    exe.symlink_to("/bin/sleep")
    proc = subprocess.Popen([str(exe), "30"])
    try:
        assert b"recorder_node" in _wait_cmdline(proc.pid)
        assert active_lock._pid_is_recorder(proc.pid) is True
    finally:
        proc.kill()
        proc.wait()


def test_pid_is_recorder_without_proc_is_not_recorder(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # /proc が読めない場合は「録画ノードではない」= orphan として上書きされ、
    # 次の録画開始で回復する (マージ済み実装 #9 の挙動に合わせる)。
    # pathlib.Path.read_bytes をプロセス全体で差し替えると他の読取を
    # 巻き込むため、切り出したヘルパだけを差し替える
    def raise_oserror(_pid: int) -> bytes:
        raise OSError("no /proc")

    monkeypatch.setattr(active_lock, "_read_cmdline", raise_oserror)
    assert active_lock._pid_is_recorder(1) is False
