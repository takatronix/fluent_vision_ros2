import os
from pathlib import Path
import subprocess

import pytest

from fv_episode_recorder.bag_recorder import BagRecorder


class FakeProcess:
    pid = 123

    def __init__(self, *, wait_timeout: bool = False):
        self.returncode = None
        self.wait_timeout = wait_timeout
        self.wait_calls = 0

    def poll(self):
        return self.returncode

    def wait(self, timeout: float):
        self.wait_calls += 1
        if self.wait_timeout:
            raise subprocess.TimeoutExpired("ros2", timeout)
        self.returncode = 0
        return 0

    def terminate(self) -> None:
        pass

    def kill(self) -> None:
        pass


def _started_recorder(tmp_path: Path, process: FakeProcess) -> BagRecorder:
    bag_dir = tmp_path / "bag"
    bag_dir.mkdir()
    recorder = BagRecorder()
    recorder._proc = process
    recorder._bag_dir = bag_dir
    recorder._topics = ["/joint_states"]
    return recorder


def test_request_stop_signals_without_waiting_then_stop_verifies_metadata(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    process = FakeProcess()
    recorder = _started_recorder(tmp_path, process)
    (recorder._bag_dir / "metadata.yaml").write_text("rosbag2_bagfile_information: {}")
    signals: list[tuple[int, int]] = []
    monkeypatch.setattr(os, "getpgid", lambda pid: pid)
    monkeypatch.setattr(os, "killpg", lambda pid, sig: signals.append((pid, sig)))

    recorder.request_stop()

    assert process.wait_calls == 0
    assert signals == [(process.pid, 2)]
    summary = recorder.stop()
    assert process.wait_calls == 1
    assert summary["topics"] == ["/joint_states"]


def test_stop_rejects_missing_rosbag_metadata(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    recorder = _started_recorder(tmp_path, FakeProcess())
    monkeypatch.setattr(os, "getpgid", lambda pid: pid)
    monkeypatch.setattr(os, "killpg", lambda _pid, _sig: None)

    with pytest.raises(RuntimeError, match="metadata.yaml was not finalized"):
        recorder.stop()


def test_request_stop_can_retry_after_signal_failure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    recorder = _started_recorder(tmp_path, FakeProcess())
    calls = 0

    def fail_once(_pid: int, _sig: int) -> None:
        nonlocal calls
        calls += 1
        if calls == 1:
            raise PermissionError("signal denied")

    monkeypatch.setattr(os, "getpgid", lambda pid: pid)
    monkeypatch.setattr(os, "killpg", fail_once)

    with pytest.raises(PermissionError, match="signal denied"):
        recorder.request_stop()
    assert not recorder._stop_requested

    recorder.request_stop()
    assert recorder._stop_requested
    assert calls == 2


def test_abort_fails_if_process_survives_sigkill(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    recorder = _started_recorder(tmp_path, FakeProcess(wait_timeout=True))
    monkeypatch.setattr(os, "getpgid", lambda pid: pid)
    monkeypatch.setattr(os, "killpg", lambda _pid, _sig: None)

    with pytest.raises(RuntimeError, match="did not exit after SIGKILL"):
        recorder.abort()
