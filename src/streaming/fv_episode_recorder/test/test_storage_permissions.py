from __future__ import annotations

import os
import stat
import sys
import types
from pathlib import Path

_ulid = types.ModuleType("ulid")


class _ULID:
    def __str__(self) -> str:
        return "01TESTASYNCSTOP0000000000"


_ulid.ULID = _ULID
sys.modules.setdefault("ulid", _ulid)

from fv_episode_recorder.episode_store import EpisodeMeta, EpisodeStore


def test_episode_files_inherit_output_group(tmp_path: Path) -> None:
    output_dir = tmp_path / "datasets"
    output_dir.mkdir()
    shared_gid = output_dir.stat().st_gid
    previous_umask = os.umask(0o002)
    try:
        store = EpisodeStore(output_dir)
        episode_dir = store.start_episode(
            EpisodeMeta(
                episode_id="01TESTPERMISSIONS000000000",
                task_description="pick",
                profile="piper_single_teleop",
                started_at="2026-07-10T00:00:00.000000Z",
            )
        )
        camera_dir = episode_dir / "videos" / "top_camera"
        camera_dir.mkdir()
        video_path = camera_dir / "0000.mp4"
        video_path.write_bytes(b"video")
    finally:
        os.umask(previous_umask)

    for directory in (
        output_dir / "episodes",
        episode_dir,
        episode_dir / "bag",
        episode_dir / "videos",
        camera_dir,
    ):
        directory_stat = directory.stat()
        assert directory_stat.st_gid == shared_gid
        assert directory_stat.st_mode & stat.S_ISGID
        assert directory_stat.st_mode & stat.S_IWGRP

    video_stat = video_path.stat()
    assert video_stat.st_gid == shared_gid
    assert video_stat.st_mode & stat.S_IWGRP
