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


def test_finished_video_is_read_only_and_still_hardlinkable(tmp_path: Path) -> None:
    store = EpisodeStore(tmp_path)
    episode_dir = store.start_episode(
        EpisodeMeta(
            episode_id="01TESTIMMUTABLE0000000000",
            task_description="pick",
            profile="piper_single_teleop",
            started_at="2026-07-10T00:00:00.000000Z",
        )
    )
    camera_dir = episode_dir / "videos" / "top_camera"
    camera_dir.mkdir()
    source = camera_dir / "0000.mp4"
    sidecar = camera_dir / "frames.parquet"
    source.write_bytes(b"video")
    sidecar.write_bytes(b"frames")

    store.make_episode_videos_read_only(episode_dir)
    linked = tmp_path / "linked.mp4"
    linked.hardlink_to(source)

    write_bits = stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH
    assert source.stat().st_uid == tmp_path.stat().st_uid
    assert source.stat().st_gid == tmp_path.stat().st_gid
    assert source.stat().st_mode & write_bits == 0
    assert sidecar.stat().st_mode & write_bits == 0
    assert linked.stat().st_ino == source.stat().st_ino


def test_historical_source_migration_only_changes_finished_success(
    tmp_path: Path,
) -> None:
    store = EpisodeStore(tmp_path)
    finished_dir = store.start_episode(
        EpisodeMeta(
            episode_id="01TESTFINISHED00000000000",
            task_description="pick",
            profile="piper_single_teleop",
            started_at="2026-07-10T00:00:00.000000Z",
            timeline_start_ros_ns=1_000_000_000,
        )
    )
    finished_video = finished_dir / "videos" / "top_camera" / "0000.mp4"
    finished_video.parent.mkdir()
    finished_video.write_bytes(b"finished")
    store.stop_active("success", 2_000_000_000)

    failed_dir = store.start_episode(
        EpisodeMeta(
            episode_id="01TESTFAILED000000000000",
            task_description="pick",
            profile="piper_single_teleop",
            started_at="2026-07-10T00:00:01.000000Z",
            timeline_start_ros_ns=2_000_000_000,
        )
    )
    failed_video = failed_dir / "videos" / "top_camera" / "0000.mp4"
    failed_video.parent.mkdir()
    failed_video.write_bytes(b"failed")
    store.stop_active("abort", 3_000_000_000)

    assert store.migrate_finished_episode_sources_read_only() == [
        "01TESTFINISHED00000000000"
    ]
    write_bits = stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH
    assert finished_video.stat().st_mode & write_bits == 0
    assert failed_video.stat().st_mode & write_bits != 0
    assert store.migrate_finished_episode_sources_read_only() == []
