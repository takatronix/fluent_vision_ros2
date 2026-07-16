from dataclasses import asdict
import json
import os
from pathlib import Path
import shutil
import stat

import pytest

from fv_episode_recorder import episode_index
from fv_episode_recorder.episode_store import (
    DuplicateEpisodeIdError,
    EPISODE_DIR_MODE,
    EpisodeMeta,
    EpisodeMetaV2,
    EpisodeIndexSyncError,
    EpisodeSchemaError,
    EpisodeStore,
    FINISHED_PAYLOAD_DIR_MODE,
    FINISHED_PAYLOAD_FILE_MODE,
    META_FILE_MODE,
    read_episode_meta,
)
from fv_episode_recorder.retention import RetentionPolicy, RetentionRunner
from fv_episode_recorder.schemas import MergeEpisodeTagsRequest


STARTED_AT = "2026-07-13T00:00:00.000000Z"


def _meta(episode_id: str, tags: list[str] | None = None) -> EpisodeMeta:
    return EpisodeMeta(
        episode_id=episode_id,
        task_description="storage contract test",
        profile="test_profile",
        tags=tags or [],
        started_at=STARTED_AT,
    )


def _finish(store: EpisodeStore, episode_id: str, outcome: str = "success") -> Path:
    ep_dir = store.start_episode(_meta(episode_id, ["tag-a"]))
    _complete_active(store, outcome)
    return ep_dir


def _complete_active(store: EpisodeStore, outcome: str) -> None:
    meta, ep_dir, index_error = store.begin_finalization(outcome)
    assert index_error is None
    if outcome == "success":
        meta.state = "finished"
        store.protect_finished_payload_sources(ep_dir)
    elif outcome == "abort":
        meta.state = "failed"
    else:
        meta.state = "discarded"
    store._write_meta(ep_dir, meta)


def _raw_meta_path(root: Path, name: str) -> Path:
    path = root / "episodes" / "test_profile" / "2026-07-13" / name / "meta.json"
    path.parent.mkdir(parents=True)
    return path


def test_v1_migrates_forward_and_unknown_versions_fail_closed(tmp_path: Path) -> None:
    meta_path = _raw_meta_path(tmp_path, "legacy")
    data = asdict(_meta("episode-v1"))
    data["schema_version"] = 1
    meta_path.write_text(json.dumps(data))

    meta, migrated = read_episode_meta(meta_path)

    assert meta.schema_version == 2
    assert migrated["schema_version"] == 2
    stored = json.loads(meta_path.read_text())
    assert stored["schema_version"] == 2

    for name, version in (("missing", None), ("future", 3)):
        invalid_path = _raw_meta_path(tmp_path, name)
        invalid = asdict(_meta(f"episode-{name}"))
        if version is None:
            invalid.pop("schema_version")
        else:
            invalid["schema_version"] = version
        invalid_path.write_text(json.dumps(invalid))
        with pytest.raises(EpisodeSchemaError):
            read_episode_meta(invalid_path)


def test_v1_unknown_fields_fail_without_rewriting_metadata(tmp_path: Path) -> None:
    meta_path = _raw_meta_path(tmp_path, "unknown-v1-field")
    data = asdict(_meta("episode-v1-unknown"))
    data["schema_version"] = 1
    data["timeline_start_ros_ns"] = 1
    meta_path.write_text(json.dumps(data))

    with pytest.raises(EpisodeSchemaError):
        read_episode_meta(meta_path)

    assert json.loads(meta_path.read_text()) == data


def test_v2_schema_excludes_unowned_timing_fields(tmp_path: Path) -> None:
    forbidden = {
        "timeline_start_ros_ns",
        "timeline_end_ros_ns",
        "stop_frame_count_per_camera",
        "video_timing_mode",
        "pts_origin",
        "video_time_base",
    }
    assert forbidden.isdisjoint(EpisodeMeta.__annotations__)
    assert set(EpisodeMetaV2.model_fields) == set(EpisodeMeta.__annotations__)

    meta_path = _raw_meta_path(tmp_path, "unknown-field")
    data = asdict(_meta("episode-unknown-field"))
    data["timeline_start_ros_ns"] = 1
    meta_path.write_text(json.dumps(data))
    with pytest.raises(EpisodeSchemaError, match="Extra inputs are not permitted"):
        read_episode_meta(meta_path)

    for field_name in (
        "video_timing_mode",
        "video_pts_origin_ros_ns",
        "video_time_base_num",
        "video_time_base_den",
    ):
        nested_path = _raw_meta_path(tmp_path, f"nested-{field_name}")
        nested = asdict(_meta(f"episode-{field_name}"))
        nested["cameras"] = [{
            "name": "camera",
            "topic": "/camera/image",
            field_name: 1,
        }]
        nested_path.write_text(json.dumps(nested))
        with pytest.raises(EpisodeSchemaError, match="Extra inputs are not permitted"):
            read_episode_meta(nested_path)


@pytest.mark.parametrize(
    ("field_name", "invalid_value"),
    (
        ("pinned", "false"),
        ("duration_s", {"seconds": 1}),
        ("cameras", "not-a-list"),
        ("recorded_topics", {"topic": "/topic"}),
        ("markers", "not-a-list"),
        ("source", 1),
    ),
)
def test_v2_rejects_invalid_top_level_types(
    tmp_path: Path,
    field_name: str,
    invalid_value,
) -> None:
    meta_path = _raw_meta_path(tmp_path, f"invalid-{field_name}")
    data = asdict(_meta(f"episode-invalid-{field_name}"))
    data[field_name] = invalid_value
    meta_path.write_text(json.dumps(data))

    with pytest.raises(EpisodeSchemaError):
        read_episode_meta(meta_path)


def test_v2_validates_camera_segment_and_recorded_topic_types(tmp_path: Path) -> None:
    valid_path = _raw_meta_path(tmp_path, "valid-nested")
    valid = asdict(_meta("episode-valid-nested"))
    valid["cameras"] = [{
        "name": "camera",
        "topic": "/camera/image",
        "width": 640,
        "height": 480,
        "fps_nominal": 30,
        "fps_actual": 29.9,
        "frame_count": 10,
        "segments": [{"file": "0000.mp4", "frame_count": 10, "size_bytes": 100}],
    }]
    valid["recorded_topics"] = [{
        "topic": "/joint_states",
        "role": "state",
        "qos": "reliable",
        "stamp_source": "message_header",
    }]
    valid_path.write_text(json.dumps(valid))
    read_episode_meta(valid_path)

    for name, mutate in (
        ("segment", lambda data: data["cameras"][0]["segments"][0].update(size_bytes="100")),
        ("topic", lambda data: data["recorded_topics"][0].update(qos=1)),
    ):
        invalid_path = _raw_meta_path(tmp_path, f"invalid-{name}")
        invalid = json.loads(json.dumps(valid))
        mutate(invalid)
        invalid_path.write_text(json.dumps(invalid))
        with pytest.raises(EpisodeSchemaError):
            read_episode_meta(invalid_path)


def test_startup_rebuild_repairs_same_count_state_and_tag_drift(tmp_path: Path) -> None:
    store = EpisodeStore(tmp_path)
    _finish(store, "episode-index")
    store.index._conn.execute(
        "UPDATE episodes SET state = 'failed', tags = '[\"stale\"]' "
        "WHERE episode_id = 'episode-index'"
    )

    assert store.index.rebuild_from_filesystem() == 1
    rows, _cursor = store.index.list()
    assert rows[0]["episode_schema_version"] == 2
    assert rows[0]["state"] == "finished"
    assert rows[0]["tags"] == ["tag-a"]


def test_duplicate_episode_ids_are_rejected_without_replacing_index(tmp_path: Path) -> None:
    store = EpisodeStore(tmp_path)
    original_dir = _finish(store, "episode-duplicate")
    duplicate_dir = original_dir.parent / "duplicate-directory"
    shutil.copytree(original_dir, duplicate_dir)

    with pytest.raises(DuplicateEpisodeIdError):
        store.index.rebuild_from_filesystem()
    assert store.index.total_count() == 1
    with pytest.raises(DuplicateEpisodeIdError):
        store.get_episode("episode-duplicate")


def test_incomplete_episode_becomes_failed_without_removing_files(tmp_path: Path) -> None:
    store = EpisodeStore(tmp_path)
    ep_dir = store.start_episode(_meta("episode-orphan"))
    artifact = ep_dir / "bag" / "partial.db3"
    artifact.write_bytes(b"partial")
    store.active.state = "finalizing"
    store.active.outcome = "success"
    store._write_meta(ep_dir, store.active)

    assert store.index.rebuild_from_filesystem() == 1
    assert store.fail_incomplete_episodes() == 1
    meta, _data = read_episode_meta(ep_dir / "meta.json")
    assert meta.state == "failed"
    assert meta.outcome == "abort"
    assert artifact.read_bytes() == b"partial"


def test_begin_finalization_does_not_scan_episode_size(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    store = EpisodeStore(tmp_path)
    store.start_episode(_meta("episode-fast-stop"))

    def fail_size_scan(_episode_dir: Path) -> int:
        raise AssertionError("size scan must not run before the 202 response")

    monkeypatch.setattr(episode_index, "_ep_dir_size", fail_size_scan)
    meta, _ep_dir, index_error = store.begin_finalization("success")

    assert meta.state == "finalizing"
    assert index_error is None


def test_terminal_size_scan_runs_outside_index_lock(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    store = EpisodeStore(tmp_path)
    ep_dir = _finish(store, "episode-size-scan")
    meta, data = read_episode_meta(ep_dir / "meta.json")

    def assert_unlocked(_episode_dir: Path) -> int:
        acquired = store.index._lock.acquire(blocking=False)
        assert acquired
        store.index._lock.release()
        return 123

    monkeypatch.setattr(episode_index, "_ep_dir_size", assert_unlocked)
    store.index.upsert(data, ep_dir, refresh_size=True)

    rows, _cursor = store.index.list()
    assert meta.state == "finished"
    assert rows[0]["size_bytes"] == 123


def test_finalizing_episode_is_excluded_from_delete_and_retention(tmp_path: Path) -> None:
    store = EpisodeStore(tmp_path)
    ep_dir = store.start_episode(_meta("episode-finalizing"))
    store.begin_finalization("success")

    with pytest.raises(RuntimeError, match="state=finalizing"):
        store.delete_episode("episode-finalizing", force=True)

    runner = RetentionRunner(store)
    result = runner.tick(policy=RetentionPolicy(
        enabled=True,
        max_episodes=0,
        grace_period_s=0,
    ))

    assert result["candidates"] == []
    assert ep_dir.exists()


def test_tag_merge_validates_deduplicates_and_updates_index(tmp_path: Path) -> None:
    store = EpisodeStore(tmp_path)
    _finish(store, "episode-tags")

    request = MergeEpisodeTagsRequest(tags=["tag-b", "tag-a", "tag-b"])
    updated = store.merge_episode_tags("episode-tags", request.tags)

    assert updated is not None
    assert updated["tags"] == ["tag-a", "tag-b"]
    rows, _cursor = store.index.list()
    assert rows[0]["tags"] == ["tag-a", "tag-b"]
    with pytest.raises(EpisodeSchemaError):
        store.merge_episode_tags("episode-tags", [" invalid"])


def test_finalizing_episode_rejects_metadata_mutation(tmp_path: Path) -> None:
    store = EpisodeStore(tmp_path)
    store.start_episode(_meta("episode-finalizing-mutation"))
    store.begin_finalization("success")

    with pytest.raises(RuntimeError, match="state=finalizing"):
        store.patch_episode_meta("episode-finalizing-mutation", {"pinned": True})
    with pytest.raises(RuntimeError, match="state=finalizing"):
        store.merge_episode_tags("episode-finalizing-mutation", ["tag-b"])
    with pytest.raises(RuntimeError, match="state=finalizing"):
        store.add_finalized_marker("episode-finalizing-mutation", {"marker_id": "m1"})


def test_index_projection_failure_is_reported_and_startup_rebuild_repairs_it(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    store = EpisodeStore(tmp_path)
    _finish(store, "episode-index-failure")
    original_upsert = store.index.upsert

    def fail_upsert(meta_dict, ep_dir) -> None:
        raise RuntimeError("injected index failure")

    monkeypatch.setattr(store.index, "upsert", fail_upsert)
    with pytest.raises(EpisodeIndexSyncError, match="startup index rebuild required"):
        store.merge_episode_tags("episode-index-failure", ["tag-b"])

    _meta, stored = read_episode_meta(store.get_episode("episode-index-failure")[1] / "meta.json")
    assert stored["tags"] == ["tag-a", "tag-b"]
    rows, _cursor = store.index.list()
    assert rows[0]["tags"] == ["tag-a"]

    monkeypatch.setattr(store.index, "upsert", original_upsert)
    assert store.index.rebuild_from_filesystem() == 1
    rows, _cursor = store.index.list()
    assert rows[0]["tags"] == ["tag-a", "tag-b"]


def test_retention_deletion_removes_index_row(tmp_path: Path) -> None:
    store = EpisodeStore(tmp_path)
    ep_dir = store.start_episode(_meta("episode-retention", ["tag-a"]))
    camera_dir = ep_dir / "videos" / "camera"
    camera_dir.mkdir()
    (camera_dir / "0000.mp4").write_bytes(b"video")
    _complete_active(store, "success")
    runner = RetentionRunner(store)

    result = runner.tick(policy=RetentionPolicy(
        enabled=True,
        max_episodes=0,
        grace_period_s=0,
    ))

    assert [item["episode_id"] for item in result["deleted"]] == ["episode-retention"]
    assert not ep_dir.exists()
    assert store.index.total_count() == 0


def test_delete_index_failure_retains_episode_files(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    store = EpisodeStore(tmp_path)
    ep_dir = _finish(store, "episode-delete-index-failure")
    original_delete = store.index.delete

    def fail_delete(episode_id: str) -> None:
        raise RuntimeError("injected index failure")

    monkeypatch.setattr(store.index, "delete", fail_delete)
    with pytest.raises(EpisodeIndexSyncError, match="files retained"):
        store.delete_episode("episode-delete-index-failure")

    assert ep_dir.exists()
    assert store.index.total_count() == 1
    monkeypatch.setattr(store.index, "delete", original_delete)
    assert store.index.rebuild_from_filesystem() == 1
    assert store.index.total_count() == 1


def test_successful_payload_sources_are_read_only_and_historical_scope_is_narrow(
    tmp_path: Path,
) -> None:
    store = EpisodeStore(tmp_path)
    success_dir = store.start_episode(_meta("episode-success"))
    success_bag = success_dir / "bag" / "bag_0.db3"
    success_bag.write_bytes(b"bag")
    success_video = success_dir / "videos" / "camera" / "0000.mp4"
    success_video.parent.mkdir()
    success_video.write_bytes(b"video")
    _complete_active(store, "success")

    failed_dir = store.start_episode(_meta("episode-failed"))
    failed_bag = failed_dir / "bag" / "bag_0.db3"
    failed_bag.write_bytes(b"bag")
    failed_video = failed_dir / "videos" / "camera" / "0000.mp4"
    failed_video.parent.mkdir()
    failed_video.write_bytes(b"video")
    _complete_active(store, "abort")

    success_bag.chmod(0o660)
    success_video.chmod(0o660)
    failed_bag.chmod(0o660)
    failed_video.chmod(0o660)
    assert store.migrate_finished_payload_permissions() == 1

    assert stat.S_IMODE(success_dir.stat().st_mode) == EPISODE_DIR_MODE
    assert stat.S_IMODE((success_dir / "meta.json").stat().st_mode) == META_FILE_MODE
    assert stat.S_IMODE(success_bag.parent.stat().st_mode) == FINISHED_PAYLOAD_DIR_MODE
    assert stat.S_IMODE(success_bag.stat().st_mode) == FINISHED_PAYLOAD_FILE_MODE
    assert stat.S_IMODE(success_video.parent.stat().st_mode) == FINISHED_PAYLOAD_DIR_MODE
    assert stat.S_IMODE(success_video.stat().st_mode) == FINISHED_PAYLOAD_FILE_MODE
    assert stat.S_IMODE(failed_bag.stat().st_mode) == 0o660
    assert stat.S_IMODE(failed_video.stat().st_mode) == 0o660
    output_stat = tmp_path.stat()
    for path in (
        success_dir,
        success_bag.parent,
        success_bag,
        success_video.parent,
        success_video,
    ):
        assert path.stat().st_uid == output_stat.st_uid
        assert path.stat().st_gid == output_stat.st_gid
    linked_bag = tmp_path / "linked.db3"
    linked_video = tmp_path / "linked.mp4"
    os.link(success_bag, linked_bag)
    os.link(success_video, linked_video)
    assert linked_bag.read_bytes() == b"bag"
    assert linked_video.read_bytes() == b"video"
