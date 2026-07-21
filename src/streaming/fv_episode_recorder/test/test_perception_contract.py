from pathlib import Path
from types import SimpleNamespace

from fv_episode_recorder.environment_annotation_bridge import (
    EnvironmentAnnotationBridge,
)
from fv_episode_recorder.episode_store import EpisodeMeta, EpisodeStore, utc_now_iso
from fv_episode_recorder.marker_manager import MarkerManager


class _Logger:
    def debug(self, _message: str) -> None:
        pass

    def warning(self, _message: str) -> None:
        pass


class _Node:
    def __init__(self) -> None:
        self._logger = _Logger()

    def get_logger(self) -> _Logger:
        return self._logger


def _finalize(store: EpisodeStore, markers: MarkerManager) -> None:
    active = store.active
    assert active is not None
    episode_id = active.episode_id
    pending_markers = markers.flush(episode_id)
    meta, episode_dir = store.stop_active("success")
    meta.markers = pending_markers
    store._write_meta(episode_dir, meta)


def test_environment_annotation_uses_real_episode_marker_and_updates_correction(
    tmp_path: Path,
) -> None:
    store = EpisodeStore(tmp_path)
    markers = MarkerManager()
    bridge = EnvironmentAnnotationBridge(
        _Node(), store, markers, enabled=False
    )
    recording_id = "recording-episode"
    change_id = "visual-change-episode"
    store.start_episode(EpisodeMeta(
        episode_id=recording_id,
        task_description="record robot operation",
        profile="test",
        started_at=utc_now_iso(),
    ))

    bridge._on_change(SimpleNamespace(episode_id=change_id, state="started"))
    bridge._on_annotation(SimpleNamespace(
        episode_id=change_id,
        annotation_id=f"{change_id}:moss",
        text="赤い箱が机に置かれた。",
    ))
    bridge._on_change(SimpleNamespace(episode_id=change_id, state="ended"))

    live_markers = markers.list(recording_id)
    assert len(live_markers) == 1
    assert live_markers[0].episode_id == recording_id
    assert live_markers[0].task_description == "赤い箱が机に置かれた。"
    assert live_markers[0].stopped_at is not None
    assert live_markers[0].attributes == [
        {"key": "environment_change_id", "value": change_id}
    ]

    _finalize(store, markers)
    bridge._on_annotation(SimpleNamespace(
        episode_id=change_id,
        annotation_id=f"{change_id}:moss",
        text="赤い工具箱が机の中央に置かれた。",
    ))

    finalized, _episode_dir = store.get_episode(recording_id)
    assert finalized.markers[0]["task_description"] == (
        "赤い工具箱が机の中央に置かれた。"
    )
    assert bridge.search("さっき赤い工具箱はどこに置かれた？", 5) == [
        "赤い工具箱が机の中央に置かれた。"
    ]


def test_environment_change_is_not_persisted_without_active_recording(
    tmp_path: Path,
) -> None:
    store = EpisodeStore(tmp_path)
    markers = MarkerManager()
    bridge = EnvironmentAnnotationBridge(
        _Node(), store, markers, enabled=False
    )

    bridge._on_change(SimpleNamespace(episode_id="orphan-change", state="started"))
    bridge._on_annotation(SimpleNamespace(
        episode_id="orphan-change",
        annotation_id="orphan-change:moss",
        text="保存されない記述",
    ))

    assert bridge.search("保存", 5) == []
    assert store.list_episodes() == []
