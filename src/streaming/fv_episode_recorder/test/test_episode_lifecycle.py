import asyncio
import json
from pathlib import Path

import pytest

from fv_episode_recorder.api_server import (
    EpisodeLifecycleError,
    _start_episode,
    start_episode,
    stop_episode,
)
from fv_episode_recorder.episode_store import EpisodeMeta, EpisodeStore, utc_now_iso
from fv_episode_recorder.marker_manager import MarkerManager
from fv_episode_recorder.schemas import StartEpisodeRequest, StopEpisodeRequest


class _Lock:
    def __init__(
        self, *, acquire_error: bool = False, release_error: bool = False
    ) -> None:
        self.acquired = 0
        self.released = 0
        self.acquire_error = acquire_error
        self.release_error = release_error

    def acquire(self, _episode_id: str, _episode_dir: Path) -> None:
        self.acquired += 1
        if self.acquire_error:
            raise RuntimeError("lock acquire")

    def release(self) -> None:
        self.released += 1
        if self.release_error:
            raise RuntimeError("lock release")


class _Bag:
    def __init__(
        self, *, active: bool = False, start_error: bool = False,
        stop_error: bool = False,
    ) -> None:
        self.active = active
        self.start_error = start_error
        self.stop_error = stop_error
        self.aborted = 0

    def start(self, _bag_dir: Path, _topics: list[str]) -> None:
        if self.start_error:
            raise RuntimeError("bag start")
        self.active = True

    def stop(self, timeout_s: float) -> dict:
        if self.stop_error:
            raise RuntimeError("bag stop")
        self.active = False
        return {"size_bytes": 10, "split_count": 1}

    def abort(self) -> None:
        self.aborted += 1
        self.active = False


class _Cameras:
    def __init__(
        self, *, start_error: bool = False, stop_error: bool = False
    ) -> None:
        self.start_error = start_error
        self.stop_error = stop_error
        self.stopped = 0

    def start_all(self, *_args, **_kwargs) -> list[dict]:
        if self.start_error:
            raise RuntimeError("camera start")
        return []

    def stop_all(self) -> list[dict]:
        self.stopped += 1
        if self.stop_error:
            raise RuntimeError("camera stop")
        return []

    def apply_depth_frame_counts(self, _counts: dict) -> None:
        pass


class _Depth:
    def __init__(self, *, start_error: bool = False, stop_error: bool = False) -> None:
        self.start_error = start_error
        self.stop_error = stop_error
        self.started = 0
        self.stopped = 0

    def start_all(self, _cameras: list[dict]) -> None:
        self.started += 1
        if self.start_error:
            raise RuntimeError("depth start")

    def stop_all(self) -> dict:
        self.stopped += 1
        if self.stop_error:
            raise RuntimeError("depth stop")
        return {}


class _FailingMarkers(MarkerManager):
    def flush(self, _episode_id: str) -> list[dict]:
        raise RuntimeError("marker flush")


def _app(tmp_path: Path, **overrides):
    defaults = {
        "store": EpisodeStore(tmp_path / "data"),
        "bag_recorder": _Bag(),
        "camera_pool": _Cameras(),
        "active_lock": _Lock(),
        "marker_manager": MarkerManager(),
        "depth_pool": None,
        "mux_tracker": None,
        "get_profile": lambda _profile: {},
    }
    defaults.update(overrides)
    return defaults


def test_start_failure_rolls_back_every_started_resource(tmp_path: Path) -> None:
    depth = _Depth(start_error=True)
    bag = _Bag()
    cameras = _Cameras()
    lock = _Lock()
    app = _app(
        tmp_path,
        depth_pool=depth,
        bag_recorder=bag,
        camera_pool=cameras,
        active_lock=lock,
        get_profile=lambda _profile: {
            "episode_recorder": {
                "cameras": [
                    {"name": "depth", "topic": "/depth", "kind": "depth"}
                ]
            }
        },
    )

    async def _run() -> None:
        with pytest.raises(EpisodeLifecycleError) as error:
            await start_episode(app, StartEpisodeRequest(
                task_description="Environment change",
                profile="test",
                tags=["environment_change", "auto_recorded"],
            ))
        assert error.value.payload["error"] == "depth_republisher_failed"

    asyncio.run(_run())
    assert app["store"].active is None
    assert depth.started == 1
    assert depth.stopped == 1
    assert cameras.stopped == 1
    assert bag.aborted == 1
    assert lock.acquired == 1
    assert lock.released == 1
    episodes = app["store"].list_episodes()
    assert len(episodes) == 1
    assert episodes[0][0].state == "failed"


def test_non_depth_start_failures_keep_existing_best_effort_contract(
    tmp_path: Path, monkeypatch,
) -> None:
    bag = _Bag(start_error=True)
    cameras = _Cameras(start_error=True)
    lock = _Lock(acquire_error=True)
    app = _app(
        tmp_path,
        bag_recorder=bag,
        camera_pool=cameras,
        active_lock=lock,
        get_profile=lambda _profile: {
            "episode_recorder": {
                "record_topics_override": ["/joint_states"],
                "cameras": [{"name": "color", "topic": "/color"}],
            }
        },
    )
    monkeypatch.setattr(
        "fv_episode_recorder.api_server.BagRecorder.available", lambda: True
    )

    class _Request:
        async def json(self):
            return {
                "task_description": "manual recording",
                "profile": "test",
            }

    request = _Request()
    request.app = app
    response = asyncio.run(_start_episode(request))
    payload = json.loads(response.body)

    assert response.status == 201
    assert payload["episode_id"] == app["store"].active.episode_id
    assert payload["preflight"]["bag_started"] is False
    assert payload["preflight"]["cameras_started"] == []
    assert lock.acquired == 1


def test_stop_completes_finalize_and_lock_release_after_cleanup_errors(
    tmp_path: Path,
) -> None:
    bag = _Bag(active=True, stop_error=True)
    depth = _Depth(stop_error=True)
    cameras = _Cameras(stop_error=True)
    lock = _Lock(release_error=True)
    app = _app(
        tmp_path,
        bag_recorder=bag,
        depth_pool=depth,
        camera_pool=cameras,
        active_lock=lock,
        marker_manager=_FailingMarkers(),
    )
    app["store"].start_episode(EpisodeMeta(
        episode_id="episode-a",
        task_description="test",
        profile="test",
        started_at=utc_now_iso(),
    ))

    response = asyncio.run(stop_episode(
        app, "episode-a", StopEpisodeRequest(outcome="success")
    ))

    assert response.episode_id == "episode-a"
    assert app["store"].active is None
    assert bag.aborted == 1
    assert depth.stopped == 1
    assert cameras.stopped == 1
    assert lock.released == 1
    finalized, _episode_dir = app["store"].get_episode("episode-a")
    assert finalized.state == "finished"
    assert finalized.outcome == "success"


def test_store_finalize_failure_retains_lock_for_active_episode(
    tmp_path: Path,
) -> None:
    lock = _Lock()
    store = EpisodeStore(tmp_path / "data")
    episode_dir = store.start_episode(EpisodeMeta(
        episode_id="episode-a",
        task_description="test",
        profile="test",
        started_at=utc_now_iso(),
    ))
    lock.acquire("episode-a", episode_dir)
    original_stop = store.stop_active

    def _fail_stop(_outcome: str):
        raise RuntimeError("store finalize")

    store.stop_active = _fail_stop
    app = _app(tmp_path, store=store, active_lock=lock)

    async def _run() -> None:
        with pytest.raises(EpisodeLifecycleError) as error:
            await stop_episode(
                app, "episode-a", StopEpisodeRequest(outcome="success")
            )
        assert error.value.payload["error"] == "episode_finalize_failed"

    asyncio.run(_run())
    assert store.active is not None
    assert store.active.episode_id == "episode-a"
    assert lock.released == 0
    store.stop_active = original_stop


def test_start_rollback_failure_retains_lock_for_active_episode(
    tmp_path: Path,
) -> None:
    lock = _Lock()
    store = EpisodeStore(tmp_path / "data")
    original_stop = store.stop_active

    def _fail_stop(_outcome: str):
        raise RuntimeError("store rollback")

    store.stop_active = _fail_stop
    app = _app(
        tmp_path,
        store=store,
        active_lock=lock,
        depth_pool=_Depth(start_error=True),
        get_profile=lambda _profile: {
            "episode_recorder": {
                "cameras": [
                    {"name": "depth", "topic": "/depth", "kind": "depth"}
                ]
            }
        },
    )

    async def _run() -> None:
        with pytest.raises(EpisodeLifecycleError):
            await start_episode(app, StartEpisodeRequest(
                task_description="Environment change",
                profile="test",
                record_bag=False,
            ))

    asyncio.run(_run())
    assert store.active is not None
    assert lock.acquired == 1
    assert lock.released == 0
    store.stop_active = original_stop
