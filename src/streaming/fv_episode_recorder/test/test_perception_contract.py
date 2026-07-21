import asyncio
from pathlib import Path
from types import SimpleNamespace

from fv_episode_recorder.api_server import start_episode, stop_episode
from fv_episode_recorder.environment_annotation_bridge import (
    EnvironmentAnnotationBridge,
)
from fv_episode_recorder.episode_store import EpisodeMeta, EpisodeStore, utc_now_iso
from fv_episode_recorder.marker_manager import MarkerManager
from fv_episode_recorder.schemas import StartEpisodeRequest, StopEpisodeRequest


class _Logger:
    def debug(self, _message: str) -> None:
        pass

    def info(self, _message: str) -> None:
        pass

    def warning(self, _message: str) -> None:
        pass


class _Node:
    def __init__(self) -> None:
        self._logger = _Logger()

    def get_logger(self) -> _Logger:
        return self._logger


class _BagRecorder:
    active = False

    def abort(self) -> None:
        pass

    def stop(self, timeout_s: float) -> dict:
        return {"size_bytes": 0, "split_count": 0}


class _CameraPool:
    def start_all(self, *_args, **_kwargs) -> list[dict]:
        return []

    def stop_all(self) -> list[dict]:
        return []

    def apply_depth_frame_counts(self, _counts: dict) -> None:
        pass


def _make_app(tmp_path: Path):
    store = EpisodeStore(tmp_path / "data")
    markers = MarkerManager()
    app = {
        "store": store,
        "bag_recorder": _BagRecorder(),
        "camera_pool": _CameraPool(),
        "active_lock": None,
        "marker_manager": markers,
        "depth_pool": None,
        "mux_tracker": None,
        "get_profile": lambda _profile: {},
    }
    profiles_dir = tmp_path / "profiles"
    profiles_dir.mkdir()
    (profiles_dir / "test.yaml").write_text("profile: {}\n")
    return app, store, markers, profiles_dir


def _bind_bridge(app, profiles_dir: Path):
    bridge = EnvironmentAnnotationBridge(
        _Node(), app["store"], app["marker_manager"], enabled=False
    )
    starts = []
    stops = []

    async def _start(request):
        starts.append(request)
        return await start_episode(app, request)

    async def _stop(episode_id, request):
        stops.append((episode_id, request))
        return await stop_episode(app, episode_id, request)

    bridge.bind_lifecycle(
        asyncio.get_running_loop(),
        start_episode=_start,
        stop_episode=_stop,
        profiles_dir=profiles_dir,
        auto_profile_override="test",
    )
    return bridge, starts, stops


def _change(change_id: str, state: str):
    return SimpleNamespace(episode_id=change_id, state=state)


def _annotation(change_id: str, text: str):
    return SimpleNamespace(episode_id=change_id, text=text)


def test_no_active_change_auto_records_and_applies_early_annotation(
    tmp_path: Path,
) -> None:
    async def _run() -> None:
        app, store, _markers, profiles_dir = _make_app(tmp_path)
        bridge, starts, stops = _bind_bridge(app, profiles_dir)

        bridge._on_annotation(_annotation("change-a", "赤い箱が置かれた。"))
        bridge._on_change(_change("change-a", "started"))
        await bridge.wait_idle()

        active = store.active
        assert active is not None
        assert active.task_description == "Environment change"
        assert active.tags == ["environment_change", "auto_recorded"]
        episode_id = active.episode_id

        bridge._on_change(_change("change-a", "ended"))
        await bridge.wait_idle()

        assert store.active is None
        assert len(starts) == 1
        assert [item[0] for item in stops] == [episode_id]
        finalized, _episode_dir = store.get_episode(episode_id)
        assert finalized.outcome == "success"
        assert finalized.markers[0]["task_description"] == "赤い箱が置かれた。"
        assert finalized.markers[0]["stopped_at"] is not None
        assert bridge.search("赤い箱", 5) == ["赤い箱が置かれた。"]

    asyncio.run(_run())


def test_preexisting_recording_gets_marker_but_is_not_stopped(tmp_path: Path) -> None:
    async def _run() -> None:
        app, store, markers, profiles_dir = _make_app(tmp_path)
        bridge, starts, stops = _bind_bridge(app, profiles_dir)
        store.start_episode(EpisodeMeta(
            episode_id="manual",
            task_description="manual recording",
            profile="test",
            started_at=utc_now_iso(),
        ))

        bridge._on_change(_change("change-a", "started"))
        bridge._on_change(_change("change-a", "ended"))
        await bridge.wait_idle()

        assert store.active is not None
        assert store.active.episode_id == "manual"
        assert len(markers.list("manual")) == 1
        assert markers.list("manual")[0].stopped_at is not None
        assert starts == []
        assert stops == []

    asyncio.run(_run())


def test_duplicate_change_messages_are_idempotent(tmp_path: Path) -> None:
    async def _run() -> None:
        app, store, _markers, profiles_dir = _make_app(tmp_path)
        bridge, starts, stops = _bind_bridge(app, profiles_dir)
        for _ in range(2):
            bridge._on_change(_change("change-a", "started"))
        await bridge.wait_idle()
        episode_id = store.active.episode_id
        for _ in range(2):
            bridge._on_change(_change("change-a", "ended"))
        await bridge.wait_idle()

        finalized, _episode_dir = store.get_episode(episode_id)
        assert len(starts) == 1
        assert len(stops) == 1
        assert len(finalized.markers) == 1

    asyncio.run(_run())


def test_failed_auto_start_can_retry_the_same_change_id(tmp_path: Path) -> None:
    async def _run() -> None:
        app, store, _markers, profiles_dir = _make_app(tmp_path)
        bridge = EnvironmentAnnotationBridge(
            _Node(), store, app["marker_manager"], enabled=False
        )
        attempts = 0

        async def _start(request):
            nonlocal attempts
            attempts += 1
            if attempts == 1:
                raise RuntimeError("temporary start failure")
            return await start_episode(app, request)

        bridge.bind_lifecycle(
            asyncio.get_running_loop(),
            start_episode=_start,
            stop_episode=lambda episode_id, request: stop_episode(
                app, episode_id, request
            ),
            profiles_dir=profiles_dir,
            auto_profile_override="test",
        )

        bridge._on_change(_change("change-a", "started"))
        await bridge.wait_idle()
        assert store.active is None

        bridge._on_change(_change("change-a", "started"))
        await bridge.wait_idle()
        assert attempts == 2
        assert store.active is not None

        bridge._on_change(_change("change-a", "ended"))
        await bridge.wait_idle()
        assert store.active is None

    asyncio.run(_run())


def test_failed_auto_stop_can_retry_the_same_ended_change(tmp_path: Path) -> None:
    async def _run() -> None:
        app, store, _markers, profiles_dir = _make_app(tmp_path)
        bridge = EnvironmentAnnotationBridge(
            _Node(), store, app["marker_manager"], enabled=False
        )
        stop_attempts = 0

        async def _stop(episode_id, request):
            nonlocal stop_attempts
            stop_attempts += 1
            if stop_attempts == 1:
                raise RuntimeError("temporary stop failure")
            return await stop_episode(app, episode_id, request)

        bridge.bind_lifecycle(
            asyncio.get_running_loop(),
            start_episode=lambda request: start_episode(app, request),
            stop_episode=_stop,
            profiles_dir=profiles_dir,
            auto_profile_override="test",
        )
        bridge._on_change(_change("change-a", "started"))
        bridge._on_change(_change("change-b", "started"))
        await bridge.wait_idle()

        bridge._on_change(_change("change-a", "ended"))
        await bridge.wait_idle()
        assert store.active is not None
        assert stop_attempts == 0

        bridge._on_change(_change("change-b", "ended"))
        await bridge.wait_idle()
        assert store.active is not None
        assert stop_attempts == 1

        bridge._on_change(_change("change-b", "ended"))
        await bridge.wait_idle()
        assert stop_attempts == 2
        assert store.active is None

    asyncio.run(_run())


def test_auto_record_waits_for_all_markers_when_a_ends_before_b(
    tmp_path: Path,
) -> None:
    async def _run() -> None:
        app, store, _markers, profiles_dir = _make_app(tmp_path)
        bridge, starts, stops = _bind_bridge(app, profiles_dir)
        bridge._on_change(_change("change-a", "started"))
        bridge._on_change(_change("change-b", "started"))
        await bridge.wait_idle()
        episode_id = store.active.episode_id

        bridge._on_change(_change("change-a", "ended"))
        await bridge.wait_idle()
        assert store.active is not None
        assert store.active.episode_id == episode_id
        assert stops == []

        bridge._on_change(_change("change-b", "ended"))
        await bridge.wait_idle()
        assert store.active is None
        finalized, _episode_dir = store.get_episode(episode_id)
        assert len(starts) == 1
        assert len(stops) == 1
        assert len(finalized.markers) == 2
        assert all(marker["stopped_at"] is not None for marker in finalized.markers)

    asyncio.run(_run())


def test_auto_record_waits_for_all_markers_when_b_ends_before_a(
    tmp_path: Path,
) -> None:
    async def _run() -> None:
        app, store, _markers, profiles_dir = _make_app(tmp_path)
        bridge, _starts, stops = _bind_bridge(app, profiles_dir)
        bridge._on_change(_change("change-a", "started"))
        bridge._on_change(_change("change-b", "started"))
        await bridge.wait_idle()
        episode_id = store.active.episode_id

        bridge._on_change(_change("change-b", "ended"))
        await bridge.wait_idle()
        assert store.active is not None
        assert store.active.episode_id == episode_id
        assert stops == []

        bridge._on_change(_change("change-a", "ended"))
        await bridge.wait_idle()
        assert store.active is None
        assert len(stops) == 1

    asyncio.run(_run())


def test_auto_marker_ends_do_not_stop_manual_replacement_episode(
    tmp_path: Path,
) -> None:
    async def _run() -> None:
        app, store, _markers, profiles_dir = _make_app(tmp_path)
        bridge, _starts, stops = _bind_bridge(app, profiles_dir)
        bridge._on_change(_change("change-a", "started"))
        bridge._on_change(_change("change-b", "started"))
        await bridge.wait_idle()
        owned_episode_id = store.active.episode_id

        await stop_episode(
            app, owned_episode_id, StopEpisodeRequest(outcome="success")
        )
        replacement = await start_episode(app, StartEpisodeRequest(
            task_description="manual replacement",
            profile="test",
            record_bag=False,
        ))

        bridge._on_change(_change("change-a", "ended"))
        bridge._on_change(_change("change-b", "ended"))
        await bridge.wait_idle()
        assert store.active is not None
        assert store.active.episode_id == replacement.episode_id
        assert stops == []

    asyncio.run(_run())


def test_ros_callback_only_copies_and_schedules_message(tmp_path: Path) -> None:
    app, _store, _markers, profiles_dir = _make_app(tmp_path)
    bridge = EnvironmentAnnotationBridge(
        _Node(), app["store"], app["marker_manager"], enabled=False
    )

    class _Loop:
        def __init__(self):
            self.calls = []

        def call_soon_threadsafe(self, callback, *args):
            self.calls.append((callback, args))

    loop = _Loop()
    bridge.bind_lifecycle(
        loop,
        start_episode=lambda _request: None,
        stop_episode=lambda _episode_id, _request: None,
        profiles_dir=profiles_dir,
        auto_profile_override="test",
    )
    message = _change("change-a", "started")
    bridge._on_change(message)
    message.episode_id = "mutated"

    assert len(loop.calls) == 1
    callback, args = loop.calls[0]
    assert callback == bridge._enqueue_event
    assert args == (("change", "change-a", "started"),)


def test_auto_record_does_not_start_without_an_existing_profile(
    tmp_path: Path, monkeypatch,
) -> None:
    async def _run() -> None:
        app, store, _markers, profiles_dir = _make_app(tmp_path)
        monkeypatch.setenv("HOME", str(tmp_path / "empty-home"))
        monkeypatch.setenv("VLABOR_PROFILE", "also-missing")
        bridge = EnvironmentAnnotationBridge(
            _Node(), store, app["marker_manager"], enabled=False
        )
        starts = []

        async def _start(request):
            starts.append(request)
            return await start_episode(app, request)

        bridge.bind_lifecycle(
            asyncio.get_running_loop(),
            start_episode=_start,
            stop_episode=lambda _episode_id, _request: None,
            profiles_dir=profiles_dir,
            auto_profile_override="missing",
        )
        bridge._on_change(_change("change-a", "started"))
        await bridge.wait_idle()

        assert store.active is None
        assert starts == []

    asyncio.run(_run())


def test_late_annotation_for_ended_change_without_marker_is_not_buffered(
    tmp_path: Path,
) -> None:
    async def _run() -> None:
        app, _store, _markers, profiles_dir = _make_app(tmp_path)
        bridge, _starts, _stops = _bind_bridge(app, profiles_dir)
        bridge._on_annotation(_annotation("change-a", "early"))
        bridge._on_change(_change("change-a", "ended"))
        bridge._on_annotation(_annotation("change-a", "late"))
        await bridge.wait_idle()

        assert "change-a" not in bridge._pending_annotations

    asyncio.run(_run())


def test_annotation_after_finalize_updates_correction(tmp_path: Path) -> None:
    async def _run() -> None:
        app, store, _markers, profiles_dir = _make_app(tmp_path)
        bridge, _starts, _stops = _bind_bridge(app, profiles_dir)
        bridge._on_change(_change("change-a", "started"))
        bridge._on_annotation(_annotation("change-a", "赤い箱が置かれた。"))
        await bridge.wait_idle()
        episode_id = store.active.episode_id
        bridge._on_change(_change("change-a", "ended"))
        await bridge.wait_idle()

        bridge._on_annotation(_annotation(
            "change-a", "赤い工具箱が中央に置かれた。"
        ))
        await bridge.wait_idle()
        finalized, _episode_dir = store.get_episode(episode_id)
        assert finalized.markers[0]["task_description"] == (
            "赤い工具箱が中央に置かれた。"
        )

    asyncio.run(_run())
