from __future__ import annotations

import asyncio
import json
import sys
import threading
import types
import unittest
from unittest.mock import patch
from dataclasses import dataclass, field
from pathlib import Path
from tempfile import TemporaryDirectory


def _install_ros_stubs() -> None:
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_qos = types.ModuleType("rclpy.qos")
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    rclpy_node.Node = type("Node", (), {})
    rclpy_qos.qos_profile_sensor_data = object()
    sensor_msgs_msg.CompressedImage = type("CompressedImage", (), {})
    sensor_msgs_msg.Image = type("Image", (), {})
    sys.modules.setdefault("rclpy", rclpy)
    sys.modules.setdefault("rclpy.node", rclpy_node)
    sys.modules.setdefault("rclpy.qos", rclpy_qos)
    sys.modules.setdefault("sensor_msgs", sensor_msgs)
    sys.modules.setdefault("sensor_msgs.msg", sensor_msgs_msg)


_install_ros_stubs()

from fv_episode_recorder.api_server import (  # noqa: E402
    _failed_start_response,
    _run_finalizer,
    _shutdown_finalizers,
    _stop_episode,
)
from fv_episode_recorder.episode_store import EpisodeMeta, EpisodeStore  # noqa: E402
from fv_episode_recorder.finalization import (  # noqa: E402
    ActiveRecording,
    FinalizationJob,
    finalize_episode,
    wait_for_finalizers,
)


@dataclass
class FakeMeta:
    episode_id: str = "episode-old"
    state: str = "finalizing"
    outcome: str = "success"
    bag_split_count: int = 0
    cameras: list[dict] = field(default_factory=list)
    markers: list[dict] = field(default_factory=list)
    controller_at_end: dict | None = None
    finalization_failures: list[dict[str, str]] = field(default_factory=list)


class FakeStore:
    def __init__(
        self,
        meta: FakeMeta,
        episode_dir: Path,
        fail_writes: int = 0,
        fail_discard: bool = False,
    ):
        self.active = meta
        self.episode_dir = episode_dir
        self.fail_writes = fail_writes
        self.written_states: list[str] = []
        self.discarded = False
        self.fail_discard = fail_discard

    def begin_finalization(self, outcome: str):
        meta = self.active
        meta.outcome = outcome
        meta.state = "finalizing"
        self._write_meta(self.episode_dir, meta)
        self.active = None
        return meta, self.episode_dir, None

    def start_new(self) -> None:
        if self.active is not None:
            raise RuntimeError("episode is still owned")
        self.active = FakeMeta(episode_id="episode-new", state="recording")

    def _write_meta(
        self,
        episode_dir: Path,
        meta: FakeMeta,
        refresh_size: bool = False,
    ) -> None:
        if self.fail_writes:
            self.fail_writes -= 1
            raise OSError("metadata write failed")
        self.written_states.append(meta.state)


    def protect_finished_payload_sources(self, _episode_dir: Path) -> None:
        pass

    def discard_finalizing_episode(self, _episode_id: str) -> bool:
        if self.fail_discard:
            raise OSError("discard failed")
        self.discarded = True
        return True


class FakeBag:
    def __init__(self, order: list[str], release: threading.Event | None = None,
                 failure: Exception | None = None,
                 request_failure: Exception | None = None):
        self.started = True
        self.order = order
        self.release = release
        self.failure = failure
        self.request_failure = request_failure
        self.stop_requested = False
        self.abort_count = 0

    def request_stop(self) -> None:
        self.order.append("bag_request")
        if self.request_failure is not None:
            raise self.request_failure
        self.stop_requested = True

    def stop(self, timeout_s: float = 10.0) -> dict:
        self.order.append("bag")
        if self.release is not None:
            self.release.wait()
        if self.failure is not None:
            raise self.failure
        return {"size_bytes": 10, "split_count": 1}

    def _summary(self) -> dict:
        return {"size_bytes": 0, "split_count": 0}

    def abort(self) -> None:
        self.abort_count += 1
        if self.release is not None:
            self.release.set()


class FakeCameraPool:
    def __init__(self, order: list[str], failures: list[dict[str, str]] | None = None):
        self.order = order
        self.failures = failures or []
        self.stop_requested = False
        self.abort_count = 0
        self.pending_cleanup = False
        self.cleanup_calls = 0

    def request_stop(self) -> list[dict[str, str]]:
        self.order.append("camera_request")
        self.stop_requested = True
        return []

    def apply_depth_frame_counts(self, counts: dict[str, int]) -> None:
        pass

    def finalize_all(self):
        self.order.append("camera")
        return ([{
            "name": "front",
            "topic": "/front/image",
            "frame_count": 3,
            "segments": [],
        }], self.failures)

    def abort_all(self) -> None:
        self.abort_count += 1

    def cleanup_pending(self) -> None:
        self.cleanup_calls += 1
        self.pending_cleanup = False

    def has_pending_cleanup(self) -> bool:
        return self.pending_cleanup


class FakeDepthPool:
    def __init__(self, order: list[str]):
        self.order = order

    def request_stop(self) -> dict[str, int]:
        self.order.append("depth_request")
        return {"depth": 2}

    def stop_all(self) -> dict[str, int]:
        self.order.append("depth")
        return {"depth": 2}

    def has_pending_cleanup(self) -> bool:
        return False


class FakeMarkers:
    def flush(self, episode_id: str) -> list[dict]:
        return [{"marker_id": "marker-1"}]


class FakeLock:
    def __init__(self):
        self.released = False

    def release(self) -> None:
        self.released = True


class FakeRequest:
    def __init__(self, app: dict, outcome: str = "success"):
        self.app = app
        self.match_info = {"episode_id": "episode-old"}
        self._body = {"outcome": outcome}

    async def json(self) -> dict:
        return self._body


def make_job(tmp_path: Path, *, bag_failure=None, camera_failures=None,
             fail_writes: int = 0, fail_discard: bool = False,
             release: threading.Event | None = None):
    order: list[str] = []
    meta = FakeMeta()
    store = FakeStore(
        meta,
        tmp_path,
        fail_writes=fail_writes,
        fail_discard=fail_discard,
    )
    bag = FakeBag(order, release=release, failure=bag_failure)
    camera = FakeCameraPool(order, failures=camera_failures)
    job = FinalizationJob(
        store=store,
        meta=meta,
        episode_dir=tmp_path,
        bag_recorder=bag,
        camera_pool=camera,
        depth_pool=None,
        marker_manager=FakeMarkers(),
        controller_at_end={"arm": {"source": "ai"}},
    )
    return job, store, bag, camera, order


class FinalizationTests(unittest.TestCase):
    def test_success_flushes_bag_then_camera_and_commits_finished(self):
        with TemporaryDirectory() as tmp:
            job, store, _, _, order = make_job(Path(tmp))
            finalize_episode(job)

        self.assertEqual(order, ["bag", "camera"])
        self.assertEqual(job.meta.state, "finished")
        self.assertEqual(job.meta.outcome, "success")
        self.assertEqual(job.meta.finalization_failures, [])
        self.assertEqual(store.written_states, ["finished"])

    def test_bag_camera_or_metadata_failure_commits_failed_abort_detail(self):
        camera_failure = {
            "component": "camera:front",
            "error_type": "RuntimeError",
            "detail": "encoder failed",
        }
        with TemporaryDirectory() as tmp:
            job, store, _, _, _ = make_job(
                Path(tmp), bag_failure=RuntimeError("bag failed"),
                camera_failures=[camera_failure], fail_writes=1,
            )
            finalize_episode(job)

        self.assertEqual(job.meta.state, "failed")
        self.assertEqual(job.meta.outcome, "abort")
        self.assertEqual(
            {failure["component"] for failure in job.meta.finalization_failures},
            {"metadata", "bag", "camera:front"},
        )
        self.assertEqual(store.written_states, ["failed"])

    def test_forced_timeout_does_not_overwrite_committed_success(self):
        with TemporaryDirectory() as tmp:
            job, _, _, _, _ = make_job(Path(tmp))
            finalize_episode(job)
            job.force_terminal_failure()

        self.assertEqual(job.meta.state, "finished")
        self.assertEqual(job.meta.outcome, "success")
        self.assertEqual(job.meta.finalization_failures, [])

    def test_discard_is_terminal_only_after_directory_deletion(self):
        with TemporaryDirectory() as tmp:
            job, store, _, _, _ = make_job(Path(tmp))
            job.meta.outcome = "discard"
            finalize_episode(job)

        self.assertTrue(store.discarded)
        self.assertTrue(job._terminal_committed)
        self.assertEqual(store.written_states, [])

    def test_discard_failure_is_persisted_as_failed_abort(self):
        with TemporaryDirectory() as tmp:
            job, store, _, _, _ = make_job(Path(tmp), fail_discard=True)
            job.meta.outcome = "discard"
            finalize_episode(job)

        self.assertFalse(store.discarded)
        self.assertEqual(job.meta.state, "failed")
        self.assertEqual(job.meta.outcome, "abort")
        self.assertEqual(job.meta.finalization_failures[0]["component"], "discard")
        self.assertEqual(store.written_states, ["failed"])

    def test_discard_index_failure_keeps_files_for_terminal_failure_metadata(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            store = EpisodeStore(root)
            store.start_episode(EpisodeMeta(
                episode_id="episode-discard-index-failure",
                task_description="discard contract",
                profile="test_profile",
                started_at="2026-07-13T00:00:00.000000Z",
            ))
            meta, episode_dir, index_error = store.begin_finalization("discard")
            self.assertIsNone(index_error)
            order: list[str] = []
            job = FinalizationJob(
                store=store,
                meta=meta,
                episode_dir=episode_dir,
                bag_recorder=FakeBag(order),
                camera_pool=FakeCameraPool(order),
                depth_pool=None,
                marker_manager=FakeMarkers(),
                controller_at_end=None,
            )

            with patch.object(
                store.index,
                "delete",
                side_effect=RuntimeError("index delete failed"),
            ):
                finalize_episode(job)

            stored, stored_dir = store.get_episode("episode-discard-index-failure")
            files_retained = stored_dir.exists()

        self.assertEqual(stored.state, "failed")
        self.assertEqual(stored.outcome, "abort")
        self.assertEqual(stored.finalization_failures[0]["component"], "discard")
        self.assertTrue(files_retained)

    def test_real_store_accepts_finalizer_metadata_and_refreshes_size(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            store = EpisodeStore(root)
            meta = EpisodeMeta(
                episode_id="episode-real-store",
                task_description="finalization contract",
                profile="test_profile",
                started_at="2026-07-13T00:00:00.000000Z",
            )
            episode_dir = store.start_episode(meta)
            video_dir = episode_dir / "videos" / "front"
            video_dir.mkdir()
            (video_dir / "0000.mp4").write_bytes(b"video")
            meta, episode_dir, index_error = store.begin_finalization("success")
            self.assertIsNone(index_error)
            order: list[str] = []
            job = FinalizationJob(
                store=store,
                meta=meta,
                episode_dir=episode_dir,
                bag_recorder=FakeBag(order),
                camera_pool=FakeCameraPool(order),
                depth_pool=None,
                marker_manager=FakeMarkers(),
                controller_at_end={"arm": {"source": "ai"}},
            )

            finalize_episode(job)

            stored, _episode_dir = store.get_episode("episode-real-store")
            rows, _cursor = store.index.list()

        self.assertEqual(stored.state, "finished")
        self.assertEqual(stored.outcome, "success")
        self.assertGreater(rows[0]["size_bytes"], 0)


class AsyncLifecycleTests(unittest.IsolatedAsyncioTestCase):
    async def test_stop_detaches_ownership_returns_202_and_new_episode_can_start(self):
        with TemporaryDirectory() as tmp:
            release = threading.Event()
            job, store, bag, camera, _ = make_job(Path(tmp), release=release)
            lock = FakeLock()
            app = {
                "store": store,
                "active_recording": ActiveRecording(
                    episode_id="episode-old",
                    bag_recorder=bag,
                    camera_pool=camera,
                    depth_pool=FakeDepthPool(job.bag_recorder.order),
                ),
                "active_lock": lock,
                "marker_manager": job.marker_manager,
                "mux_tracker": None,
                "finalization_jobs": {},
            }

            response = await _stop_episode(FakeRequest(app))
            payload = json.loads(response.body)
            finalizer_task = app["finalization_jobs"]["episode-old"][1]
            self.assertEqual(
                job.bag_recorder.order,
                ["camera_request", "depth_request", "bag_request"],
            )
            store.start_new()
            release.set()
            await finalizer_task

        self.assertEqual(response.status, 202)
        self.assertEqual(payload["state"], "finalizing")
        self.assertEqual(store.active.episode_id, "episode-new")
        self.assertTrue(lock.released)
        self.assertTrue(bag.stop_requested)
        self.assertTrue(camera.stop_requested)
        self.assertEqual(payload["finalization_pending"], True)
        self.assertEqual(
            job.bag_recorder.order,
            [
                "camera_request",
                "depth_request",
                "bag_request",
                "bag",
                "depth",
                "camera",
            ],
        )

    async def test_start_failure_returns_500_and_finalizes_failed(self):
        with TemporaryDirectory() as tmp:
            job, store, bag, camera, order = make_job(Path(tmp))
            lock = FakeLock()
            recording = ActiveRecording(
                episode_id="episode-old",
                bag_recorder=bag,
                camera_pool=camera,
                depth_pool=FakeDepthPool(order),
            )
            app = {
                "store": store,
                "active_recording": recording,
                "active_lock": lock,
                "marker_manager": job.marker_manager,
                "mux_tracker": None,
                "finalization_jobs": {},
            }

            response = _failed_start_response(
                app,
                recording,
                "camera",
                RuntimeError("encoder probe failed"),
            )
            payload = json.loads(response.body)
            finalizer_task = app["finalization_jobs"]["episode-old"][1]
            await finalizer_task

        self.assertEqual(response.status, 500)
        self.assertEqual(payload["error"], "episode_start_failed")
        self.assertIsNone(app["active_recording"])
        self.assertTrue(lock.released)
        self.assertEqual(job.meta.state, "failed")
        self.assertIn(
            "camera",
            [failure["component"] for failure in job.meta.finalization_failures],
        )

    async def test_terminal_job_retains_cleanup_ownership(self):
        with TemporaryDirectory() as tmp:
            job, _, _, camera, _ = make_job(Path(tmp))
            camera.pending_cleanup = True
            app = {"finalization_jobs": {}}
            task = asyncio.create_task(_run_finalizer(app, job))
            app["finalization_jobs"][job.meta.episode_id] = (job, task)

            await task

        self.assertIn(job.meta.episode_id, app["finalization_jobs"])
        self.assertTrue(job.has_pending_cleanup())

    async def test_quiesce_failure_releases_active_ownership_and_aborts(self):
        with TemporaryDirectory() as tmp:
            job, store, _bag, camera, order = make_job(Path(tmp))
            bag = FakeBag(order, request_failure=PermissionError("signal denied"))
            lock = FakeLock()
            app = {
                "store": store,
                "active_recording": ActiveRecording(
                    episode_id="episode-old",
                    bag_recorder=bag,
                    camera_pool=camera,
                    depth_pool=FakeDepthPool(order),
                ),
                "active_lock": lock,
                "marker_manager": job.marker_manager,
                "mux_tracker": None,
                "finalization_jobs": {},
            }

            response = await _stop_episode(FakeRequest(app))
            payload = json.loads(response.body)
            finalizer_task = app["finalization_jobs"]["episode-old"][1]
            await finalizer_task

        self.assertEqual(response.status, 500)
        self.assertEqual(payload["error"], "quiesce_failed")
        self.assertIsNone(app["active_recording"])
        self.assertTrue(lock.released)
        self.assertEqual(job.meta.state, "failed")
        self.assertEqual(job.meta.outcome, "abort")

    async def test_shutdown_retries_cleanup_for_job_completed_during_wait(self):
        with TemporaryDirectory() as tmp:
            release = threading.Event()
            job, _, _, camera, _ = make_job(Path(tmp), release=release)
            camera.pending_cleanup = True
            app = {"finalization_jobs": {}}
            task = asyncio.create_task(_run_finalizer(app, job))
            app["finalization_jobs"][job.meta.episode_id] = (job, task)
            asyncio.get_running_loop().call_later(0.01, release.set)

            await _shutdown_finalizers(app)

        self.assertTrue(task.done())
        self.assertEqual(camera.cleanup_calls, 1)
        self.assertFalse(camera.pending_cleanup)

    async def test_shutdown_timeout_forces_failure_without_touching_new_resources(self):
        with TemporaryDirectory() as tmp:
            old_job, _, old_bag, old_camera, _ = make_job(Path(tmp))
            new_order: list[str] = []
            new_bag = FakeBag(new_order)
            new_camera = FakeCameraPool(new_order)
            pending = asyncio.create_task(asyncio.Event().wait())

            await wait_for_finalizers([(old_job, pending)], timeout_s=0)

        self.assertTrue(pending.cancelled())
        self.assertEqual(old_job.meta.state, "failed")
        self.assertEqual(old_job.meta.outcome, "abort")
        self.assertEqual(old_bag.abort_count, 1)
        self.assertEqual(old_camera.abort_count, 1)
        self.assertEqual(new_bag.abort_count, 0)
        self.assertEqual(new_camera.abort_count, 0)
        self.assertEqual(
            old_job.meta.finalization_failures[0]["error_type"],
            "FinalizationTimeout",
        )

if __name__ == "__main__":
    unittest.main()
