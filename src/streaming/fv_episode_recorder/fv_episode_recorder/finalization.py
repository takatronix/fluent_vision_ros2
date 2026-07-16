"""Per-episode background finalization."""

from __future__ import annotations

import asyncio
import logging
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Callable, Optional

if TYPE_CHECKING:
    from .bag_recorder import BagRecorder
    from .camera_writer import CameraWriterPool
    from .depth_republisher import DepthRepublisherPool
    from .episode_store import EpisodeMeta, EpisodeStore, JsonObject
    from .marker_manager import MarkerManager


LOG = logging.getLogger("fv_episode_recorder.finalization")


@dataclass
class ActiveRecording:
    episode_id: str
    bag_recorder: "BagRecorder"
    camera_pool: "CameraWriterPool"
    depth_pool: Optional["DepthRepublisherPool"]


@dataclass
class FinalizationJob:
    store: "EpisodeStore"
    meta: "EpisodeMeta"
    episode_dir: Path
    bag_recorder: "BagRecorder"
    camera_pool: "CameraWriterPool"
    depth_pool: Optional["DepthRepublisherPool"]
    marker_manager: "MarkerManager"
    controller_at_end: Optional[dict[str, "JsonObject"]]
    failures: list[dict[str, str]] = field(default_factory=list)
    forced_failure: threading.Event = field(default_factory=threading.Event)
    _terminal_lock: threading.Lock = field(default_factory=threading.Lock)
    _terminal_committed: bool = False

    def has_pending_cleanup(self) -> bool:
        return (
            self.camera_pool.has_pending_cleanup()
            or (
                self.depth_pool is not None
                and self.depth_pool.has_pending_cleanup()
            )
        )

    def cleanup_pending_resources(self) -> None:
        self.camera_pool.cleanup_pending()
        if self.depth_pool is not None and self.depth_pool.has_pending_cleanup():
            self.depth_pool.stop_all()

    def add_failure(self, component: str, exc: BaseException) -> None:
        self.failures.append({
            "component": component,
            "error_type": type(exc).__name__,
            "detail": str(exc),
        })

    def force_terminal_failure(self) -> None:
        timeout_failure = {
            "component": "shutdown",
            "error_type": "FinalizationTimeout",
            "detail": "finalization exceeded the shutdown timeout",
        }
        with self._terminal_lock:
            if self._terminal_committed:
                return
            self.forced_failure.set()
            self.failures.append(timeout_failure)
            try:
                self._commit_terminal()
            except Exception as commit_exc:
                self.failures.append({
                    "component": "metadata",
                    "error_type": type(commit_exc).__name__,
                    "detail": str(commit_exc),
                })
                self.meta.state = "failed"
                self.meta.outcome = "abort"
                self.meta.finalization_failures = list(self.failures)
                try:
                    self.store._write_meta(  # noqa: SLF001
                        self.episode_dir,
                        self.meta,
                        refresh_size=True,
                    )
                    self._terminal_committed = True
                except Exception:
                    LOG.exception(
                        "timeout terminal metadata commit failed: %s",
                        self.meta.episode_id,
                    )
        try:
            self.bag_recorder.abort()
        except Exception:
            LOG.exception("timeout bag cleanup failed: %s", self.meta.episode_id)
        try:
            self.camera_pool.abort_all()
        except Exception:
            LOG.exception("timeout camera cleanup failed: %s", self.meta.episode_id)
        if self.depth_pool is not None:
            try:
                self.depth_pool.stop_all()
            except Exception:
                LOG.exception("timeout depth cleanup failed: %s", self.meta.episode_id)

    def fail_terminal(
        self, component: str, exc: BaseException, error_type: Optional[str] = None,
    ) -> None:
        with self._terminal_lock:
            if self._terminal_committed:
                return
            self.failures.append({
                "component": component,
                "error_type": error_type or type(exc).__name__,
                "detail": str(exc),
            })
            try:
                self._commit_terminal()
            except Exception as commit_exc:
                self.add_failure("metadata", commit_exc)
                self.meta.state = "failed"
                self.meta.outcome = "abort"
                self.meta.finalization_failures = list(self.failures)
                try:
                    self.store._write_meta(  # noqa: SLF001
                        self.episode_dir,
                        self.meta,
                        refresh_size=True,
                    )
                    self._terminal_committed = True
                except Exception:
                    LOG.exception("terminal metadata commit failed: %s", self.meta.episode_id)

    def _commit_terminal(self) -> None:
        if self.failures or self.meta.outcome == "abort" or self.forced_failure.is_set():
            self.meta.state = "failed"
            self.meta.outcome = "abort"
        elif self.meta.outcome == "discard":
            self.meta.state = "discarded"
        else:
            self.meta.state = "finished"
        self.meta.finalization_failures = list(self.failures)
        self.store._write_meta(  # noqa: SLF001
            self.episode_dir,
            self.meta,
            refresh_size=True,
        )
        self._terminal_committed = True


def finalize_episode(job: FinalizationJob) -> None:
    bag_summary: dict[str, str | int | None | list[str]] = {
        "size_bytes": 0,
        "split_count": 0,
    }
    if job.bag_recorder.started:
        try:
            bag_summary = job.bag_recorder.stop(timeout_s=10.0)
        except Exception as exc:
            job.add_failure("bag", exc)
            bag_summary = job.bag_recorder._summary()  # noqa: SLF001

    if job.depth_pool is not None:
        try:
            depth_frame_counts = job.depth_pool.stop_all()
            if depth_frame_counts:
                job.camera_pool.apply_depth_frame_counts(depth_frame_counts)
        except Exception as exc:
            job.add_failure("depth", exc)

    camera_summaries, camera_failures = job.camera_pool.finalize_all()
    job.failures.extend(camera_failures)

    try:
        pending_markers = job.marker_manager.flush(job.meta.episode_id)
    except Exception as exc:
        job.add_failure("metadata", exc)
        pending_markers = []

    job.meta.bag_split_count = int(bag_summary.get("split_count", 0))
    if camera_summaries:
        job.meta.cameras = camera_summaries
    if pending_markers:
        job.meta.markers = pending_markers
    job.meta.controller_at_end = job.controller_at_end

    if job.meta.outcome == "discard":
        try:
            job.store.discard_finalizing_episode(job.meta.episode_id)
        except Exception as exc:
            job.add_failure("discard", exc)
        else:
            with job._terminal_lock:
                job._terminal_committed = True
            return

    if not job.failures and job.meta.outcome == "success":
        try:
            job.store.protect_finished_payload_sources(job.episode_dir)
        except Exception as exc:
            job.add_failure("payload_permissions", exc)

    with job._terminal_lock:
        if not job._terminal_committed:
            try:
                job._commit_terminal()
            except Exception as exc:
                job.add_failure("metadata", exc)
                job.meta.state = "failed"
                job.meta.outcome = "abort"
                job.meta.finalization_failures = list(job.failures)
                try:
                    job.store._write_meta(  # noqa: SLF001
                        job.episode_dir,
                        job.meta,
                        refresh_size=True,
                    )
                    job._terminal_committed = True
                except Exception:
                    LOG.exception("terminal metadata commit failed: %s", job.meta.episode_id)

async def wait_for_finalizers(
    jobs: list[tuple[FinalizationJob, asyncio.Task[None]]], timeout_s: float,
) -> set[str]:
    if not jobs:
        return set()
    tasks = [task for _, task in jobs]
    _, pending = await asyncio.wait(tasks, timeout=timeout_s)
    if not pending:
        return set()
    timed_out_ids = {
        job.meta.episode_id
        for job, task in jobs
        if task in pending
    }
    force_tasks = [
        asyncio.create_task(run_in_daemon(job.force_terminal_failure))
        for job, task in jobs if task in pending
    ]
    _, pending_force = await asyncio.wait(force_tasks, timeout=5.0)
    if pending_force:
        LOG.error("forced terminal commit exceeded shutdown timeout")
    for task in pending_force:
        task.cancel()
    for task in pending:
        task.cancel()
    await asyncio.gather(*force_tasks, *pending, return_exceptions=True)
    return timed_out_ids


async def run_in_daemon(function: Callable[[], None]) -> None:
    loop = asyncio.get_running_loop()
    done = loop.create_future()

    def set_result() -> None:
        if not done.done():
            done.set_result(None)

    def set_exception(exc: BaseException) -> None:
        if not done.done():
            done.set_exception(exc)

    def run() -> None:
        try:
            function()
        except BaseException as exc:
            try:
                loop.call_soon_threadsafe(set_exception, exc)
            except RuntimeError:
                pass
        else:
            try:
                loop.call_soon_threadsafe(set_result)
            except RuntimeError:
                pass

    threading.Thread(target=run, daemon=True).start()
    await done
