"""aiohttp REST API for fv_episode_recorder.

Phase 1 Step 1: minimal endpoints only.
    POST /api/v1/episodes        — start
    POST /api/v1/episodes/{id}/stop  — stop
    GET  /api/v1/episodes        — list
    GET  /api/v1/episodes/{id}   — get one
    GET  /api/v1/healthz         — liveness

Other endpoints (markers, replay, export, disk, retention) land in later steps.
"""

from __future__ import annotations

import logging
import shutil
from dataclasses import asdict
from pathlib import Path
from typing import Any, Optional

from aiohttp import web

from .active_lock import ActiveLock
from .bag_recorder import BagRecorder
from .camera_writer import CameraWriterPool
from .episode_store import EpisodeMeta, EpisodeStore, new_episode_id, utc_now_iso
from .marker_manager import MarkerManager
from .preflight import run_preflight
from .schemas import (
    EpisodeSummary,
    ListEpisodesResponse,
    StartEpisodeRequest,
    StartEpisodeResponse,
    StopEpisodeRequest,
    StopEpisodeResponse,
)

LOG = logging.getLogger("fv_episode_recorder.api")


@web.middleware
async def _cors_middleware(request, handler):
    """Permissive CORS so the vlabor dashboard at :8888 can fetch from :8083.
    Same-LAN dev convenience; tighten for production deploys."""
    if request.method == "OPTIONS":
        return web.Response(
            status=204,
            headers={
                "Access-Control-Allow-Origin": "*",
                "Access-Control-Allow-Methods": "GET,POST,PATCH,DELETE,OPTIONS",
                "Access-Control-Allow-Headers": "Content-Type,Authorization",
                "Access-Control-Max-Age": "86400",
            },
        )
    resp = await handler(request)
    resp.headers["Access-Control-Allow-Origin"] = "*"
    return resp


def build_app(
    store: EpisodeStore,
    bag_recorder: BagRecorder,
    camera_pool: CameraWriterPool,
    active_lock: Optional[ActiveLock] = None,
    marker_manager: Optional[MarkerManager] = None,
    get_profile=None,  # callable: name -> dict; None = no profile lookup (override only)
) -> web.Application:
    app = web.Application(middlewares=[_cors_middleware])
    app["store"] = store
    app["bag_recorder"] = bag_recorder
    app["camera_pool"] = camera_pool
    app["active_lock"] = active_lock
    app["marker_manager"] = marker_manager or MarkerManager()
    app["get_profile"] = get_profile

    app.router.add_get("/api/v1/healthz", _healthz)
    app.router.add_post("/api/v1/episodes", _start_episode)
    app.router.add_post("/api/v1/episodes/{episode_id}/stop", _stop_episode)
    app.router.add_post("/api/v1/episodes/{episode_id}/recover", _recover_episode)
    app.router.add_get("/api/v1/episodes", _list_episodes)
    app.router.add_get("/api/v1/episodes/{episode_id}", _get_episode)
    app.router.add_get("/api/v1/episodes/{episode_id}/files/{tail:.*}", _episode_file)
    app.router.add_get("/api/v1/disk/status", _disk_status)

    # Phase 2: marker endpoints (subtask / event / note + post-hoc edits)
    app.router.add_get("/api/v1/episodes/{episode_id}/markers", _list_markers)
    app.router.add_post("/api/v1/episodes/{episode_id}/markers/start", _start_marker)
    app.router.add_post("/api/v1/markers/{marker_id}/stop", _stop_marker)
    app.router.add_patch("/api/v1/markers/{marker_id}", _patch_marker)
    app.router.add_delete("/api/v1/markers/{marker_id}", _delete_marker)

    return app


async def _healthz(request: web.Request) -> web.Response:
    store: EpisodeStore = request.app["store"]
    camera_pool: CameraWriterPool = request.app["camera_pool"]
    marker_manager: MarkerManager = request.app["marker_manager"]
    payload = {"status": "ok", "active_episode": None}
    if store.active is not None:
        meta = store.active
        active_markers = [
            {
                "marker_id": m.marker_id,
                "kind": m.kind,
                "task_description": m.task_description,
                "started_at": m.started_at,
            }
            for m in marker_manager.active(meta.episode_id)
        ]
        payload["active_episode"] = {
            "episode_id": meta.episode_id,
            "task_description": meta.task_description,
            "profile": meta.profile,
            "started_at": meta.started_at,
            "fps_per_camera": camera_pool.frame_counts(),
            "active_markers": active_markers,
        }
    return web.json_response(payload)


# ---------- Marker handlers ----------

async def _list_markers(request: web.Request) -> web.Response:
    episode_id = request.match_info["episode_id"]
    mm: MarkerManager = request.app["marker_manager"]
    store: EpisodeStore = request.app["store"]
    in_memory = [m.to_meta_dict() for m in mm.list(episode_id)]
    if in_memory:
        return web.json_response({"episode_id": episode_id, "markers": in_memory})
    # fall back to finalized episode meta
    found = store.get_episode(episode_id)
    if found is None:
        return web.json_response({"error": "not_found"}, status=404)
    meta, _ = found
    return web.json_response({"episode_id": episode_id, "markers": meta.markers})


async def _start_marker(request: web.Request) -> web.Response:
    episode_id = request.match_info["episode_id"]
    try:
        body = await request.json()
    except Exception:
        body = {}
    task_desc = (body.get("task_description") or "").strip()
    if not task_desc:
        return web.json_response({"error": "task_description_required"}, status=400)
    kind = body.get("kind") or body.get("marker_kind") or "subtask"
    tags = body.get("tags") or []
    store: EpisodeStore = request.app["store"]
    if store.active is None or store.active.episode_id != episode_id:
        return web.json_response(
            {"error": "episode_not_active",
             "detail": "markers can only be added to the active recording episode in Phase 2"},
            status=409,
        )
    mm: MarkerManager = request.app["marker_manager"]
    try:
        m = mm.start(episode_id, task_desc, kind=kind, tags=tags)
    except ValueError as exc:
        return web.json_response({"error": "invalid_kind", "detail": str(exc)}, status=400)
    LOG.info("marker started: %s episode=%s kind=%s task=%s",
             m.marker_id, episode_id, kind, task_desc)
    return web.json_response(m.to_meta_dict(), status=201)


async def _stop_marker(request: web.Request) -> web.Response:
    marker_id = request.match_info["marker_id"]
    try:
        body = await request.json()
    except Exception:
        body = {}
    outcome = body.get("outcome", "success")
    mm: MarkerManager = request.app["marker_manager"]
    m = mm.stop(marker_id, outcome=outcome)
    if m is None:
        return web.json_response({"error": "not_found"}, status=404)
    LOG.info("marker stopped: %s outcome=%s", marker_id, outcome)
    return web.json_response(m.to_meta_dict())


async def _patch_marker(request: web.Request) -> web.Response:
    marker_id = request.match_info["marker_id"]
    try:
        body = await request.json()
    except Exception:
        body = {}
    mm: MarkerManager = request.app["marker_manager"]
    m = mm.patch(marker_id, **body)
    if m is None:
        return web.json_response({"error": "not_found"}, status=404)
    return web.json_response(m.to_meta_dict())


async def _delete_marker(request: web.Request) -> web.Response:
    marker_id = request.match_info["marker_id"]
    mm: MarkerManager = request.app["marker_manager"]
    if not mm.delete(marker_id):
        return web.json_response({"error": "not_found"}, status=404)
    return web.json_response({"marker_id": marker_id, "deleted": True})


async def _start_episode(request: web.Request) -> web.Response:
    try:
        raw = await request.json()
        req = StartEpisodeRequest(**raw)
    except Exception as exc:
        return web.json_response({"error": "invalid_request", "detail": str(exc)}, status=400)

    store: EpisodeStore = request.app["store"]
    if store.active is not None:
        return web.json_response(
            {"error": "conflict", "detail": f"episode {store.active.episode_id} is already active"},
            status=409,
        )

    ep_id = new_episode_id()
    started_at = utc_now_iso()

    # Step 4: profile-driven topic + camera discovery (override > profile auto)
    from .topic_discovery import discover_cameras, discover_topics
    get_profile = request.app.get("get_profile")
    profile_dict = get_profile(req.profile) if callable(get_profile) else {}

    if req.record_topics_override:
        recorded_topics_meta = req.record_topics_override
        bag_topics = [t["topic"] if isinstance(t, dict) else t for t in req.record_topics_override]
        discovery_source = "request_override"
    elif req.record_bag_topics:
        recorded_topics_meta = [
            {"topic": t, "role": "unknown", "qos": "default", "stamp_source": "rosbag_recv"}
            for t in req.record_bag_topics
        ]
        bag_topics = list(req.record_bag_topics)
        discovery_source = "request_override"
    elif profile_dict:
        recorded_topics_meta = discover_topics(profile_dict)
        bag_topics = [d["topic"] for d in recorded_topics_meta]
        discovery_source = "profile"
    else:
        recorded_topics_meta = []
        bag_topics = []
        discovery_source = "none"

    if req.cameras_override is not None:
        cameras_resolved = req.cameras_override
    elif profile_dict:
        cameras_resolved = discover_cameras(profile_dict)
    else:
        cameras_resolved = []

    # Step 5: disk preflight (refuse with 503 if not enough room).
    preflight_cfg = (req.model_dump().get("preflight") or {}) if hasattr(req, "model_dump") else {}
    preflight = run_preflight(
        output_dir=store.output_dir,
        n_cameras=len(cameras_resolved),
        n_bag_topics=len(bag_topics),
        expected_duration_s=req.expected_duration_s,
        safety_margin_gb=float(preflight_cfg.get("safety_margin_gb", 5.0))
        if isinstance(preflight_cfg, dict) else 5.0,
        skip=bool(preflight_cfg.get("skip", False)) if isinstance(preflight_cfg, dict) else False,
    )
    if not preflight["passed"]:
        LOG.warning("episode rejected by preflight: %s", preflight.get("blockers"))
        return web.json_response(
            {"error": "disk_preflight_failed", "preflight": preflight,
             "code": "EPISODE_DISK_PREFLIGHT_FAILED"},
            status=503,
        )

    meta = EpisodeMeta(
        episode_id=ep_id,
        state="recording",
        task_description=req.task_description,
        profile=req.profile,
        robot_id=req.robot_id,
        operator=req.operator,
        tags=req.tags,
        parent_session_id=req.parent_session_id,
        expected_duration_s=req.expected_duration_s,
        started_at=started_at,
        env_config=req.env_config,
        recorded_topics=recorded_topics_meta,
        cameras=cameras_resolved,
        topic_discovery_source=discovery_source,
    )
    try:
        ep_dir = store.start_episode(meta)
    except RuntimeError as exc:
        return web.json_response({"error": "conflict", "detail": str(exc)}, status=409)

    # Step 6: acquire active lock
    active_lock: Optional[ActiveLock] = request.app.get("active_lock")
    if active_lock is not None:
        try:
            active_lock.acquire(ep_id, ep_dir)
        except Exception as exc:
            LOG.warning("active lock acquire failed: %s", exc)

    # Step 2: start bag recorder if topics provided and record_bag=true
    bag_recorder: BagRecorder = request.app["bag_recorder"]
    bag_started = False
    if req.record_bag and bag_topics:
        if not BagRecorder.available():
            LOG.warning("ros2 not available, skipping bag record")
        else:
            try:
                bag_recorder.start(ep_dir / "bag", bag_topics)
                bag_started = True
            except Exception as exc:
                LOG.error("bag recorder start failed: %s", exc)
                # Don't fail the whole episode; record meta and continue.

    # Step 3+4: start camera writers (resolved from override or profile)
    camera_pool: CameraWriterPool = request.app["camera_pool"]
    cameras_started: list[dict] = []
    if cameras_resolved:
        try:
            cameras_started = camera_pool.start_all(ep_dir, cameras_resolved, fps=req.fps)
            meta.cameras = cameras_started
            store._write_meta(ep_dir, meta)  # noqa: SLF001
        except Exception as exc:
            LOG.error("camera_pool start failed: %s", exc)

    resp = StartEpisodeResponse(
        episode_id=ep_id,
        started_at=started_at,
        bag_path=str((ep_dir / "bag").resolve()),
        meta_path=str((ep_dir / "meta.json").resolve()),
        recorded_topics_resolved=meta.recorded_topics,
        preflight={**preflight, "bag_started": bag_started,
                   "cameras_started": [c["name"] for c in cameras_started]},
    )
    LOG.info("episode started: %s task=%s profile=%s bag_topics=%d bag_started=%s",
             ep_id, req.task_description, req.profile, len(bag_topics), bag_started)
    return web.json_response(resp.model_dump(), status=201)


async def _stop_episode(request: web.Request) -> web.Response:
    episode_id = request.match_info["episode_id"]
    try:
        raw = await request.json()
        req = StopEpisodeRequest(**raw)
    except Exception as exc:
        return web.json_response({"error": "invalid_request", "detail": str(exc)}, status=400)

    store: EpisodeStore = request.app["store"]
    if store.active is None:
        return web.json_response({"error": "no_active_episode"}, status=409)
    if store.active.episode_id != episode_id:
        return web.json_response(
            {"error": "episode_mismatch",
             "detail": f"active is {store.active.episode_id}, requested {episode_id}"},
            status=409,
        )

    # Step 2: stop bag recorder first so metadata.yaml is flushed before meta write
    bag_recorder: BagRecorder = request.app["bag_recorder"]
    bag_summary: dict = {"size_bytes": 0, "split_count": 0}
    if bag_recorder.active:
        bag_summary = bag_recorder.stop(timeout_s=10.0)
    elif req.outcome == "discard":
        bag_recorder.abort()

    # Step 3: stop camera writers, gather frame_count summaries
    camera_pool: CameraWriterPool = request.app["camera_pool"]
    camera_summaries: list[dict] = []
    frame_count_per_camera: dict[str, int] = {}
    video_size_bytes = 0
    if camera_pool.is_active():
        camera_summaries = camera_pool.stop_all()
        for cs in camera_summaries:
            frame_count_per_camera[cs["name"]] = cs["frame_count"]
            for seg in cs.get("segments", []):
                video_size_bytes += int(seg.get("size_bytes", 0))

    # Flush markers (Phase 2) from in-memory manager into meta.json
    mm: MarkerManager = request.app["marker_manager"]
    pending_markers = mm.flush(episode_id)

    meta, ep_dir = store.stop_active(req.outcome)
    # Update meta with bag size + split_count + camera summaries + markers
    meta.bag_split_count = int(bag_summary.get("split_count", 0))
    if camera_summaries:
        meta.cameras = camera_summaries
    if pending_markers:
        meta.markers = pending_markers
    store._write_meta(ep_dir, meta)  # noqa: SLF001 — local helper, store is single owner

    # If discard, blow away the dir entirely
    if req.outcome == "discard":
        shutil.rmtree(ep_dir, ignore_errors=True)
        LOG.info("episode discarded and dir removed: %s", episode_id)

    # Step 6: release active lock
    active_lock: Optional[ActiveLock] = request.app.get("active_lock")
    if active_lock is not None:
        active_lock.release()

    resp = StopEpisodeResponse(
        episode_id=meta.episode_id,
        state=meta.state,
        duration_s=meta.duration_s or 0.0,
        frame_count_per_camera=frame_count_per_camera,
        bag_size_bytes=int(bag_summary.get("size_bytes", 0)),
        video_size_bytes=video_size_bytes,
        manifest_pending=True,
    )
    LOG.info(
        "episode stopped: %s outcome=%s duration=%.1fs bag=%d video=%d frames=%s",
        episode_id, req.outcome, resp.duration_s,
        resp.bag_size_bytes, resp.video_size_bytes, frame_count_per_camera,
    )
    return web.json_response(resp.model_dump(), status=200)


async def _list_episodes(request: web.Request) -> web.Response:
    store: EpisodeStore = request.app["store"]
    try:
        limit = int(request.query.get("limit", "50"))
    except ValueError:
        limit = 50
    limit = max(1, min(limit, 500))

    summaries: list[dict[str, Any]] = []
    for meta in store.list_episodes(limit=limit):
        summaries.append(EpisodeSummary(
            episode_id=meta.episode_id,
            state=meta.state,
            task_description=meta.task_description,
            profile=meta.profile,
            robot_id=meta.robot_id,
            started_at=meta.started_at,
            stopped_at=meta.stopped_at,
            duration_s=meta.duration_s,
            outcome=meta.outcome,
            pinned=meta.pinned,
            marker_count=len(meta.markers),
            tags=meta.tags,
            source=meta.source,
        ).model_dump())
    return web.json_response(ListEpisodesResponse(episodes=summaries).model_dump())


async def _get_episode(request: web.Request) -> web.Response:
    episode_id = request.match_info["episode_id"]
    store: EpisodeStore = request.app["store"]
    found = store.get_episode(episode_id)
    if found is None:
        return web.json_response({"error": "not_found"}, status=404)
    meta, _ep_dir = found
    return web.json_response(asdict(meta))


async def _disk_status(request: web.Request) -> web.Response:
    """Disk + retention status snapshot (Phase 1.5 minimal).

    Returns: current usage, recent write rate, estimated "hours of recording
    left at current rate", episode count + total bytes.
    """
    import time
    store: EpisodeStore = request.app["store"]
    output_dir = store.output_dir
    usage = shutil.disk_usage(output_dir)
    percent_used = round(100.0 * usage.used / usage.total, 1) if usage.total else 0.0
    percent_free = round(100.0 * usage.free / usage.total, 1) if usage.total else 0.0

    # Episode aggregate (cheap walk; Phase 2 will switch to sqlite index).
    episode_count = 0
    total_episode_bytes = 0
    profile_counts: dict[str, int] = {}
    episodes_root = output_dir / "episodes"
    if episodes_root.exists():
        for ep_dir in episodes_root.glob("*/*/*"):
            if not ep_dir.is_dir():
                continue
            episode_count += 1
            profile = ep_dir.parts[-3]
            profile_counts[profile] = profile_counts.get(profile, 0) + 1
            try:
                for f in ep_dir.rglob("*"):
                    if f.is_file():
                        total_episode_bytes += f.stat().st_size
            except OSError:
                continue

    # Active episode write rate estimate — walk store.active_dir (the recorder
    # already knows where it is; folder name format is no longer ULID-only).
    active_rate_bytes_per_s = 0.0
    active_elapsed_s = 0.0
    if store.active is not None and store.active_dir is not None:
        meta = store.active
        from datetime import datetime, timezone
        started = datetime.strptime(meta.started_at, "%Y-%m-%dT%H:%M:%S.%fZ").replace(tzinfo=timezone.utc)
        active_elapsed_s = max((datetime.now(timezone.utc) - started).total_seconds(), 1.0)
        total_active_bytes = 0
        for f in store.active_dir.rglob("*"):
            if f.is_file():
                try:
                    total_active_bytes += f.stat().st_size
                except OSError:
                    pass
        active_rate_bytes_per_s = total_active_bytes / active_elapsed_s

    # Hours of recording left at current rate (only meaningful if recording).
    hours_left = None
    if active_rate_bytes_per_s > 0:
        hours_left = round(usage.free / (active_rate_bytes_per_s * 3600), 2)

    return web.json_response({
        "path": str(output_dir),
        "bytes_used": usage.used,
        "bytes_free": usage.free,
        "bytes_total": usage.total,
        "percent_used": percent_used,
        "percent_free": percent_free,
        "episode_count": episode_count,
        "episode_count_by_profile": profile_counts,
        "total_episode_bytes": total_episode_bytes,
        "active": {
            "rate_bytes_per_s": int(active_rate_bytes_per_s),
            "elapsed_s": round(active_elapsed_s, 1),
            "hours_left_at_current_rate": hours_left,
        } if store.active else None,
        "policy": {
            "warn_pct_free": 20.0,
            "crit_pct_free": 10.0,
            "note": "Phase 1.5: thresholds hardcoded; Phase 2 will read from profile.episode_recorder.retention",
        },
    })


async def _episode_file(request: web.Request) -> web.Response:
    """Serve a file from inside an episode directory (mp4, parquet, bag, etc).

    URL: /api/v1/episodes/{episode_id}/files/{tail}
    e.g. /api/v1/episodes/01KSX.../files/videos/top_camera/0000.mp4

    Path traversal protected — resolved path must stay under the episode dir.
    """
    store: EpisodeStore = request.app["store"]
    episode_id = request.match_info["episode_id"]
    tail = request.match_info["tail"]
    found = store.get_episode(episode_id)
    if found is None:
        return web.json_response({"error": "not_found"}, status=404)
    _meta, ep_dir = found
    path = (ep_dir / tail).resolve()
    ep_dir_resolved = ep_dir.resolve()
    try:
        path.relative_to(ep_dir_resolved)
    except ValueError:
        return web.json_response({"error": "forbidden", "detail": "path escape"}, status=403)
    if not path.exists() or not path.is_file():
        return web.json_response({"error": "not_found", "path": str(path.relative_to(ep_dir_resolved))}, status=404)
    return web.FileResponse(path)


async def _recover_episode(request: web.Request) -> web.Response:
    """Operator action for orphaned episodes (recorder crash mid-record).

    body: {action: "finalize_aborted" | "discard"}
      finalize_aborted: keep files, mark state=failed/outcome=abort, release lock
      discard:          rmtree the episode dir, release lock
    """
    episode_id = request.match_info["episode_id"]
    try:
        raw = await request.json()
    except Exception:
        raw = {}
    action = (raw.get("action") or "").strip()
    if action not in ("finalize_aborted", "discard"):
        return web.json_response(
            {"error": "invalid_action",
             "detail": "action must be 'finalize_aborted' or 'discard'"},
            status=400,
        )

    store: EpisodeStore = request.app["store"]
    active_lock: Optional[ActiveLock] = request.app.get("active_lock")
    found = store.get_episode(episode_id)
    if found is None:
        return web.json_response({"error": "not_found"}, status=404)
    meta, ep_dir = found

    if action == "discard":
        shutil.rmtree(ep_dir, ignore_errors=True)
        result = {"episode_id": episode_id, "state": "discarded", "files_removed": True}
    else:  # finalize_aborted
        meta.state = "failed"
        meta.outcome = "abort"
        if meta.stopped_at is None:
            meta.stopped_at = utc_now_iso()
        store._write_meta(ep_dir, meta)  # noqa: SLF001
        result = {"episode_id": episode_id, "state": meta.state, "outcome": meta.outcome}

    if active_lock is not None:
        # Only release the lock if it actually refers to this episode_id.
        lock_data = active_lock.read()
        if lock_data and lock_data.get("episode_id") == episode_id:
            active_lock.release()
            result["lock_released"] = True

    LOG.info("episode recovered: %s action=%s", episode_id, action)
    return web.json_response(result, status=200)
