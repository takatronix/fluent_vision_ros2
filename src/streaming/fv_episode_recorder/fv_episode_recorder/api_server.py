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
from .batch_runner import BatchConfig, BatchRunner
from .replay_runner import ReplayRunner
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
    mux_tracker=None,  # MuxTracker — used to label each ep VLA vs teleop
    retention_runner=None,  # RetentionRunner — exposes plan/tick via /retention/*
    get_profile=None,  # callable: name -> dict; None = no profile lookup (override only)
) -> web.Application:
    app = web.Application(middlewares=[_cors_middleware])
    app["store"] = store
    app["mux_tracker"] = mux_tracker
    app["batch_runner"] = BatchRunner()
    app["retention_runner"] = retention_runner
    app["replay_runner"] = ReplayRunner()
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
    app.router.add_delete("/api/v1/episodes/{episode_id}", _delete_episode)
    app.router.add_patch("/api/v1/episodes/{episode_id}", _patch_episode)
    app.router.add_get("/api/v1/episodes/{episode_id}/files/{tail:.*}", _episode_file)
    app.router.add_get("/api/v1/episodes/{episode_id}/depth_preview/{camera}/{frame}", _depth_preview)
    app.router.add_get("/api/v1/episodes/{episode_id}/joints", _episode_joints)
    app.router.add_get("/api/v1/episodes/{episode_id}/joints.csv", _episode_joints_csv)
    # Batch loop (LeRobot-style auto-cycle) so the operator clicks once for 100 runs.
    app.router.add_post("/api/v1/episodes/batch_start", _batch_start)
    app.router.add_get("/api/v1/episodes/batch_status", _batch_status)
    app.router.add_post("/api/v1/episodes/batch_stop", _batch_stop)
    app.router.add_post("/api/v1/episodes/batch_retry", _batch_retry)
    app.router.add_post("/api/v1/episodes/batch_skip", _batch_skip)
    # Retention — read current plan, run a manual tick, or update the policy
    # (operator-facing settings live in the dashboard popover, persisted via PUT).
    app.router.add_get("/api/v1/retention/status", _retention_status)
    app.router.add_post("/api/v1/retention/run", _retention_run)
    app.router.add_get("/api/v1/retention/policy", _retention_get_policy)
    app.router.add_put("/api/v1/retention/policy", _retention_put_policy)
    # Replay — Phase 3b/3c minimum (sim only, no safety gate yet)
    app.router.add_post("/api/v1/episodes/{episode_id}/replay", _replay_start)
    app.router.add_get("/api/v1/replay/status", _replay_status)
    app.router.add_post("/api/v1/replay/stop", _replay_stop)
    app.router.add_get("/api/v1/episodes/{episode_id}/download.tar", _episode_download_tar)
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
    # Include batch_runner state so the dashboard's WS recorder_status
    # carries both episode + batch info — no second poll required, no race
    # between WS recorder_status and the separate batch_status polling.
    runner = request.app.get("batch_runner")
    if runner is not None and runner.state is not None:
        payload["batch"] = runner.state.to_dict()
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
    if kind not in ("subtask", "event", "note"):
        return web.json_response({"error": "invalid_kind", "detail": f"kind must be subtask/event/note, got {kind}"}, status=400)
    tags = body.get("tags") or []
    store: EpisodeStore = request.app["store"]

    # Active-episode path → live marker (real-time start).
    if store.active is not None and store.active.episode_id == episode_id:
        mm: MarkerManager = request.app["marker_manager"]
        try:
            m = mm.start(episode_id, task_desc, kind=kind, tags=tags)
        except ValueError as exc:
            return web.json_response({"error": "invalid_kind", "detail": str(exc)}, status=400)
        LOG.info("marker started: %s episode=%s kind=%s task=%s",
                 m.marker_id, episode_id, kind, task_desc)
        return web.json_response(m.to_meta_dict(), status=201)

    # Finalized-episode path (Phase 4 review workflow): the client supplies
    # explicit started_at (and stopped_at for subtask) computed from the
    # joint-chart / video-timeline click position.
    started_at = body.get("started_at")
    stopped_at = body.get("stopped_at")
    if not started_at:
        return web.json_response(
            {"error": "started_at_required",
             "detail": "for finalized episodes the client must supply started_at (ISO 8601 UTC)"},
            status=400,
        )
    if kind == "subtask" and not stopped_at:
        return web.json_response(
            {"error": "stopped_at_required",
             "detail": "subtask markers on finalized episodes need stopped_at"},
            status=400,
        )
    from ulid import ULID
    marker = {
        "marker_id": str(ULID()),
        "kind": kind,
        "task_description": task_desc,
        "tags": tags,
        "started_at": started_at,
        "stopped_at": stopped_at if kind == "subtask" else None,
        "outcome": body.get("outcome"),
        "rev": 1,
    }
    saved = store.add_finalized_marker(episode_id, marker)
    if saved is None:
        return web.json_response({"error": "episode_not_found"}, status=404)
    LOG.info("marker added (finalized): %s episode=%s kind=%s task=%s",
             marker["marker_id"], episode_id, kind, task_desc)
    return web.json_response(saved, status=201)


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
    if m is not None:
        return web.json_response(m.to_meta_dict())
    # Not in the active episode's in-memory state — search finalized meta.json
    # files. This is what the timeline-drag editor in fv_episode_ui hits when
    # editing markers on past episodes.
    store: EpisodeStore = request.app["store"]
    updated = store.patch_finalized_marker(marker_id, body)
    if updated is not None:
        return web.json_response(updated)
    return web.json_response({"error": "not_found"}, status=404)


async def _delete_marker(request: web.Request) -> web.Response:
    marker_id = request.match_info["marker_id"]
    mm: MarkerManager = request.app["marker_manager"]
    if mm.delete(marker_id):
        return web.json_response({"marker_id": marker_id, "deleted": True})
    # Fall back to meta.json rewrite for finalized episodes.
    store: EpisodeStore = request.app["store"]
    if store.delete_finalized_marker(marker_id):
        return web.json_response({"marker_id": marker_id, "deleted": True})
    return web.json_response({"error": "not_found"}, status=404)


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

    # Snapshot mux state so the recording is labeled VLA vs teleop without
    # having to decode the bag.
    mux_tracker = request.app.get("mux_tracker")
    controller_at_start = mux_tracker.snapshot() if mux_tracker is not None else None

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
        controller_at_start=controller_at_start,
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
    # Snapshot mux state at stop — used by the UI to tell operator-after-
    # the-fact whether THIS run was VLA (with which controller) or teleop.
    mux_tracker = request.app.get("mux_tracker")
    if mux_tracker is not None:
        meta.controller_at_end = mux_tracker.snapshot()
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
    """sqlite-indexed list with cursor pagination + filters.
    Query params:
      limit       (default 50, max 500)
      cursor      pass back next_cursor from previous response
      profile     filter by profile name
      batch_id    filter by batch tag value
      pinned_only "true" to filter
      env         "sim" or "real"
    """
    store: EpisodeStore = request.app["store"]
    try:
        limit = int(request.query.get("limit", "50"))
    except ValueError:
        limit = 50
    limit = max(1, min(limit, 500))

    rows, next_cursor, total = store.list_episodes_indexed(
        limit=limit,
        cursor=request.query.get("cursor") or None,
        profile=request.query.get("profile") or None,
        batch_id=request.query.get("batch_id") or None,
        pinned_only=(request.query.get("pinned_only", "").lower() == "true"),
        env=request.query.get("env") or None,
    )
    summaries = [
        EpisodeSummary(
            episode_id=r["episode_id"],
            state=r["state"],
            task_description=r["task_description"],
            profile=r["profile"],
            robot_id=r.get("robot_id"),
            started_at=r["started_at"],
            stopped_at=r.get("stopped_at"),
            duration_s=r.get("duration_s"),
            outcome=r.get("outcome"),
            pinned=bool(r.get("pinned")),
            size_bytes=int(r.get("size_bytes") or 0),
            marker_count=int(r.get("marker_count") or 0),
            tags=r.get("tags") or [],
            source=r.get("source") or "local",
            env=r.get("env") or "real",
            controller_label=r.get("controller_label"),
        ).model_dump()
        for r in rows
    ]
    return web.json_response({
        "episodes": summaries,
        "next_cursor": next_cursor,
        "total": total,
    })


def _summarize_controller(snap: Optional[dict[str, Any]]) -> Optional[str]:
    """Pick one short label from a mux snapshot. Prefers the first arm
    found; multi-arm rigs may want per-arm rendering in the future."""
    if not snap:
        return None
    for _topic, d in snap.items():
        if not isinstance(d, dict):
            continue
        src = (d.get("source") or "").strip()
        if src in ("ai", "ai_route"):
            ctl = (d.get("ai_controller") or "ai").strip() or "ai"
            return f"VLA: {ctl}"
        if src == "leader":
            return "teleop"
        if src == "none":
            return "停止"
        if src:
            return src
    return None


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


async def _episode_joints(request: web.Request) -> web.Response:
    """Extract sensor_msgs/JointState time series from the episode's bag.

    Phase 3a replay-preview support: the play modal renders a synced line
    chart so the operator can see the joints alongside the video. The bag
    is opened with rosbag2_py and only JointState messages are decoded —
    everything else (tf, mux, images) is skipped to keep this fast.

    Output: {
        episode_id, started_at, duration_s,
        joints: [{topic, names: [str], t_ms: [int], q: [[float, ...]]}, ...]
    }
    Timestamps are relative to the episode's started_at (in milliseconds)
    so the frontend can map them straight onto the video timeline."""
    from datetime import datetime, timezone
    store: EpisodeStore = request.app["store"]
    episode_id = request.match_info["episode_id"]
    found = store.get_episode(episode_id)
    if found is None:
        return web.json_response({"error": "not_found"}, status=404)
    meta, ep_dir = found
    bag_dir = ep_dir / "bag"
    if not bag_dir.exists():
        return web.json_response({"episode_id": episode_id, "joints": [], "warning": "no bag"})

    # Filter: query param `topic=` restricts to one topic (default = all
    # JointState topics found in the bag).
    only_topic = request.query.get("topic")

    def _decode_bag() -> dict:
        # Runs in a thread (executor) to avoid blocking the aiohttp loop —
        # bag decode of a 1 hour episode can take a few hundred ms.
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )
        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(storage_options, converter_options)
        except Exception as exc:
            return {"error": "bag_open_failed", "detail": str(exc)}

        topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        joint_topics = {n: tp for n, tp in topic_types.items()
                        if tp == "sensor_msgs/msg/JointState"
                        and (only_topic is None or n == only_topic)}
        if not joint_topics:
            return {"joints": [], "warning": "no JointState topics in bag",
                    "all_topics": list(topic_types.keys())}

        # Episode start as nanoseconds (UTC) — used to compute relative offsets.
        ep_start_ns = int(datetime.strptime(
            meta.started_at, "%Y-%m-%dT%H:%M:%S.%fZ"
        ).replace(tzinfo=timezone.utc).timestamp() * 1e9)

        per_topic: dict[str, dict] = {}
        msg_cls_cache: dict[str, type] = {}
        while reader.has_next():
            topic, raw, t_ns = reader.read_next()
            if topic not in joint_topics:
                continue
            cls = msg_cls_cache.get(topic)
            if cls is None:
                cls = get_message(joint_topics[topic])
                msg_cls_cache[topic] = cls
            msg = deserialize_message(raw, cls)
            entry = per_topic.setdefault(topic, {"topic": topic, "names": list(msg.name or []),
                                                 "t_ms": [], "q": []})
            # Use bag receive time for the X axis — joint_state stamps are
            # usually identical to receive time and this saves a stamp-vs-recv
            # decision the caller doesn't care about.
            t_ms = (t_ns - ep_start_ns) // 1_000_000
            entry["t_ms"].append(int(t_ms))
            entry["q"].append([float(x) for x in msg.position])
        return {"joints": list(per_topic.values())}

    import asyncio
    loop = asyncio.get_running_loop()
    try:
        result = await loop.run_in_executor(None, _decode_bag)
    except Exception as exc:
        return web.json_response({"error": "decode_failed", "detail": str(exc)}, status=500)
    result["episode_id"] = episode_id
    result["started_at"] = meta.started_at
    result["duration_s"] = meta.duration_s
    return web.json_response(result)


async def _episode_download_tar(request: web.Request) -> web.StreamResponse:
    """Stream the whole episode dir as an uncompressed tar.

    Use case: teleop-vs-VLA jerkiness investigation — operator wants the bag
    + mp4s on a laptop for offline pandas / rosbag2 analysis. Streams via
    `tar -cf -` so we never buffer the whole thing in memory."""
    import asyncio
    store: EpisodeStore = request.app["store"]
    episode_id = request.match_info["episode_id"]
    found = store.get_episode(episode_id)
    if found is None:
        return web.json_response({"error": "not_found"}, status=404)
    meta, ep_dir = found
    safe_name = (meta.task_description or "episode").replace("/", "_").replace(" ", "_")[:40]
    fname = f"{safe_name}_{episode_id[-8:]}.tar"
    response = web.StreamResponse(
        status=200,
        reason="OK",
        headers={
            "Content-Type": "application/x-tar",
            "Content-Disposition": f'attachment; filename="{fname}"',
            "X-Episode-Id": episode_id,
        },
    )
    await response.prepare(request)
    # Use -C parent so the tar's top-level entry is the episode folder name.
    proc = await asyncio.create_subprocess_exec(
        "tar", "-cf", "-", "-C", str(ep_dir.parent), ep_dir.name,
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
    )
    try:
        assert proc.stdout is not None
        while True:
            chunk = await proc.stdout.read(64 * 1024)
            if not chunk:
                break
            await response.write(chunk)
    finally:
        try:
            await proc.wait()
        except Exception:
            pass
    await response.write_eof()
    return response


async def _episode_joints_csv(request: web.Request) -> web.Response:
    """Convenience CSV export of joint angles for one topic.

    Same data as /joints (JSON) but flat CSV — pandas-friendly:
        t_s, <joint_name_0>, <joint_name_1>, ...
    Angles are converted from radians → degrees so the CSV matches the
    chart displayed in the play modal."""
    # Reuse the JSON endpoint's decoder.
    json_resp = await _episode_joints(request)
    if json_resp.status != 200:
        return json_resp
    import json as _json
    data = _json.loads(json_resp.body)
    only_topic = request.query.get("topic")
    series_list = data.get("joints", [])
    if only_topic:
        series_list = [s for s in series_list if s["topic"] == only_topic]
    if not series_list:
        return web.Response(status=404, text="no joint topic")
    s = series_list[0]
    import math
    lines = []
    header = ["t_s"] + (s["names"] or [f"j{i}" for i in range(len(s["q"][0]) if s["q"] else 0)])
    lines.append(",".join(header))
    for t_ms, row in zip(s["t_ms"], s["q"]):
        lines.append(",".join([f"{t_ms/1000.0:.3f}"] + [f"{v*180/math.pi:.4f}" for v in row]))
    body = "\n".join(lines) + "\n"
    return web.Response(
        text=body,
        content_type="text/csv",
        headers={"Content-Disposition": f'attachment; filename="joints_{request.match_info["episode_id"][-8:]}.csv"'},
    )


async def _depth_preview(request: web.Request) -> web.Response:
    """Render a single 16-bit depth PNG into a colormapped JPEG so it can
    be previewed in the browser (HTML5 video can't carry 16-bit single-
    channel and direct PNG can't show the depth range).

    URL: /api/v1/episodes/{episode_id}/depth_preview/{camera}/{frame:06d}.jpg
    Query:
        cmap = turbo (default) | jet | viridis | plasma | inferno
        max  = max depth in mm to clip at (default 4000 for indoor 4m)
    """
    import cv2
    store: EpisodeStore = request.app["store"]
    episode_id = request.match_info["episode_id"]
    camera = request.match_info["camera"]
    frame = request.match_info["frame"]
    found = store.get_episode(episode_id)
    if found is None:
        return web.Response(status=404, text="episode not found")
    _meta, ep_dir = found
    png_path = (ep_dir / "videos" / camera / frame).resolve()
    # path traversal guard
    if not str(png_path).startswith(str(ep_dir.resolve())):
        return web.Response(status=400, text="bad path")
    if not png_path.exists() or not png_path.suffix.lower() in (".png", ".jpg"):
        # accept .jpg in the URL but rewrite to .png on disk
        png_path = png_path.with_suffix(".png")
        if not png_path.exists():
            return web.Response(status=404, text="frame not found")

    arr = cv2.imread(str(png_path), cv2.IMREAD_UNCHANGED)
    if arr is None:
        return web.Response(status=500, text="png decode failed")

    try:
        max_mm = max(100, min(20000, int(request.query.get("max", "4000"))))
    except ValueError:
        max_mm = 4000

    if arr.dtype != cv2.CV_8U and arr.ndim == 2:
        # Treat 0 as "no data" so it stays black; clip the rest to [1, max_mm].
        mask = arr > 0
        scaled = arr.astype("float32")
        scaled[~mask] = 0
        scaled = (scaled.clip(0, max_mm) / max_mm * 255).astype("uint8")
    else:
        scaled = arr.astype("uint8") if arr.dtype != "uint8" else arr

    cmap_name = (request.query.get("cmap") or "turbo").lower()
    cmap_map = {
        "turbo":   cv2.COLORMAP_TURBO,
        "jet":     cv2.COLORMAP_JET,
        "viridis": cv2.COLORMAP_VIRIDIS,
        "plasma":  cv2.COLORMAP_PLASMA,
        "inferno": cv2.COLORMAP_INFERNO,
    }
    color = cv2.applyColorMap(scaled, cmap_map.get(cmap_name, cv2.COLORMAP_TURBO))
    # Black out the "no data" pixels so nearest 0mm doesn't bleed bright red.
    if arr.ndim == 2:
        color[arr == 0] = (0, 0, 0)

    ok, jpg = cv2.imencode(".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        return web.Response(status=500, text="jpeg encode failed")
    return web.Response(
        body=jpg.tobytes(),
        content_type="image/jpeg",
        headers={"Cache-Control": "public, max-age=3600"},
    )


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


async def _batch_start(request: web.Request) -> web.Response:
    """Kick off a LeRobot-style cycle: [setup_s → record_s] × count.

    Body:
        task_description: str (required)
        profile: str (required)
        count: int (required, >=1)
        record_s: float (required, seconds per episode)
        setup_s: float (default 5.0, scene-reset countdown before each ep)
        tags: list[str] (optional)
        outcome_on_timeout: str (default "success")
    """
    try:
        body = await request.json()
    except Exception:
        body = {}
    required = ["task_description", "profile", "count", "record_s"]
    missing = [k for k in required if body.get(k) in (None, "", 0)]
    if missing:
        return web.json_response({"error": "missing_fields", "fields": missing}, status=400)
    try:
        cfg = BatchConfig(
            task_description=str(body["task_description"]),
            profile=str(body["profile"]),
            count=max(1, int(body["count"])),
            record_s=max(0.5, float(body["record_s"])),
            setup_s=max(0.0, float(body.get("setup_s", 5.0))),
            tags=list(body.get("tags") or []),
            outcome_on_timeout=str(body.get("outcome_on_timeout", "success")),
        )
    except (TypeError, ValueError) as exc:
        return web.json_response({"error": "invalid_request", "detail": str(exc)}, status=400)

    runner: BatchRunner = request.app["batch_runner"]
    if runner.is_running():
        return web.json_response(
            {"error": "batch_already_running", "current": runner.state.to_dict() if runner.state else None},
            status=409,
        )

    # Run the existing start/stop handlers in-process — keeps a single
    # code path (preflight, profile resolution, mux snapshot, lock).
    # aiohttp doesn't have a public sub-request API; a tiny shim is
    # enough since both handlers only use `request.json()`, `request.app`,
    # and `request.match_info`.
    async def _start(body: dict) -> dict:
        class _Shim:
            def __init__(self, parent, payload):
                self.app = parent.app
                self.match_info = {}
                self._payload = payload
            async def json(self):
                return self._payload
            @property
            def query(self):
                return {}
        resp = await _start_episode(_Shim(request, body))  # type: ignore[arg-type]
        if resp.status >= 400:
            raise RuntimeError(f"start_episode HTTP {resp.status}: {resp.body.decode('utf-8', 'replace')[:200]}")
        import json as _json
        return _json.loads(resp.body)

    async def _stop(ep_id: str, outcome: str) -> dict:
        class _Shim:
            def __init__(self, parent, ep, payload):
                self.app = parent.app
                self.match_info = {"episode_id": ep}
                self._payload = payload
            async def json(self):
                return self._payload
        resp = await _stop_episode(_Shim(request, ep_id, {"outcome": outcome}))  # type: ignore[arg-type]
        if resp.status >= 400:
            raise RuntimeError(f"stop_episode HTTP {resp.status}: {resp.body.decode('utf-8', 'replace')[:200]}")
        import json as _json
        return _json.loads(resp.body)

    state = await runner.start(cfg, _start, _stop)
    return web.json_response(state.to_dict(), status=202)


async def _batch_status(request: web.Request) -> web.Response:
    runner: BatchRunner = request.app["batch_runner"]
    if runner.state is None:
        return web.json_response({"phase": "idle"})
    return web.json_response(runner.state.to_dict())


async def _batch_stop(request: web.Request) -> web.Response:
    runner: BatchRunner = request.app["batch_runner"]
    if not runner.is_running():
        return web.json_response({"error": "no_batch_running"}, status=404)
    await runner.stop()
    return web.json_response({"stopping": True, "state": runner.state.to_dict() if runner.state else None})


async def _batch_retry(request: web.Request) -> web.Response:
    """Discard the current iteration's episode and redo the same index.
    No-op if no batch is running."""
    runner: BatchRunner = request.app["batch_runner"]
    if not runner.is_running():
        return web.json_response({"error": "no_batch_running"}, status=404)
    if not runner.retry():
        return web.json_response({"error": "retry_failed"}, status=500)
    return web.json_response({"retrying": True, "state": runner.state.to_dict() if runner.state else None})


async def _batch_skip(request: web.Request) -> web.Response:
    """Finalize the current iteration's episode with the configured outcome
    and advance to the next index immediately (don't wait for record_s)."""
    runner: BatchRunner = request.app["batch_runner"]
    if not runner.is_running():
        return web.json_response({"error": "no_batch_running"}, status=404)
    if not runner.skip():
        return web.json_response({"error": "skip_failed"}, status=500)
    return web.json_response({"skipping": True, "state": runner.state.to_dict() if runner.state else None})


async def _retention_status(request: web.Request) -> web.Response:
    """Dry-run plan: return what would be deleted under the current policy
    without actually deleting. Mirror of POST /retention/run?dry_run=true."""
    runner = request.app.get("retention_runner")
    if runner is None:
        return web.json_response({"enabled": False, "error": "retention_runner_not_wired"}, status=503)
    import asyncio
    loop = asyncio.get_running_loop()
    # tick() walks the FS — run in executor so the aiohttp loop stays free.
    summary = await loop.run_in_executor(None, lambda: runner.tick(dry_run=True))
    return web.json_response(summary)


async def _retention_run(request: web.Request) -> web.Response:
    """Manual retention tick. dry_run=true (default) just returns the plan;
    dry_run=false executes ripe deletes (still respecting grace period).
    Optional body: a policy override for THIS tick only (used by the
    "プレビュー" button so the operator can preview a hypothetical policy
    before saving it via PUT /retention/policy)."""
    runner = request.app.get("retention_runner")
    if runner is None:
        return web.json_response({"enabled": False, "error": "retention_runner_not_wired"}, status=503)
    dry_run = (request.query.get("dry_run", "true").lower() != "false")
    body = {}
    try:
        body = await request.json() if request.can_read_body else {}
    except Exception:
        body = {}
    from .retention import RetentionPolicy
    override = RetentionPolicy.from_dict(body) if body else None
    import asyncio
    loop = asyncio.get_running_loop()
    summary = await loop.run_in_executor(None, lambda: runner.tick(dry_run=dry_run, policy=override))
    return web.json_response(summary)


async def _replay_start(request: web.Request) -> web.Response:
    """Phase 3b/3c MVP — spawns `ros2 bag play <bag_dir>` so the recorded
    joint_cmd / tf / mux topics get republished. Downstream subscribers
    (mujoco arm in sim, or inference_bridge in policy_feed mode) pick the
    messages up as if live.

    Body:
      mode: "bag_play" (default) | "preview" | "policy_feed"
      speed: 0.1..4.0 (default 1.0)
      start_offset_s: float (default 0)
      duration_s: float (optional — SIGINT bag play at this elapsed time)
      marker_id: str (optional — resolves to start_offset_s from marker.started_at)

    SAFETY: no physical safety gate in this MVP. Caller is expected to ensure
    sim profile / mux is configured for bag_replay source. Phase 3c will add
    person/home_pose/e-stop precheck + ack token before real-hardware mode.
    """
    from datetime import datetime, timezone
    store: EpisodeStore = request.app["store"]
    episode_id = request.match_info["episode_id"]
    found = store.get_episode(episode_id)
    if found is None:
        return web.json_response({"error": "not_found"}, status=404)
    meta, ep_dir = found
    bag_dir = ep_dir / "bag"
    if not bag_dir.exists() or not (bag_dir / "metadata.yaml").exists():
        return web.json_response({"error": "no_bag", "detail": f"missing bag at {bag_dir}"}, status=400)

    runner: ReplayRunner = request.app["replay_runner"]
    if runner.is_running():
        return web.json_response(
            {"error": "replay_busy", "current": runner.state.to_dict() if runner.state else None},
            status=409,
        )
    # Refuse concurrent record + replay so the runners don't fight over /tf
    # and joint_cmd topics (and so the operator doesn't accidentally record
    # the replay).
    if store.active is not None:
        return web.json_response(
            {"error": "recording_active",
             "detail": "stop the active recording before starting a replay"},
            status=409,
        )

    try:
        body = await request.json() if request.can_read_body else {}
    except Exception:
        body = {}

    mode = body.get("mode") or "bag_play"
    speed = float(body.get("speed", 1.0))
    start_offset_s = float(body.get("start_offset_s", 0.0))
    duration_s = body.get("duration_s")
    if duration_s is not None:
        duration_s = float(duration_s)

    # Resolve marker_id → start_offset_s / duration_s if provided.
    marker_id = body.get("marker_id")
    if marker_id:
        for m in (meta.markers or []):
            if m.get("marker_id") == marker_id:
                try:
                    ep_start = datetime.strptime(meta.started_at, "%Y-%m-%dT%H:%M:%S.%fZ").replace(tzinfo=timezone.utc)
                    m_start = datetime.strptime(m["started_at"], "%Y-%m-%dT%H:%M:%S.%fZ").replace(tzinfo=timezone.utc)
                    start_offset_s = max(0.0, (m_start - ep_start).total_seconds())
                    if m.get("stopped_at"):
                        m_stop = datetime.strptime(m["stopped_at"], "%Y-%m-%dT%H:%M:%S.%fZ").replace(tzinfo=timezone.utc)
                        duration_s = (m_stop - m_start).total_seconds()
                except Exception as exc:
                    return web.json_response(
                        {"error": "marker_time_parse_failed", "detail": str(exc)}, status=400)
                break

    try:
        state = await runner.start(
            episode_id=episode_id,
            bag_dir=bag_dir,
            speed=speed,
            start_offset_s=start_offset_s,
            duration_s=duration_s,
            mode=mode,
        )
    except (RuntimeError, FileNotFoundError) as exc:
        return web.json_response({"error": "replay_start_failed", "detail": str(exc)}, status=500)
    return web.json_response(state.to_dict(), status=202)


async def _replay_status(request: web.Request) -> web.Response:
    runner: ReplayRunner = request.app["replay_runner"]
    if runner.state is None:
        return web.json_response({"phase": "idle"})
    return web.json_response(runner.state.to_dict())


async def _replay_stop(request: web.Request) -> web.Response:
    runner: ReplayRunner = request.app["replay_runner"]
    if not runner.is_running():
        return web.json_response({"error": "no_replay_running"}, status=404)
    await runner.stop()
    return web.json_response({"stopping": True, "state": runner.state.to_dict() if runner.state else None})


async def _retention_get_policy(request: web.Request) -> web.Response:
    """Current effective policy (operator override > profile default > disabled)."""
    runner = request.app.get("retention_runner")
    if runner is None:
        return web.json_response({"enabled": False, "error": "retention_runner_not_wired"}, status=503)
    return web.json_response(runner.current_policy().to_dict())


async def _retention_put_policy(request: web.Request) -> web.Response:
    """Save an operator override policy. Pass {"enabled": false} or DELETE to
    clear and fall back to the profile default."""
    runner = request.app.get("retention_runner")
    if runner is None:
        return web.json_response({"enabled": False, "error": "retention_runner_not_wired"}, status=503)
    try:
        body = await request.json()
    except Exception:
        body = {}
    from .retention import RetentionPolicy
    if not body or body.get("clear"):
        runner.clear_override()
    else:
        runner.set_policy(RetentionPolicy.from_dict(body))
    return web.json_response(runner.current_policy().to_dict())


async def _patch_episode(request: web.Request) -> web.Response:
    """Update a small allow-list of episode fields (trim, task_description,
    tags, pinned). Used by the play modal's clip / rename / pin actions."""
    episode_id = request.match_info["episode_id"]
    try:
        body = await request.json()
    except Exception:
        body = {}
    store: EpisodeStore = request.app["store"]
    updated = store.patch_episode_meta(episode_id, body)
    if updated is None:
        return web.json_response({"error": "not_found"}, status=404)
    return web.json_response(updated)


async def _delete_episode(request: web.Request) -> web.Response:
    """Hard-delete an episode directory. ?force=true bypasses the pin guard.

    Refuses to delete the currently active recording (must stop first)."""
    episode_id = request.match_info["episode_id"]
    force = request.query.get("force", "").lower() in ("1", "true", "yes")
    store: EpisodeStore = request.app["store"]
    try:
        removed = store.delete_episode(episode_id, force=force)
    except PermissionError as exc:
        return web.json_response({"error": "pinned", "detail": str(exc)}, status=409)
    except RuntimeError as exc:
        return web.json_response({"error": "active", "detail": str(exc)}, status=409)
    if not removed:
        return web.json_response({"error": "not_found"}, status=404)
    LOG.info("episode deleted: %s force=%s", episode_id, force)
    return web.json_response({"episode_id": episode_id, "deleted": True}, status=200)


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
