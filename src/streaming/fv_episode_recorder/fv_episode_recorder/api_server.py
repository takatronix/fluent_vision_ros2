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
from dataclasses import asdict
from typing import Any

from aiohttp import web

from .episode_store import EpisodeMeta, EpisodeStore, new_episode_id, utc_now_iso
from .schemas import (
    EpisodeSummary,
    ListEpisodesResponse,
    StartEpisodeRequest,
    StartEpisodeResponse,
    StopEpisodeRequest,
    StopEpisodeResponse,
)

LOG = logging.getLogger("fv_episode_recorder.api")


def build_app(store: EpisodeStore) -> web.Application:
    app = web.Application()
    app["store"] = store

    app.router.add_get("/api/v1/healthz", _healthz)
    app.router.add_post("/api/v1/episodes", _start_episode)
    app.router.add_post("/api/v1/episodes/{episode_id}/stop", _stop_episode)
    app.router.add_get("/api/v1/episodes", _list_episodes)
    app.router.add_get("/api/v1/episodes/{episode_id}", _get_episode)

    return app


async def _healthz(request: web.Request) -> web.Response:
    return web.json_response({
        "status": "ok",
        "active_episode": (
            request.app["store"].active.episode_id if request.app["store"].active else None
        ),
    })


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
        recorded_topics=req.record_topics_override or [],
        cameras=req.cameras_override or [],
        topic_discovery_source="request_override" if req.record_topics_override else "profile",
    )
    try:
        ep_dir = store.start_episode(meta)
    except RuntimeError as exc:
        return web.json_response({"error": "conflict", "detail": str(exc)}, status=409)

    resp = StartEpisodeResponse(
        episode_id=ep_id,
        started_at=started_at,
        bag_path=str((ep_dir / "bag").resolve()),
        meta_path=str((ep_dir / "meta.json").resolve()),
        recorded_topics_resolved=meta.recorded_topics,
        preflight={"passed": True, "estimated_bytes": 0, "available_bytes": 0,
                   "margin_bytes": 0, "note": "phase1-step1: preflight stub"},
    )
    LOG.info("episode started: %s task=%s profile=%s", ep_id, req.task_description, req.profile)
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

    meta, _ep_dir = store.stop_active(req.outcome)
    resp = StopEpisodeResponse(
        episode_id=meta.episode_id,
        state=meta.state,
        duration_s=meta.duration_s or 0.0,
        manifest_pending=True,
    )
    LOG.info("episode stopped: %s outcome=%s duration=%.1fs", episode_id, req.outcome, resp.duration_s)
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
