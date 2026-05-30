"""Pydantic request/response schemas for fv_episode_recorder REST API.

API base URL: http://<host>:8083/api/v1
Phase 1 Step 1 — minimal start/stop/list/get only. Other endpoints land later.
"""

from __future__ import annotations

from typing import Any, Optional

from pydantic import BaseModel, Field


# ---------- Episode lifecycle ----------

class StartEpisodeRequest(BaseModel):
    task_description: str = Field(..., min_length=1)
    profile: str = Field(..., min_length=1)
    robot_id: Optional[str] = None
    operator: Optional[str] = None
    tags: list[str] = Field(default_factory=list)
    parent_session_id: Optional[str] = None
    expected_duration_s: Optional[float] = Field(default=None, ge=0)
    env_config: Optional[dict[str, Any]] = None
    record_topics_override: Optional[list[dict[str, Any]]] = None
    record_bag_topics: Optional[list[str]] = None  # Step 2: simple topic name list for bag
    cameras_override: Optional[list[dict[str, Any]]] = None
    fps: int = Field(default=30, ge=1, le=120)
    record_bag: bool = True


class StartEpisodeResponse(BaseModel):
    episode_id: str
    started_at: str
    bag_path: str
    meta_path: str
    recorded_topics_resolved: list[dict[str, Any]] = Field(default_factory=list)
    preflight: dict[str, Any] = Field(default_factory=dict)


class StopEpisodeRequest(BaseModel):
    outcome: str = Field(..., pattern="^(success|abort|discard)$")


class StopEpisodeResponse(BaseModel):
    episode_id: str
    state: str
    duration_s: float
    frame_count_per_camera: dict[str, int] = Field(default_factory=dict)
    bag_size_bytes: int = 0
    video_size_bytes: int = 0
    manifest_pending: bool = True


# ---------- List / Get ----------

class EpisodeSummary(BaseModel):
    episode_id: str
    state: str
    task_description: str
    profile: str
    robot_id: Optional[str] = None
    started_at: str
    stopped_at: Optional[str] = None
    duration_s: Optional[float] = None
    outcome: Optional[str] = None
    pinned: bool = False
    size_bytes: int = 0
    marker_count: int = 0
    derived_from: Optional[str] = None
    tags: list[str] = Field(default_factory=list)
    source: str = "local"


class ListEpisodesResponse(BaseModel):
    episodes: list[EpisodeSummary]
    next_cursor: Optional[str] = None
