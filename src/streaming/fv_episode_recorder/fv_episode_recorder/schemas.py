"""Pydantic request/response schemas for fv_episode_recorder REST API.

API base URL: http://<host>:8083/api/v1
Phase 1 Step 1 — minimal start/stop/list/get only. Other endpoints land later.
"""

from __future__ import annotations

from typing import Any, Optional

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from .episode_store import normalize_episode_tags


# ---------- Episode lifecycle ----------

class CameraSelector(BaseModel):
    model_config = ConfigDict(extra="forbid", str_strip_whitespace=True)

    name: Optional[str] = Field(default=None, min_length=1)
    topic: Optional[str] = Field(default=None, min_length=1)
    kind: Optional[str] = Field(default=None, min_length=1)

    @model_validator(mode="after")
    def require_condition(self) -> "CameraSelector":
        if self.name is None and self.topic is None and self.kind is None:
            raise ValueError("camera selector requires name, topic, or kind")
        return self


class TopicSelector(BaseModel):
    model_config = ConfigDict(extra="forbid", str_strip_whitespace=True)

    topic: Optional[str] = Field(default=None, min_length=1)
    role: Optional[str] = Field(default=None, min_length=1)

    @model_validator(mode="after")
    def require_condition(self) -> "TopicSelector":
        if self.topic is None and self.role is None:
            raise ValueError("topic selector requires topic or role")
        return self


class RecordingResourceSelectors(BaseModel):
    model_config = ConfigDict(extra="forbid")

    cameras: Optional[list[CameraSelector]] = None
    topics: Optional[list[TopicSelector]] = None


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
    include: Optional[RecordingResourceSelectors] = None
    exclude: Optional[RecordingResourceSelectors] = None
    fps: int = Field(default=30, ge=1, le=120)
    record_bag: bool = True

    @field_validator("tags")
    @classmethod
    def validate_tags(cls, tags: list[str]) -> list[str]:
        return normalize_episode_tags(tags)


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
    finalization_pending: bool = True


class MergeEpisodeTagsRequest(BaseModel):
    tags: list[str]

    @field_validator("tags")
    @classmethod
    def validate_tags(cls, tags: list[str]) -> list[str]:
        return normalize_episode_tags(tags)


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
    # UX-only derived flags so the list table can render badges without
    # decoding meta.json client-side.
    env: str = "real"          # "real" | "sim"  (profile.endswith("_sim"))
    controller_label: Optional[str] = None  # e.g. "VLA: pi0", "teleop", "停止"


class ListEpisodesResponse(BaseModel):
    episodes: list[EpisodeSummary]
    next_cursor: Optional[str] = None
    total: int = 0
