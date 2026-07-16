"""Episode store — directory layout + meta.json + (Phase 2) sqlite index.

Phase 1 Step 1 — minimum viable: ULID id, dir layout, meta.json read/write.
sqlite index / FTS / tags table are Phase 2.
"""

from __future__ import annotations

import json
import os
import re
import threading
from collections.abc import Callable
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Literal, Optional

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    JsonValue,
    StrictBool,
    StrictFloat,
    StrictInt,
    StrictStr,
    ValidationError,
    model_validator,
)
from ulid import ULID


SCHEMA_VERSION = 2
EPISODE_DIR_MODE = 0o2770
META_FILE_MODE = 0o660
FINISHED_PAYLOAD_DIR_MODE = 0o750
FINISHED_PAYLOAD_FILE_MODE = 0o440
MAX_EPISODE_TAGS = 64
MAX_EPISODE_TAG_LENGTH = 128

# Finished payloads are owned by the output root owner. This keeps hardlinks
# available when a privileged recorder writes into a host-owned bind mount.

JsonObject = dict[str, JsonValue]
Number = StrictInt | StrictFloat


class EpisodeSchemaError(ValueError):
    """meta.json is missing a supported schema or violates that schema."""


class DuplicateEpisodeIdError(EpisodeSchemaError):
    """The same episode ID is present in more than one directory."""


class EpisodeIndexSyncError(Exception):
    """meta.json changed but its disposable index projection did not."""


def _migrate_v1_to_v2(data: JsonObject) -> JsonObject:
    migrated = dict(data)
    migrated["schema_version"] = 2
    return migrated


# Each entry migrates exactly one version forward. Add future migrations here;
# metadata without a complete path to SCHEMA_VERSION is rejected.
EPISODE_MIGRATIONS: dict[int, Callable[[JsonObject], JsonObject]] = {
    1: _migrate_v1_to_v2,
}

# Reject filesystem-hostile chars + control chars. Japanese / non-ASCII kept
# (ext4 is utf-8 native; ls/ros2 bag handle them fine).
_FORBIDDEN_FS_CHARS = re.compile(r'[\\/:*?"<>|\x00-\x1f]+')
_WHITESPACE = re.compile(r'\s+')
_MULTI_UNDERSCORE = re.compile(r'_+')


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def utc_today_str() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%d")


def new_episode_id() -> str:
    return str(ULID())


def slugify_task(task: str, max_len: int = 40) -> str:
    """Build a filesystem-friendly slug from task_description for the episode
    directory name. Japanese / hiragana / kanji are preserved (ext4 utf-8);
    only path-hostile chars are stripped."""
    if not task:
        return "untitled"
    s = task.strip()
    s = _FORBIDDEN_FS_CHARS.sub("_", s)
    s = _WHITESPACE.sub("_", s)
    s = _MULTI_UNDERSCORE.sub("_", s)
    s = s.strip("._")
    if not s:
        return "untitled"
    if len(s) > max_len:
        s = s[:max_len].rstrip("._")
    return s


@dataclass
class EpisodeMeta:
    schema_version: int = SCHEMA_VERSION
    episode_id: str = ""
    state: str = "recording"  # recording | finalizing | finished | failed | discarded
    task_description: str = ""
    profile: str = ""
    robot_id: Optional[str] = None
    operator: Optional[str] = None
    tags: list[str] = field(default_factory=list)
    parent_session_id: Optional[str] = None
    expected_duration_s: Optional[float] = None
    started_at: str = ""
    stopped_at: Optional[str] = None
    duration_s: Optional[float] = None
    outcome: Optional[str] = None
    pinned: bool = False
    pin_reason: Optional[str] = None
    cameras: list[JsonObject] = field(default_factory=list)
    recorded_topics: list[JsonObject] = field(default_factory=list)
    topic_discovery_source: str = "profile"
    bag_path: str = "bag/"
    bag_split_count: int = 0
    annotation_revisions: list[JsonObject] = field(default_factory=list)
    markers: list[JsonObject] = field(default_factory=list)
    recorder_version: str = "fv_episode_recorder/0.1.0"
    container_image_tag: Optional[str] = None
    stale_input_events: list[JsonObject] = field(default_factory=list)
    quality_metrics: Optional[JsonObject] = None
    manifest_file: Optional[str] = None
    # remote import (Phase 4.5+ reserved)
    source: str = "local"  # local | remote_imported | remote_proxy
    remote_manifest_url: Optional[str] = None
    remote_signed_urls: Optional[JsonObject] = None
    # env / scene (Phase 2.7+ optional)
    env_config: Optional[JsonObject] = None
    # Non-destructive clip trim. Seconds from started_at. None = no trim.
    # Playback / joint chart / future exports honor these; underlying bag +
    # mp4 stay intact so the operator can widen the trim later.
    trim_start_s: Optional[float] = None
    trim_end_s: Optional[float] = None
    # Mux / controller snapshot — used to answer "was this VLA or teleop?"
    # without decoding the bag. Populated by recorder_node from the latest
    # `*/teleop_mux/status` JSON at start + stop.
    # Shape: {"<arm_topic>": {"source": "ai|leader|none",
    #                          "ai_controller": "<name>", "ai_route_key": "...",
    #                          "enabled": bool}}
    controller_at_start: Optional[dict[str, JsonObject]] = None
    controller_at_end: Optional[dict[str, JsonObject]] = None
    finalization_failures: list[dict[str, str]] = field(default_factory=list)


class _StrictMetaModel(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)


class CameraSegmentV2(_StrictMetaModel):
    file: StrictStr
    frame_count: StrictInt
    size_bytes: StrictInt


class CameraV2(_StrictMetaModel):
    name: StrictStr
    topic: StrictStr
    codec: Optional[StrictStr] = None
    kind: Optional[StrictStr] = None
    encoding: Optional[StrictStr] = None
    width: Optional[StrictInt] = None
    height: Optional[StrictInt] = None
    fps_nominal: Optional[StrictInt] = None
    fps_actual: Optional[Number] = None
    frame_count: Optional[StrictInt] = None
    video_dir: Optional[StrictStr] = None
    sidecar_file: Optional[StrictStr] = None
    segments: list[CameraSegmentV2] = Field(default_factory=list)
    total_png_bytes: Optional[StrictInt] = None


class RecordedTopicV2(_StrictMetaModel):
    topic: StrictStr
    role: StrictStr
    qos: StrictStr
    stamp_source: StrictStr
    msg_type: Optional[StrictStr] = None


class EpisodeMetaV2(_StrictMetaModel):
    schema_version: Literal[2]
    episode_id: StrictStr
    state: Literal["recording", "finalizing", "finished", "failed", "discarded"]
    task_description: StrictStr
    profile: StrictStr
    robot_id: Optional[StrictStr] = None
    operator: Optional[StrictStr] = None
    tags: list[StrictStr] = Field(default_factory=list)
    parent_session_id: Optional[StrictStr] = None
    expected_duration_s: Optional[Number] = None
    started_at: StrictStr
    stopped_at: Optional[StrictStr] = None
    duration_s: Optional[Number] = None
    outcome: Optional[Literal["success", "abort", "discard"]] = None
    pinned: StrictBool = False
    pin_reason: Optional[StrictStr] = None
    cameras: list[CameraV2] = Field(default_factory=list)
    recorded_topics: list[RecordedTopicV2] = Field(default_factory=list)
    topic_discovery_source: StrictStr = "profile"
    bag_path: StrictStr = "bag/"
    bag_split_count: StrictInt = 0
    annotation_revisions: list[JsonObject] = Field(default_factory=list)
    markers: list[JsonObject] = Field(default_factory=list)
    recorder_version: StrictStr = "fv_episode_recorder/0.1.0"
    container_image_tag: Optional[StrictStr] = None
    stale_input_events: list[JsonObject] = Field(default_factory=list)
    quality_metrics: Optional[JsonObject] = None
    manifest_file: Optional[StrictStr] = None
    source: Literal["local", "remote_imported", "remote_proxy"] = "local"
    remote_manifest_url: Optional[StrictStr] = None
    remote_signed_urls: Optional[JsonObject] = None
    env_config: Optional[JsonObject] = None
    trim_start_s: Optional[Number] = None
    trim_end_s: Optional[Number] = None
    controller_at_start: Optional[dict[StrictStr, JsonObject]] = None
    controller_at_end: Optional[dict[StrictStr, JsonObject]] = None
    finalization_failures: list[dict[StrictStr, StrictStr]] = Field(default_factory=list)

    @model_validator(mode="after")
    def validate_contract(self) -> EpisodeMetaV2:
        if not self.episode_id or not self.task_description or not self.profile or not self.started_at:
            raise ValueError("episode_id, task_description, profile, and started_at must be non-empty")
        if normalize_episode_tags(list(self.tags)) != self.tags:
            raise ValueError("tags must be unique")
        allowed_outcomes = {
            "recording": {None},
            "finalizing": {None, "success", "abort", "discard"},
            "finished": {"success"},
            "failed": {"abort"},
            "discarded": {"discard"},
        }[self.state]
        if self.outcome not in allowed_outcomes:
            raise ValueError(
                f"episode state {self.state} does not allow outcome {self.outcome!r}"
            )
        return self


def normalize_episode_tags(tags: list[str]) -> list[str]:
    if not isinstance(tags, list):
        raise EpisodeSchemaError("tags must be a list of strings")
    normalized: list[str] = []
    seen: set[str] = set()
    for tag in tags:
        if not isinstance(tag, str):
            raise EpisodeSchemaError("tags must be a list of strings")
        if not tag or tag != tag.strip():
            raise EpisodeSchemaError("tags must be non-empty and have no surrounding whitespace")
        if len(tag) > MAX_EPISODE_TAG_LENGTH:
            raise EpisodeSchemaError(
                f"tag exceeds {MAX_EPISODE_TAG_LENGTH} characters"
            )
        if any(ord(char) < 32 or ord(char) == 127 for char in tag):
            raise EpisodeSchemaError("tags must not contain control characters")
        if tag not in seen:
            normalized.append(tag)
            seen.add(tag)
    if len(normalized) > MAX_EPISODE_TAGS:
        raise EpisodeSchemaError(f"at most {MAX_EPISODE_TAGS} tags are allowed")
    return normalized


def migrate_episode_meta(data: JsonObject) -> tuple[JsonObject, bool]:
    if "schema_version" not in data:
        raise EpisodeSchemaError("meta.json is missing schema_version")
    version = data["schema_version"]
    if type(version) is not int:
        raise EpisodeSchemaError("meta.json schema_version must be an integer")
    migrated = dict(data)
    changed = False
    while version != SCHEMA_VERSION:
        migration = EPISODE_MIGRATIONS.get(version)
        if migration is None:
            raise EpisodeSchemaError(
                f"unsupported meta.json schema_version {version}; expected {SCHEMA_VERSION}"
            )
        migrated = migration(migrated)
        next_version = migrated.get("schema_version")
        if type(next_version) is not int or next_version <= version:
            raise EpisodeSchemaError(f"invalid migration from schema_version {version}")
        version = next_version
        changed = True
    return migrated, changed


def validate_episode_meta(data: JsonObject) -> None:
    try:
        EpisodeMetaV2.model_validate(data)
    except ValidationError as exc:
        raise EpisodeSchemaError(f"invalid meta.json schema v{SCHEMA_VERSION}: {exc}") from exc


def _write_meta_file(meta_path: Path, data: JsonObject) -> None:
    validate_episode_meta(data)
    tmp_path = meta_path.with_suffix(".json.tmp")
    with tmp_path.open("w") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    tmp_path.chmod(META_FILE_MODE)
    tmp_path.replace(meta_path)


def read_episode_meta(meta_path: Path, migrate: bool = True) -> tuple[EpisodeMeta, JsonObject]:
    try:
        with meta_path.open() as f:
            raw = json.load(f)
    except (OSError, json.JSONDecodeError) as exc:
        raise EpisodeSchemaError(f"unable to read {meta_path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise EpisodeSchemaError(f"meta.json must contain a JSON object: {meta_path}")
    data, changed = migrate_episode_meta(raw)
    validate_episode_meta(data)
    if changed:
        if not migrate:
            raise EpisodeSchemaError(
                f"meta.json schema_version {raw['schema_version']} requires migration"
            )
        _write_meta_file(meta_path, data)
    return EpisodeMeta(**data), data


class EpisodeStore:
    """Filesystem-backed episode store.

    Layout:
        <output_dir>/episodes/<profile>/<YYYY-MM-DD>/<episode_id>/
            meta.json
            bag/                  # Phase 1 Step 2 で書き込み開始
            videos/<camera>/      # Phase 1 Step 3
            manifest.json         # Phase 1.5
    """

    def __init__(self, output_dir: str | Path):
        self.output_dir = Path(output_dir)
        self._lock = threading.Lock()
        output_stat = self.output_dir.stat()
        self._shared_uid = output_stat.st_uid
        self._shared_gid = output_stat.st_gid
        self._active_episode: Optional[EpisodeMeta] = None
        self._active_dir: Optional[Path] = None
        # sqlite cache is rebuilt by recorder_node on startup. Mutations below call upsert
        # so the index stays in sync with on-disk meta.json.
        from .episode_index import EpisodeIndex
        self.index = EpisodeIndex(self.output_dir)

    # ---- active episode tracking (Phase 1 Step 1 — single active enforcement) ----

    @property
    def active(self) -> Optional[EpisodeMeta]:
        return self._active_episode

    @property
    def active_dir(self) -> Optional[Path]:
        return self._active_dir

    def start_episode(self, meta: EpisodeMeta) -> Path:
        """Create episode directory and write initial meta.json. Returns episode dir."""
        with self._lock:
            if self._active_episode is not None:
                raise RuntimeError(
                    f"another episode already active: {self._active_episode.episode_id}"
                )
            validate_episode_meta(asdict(meta))
            if self.get_episode(meta.episode_id) is not None:
                raise DuplicateEpisodeIdError(
                    f"episode ID already exists: {meta.episode_id}"
                )
            ep_dir = self._episode_dir(meta)
            for directory in (
                self.output_dir / "episodes",
                self.output_dir / "episodes" / meta.profile,
                self.output_dir / "episodes" / meta.profile / meta.started_at[:10],
                ep_dir,
                ep_dir / "bag",
                ep_dir / "videos",
            ):
                directory.mkdir(mode=EPISODE_DIR_MODE, exist_ok=True)
                os.chown(directory, self._shared_uid, self._shared_gid)
                directory.chmod(EPISODE_DIR_MODE)
            self._active_episode = meta
            self._active_dir = ep_dir
            self._write_meta(ep_dir, meta)
            return ep_dir

    def begin_finalization(
        self,
        outcome: str,
    ) -> tuple[EpisodeMeta, Path, Optional[EpisodeIndexSyncError]]:
        """Persist finalizing state, then release the active episode slot."""
        with self._lock:
            if self._active_episode is None:
                raise RuntimeError("no active episode")
            meta = self._active_episode
            ep_dir = self._active_dir
            meta.stopped_at = utc_now_iso()
            meta.outcome = outcome
            started = datetime.strptime(
                meta.started_at, "%Y-%m-%dT%H:%M:%S.%fZ"
            ).replace(tzinfo=timezone.utc)
            stopped = datetime.strptime(
                meta.stopped_at, "%Y-%m-%dT%H:%M:%S.%fZ"
            ).replace(tzinfo=timezone.utc)
            meta.duration_s = (stopped - started).total_seconds()
            meta.state = "finalizing"
            data = asdict(meta)
            _write_meta_file(ep_dir / "meta.json", data)
            index_error = None
            try:
                self.index.upsert(data, ep_dir)
            except Exception as exc:
                index_error = EpisodeIndexSyncError(
                    f"finalizing metadata committed for {meta.episode_id}; "
                    f"startup index rebuild required: {exc}"
                )
            self._active_episode = None
            self._active_dir = None
            return meta, ep_dir, index_error

    # ---- list / get ----

    def list_episodes(self, limit: int = 50) -> list[tuple[EpisodeMeta, int]]:
        """Legacy filesystem-walk lister (kept for callers that don't need
        pagination — e.g. retention planner). New code should use
        `list_episodes_indexed()` for the sqlite-backed cursor pagination."""
        results: list[tuple[EpisodeMeta, int]] = []
        episodes_root = self.output_dir / "episodes"
        if not episodes_root.exists():
            return results
        seen: dict[str, Path] = {}
        for meta_path in sorted(
            episodes_root.glob("*/*/*/meta.json"), key=lambda p: p.stat().st_mtime, reverse=True
        ):
            meta, _data = read_episode_meta(meta_path)
            previous = seen.get(meta.episode_id)
            if previous is not None and previous != meta_path.parent:
                raise DuplicateEpisodeIdError(
                    f"duplicate episode ID {meta.episode_id}: {previous} and {meta_path.parent}"
                )
            seen[meta.episode_id] = meta_path.parent
            ep_dir = meta_path.parent
            size = 0
            try:
                for file_path in ep_dir.rglob("*"):
                    if file_path.is_file():
                        size += file_path.stat().st_size
            except OSError:
                pass
            results.append((meta, size))
        return results[:limit]

    def list_episodes_indexed(
        self,
        limit: int = 50,
        cursor: Optional[str] = None,
        profile: Optional[str] = None,
        batch_id: Optional[str] = None,
        pinned_only: bool = False,
        env: Optional[str] = None,
    ) -> tuple[list[dict], Optional[str], int]:
        """sqlite-backed cursor pagination. Returns (rows, next_cursor, total).
        Each row is a dict already shaped like EpisodeSummary so api_server
        can pass it through with minimal massaging."""
        rows, next_cursor = self.index.list(
            limit=limit, cursor=cursor, profile=profile,
            batch_id=batch_id, pinned_only=pinned_only, env=env,
        )
        return rows, next_cursor, self.index.total_count()

    def patch_episode_meta(self, episode_id: str, changes: dict) -> Optional[dict]:
        """Update a small allow-list of meta fields on disk (trim / task /
        pinned + reason). Returns the updated meta dict, or None
        if the episode isn't found."""
        allowed = {"trim_start_s", "trim_end_s", "task_description",
                   "pinned", "pin_reason"}
        found = self.get_episode(episode_id)
        if found is None:
            return None
        meta, ep_dir = found
        if meta.state in ("recording", "finalizing"):
            raise RuntimeError(f"cannot update episode while state={meta.state}")
        meta_path = ep_dir / "meta.json"
        _meta, data = read_episode_meta(meta_path)
        for k, v in (changes or {}).items():
            if k in allowed:
                # Allow explicit null/None to clear trim_*.
                data[k] = v
        self._write_meta_data(ep_dir, data)
        return data

    def merge_episode_tags(self, episode_id: str, tags: list[str]) -> Optional[dict]:
        """Merge validated tags into meta.json and its index row atomically per process."""
        requested = normalize_episode_tags(tags)
        with self._lock:
            found = self.get_episode(episode_id)
            if found is None:
                return None
            meta, ep_dir = found
            if meta.state in ("recording", "finalizing"):
                raise RuntimeError(f"cannot update episode while state={meta.state}")
            _stored_meta, data = read_episode_meta(ep_dir / "meta.json")
            data["tags"] = normalize_episode_tags([*data["tags"], *requested])
            self._write_meta_data(ep_dir, data)
            return data

    def add_finalized_marker(self, episode_id: str, marker: dict) -> Optional[dict]:
        """Append a marker entry into a finalized episode's meta.json.

        Phase 4 review workflow: while reviewing a past recording the operator
        often spots a moment worth tagging (collision, lifted gripper too
        early, etc.). The marker manager only handles the active episode in
        memory, so for finalized episodes we write directly into the file."""
        found = self.get_episode(episode_id)
        if found is None:
            return None
        meta, ep_dir = found
        if meta.state in ("recording", "finalizing"):
            raise RuntimeError(f"cannot update episode while state={meta.state}")
        meta_path = ep_dir / "meta.json"
        _meta, data = read_episode_meta(meta_path)
        markers = data.setdefault("markers", [])
        markers.append(marker)
        self._write_meta_data(ep_dir, data)
        return marker

    def delete_finalized_marker(self, marker_id: str) -> bool:
        """Remove a marker by id from whichever finalized meta.json holds it."""
        episodes_root = self.output_dir / "episodes"
        if not episodes_root.exists():
            return False
        for meta_path in episodes_root.glob("*/*/*/meta.json"):
            meta, data = read_episode_meta(meta_path)
            markers = data.get("markers", []) or []
            new_markers = [m for m in markers if m.get("marker_id") != marker_id]
            if len(new_markers) != len(markers):
                if meta.state in ("recording", "finalizing"):
                    raise RuntimeError(f"cannot update episode while state={meta.state}")
                data["markers"] = new_markers
                self._write_meta_data(meta_path.parent, data)
                return True
        return False

    def patch_finalized_marker(self, marker_id: str, changes: dict) -> Optional[dict]:
        """Update one marker inside a finalized episode's meta.json on disk.

        After an episode stops, MarkerManager flushes its markers and clears
        them from memory — so the in-memory PATCH path can't edit them.
        This walks meta.json files until it finds the marker_id, applies the
        allowed changes, and rewrites the file atomically. Returns the
        updated marker dict, or None if not found."""
        allowed = {"started_at", "stopped_at", "task_description", "tags",
                   "outcome", "kind", "attributes"}
        episodes_root = self.output_dir / "episodes"
        if not episodes_root.exists():
            return None
        for meta_path in episodes_root.glob("*/*/*/meta.json"):
            meta, data = read_episode_meta(meta_path)
            markers = data.get("markers", []) or []
            for m in markers:
                if m.get("marker_id") != marker_id:
                    continue
                if meta.state in ("recording", "finalizing"):
                    raise RuntimeError(f"cannot update episode while state={meta.state}")
                for k, v in (changes or {}).items():
                    if k in allowed and v is not None:
                        m[k] = v
                m["rev"] = (m.get("rev", 0) or 0) + 1
                self._write_meta_data(meta_path.parent, data)
                return m
        return None

    def delete_episode(self, episode_id: str, force: bool = False) -> bool:
        """Remove the entire episode directory. Returns True if removed.
        Refuses pinned episodes unless force=True (Phase 1 audit log via api)."""
        found = self.get_episode(episode_id)
        if found is None:
            return False
        meta, ep_dir = found
        if meta.pinned and not force:
            raise PermissionError("episode is pinned; use force=true")
        if meta.state in ("recording", "finalizing"):
            raise RuntimeError(f"cannot delete episode while state={meta.state}")
        return self._delete_episode(episode_id, ep_dir)

    def discard_finalizing_episode(self, episode_id: str) -> bool:
        found = self.get_episode(episode_id)
        if found is None:
            return False
        meta, ep_dir = found
        if meta.state != "finalizing" or meta.outcome != "discard":
            raise RuntimeError("only a finalizing discard episode can use this operation")
        return self._delete_episode(episode_id, ep_dir)

    def _delete_episode(self, episode_id: str, ep_dir: Path) -> bool:
        import shutil
        try:
            self.index.delete(episode_id)
        except Exception as exc:
            raise EpisodeIndexSyncError(
                f"episode files retained for {episode_id}; index deletion failed"
            ) from exc
        try:
            shutil.rmtree(ep_dir, ignore_errors=False)
        except Exception:
            try:
                _meta, data = read_episode_meta(ep_dir / "meta.json")
                self.index.upsert(data, ep_dir, refresh_size=True)
            except Exception as repair_exc:
                raise EpisodeIndexSyncError(
                    f"episode deletion incomplete for {episode_id}; "
                    "startup index rebuild required"
                ) from repair_exc
            raise
        return True

    def get_episode(self, episode_id: str) -> Optional[tuple[EpisodeMeta, Path]]:
        """Find an episode by full ULID. Folder name format is now
        <HHMMSS>_<slug>_<ULID8>, so glob by the ULID 8-suffix then
        verify the full episode_id against meta.json (collision-safe)."""
        episodes_root = self.output_dir / "episodes"
        if not episodes_root.exists():
            return None
        matches: list[tuple[EpisodeMeta, Path]] = []
        for meta_path in episodes_root.glob("*/*/*/meta.json"):
            meta, _data = read_episode_meta(meta_path)
            if meta.episode_id == episode_id:
                matches.append((meta, meta_path.parent))
        if len(matches) > 1:
            locations = ", ".join(str(ep_dir) for _meta, ep_dir in matches)
            raise DuplicateEpisodeIdError(
                f"duplicate episode ID {episode_id}: {locations}"
            )
        return matches[0] if matches else None

    def fail_orphan_episode(self, episode_id: str, episode_dir: Path) -> EpisodeMeta:
        """Mark a crashed recording/finalizing episode failed without deleting files."""
        found = self.get_episode(episode_id)
        if found is None:
            raise FileNotFoundError(f"orphan episode not found: {episode_id}")
        meta, ep_dir = found
        if ep_dir.resolve() != episode_dir.resolve():
            raise EpisodeSchemaError(
                f"orphan lock directory does not match metadata for {episode_id}"
            )
        if meta.state in ("recording", "finalizing"):
            meta.state = "failed"
            meta.outcome = "abort"
            if meta.stopped_at is None:
                meta.stopped_at = utc_now_iso()
            self._write_meta(ep_dir, meta)
        return meta

    def fail_incomplete_episodes(self) -> int:
        """Fail closed any recording/finalizing metadata left by a prior process."""
        episodes_root = self.output_dir / "episodes"
        if not episodes_root.exists():
            return 0
        failed = 0
        seen: dict[str, Path] = {}
        for meta_path in episodes_root.glob("*/*/*/meta.json"):
            meta, _data = read_episode_meta(meta_path)
            previous = seen.get(meta.episode_id)
            if previous is not None and previous != meta_path.parent:
                raise DuplicateEpisodeIdError(
                    f"duplicate episode ID {meta.episode_id}: {previous} and {meta_path.parent}"
                )
            seen[meta.episode_id] = meta_path.parent
            if meta.state not in ("recording", "finalizing"):
                continue
            meta.state = "failed"
            meta.outcome = "abort"
            if meta.stopped_at is None:
                meta.stopped_at = utc_now_iso()
            self._write_meta(meta_path.parent, meta)
            failed += 1
        return failed

    def migrate_finished_payload_permissions(self) -> int:
        """Apply the read-only payload contract to historical successful episodes."""
        migrated = 0
        episodes_root = self.output_dir / "episodes"
        if not episodes_root.exists():
            return migrated
        seen: dict[str, Path] = {}
        for meta_path in episodes_root.glob("*/*/*/meta.json"):
            meta, _data = read_episode_meta(meta_path)
            previous = seen.get(meta.episode_id)
            if previous is not None and previous != meta_path.parent:
                raise DuplicateEpisodeIdError(
                    f"duplicate episode ID {meta.episode_id}: {previous} and {meta_path.parent}"
                )
            seen[meta.episode_id] = meta_path.parent
            if meta.state == "finished" and meta.outcome == "success":
                self.protect_finished_payload_sources(meta_path.parent)
                migrated += 1
        return migrated

    def protect_finished_payload_sources(self, ep_dir: Path) -> None:
        for payload_root in (ep_dir / "bag", ep_dir / "videos"):
            if not payload_root.exists():
                continue
            for path in payload_root.rglob("*"):
                if path.is_file():
                    os.chown(path, self._shared_uid, self._shared_gid)
                    path.chmod(FINISHED_PAYLOAD_FILE_MODE)
                elif path.is_dir():
                    os.chown(path, self._shared_uid, self._shared_gid)
                    path.chmod(FINISHED_PAYLOAD_DIR_MODE)
            os.chown(payload_root, self._shared_uid, self._shared_gid)
            payload_root.chmod(FINISHED_PAYLOAD_DIR_MODE)

    # ---- internals ----

    def _episode_dir(self, meta: EpisodeMeta) -> Path:
        # Folder name convention: <HHMMSS>_<task-slug>_<ULID-suffix-8>
        # - HHMMSS prefix → ls sorts chronologically within the date folder
        # - task slug → operator can tell what the episode was without reading meta.json
        # - ULID suffix-8 → guarantees uniqueness even for same-second collisions
        date_str = meta.started_at[:10]            # YYYY-MM-DD
        hhmmss = meta.started_at[11:19].replace(":", "")  # HHMMSS
        slug = slugify_task(meta.task_description)
        suffix = meta.episode_id[-8:] if len(meta.episode_id) >= 8 else meta.episode_id
        folder_name = f"{hhmmss}_{slug}_{suffix}"
        return self.output_dir / "episodes" / meta.profile / date_str / folder_name

    def _write_meta(
        self,
        ep_dir: Path,
        meta: EpisodeMeta,
        refresh_size: bool = False,
    ) -> None:
        data = asdict(meta)
        self._write_meta_data(ep_dir, data, refresh_size=refresh_size)

    def _write_meta_data(
        self,
        ep_dir: Path,
        data: JsonObject,
        refresh_size: bool = False,
    ) -> None:
        _write_meta_file(ep_dir / "meta.json", data)
        # meta.json is authoritative and cannot be rolled back after rename.
        # A failed index projection is reported; recorder startup rebuilds the
        # complete disposable index from metadata before serving requests.
        try:
            self.index.upsert(data, ep_dir, refresh_size=refresh_size)
        except Exception as exc:
            raise EpisodeIndexSyncError(
                f"meta.json committed for {data['episode_id']}; startup index rebuild required"
            ) from exc
