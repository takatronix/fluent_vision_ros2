"""Episode store — directory layout + meta.json + (Phase 2) sqlite index.

Phase 1 Step 1 — minimum viable: ULID id, dir layout, meta.json read/write.
sqlite index / FTS / tags table are Phase 2.
"""

from __future__ import annotations

import json
import re
import threading
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

from ulid import ULID


SCHEMA_VERSION = 1

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
    cameras: list[dict[str, Any]] = field(default_factory=list)
    recorded_topics: list[dict[str, Any]] = field(default_factory=list)
    topic_discovery_source: str = "profile"
    bag_path: str = "bag/"
    bag_split_count: int = 0
    annotation_revisions: list[dict[str, Any]] = field(default_factory=list)
    markers: list[dict[str, Any]] = field(default_factory=list)
    recorder_version: str = "fv_episode_recorder/0.1.0"
    container_image_tag: Optional[str] = None
    stale_input_events: list[dict[str, Any]] = field(default_factory=list)
    quality_metrics: Optional[dict[str, Any]] = None
    manifest_file: Optional[str] = None
    # remote import (Phase 4.5+ reserved)
    source: str = "local"  # local | remote_imported | remote_proxy
    remote_manifest_url: Optional[str] = None
    remote_signed_urls: Optional[dict[str, Any]] = None
    # env / scene (Phase 2.7+ optional)
    env_config: Optional[dict[str, Any]] = None
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
    controller_at_start: Optional[dict[str, Any]] = None
    controller_at_end: Optional[dict[str, Any]] = None


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
        self._active_episode: Optional[EpisodeMeta] = None
        self._active_dir: Optional[Path] = None
        # sqlite cache built lazily — recorder_node calls ensure_consistent()
        # on startup to detect drift + rebuild. Mutations below call upsert
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
            ep_dir = self._episode_dir(meta)
            ep_dir.mkdir(parents=True, exist_ok=True)
            (ep_dir / "bag").mkdir(exist_ok=True)
            (ep_dir / "videos").mkdir(exist_ok=True)
            self._active_episode = meta
            self._active_dir = ep_dir
            self._write_meta(ep_dir, meta)
            return ep_dir

    def stop_active(self, outcome: str) -> tuple[EpisodeMeta, Path]:
        """Finalize active episode. Returns (meta, dir)."""
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
            if outcome == "discard":
                meta.state = "discarded"
            elif outcome == "abort":
                meta.state = "failed"
            else:
                meta.state = "finished"
            self._write_meta(ep_dir, meta)
            self._active_episode = None
            self._active_dir = None
            return meta, ep_dir

    # ---- list / get ----

    def list_episodes(self, limit: int = 50) -> list[tuple[EpisodeMeta, int]]:
        """Legacy filesystem-walk lister (kept for callers that don't need
        pagination — e.g. retention planner). New code should use
        `list_episodes_indexed()` for the sqlite-backed cursor pagination."""
        results: list[tuple[EpisodeMeta, int]] = []
        episodes_root = self.output_dir / "episodes"
        if not episodes_root.exists():
            return results
        for meta_path in sorted(
            episodes_root.glob("*/*/*/meta.json"), key=lambda p: p.stat().st_mtime, reverse=True
        ):
            try:
                with meta_path.open() as f:
                    data = json.load(f)
                meta = EpisodeMeta(**{k: v for k, v in data.items() if k in EpisodeMeta.__annotations__})
                ep_dir = meta_path.parent
                size = 0
                try:
                    for f in ep_dir.rglob("*"):
                        if f.is_file():
                            size += f.stat().st_size
                except OSError:
                    pass
                results.append((meta, size))
                if len(results) >= limit:
                    break
            except (json.JSONDecodeError, TypeError, ValueError):
                continue
        return results

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
        tags / pinned + reason). Returns the updated meta dict, or None
        if the episode isn't found."""
        allowed = {"trim_start_s", "trim_end_s", "task_description",
                   "tags", "pinned", "pin_reason", "outcome"}
        found = self.get_episode(episode_id)
        if found is None:
            return None
        _meta, ep_dir = found
        meta_path = ep_dir / "meta.json"
        try:
            with meta_path.open() as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError):
            return None
        for k, v in (changes or {}).items():
            if k in allowed:
                # Allow explicit null/None to clear trim_*.
                data[k] = v
        tmp = meta_path.with_suffix(".json.tmp")
        with tmp.open("w") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        tmp.replace(meta_path)
        try: self.index.upsert(data, ep_dir)
        except Exception: pass
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
        _meta, ep_dir = found
        meta_path = ep_dir / "meta.json"
        try:
            with meta_path.open() as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError):
            return None
        markers = data.setdefault("markers", [])
        markers.append(marker)
        tmp = meta_path.with_suffix(".json.tmp")
        with tmp.open("w") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        tmp.replace(meta_path)
        try: self.index.upsert(data, ep_dir)
        except Exception: pass
        return marker

    def delete_finalized_marker(self, marker_id: str) -> bool:
        """Remove a marker by id from whichever finalized meta.json holds it."""
        episodes_root = self.output_dir / "episodes"
        if not episodes_root.exists():
            return False
        for meta_path in episodes_root.glob("*/*/*/meta.json"):
            try:
                with meta_path.open() as f:
                    data = json.load(f)
            except (OSError, json.JSONDecodeError):
                continue
            markers = data.get("markers", []) or []
            new_markers = [m for m in markers if m.get("marker_id") != marker_id]
            if len(new_markers) != len(markers):
                data["markers"] = new_markers
                tmp = meta_path.with_suffix(".json.tmp")
                with tmp.open("w") as f:
                    json.dump(data, f, ensure_ascii=False, indent=2)
                tmp.replace(meta_path)
                try: self.index.upsert(data, meta_path.parent)
                except Exception: pass
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
            try:
                with meta_path.open() as f:
                    data = json.load(f)
            except (OSError, json.JSONDecodeError):
                continue
            markers = data.get("markers", []) or []
            for m in markers:
                if m.get("marker_id") != marker_id:
                    continue
                for k, v in (changes or {}).items():
                    if k in allowed and v is not None:
                        m[k] = v
                m["rev"] = (m.get("rev", 0) or 0) + 1
                tmp = meta_path.with_suffix(".json.tmp")
                with tmp.open("w") as f:
                    json.dump(data, f, ensure_ascii=False, indent=2)
                tmp.replace(meta_path)
                try: self.index.upsert(data, meta_path.parent)
                except Exception: pass
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
        if self._active_episode is not None and self._active_episode.episode_id == episode_id:
            raise RuntimeError("cannot delete the active recording episode")
        import shutil
        shutil.rmtree(ep_dir, ignore_errors=True)
        try: self.index.delete(episode_id)
        except Exception: pass
        return True

    def get_episode(self, episode_id: str) -> Optional[tuple[EpisodeMeta, Path]]:
        """Find an episode by full ULID. Folder name format is now
        <HHMMSS>_<slug>_<ULID8>, so glob by the ULID 8-suffix then
        verify the full episode_id against meta.json (collision-safe)."""
        episodes_root = self.output_dir / "episodes"
        if not episodes_root.exists():
            return None
        suffix = episode_id[-8:] if len(episode_id) >= 8 else episode_id
        # Try new pattern first (suffix match), then legacy pattern (ULID == folder).
        for pattern in (f"*/*/*{suffix}/meta.json", f"*/*/{episode_id}/meta.json"):
            for meta_path in episodes_root.glob(pattern):
                try:
                    with meta_path.open() as f:
                        data = json.load(f)
                except (OSError, json.JSONDecodeError):
                    continue
                if data.get("episode_id") != episode_id:
                    continue
                return (
                    EpisodeMeta(**{k: v for k, v in data.items() if k in EpisodeMeta.__annotations__}),
                    meta_path.parent,
                )
        return None

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

    def _write_meta(self, ep_dir: Path, meta: EpisodeMeta) -> None:
        meta_path = ep_dir / "meta.json"
        tmp_path = meta_path.with_suffix(".json.tmp")
        data = asdict(meta)
        with tmp_path.open("w") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        tmp_path.replace(meta_path)
        # Mirror into the sqlite index so list queries don't have to FS-walk.
        # Best-effort: a failure here doesn't block the meta.json write.
        try:
            self.index.upsert(data, ep_dir)
        except Exception:
            pass
