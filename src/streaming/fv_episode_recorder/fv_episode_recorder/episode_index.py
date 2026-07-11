"""sqlite index over EpisodeStore (Phase 2).

The on-disk meta.json files remain the source of truth — this index is a
cache that can be rebuilt at any time by walking the filesystem. It lets
the API serve large episode counts (1000+) without paying the cost of a
full FS walk + size aggregation on every request, and unlocks cursor
pagination + `GROUP BY batch_tag` for batch-folding.

Schema is intentionally narrow: only fields the list view + filters need.
For full episode details (cameras / markers / recorded_topics), callers
still read meta.json on demand (`get_episode`).

Reliability model:
  - All writes are: meta.json first (atomic rename), then index update.
  - The recorder rebuilds the index from meta.json on startup. This also
    repairs same-row-count lifecycle/tag drift after a crash between the
    atomic meta replacement and index upsert.
  - The index file is a single sqlite DB at `<output_dir>/index.db`;
    safe to delete — the next startup rebuilds.
"""

from __future__ import annotations

import json
import logging
import sqlite3
import threading
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any, Iterable, Optional

LOG = logging.getLogger("fv_episode_recorder.index")

SCHEMA_VERSION = 1

_DDL = """
CREATE TABLE IF NOT EXISTS schema_version (
    version INTEGER NOT NULL PRIMARY KEY
);

CREATE TABLE IF NOT EXISTS episodes (
    episode_id        TEXT PRIMARY KEY,
    state             TEXT NOT NULL,
    task_description  TEXT NOT NULL,
    profile           TEXT NOT NULL,
    robot_id          TEXT,
    started_at        TEXT NOT NULL,        -- ISO 8601 UTC
    stopped_at        TEXT,
    duration_s        REAL,
    outcome           TEXT,
    pinned            INTEGER NOT NULL DEFAULT 0,
    size_bytes        INTEGER NOT NULL DEFAULT 0,
    marker_count      INTEGER NOT NULL DEFAULT 0,
    tags              TEXT NOT NULL DEFAULT '[]',  -- JSON array
    source            TEXT NOT NULL DEFAULT 'local',
    env               TEXT NOT NULL DEFAULT 'real',  -- 'real' | 'sim'
    controller_label  TEXT,
    batch_id          TEXT,                  -- extracted from tags["batch:<id>"]
    ep_dir            TEXT NOT NULL,         -- absolute path on disk
    updated_at        REAL NOT NULL DEFAULT 0
);

-- Cursor pagination + filter common cases.
CREATE INDEX IF NOT EXISTS idx_episodes_started_at_desc ON episodes(started_at DESC);
CREATE INDEX IF NOT EXISTS idx_episodes_profile         ON episodes(profile, started_at DESC);
CREATE INDEX IF NOT EXISTS idx_episodes_batch_id        ON episodes(batch_id, started_at DESC);
CREATE INDEX IF NOT EXISTS idx_episodes_pinned          ON episodes(pinned, started_at DESC);
"""


def _batch_id_from_tags(tags) -> Optional[str]:
    if not tags:
        return None
    for t in tags:
        if isinstance(t, str) and t.startswith("batch:"):
            return t[6:]
    return None


def _env_from_profile(profile: Optional[str]) -> str:
    return "sim" if (profile or "").endswith("_sim") else "real"


def _summarize_controller(snap: Optional[dict]) -> Optional[str]:
    """Same logic as api_server._summarize_controller — duplicated to avoid
    a backward import; controller snapshot shape is stable."""
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


def _ep_dir_size(ep_dir: Path) -> int:
    total = 0
    try:
        for f in ep_dir.rglob("*"):
            if f.is_file():
                try:
                    total += f.stat().st_size
                except OSError:
                    continue
    except OSError:
        pass
    return total


class EpisodeIndex:
    """sqlite-backed list cache. Thread-safe (single connection + lock)."""

    def __init__(self, output_dir: Path):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.db_path = self.output_dir / "index.db"
        self._lock = threading.Lock()
        self._conn = sqlite3.connect(
            str(self.db_path),
            check_same_thread=False,  # guarded by self._lock
            isolation_level=None,     # autocommit; explicit txns when needed
        )
        self._conn.row_factory = sqlite3.Row
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute("PRAGMA synchronous=NORMAL")
        self._init_schema()

    def close(self) -> None:
        with self._lock:
            try:
                self._conn.close()
            except Exception:
                pass

    # ---- schema / rebuild ----

    def _init_schema(self) -> None:
        with self._lock:
            schema_exists = self._conn.execute(
                "SELECT 1 FROM sqlite_master WHERE type = 'table' AND name = 'schema_version'"
            ).fetchone()
            row = (
                self._conn.execute(
                    "SELECT version FROM schema_version LIMIT 1"
                ).fetchone()
                if schema_exists is not None
                else None
            )
            if schema_exists is not None and (
                row is None or row["version"] != SCHEMA_VERSION
            ):
                LOG.warning(
                    "index schema version %s != %d, dropping + rebuilding",
                    row["version"] if row is not None else "missing",
                    SCHEMA_VERSION,
                )
                self._conn.execute("DROP TABLE IF EXISTS episodes")
                self._conn.execute("DROP TABLE IF EXISTS schema_version")
            self._conn.executescript(_DDL)
            row = self._conn.execute("SELECT version FROM schema_version LIMIT 1").fetchone()
            if row is None:
                self._conn.execute("INSERT INTO schema_version(version) VALUES (?)", (SCHEMA_VERSION,))

    def rebuild_from_filesystem(self) -> int:
        """Wipe the episodes table and re-populate by walking meta.json files.
        Called on startup and as a manual recovery action. Returns count."""
        episodes_root = self.output_dir / "episodes"
        with self._lock:
            if not episodes_root.exists():
                self._conn.execute("DELETE FROM episodes")
                return 0
            self._conn.execute("BEGIN IMMEDIATE")
            try:
                self._conn.execute("DELETE FROM episodes")
                inserted = 0
                for meta_path in episodes_root.glob("*/*/*/meta.json"):
                    try:
                        with meta_path.open() as f:
                            data = json.load(f)
                    except (OSError, json.JSONDecodeError):
                        continue
                    ep_dir = meta_path.parent
                    self._insert_row_locked(data, ep_dir)
                    inserted += 1
                self._conn.execute("COMMIT")
                return inserted
            except Exception:
                self._conn.execute("ROLLBACK")
                raise

    def ensure_consistent(self) -> None:
        """Rebuild from authoritative meta.json files on every recorder startup.

        Counting files cannot detect a crash between atomic meta replacement
        and index upsert, where the row count stays equal but lifecycle/tags
        are stale.
        """
        self.rebuild_from_filesystem()

    # ---- mutators (called by EpisodeStore after meta.json writes) ----

    def upsert(self, meta_dict: dict, ep_dir: Path) -> None:
        with self._lock:
            self._insert_row_locked(meta_dict, ep_dir)

    def _insert_row_locked(self, data: dict, ep_dir: Path) -> None:
        episode_id = data.get("episode_id") or ""
        if not episode_id:
            return
        existing = self._conn.execute(
            "SELECT ep_dir FROM episodes WHERE episode_id = ?",
            (episode_id,),
        ).fetchone()
        if (
            existing is not None
            and Path(existing["ep_dir"]).resolve() != ep_dir.resolve()
        ):
            raise RuntimeError(
                f"episode_id {episode_id} maps to multiple directories: "
                f"{existing['ep_dir']} and {ep_dir}"
            )
        tags = data.get("tags") or []
        markers = data.get("markers") or []
        profile = data.get("profile") or ""
        size = _ep_dir_size(ep_dir)
        controller_snap = data.get("controller_at_end") or data.get("controller_at_start")
        self._conn.execute(
            """
            INSERT INTO episodes (
                episode_id, state, task_description, profile, robot_id,
                started_at, stopped_at, duration_s, outcome, pinned,
                size_bytes, marker_count, tags, source, env, controller_label,
                batch_id, ep_dir, updated_at
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            ON CONFLICT(episode_id) DO UPDATE SET
                state            = excluded.state,
                task_description = excluded.task_description,
                profile          = excluded.profile,
                robot_id         = excluded.robot_id,
                started_at       = excluded.started_at,
                stopped_at       = excluded.stopped_at,
                duration_s       = excluded.duration_s,
                outcome          = excluded.outcome,
                pinned           = excluded.pinned,
                size_bytes       = excluded.size_bytes,
                marker_count     = excluded.marker_count,
                tags             = excluded.tags,
                source           = excluded.source,
                env              = excluded.env,
                controller_label = excluded.controller_label,
                batch_id         = excluded.batch_id,
                ep_dir           = excluded.ep_dir,
                updated_at       = excluded.updated_at
            """,
            (
                episode_id,
                data.get("state") or "unknown",
                data.get("task_description") or "",
                profile,
                data.get("robot_id"),
                data.get("started_at") or "",
                data.get("stopped_at"),
                data.get("duration_s"),
                data.get("outcome"),
                1 if data.get("pinned") else 0,
                size,
                len(markers),
                json.dumps(tags, ensure_ascii=False),
                data.get("source") or "local",
                _env_from_profile(profile),
                _summarize_controller(controller_snap),
                _batch_id_from_tags(tags),
                str(ep_dir),
                time.time(),
            ),
        )

    def delete(self, episode_id: str) -> None:
        with self._lock:
            self._conn.execute("DELETE FROM episodes WHERE episode_id = ?", (episode_id,))

    # ---- queries ----

    def list(
        self,
        limit: int = 50,
        cursor: Optional[str] = None,         # started_at value; return rows older than this
        profile: Optional[str] = None,
        batch_id: Optional[str] = None,
        pinned_only: bool = False,
        env: Optional[str] = None,
    ) -> tuple[list[dict], Optional[str]]:
        """Cursor pagination by started_at DESC. Returns (rows, next_cursor)."""
        where = []
        params: list[Any] = []
        if cursor:
            where.append("started_at < ?")
            params.append(cursor)
        if profile:
            where.append("profile = ?")
            params.append(profile)
        if batch_id:
            where.append("batch_id = ?")
            params.append(batch_id)
        if pinned_only:
            where.append("pinned = 1")
        if env:
            where.append("env = ?")
            params.append(env)
        sql = (
            "SELECT * FROM episodes "
            + ("WHERE " + " AND ".join(where) + " " if where else "")
            + "ORDER BY started_at DESC LIMIT ?"
        )
        params.append(limit + 1)  # fetch one extra so we know if there's a next page
        with self._lock:
            rows = [dict(r) for r in self._conn.execute(sql, params).fetchall()]
        next_cursor: Optional[str] = None
        if len(rows) > limit:
            next_cursor = rows[limit - 1]["started_at"]
            rows = rows[:limit]
        # Parse tags JSON back so the caller doesn't have to.
        for r in rows:
            try:
                r["tags"] = json.loads(r.get("tags") or "[]")
            except Exception:
                r["tags"] = []
            r["pinned"] = bool(r.get("pinned"))
        return rows, next_cursor

    def total_count(self) -> int:
        with self._lock:
            return int(self._conn.execute("SELECT COUNT(*) AS c FROM episodes").fetchone()["c"])

    def find_dir_by_episode_id(self, episode_id: str) -> Optional[Path]:
        with self._lock:
            row = self._conn.execute(
                "SELECT ep_dir FROM episodes WHERE episode_id = ?",
                (episode_id,),
            ).fetchone()
        return Path(row["ep_dir"]) if row is not None else None
