"""Persistent anomaly episodes and MOSS semantic annotations.

The ROS-facing annotation contains only text.  Interval timestamps remain in
this recorder-owned database and are never exposed to the dialogue agent.
"""

from __future__ import annotations

import os
import re
import sqlite3
import threading
import time
import unicodedata
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


def default_episode_root() -> Path:
    configured = os.environ.get("FV_EPISODE_OUTPUT_DIR")
    return Path(configured).expanduser() if configured else Path.home() / ".aspa" / "episodes"


_DDL = """
CREATE TABLE IF NOT EXISTS anomaly_episodes (
    episode_id   TEXT PRIMARY KEY,
    started_at   REAL NOT NULL,
    ended_at     REAL
);

CREATE TABLE IF NOT EXISTS semantic_annotations (
    annotation_id TEXT PRIMARY KEY,
    episode_id    TEXT NOT NULL UNIQUE,
    text          TEXT NOT NULL,
    revision      INTEGER NOT NULL DEFAULT 0,
    created_at    REAL NOT NULL,
    updated_at    REAL NOT NULL,
    FOREIGN KEY (episode_id) REFERENCES anomaly_episodes(episode_id)
);

CREATE VIRTUAL TABLE IF NOT EXISTS semantic_annotations_fts USING fts5(
    text,
    content='semantic_annotations',
    content_rowid='rowid',
    tokenize='trigram'
);

CREATE TRIGGER IF NOT EXISTS semantic_annotations_ai AFTER INSERT ON semantic_annotations BEGIN
    INSERT INTO semantic_annotations_fts(rowid, text) VALUES (new.rowid, new.text);
END;

CREATE TRIGGER IF NOT EXISTS semantic_annotations_ad AFTER DELETE ON semantic_annotations BEGIN
    INSERT INTO semantic_annotations_fts(semantic_annotations_fts, rowid, text)
    VALUES ('delete', old.rowid, old.text);
END;

CREATE TRIGGER IF NOT EXISTS semantic_annotations_au
AFTER UPDATE OF text ON semantic_annotations BEGIN
    INSERT INTO semantic_annotations_fts(semantic_annotations_fts, rowid, text)
    VALUES ('delete', old.rowid, old.text);
    INSERT INTO semantic_annotations_fts(rowid, text) VALUES (new.rowid, new.text);
END;
"""


@dataclass(frozen=True)
class AnnotationWrite:
    annotation_id: str
    episode_id: str
    text: str
    revision: int
    created: bool


class AnnotationStore:
    """Thread-safe SQLite FTS5 trigram store for MOSS annotations only."""

    _WRITE_RETRY_DELAYS = (0.02, 0.05, 0.10, 0.20)

    def __init__(self, database_path: str | Path) -> None:
        self.database_path = Path(database_path)
        self.database_path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._connection = sqlite3.connect(
            str(self.database_path),
            check_same_thread=False,
            isolation_level=None,
        )
        self._connection.row_factory = sqlite3.Row
        self._connection.execute("PRAGMA busy_timeout=2000")
        self._run_write(
            lambda: self._connection.execute("PRAGMA journal_mode=WAL").fetchone()
        )
        self._connection.execute("PRAGMA synchronous=NORMAL")
        self._connection.execute("PRAGMA foreign_keys=ON")
        self._run_write(lambda: self._connection.executescript(_DDL))

    def close(self) -> None:
        with self._lock:
            self._connection.close()

    def start_episode(self, episode_id: str, started_at: Optional[float] = None) -> None:
        now = time.time() if started_at is None else float(started_at)

        def write() -> None:
            self._connection.execute(
                """
                INSERT INTO anomaly_episodes(episode_id, started_at, ended_at)
                VALUES (?, ?, NULL)
                ON CONFLICT(episode_id) DO UPDATE SET
                    started_at = excluded.started_at,
                    ended_at = NULL
                """,
                (episode_id, now),
            )

        self._run_write(write)

    def end_episode(self, episode_id: str, ended_at: Optional[float] = None) -> None:
        now = time.time() if ended_at is None else float(ended_at)

        def write() -> None:
            cursor = self._connection.execute(
                "UPDATE anomaly_episodes SET ended_at = ? WHERE episode_id = ?",
                (now, episode_id),
            )
            if cursor.rowcount == 0:
                self._connection.execute(
                    """
                    INSERT INTO anomaly_episodes(episode_id, started_at, ended_at)
                    VALUES (?, ?, ?)
                    """,
                    (episode_id, now, now),
                )

        self._run_write(write)

    def upsert_annotation(
        self,
        annotation_id: str,
        episode_id: str,
        text: str,
        updated_at: Optional[float] = None,
    ) -> AnnotationWrite:
        semantic_text = text.strip()
        if not semantic_text:
            raise ValueError("annotation text must not be empty")
        now = time.time() if updated_at is None else float(updated_at)

        def write() -> tuple[str, int, bool]:
            self._connection.execute("BEGIN IMMEDIATE")
            self._connection.execute(
                """
                INSERT INTO anomaly_episodes(episode_id, started_at, ended_at)
                VALUES (?, ?, NULL)
                ON CONFLICT(episode_id) DO NOTHING
                """,
                (episode_id, now),
            )
            existing = self._connection.execute(
                """
                SELECT annotation_id, revision
                FROM semantic_annotations WHERE episode_id = ?
                """,
                (episode_id,),
            ).fetchone()
            if existing is None:
                self._connection.execute(
                    """
                    INSERT INTO semantic_annotations(
                        annotation_id, episode_id, text, revision, created_at, updated_at
                    ) VALUES (?, ?, ?, 0, ?, ?)
                    """,
                    (annotation_id, episode_id, semantic_text, now, now),
                )
                revision = 0
                created = True
                stored_id = annotation_id
            else:
                stored_id = str(existing["annotation_id"])
                revision = int(existing["revision"]) + 1
                self._connection.execute(
                    """
                    UPDATE semantic_annotations
                    SET text = ?, revision = ?, updated_at = ?
                    WHERE annotation_id = ?
                    """,
                    (semantic_text, revision, now, stored_id),
                )
                created = False
            self._connection.execute("COMMIT")
            return stored_id, revision, created

        stored_id, revision, created = self._run_write(write)

        return AnnotationWrite(
            annotation_id=stored_id,
            episode_id=episode_id,
            text=semantic_text,
            revision=revision,
            created=created,
        )

    def _run_write(self, operation):
        last_attempt = len(self._WRITE_RETRY_DELAYS)
        for attempt in range(last_attempt + 1):
            with self._lock:
                try:
                    return operation()
                except sqlite3.OperationalError as exc:
                    if self._connection.in_transaction:
                        self._connection.execute("ROLLBACK")
                    busy = "locked" in str(exc).lower() or "busy" in str(exc).lower()
                    if not busy or attempt == last_attempt:
                        raise
                except Exception:
                    if self._connection.in_transaction:
                        self._connection.execute("ROLLBACK")
                    raise
            time.sleep(self._WRITE_RETRY_DELAYS[attempt])
        raise RuntimeError("unreachable SQLite retry state")

    def search(self, query: str, limit: int = 5) -> list[str]:
        match_query = build_fts_query(query)
        if not match_query or limit <= 0:
            return []
        bounded_limit = min(int(limit), 20)
        with self._lock:
            rows = self._connection.execute(
                """
                SELECT annotations.text
                FROM semantic_annotations_fts AS matches
                JOIN semantic_annotations AS annotations ON annotations.rowid = matches.rowid
                WHERE semantic_annotations_fts MATCH ?
                ORDER BY annotations.updated_at DESC
                LIMIT ?
                """,
                (match_query, bounded_limit),
            ).fetchall()
        return [str(row["text"]) for row in rows]

    def get_annotation(self, episode_id: str) -> Optional[AnnotationWrite]:
        with self._lock:
            row = self._connection.execute(
                """
                SELECT annotation_id, episode_id, text, revision
                FROM semantic_annotations WHERE episode_id = ?
                """,
                (episode_id,),
            ).fetchone()
        if row is None:
            return None
        return AnnotationWrite(
            annotation_id=str(row["annotation_id"]),
            episode_id=str(row["episode_id"]),
            text=str(row["text"]),
            revision=int(row["revision"]),
            created=False,
        )


_WORDS = re.compile(r"[^\W_]+", flags=re.UNICODE)


def build_fts_query(query: str) -> str:
    """Build an OR query of lexical terms accepted by the trigram tokenizer."""
    normalized = unicodedata.normalize("NFKC", query).strip()
    terms: list[str] = []
    seen: set[str] = set()
    for match in _WORDS.finditer(normalized):
        word = match.group(0)
        candidates = (
            [word]
            if word.isascii()
            else [word[index:index + 3] for index in range(len(word) - 2)]
        )
        for candidate in candidates:
            if len(candidate) < 3 or candidate in seen:
                continue
            seen.add(candidate)
            terms.append('"' + candidate.replace('"', '""') + '"')
    return " OR ".join(terms)
