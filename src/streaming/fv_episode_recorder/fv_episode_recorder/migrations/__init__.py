"""Forward-only migrations for authoritative episode metadata."""

from __future__ import annotations

import json
import logging
import os
import stat
from collections.abc import Callable
from pathlib import Path

from ..episode_schema import (
    CURRENT_EPISODE_SCHEMA_VERSION,
    EpisodeSchemaError,
    JsonObject,
    JsonValue,
    UnsupportedEpisodeSchemaVersionError,
    episode_schema_version,
    read_current_episode_document,
)
from .v1_to_v2 import migrate_v1_to_v2


LOG = logging.getLogger("fv_episode_recorder.migrations")

EpisodeMigration = Callable[[JsonObject], JsonObject]

MIGRATIONS: dict[int, EpisodeMigration] = {
    1: migrate_v1_to_v2,
}


def migrate_episode_document(data: JsonValue) -> JsonObject:
    version = episode_schema_version(data)
    if not isinstance(data, dict):
        raise EpisodeSchemaError("episode metadata must be a JSON object")
    document = data
    visited: set[int] = set()
    while version != CURRENT_EPISODE_SCHEMA_VERSION:
        if version in visited:
            raise EpisodeSchemaError(f"episode migration cycle detected at version {version}")
        visited.add(version)
        migration = MIGRATIONS.get(version)
        if migration is None:
            raise UnsupportedEpisodeSchemaVersionError(
                f"no forward migration from episode schema_version {version} "
                f"to {CURRENT_EPISODE_SCHEMA_VERSION}"
            )
        document = migration(document)
        next_version = episode_schema_version(document)
        if next_version <= version:
            raise EpisodeSchemaError(
                f"episode migration did not advance schema_version: "
                f"{version} -> {next_version}"
            )
        version = next_version
    return read_current_episode_document(document)


def migrate_episode_metadata_file(meta_path: Path) -> bool:
    try:
        raw = json.loads(meta_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise EpisodeSchemaError(f"episode metadata cannot be migrated: {meta_path}") from exc
    try:
        source_version = episode_schema_version(raw)
        migrated = migrate_episode_document(raw)
    except EpisodeSchemaError as exc:
        raise type(exc)(f"{meta_path}: {exc}") from exc
    if migrated == raw:
        return False

    source_stat = meta_path.stat()
    tmp_path = meta_path.with_suffix(".json.migrating")
    try:
        with tmp_path.open("w", encoding="utf-8") as handle:
            json.dump(migrated, handle, ensure_ascii=False, indent=2)
            handle.flush()
            os.fsync(handle.fileno())
        tmp_path.chmod(stat.S_IMODE(source_stat.st_mode))
        tmp_stat = tmp_path.stat()
        if (
            tmp_stat.st_uid != source_stat.st_uid
            or tmp_stat.st_gid != source_stat.st_gid
        ):
            os.chown(tmp_path, source_stat.st_uid, source_stat.st_gid)
        os.replace(tmp_path, meta_path)
        directory_fd = os.open(meta_path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except Exception:
        tmp_path.unlink(missing_ok=True)
        raise
    LOG.info(
        "migrated episode metadata schema %d -> %d: %s",
        source_version,
        CURRENT_EPISODE_SCHEMA_VERSION,
        meta_path,
    )
    return True


def migrate_episode_store(output_dir: Path) -> list[Path]:
    episodes_root = output_dir / "episodes"
    if not episodes_root.exists():
        return []
    migrated: list[Path] = []
    for meta_path in episodes_root.glob("*/*/*/meta.json"):
        if migrate_episode_metadata_file(meta_path):
            migrated.append(meta_path)
    return migrated
