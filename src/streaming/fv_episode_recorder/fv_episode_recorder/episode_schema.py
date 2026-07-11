"""Validation for the current authoritative episode metadata schema."""

from __future__ import annotations

from typing import TypeAlias


JsonScalar: TypeAlias = None | bool | int | float | str
JsonValue: TypeAlias = JsonScalar | list["JsonValue"] | dict[str, "JsonValue"]
JsonObject: TypeAlias = dict[str, JsonValue]

CURRENT_EPISODE_SCHEMA_VERSION = 2
_REMOVED_EPISODE_FIELDS = {
    "timeline_start_ros_ns",
    "timeline_end_ros_ns",
    "stop_frame_count_per_camera",
}
_REMOVED_CAMERA_FIELDS = {
    "video_timing_mode",
    "video_pts_origin_ros_ns",
    "video_time_base_num",
    "video_time_base_den",
}


class EpisodeSchemaError(ValueError):
    """Base error for an invalid or unsupported episode metadata schema."""


class EpisodeSchemaVersionMissingError(EpisodeSchemaError):
    """The metadata does not declare a usable schema version."""


class UnsupportedEpisodeSchemaVersionError(EpisodeSchemaError):
    """The document is not current and has no valid migration path."""


def episode_schema_version(data: JsonValue) -> int:
    if not isinstance(data, dict):
        raise EpisodeSchemaVersionMissingError("episode metadata must be a JSON object")
    version = data.get("schema_version")
    if type(version) is not int or version <= 0:
        raise EpisodeSchemaVersionMissingError(
            "episode metadata must declare a positive integer schema_version"
        )
    return version


def read_current_episode_document(data: JsonValue) -> JsonObject:
    version = episode_schema_version(data)
    if version != CURRENT_EPISODE_SCHEMA_VERSION:
        raise UnsupportedEpisodeSchemaVersionError(
            f"episode schema_version {version} is not current; "
            f"expected {CURRENT_EPISODE_SCHEMA_VERSION}"
        )
    assert isinstance(data, dict)
    _validate_common_fields(data)
    removed_episode_fields = sorted(_REMOVED_EPISODE_FIELDS.intersection(data))
    if removed_episode_fields:
        raise EpisodeSchemaError(
            "episode schema_version 2 contains removed fields: "
            + ", ".join(removed_episode_fields)
        )
    cameras = data.get("cameras")
    if isinstance(cameras, list):
        for camera in cameras:
            if not isinstance(camera, dict):
                continue
            removed_camera_fields = sorted(
                _REMOVED_CAMERA_FIELDS.intersection(camera)
            )
            if removed_camera_fields:
                raise EpisodeSchemaError(
                    "episode schema_version 2 camera contains removed fields: "
                    + ", ".join(removed_camera_fields)
                )
    return data


def _validate_common_fields(data: JsonObject) -> None:
    required_strings = ("episode_id", "state", "profile", "started_at")
    invalid = [
        field
        for field in required_strings
        if not isinstance(data.get(field), str) or not data[field]
    ]
    if invalid:
        raise EpisodeSchemaError(
            "current episode schema has invalid fields: " + ", ".join(invalid)
        )
