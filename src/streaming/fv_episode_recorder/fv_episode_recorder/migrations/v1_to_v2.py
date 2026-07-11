"""Migrate the original episode metadata contract to schema version 2."""

from __future__ import annotations

from ..episode_schema import JsonObject, JsonValue


def migrate_v1_to_v2(data: JsonObject) -> JsonObject:
    migrated = dict(data)
    migrated.pop("timeline_start_ros_ns", None)
    migrated.pop("timeline_end_ros_ns", None)
    migrated.pop("stop_frame_count_per_camera", None)
    cameras = migrated.get("cameras")
    if isinstance(cameras, list):
        migrated["cameras"] = [_migrate_camera(camera) for camera in cameras]
    migrated["schema_version"] = 2
    return migrated


def _migrate_camera(camera: JsonValue) -> JsonValue:
    if not isinstance(camera, dict):
        return camera
    migrated = dict(camera)
    migrated.pop("video_timing_mode", None)
    migrated.pop("video_pts_origin_ros_ns", None)
    migrated.pop("video_time_base_num", None)
    migrated.pop("video_time_base_den", None)
    return migrated
