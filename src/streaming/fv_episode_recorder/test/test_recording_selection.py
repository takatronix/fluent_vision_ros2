from __future__ import annotations

import pytest
from pydantic import ValidationError

from fv_episode_recorder.schemas import CameraSelector, StartEpisodeRequest, TopicSelector
from fv_episode_recorder.topic_discovery import (
    RecordingSelectionError,
    discover_cameras,
    discover_topics,
    select_cameras,
    select_topics,
)


PROFILE = {
    "episode_recorder": {
        "cameras": [
            {"name": "top_camera", "topic": "/top/image/compressed"},
            {"name": "arm_color", "topic": "/arm/color/compressed"},
            {"name": "arm_depth", "topic": "/arm/depth", "kind": "depth"},
        ],
        "record_topics_override": [
            {
                "topic": "/follower/joint_states",
                "role": "state",
                "qos": "reliable",
                "stamp_source": "message_header",
            }
        ],
    }
}


def test_depth_camera_exclusion_also_removes_derived_depth_topic() -> None:
    cameras = select_cameras(
        discover_cameras(PROFILE),
        include=None,
        exclude=[CameraSelector(kind="depth")],
    )
    topics = discover_topics(PROFILE, cameras)

    assert [camera["name"] for camera in cameras] == ["top_camera", "arm_color"]
    assert all(topic["role"] != "camera_depth_compressed" for topic in topics)
    assert all("/arm/depth" not in topic["topic"] for topic in topics)


def test_camera_and_topic_selectors_support_include_and_exclude() -> None:
    cameras = select_cameras(
        discover_cameras(PROFILE),
        include=[CameraSelector(kind="color")],
        exclude=[CameraSelector(name="top_camera")],
    )
    topics = select_topics(
        discover_topics(PROFILE, cameras),
        include=None,
        exclude=[TopicSelector(role="annotation")],
    )

    assert [camera["name"] for camera in cameras] == ["arm_color"]
    assert all(topic["role"] != "annotation" for topic in topics)


def test_recording_selector_rejects_empty_or_unmatched_conditions() -> None:
    with pytest.raises(ValidationError, match="camera selector requires"):
        StartEpisodeRequest(
            task_description="pick",
            profile="piper_single_teleop",
            exclude={"cameras": [{}]},
        )

    with pytest.raises(RecordingSelectionError, match="matched no profile cameras"):
        select_cameras(
            discover_cameras(PROFILE),
            include=None,
            exclude=[CameraSelector(name="missing_camera")],
        )
