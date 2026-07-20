from types import SimpleNamespace

import cv2
import numpy as np
import pytest


pytest.importorskip("rclpy")

from sensor_msgs.msg import CompressedImage  # noqa: E402

from fv_episode_recorder.moss_realtime_adapter_node import (  # noqa: E402
    MossRealtimeAdapterNode,
)
from fv_visual_condition_detector_py.visual_condition_detector_node import (  # noqa: E402
    VisualConditionDetectorNode,
)


def test_d555_compressed_image_reaches_detector_and_moss_without_reencoding() -> None:
    source = np.zeros((24, 32, 3), dtype=np.uint8)
    source[:, :, 1] = 180
    ok, encoded = cv2.imencode(".jpg", source)
    assert ok

    message = CompressedImage()
    message.format = "bgr8; jpeg compressed bgr8"
    message.data = encoded.tobytes()

    decoded = VisualConditionDetectorNode._imgmsg_to_bgr(message)
    assert decoded.shape == source.shape

    relayed: list[CompressedImage] = []
    detector_harness = SimpleNamespace(
        compressed_relay_pub=SimpleNamespace(publish=relayed.append),
        compressed_relay_frequency=1.0,
        _last_compressed_relay_time=0.0,
    )
    VisualConditionDetectorNode._relay_compressed_image(
        detector_harness,
        message,
        now=1.0,
    )
    assert relayed == [message]

    forwarded: list[bytes] = []
    harness = SimpleNamespace(
        _should_sample=lambda now: True,
        _is_supported_encoded_image=MossRealtimeAdapterNode._is_supported_encoded_image,
        _queue_frame=lambda now, data: forwarded.append(data),
    )
    MossRealtimeAdapterNode._on_compressed_image(harness, message)

    assert forwarded == [bytes(message.data)]


def test_canonical_detector_fails_when_model_is_missing() -> None:
    errors: list[str] = []
    harness = SimpleNamespace(
        model_path="/definitely/missing/visual-condition.onnx",
        require_model=True,
        get_logger=lambda: SimpleNamespace(error=errors.append),
    )

    with pytest.raises(RuntimeError, match="Model file not found"):
        VisualConditionDetectorNode._load_model(harness)

    assert errors == [
        "Model file not found: /definitely/missing/visual-condition.onnx"
    ]
