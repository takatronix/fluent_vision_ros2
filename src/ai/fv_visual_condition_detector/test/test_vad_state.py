#!/usr/bin/env python3
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from fv_visual_condition_detector_py.vad_state import VadState  # noqa: E402


def test_min_on_min_off() -> None:
    vad = VadState(threshold=0.5, min_on_sec=0.3, min_off_sec=0.2)

    assert vad.update(0.7, 0.0) is None
    assert vad.state is False
    event = vad.update(0.7, 0.31)
    assert event is not None
    assert event.action == "start"
    assert vad.state is True

    assert vad.update(0.2, 0.4) is None
    assert vad.state is True
    event = vad.update(0.2, 0.61)
    assert event is not None
    assert event.action == "stop"
    assert vad.state is False


if __name__ == "__main__":
    test_min_on_min_off()
