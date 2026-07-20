#!/usr/bin/env python3
"""Resolve registered FluentVision events into SYSTEM speech and cue PCM."""

from __future__ import annotations

from array import array
from dataclasses import asdict, dataclass
import json
import time
import uuid
import wave

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from fluent_dialogue_dora_interfaces.msg import AudioFrame

from .event_registry import EventRegistry


@dataclass(frozen=True)
class SayRequest:
    kind: str
    utterance_id: str
    text: str

    def to_json(self) -> str:
        return json.dumps(asdict(self), ensure_ascii=False, separators=(",", ":"))


@dataclass(frozen=True)
class WavPcm:
    data: bytes
    sample_rate_hz: int
    channels: int


def load_wav(path: str) -> WavPcm:
    with wave.open(path, "rb") as wav:
        if wav.getcomptype() != "NONE":
            raise ValueError(f"cue WAV must be uncompressed PCM: {path}")
        if wav.getsampwidth() != 2:
            raise ValueError(f"cue WAV must be PCM16: {path}")
        channels = wav.getnchannels()
        sample_rate_hz = wav.getframerate()
        data = wav.readframes(wav.getnframes())
    if channels <= 0 or sample_rate_hz <= 0 or not data:
        raise ValueError(f"cue WAV is empty or has invalid format: {path}")
    return WavPcm(data=data, sample_rate_hz=sample_rate_hz, channels=channels)


class SoundboardNode(Node):
    def __init__(self) -> None:
        super().__init__("fv_soundboard")
        share = get_package_share_directory("fv_soundboard")
        self.declare_parameter("registry_file", f"{share}/config/events.yaml")
        self.declare_parameter("sounds_dir", f"{share}/sounds")
        self._registry = EventRegistry.load(
            str(self.get_parameter("registry_file").value),
            str(self.get_parameter("sounds_dir").value),
        )

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._say_pub = self.create_publisher(String, "/aspa/tts/say", qos)
        self._cue_pub = self.create_publisher(AudioFrame, "/audio/cue/frame", qos)
        self._marker_pub = self.create_publisher(String, "/fv/event/active", qos)
        self.create_subscription(String, "/fv/event", self._on_event, qos)
        self.get_logger().info(
            f"fv_soundboard ready: {len(self._registry)} fixed events"
        )

    def _on_event(self, msg: String) -> None:
        try:
            event = self._registry.resolve(msg.data)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return

        suffix = uuid.uuid4().hex
        if event.record:
            self._marker_pub.publish(String(data=event.record))
        if event.sound_path:
            try:
                audio = load_wav(event.sound_path)
            except (OSError, ValueError, wave.Error) as exc:
                self.get_logger().error(str(exc))
                return
            cue = AudioFrame()
            cue.header.stamp = self.get_clock().now().to_msg()
            cue.source_id = "fv_soundboard"
            cue.stream_id = f"cue-{event.name}-{suffix}"
            cue.seq = 0
            cue.sample_index = 0
            cue.capture_time_ns = time.time_ns()
            cue.frame_count = len(audio.data) // (audio.channels * 2)
            cue.encoding = "PCM16LE"
            cue.sample_rate_hz = audio.sample_rate_hz
            cue.channels = audio.channels
            cue.bit_depth = 16
            cue.layout = "interleaved"
            cue.data = array("B", audio.data)
            cue.final = True
            self._cue_pub.publish(cue)

        if event.say:
            request = SayRequest(
                kind="system",
                utterance_id=f"system-{event.name}-{suffix}",
                text=event.say,
            )
            self._say_pub.publish(String(data=request.to_json()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SoundboardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
