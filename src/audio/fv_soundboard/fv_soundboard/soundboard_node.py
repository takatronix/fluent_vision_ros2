#!/usr/bin/env python3
"""Resolve registered FluentVision events into SYSTEM speech and cue PCM."""

from __future__ import annotations

import time
import uuid
import wave
from array import array

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from fv_audio_interfaces.msg import (
    TtsRequest,
    TtsResult,
    VoiceMaintenanceState,
)
from fv_speech_interfaces.msg import AudioFrame

from .contracts import SoundSettings
from .event_registry import EventRegistry
from .wav_pcm import load_wav


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
        self._settings = SoundSettings(True, frozenset())

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._say_pub = self.create_publisher(
            TtsRequest, "/aspa/dialogue/tts/request", qos
        )
        self._tts_result_pub = self.create_publisher(
            TtsResult, "/aspa/tts/result", qos
        )
        self._cue_pub = self.create_publisher(AudioFrame, "/audio/cue/frame", qos)
        self._marker_pub = self.create_publisher(String, "/fv/event/active", qos)
        self.create_subscription(String, "/fv/event", self._on_event, qos)
        self.create_subscription(String, "/fv/sound/play", self._on_preview, qos)
        settings_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            String, "/fv/sound/settings", self._on_settings, settings_qos
        )
        self._maintenance_state = None
        self._maintenance_ack_pub = self.create_publisher(
            VoiceMaintenanceState,
            "/fv/soundboard/voice-maintenance",
            settings_qos,
        )
        self.create_subscription(
            VoiceMaintenanceState,
            "/aspa/voice/maintenance",
            self._on_voice_maintenance,
            settings_qos,
        )
        self.get_logger().info(
            f"fv_soundboard ready: {len(self._registry)} fixed events"
        )

    def _on_settings(self, msg: String) -> None:
        try:
            settings = SoundSettings.from_json(msg.data)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        self._settings = settings
        self.get_logger().info(
            "sound settings applied: robot=%s disabled=%d"
            % (
                "on" if settings.robot_enabled else "off",
                len(settings.disabled_events),
            )
        )

    def _on_event(self, msg: String) -> None:
        self._handle_event(msg.data, preview=False)

    def _on_preview(self, msg: String) -> None:
        self._handle_event(msg.data, preview=True)

    def _on_voice_maintenance(
            self, msg: VoiceMaintenanceState) -> None:
        try:
            _validate_voice_maintenance(msg)
        except ValueError as exc:
            self._maintenance_state = None
            self.get_logger().error(str(exc))
            return
        self._maintenance_state = msg
        self._maintenance_ack_pub.publish(msg)

    def _handle_event(self, payload: str, *, preview: bool) -> None:
        try:
            event = self._registry.resolve(payload)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return

        suffix = uuid.uuid4().hex
        if not preview and event.record:
            self._marker_pub.publish(String(data=event.record))
        settings = self._settings
        if not preview and (
            not settings.robot_enabled or event.name in settings.disabled_events
        ):
            return
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
            request_id = str(uuid.uuid4())
            utterance_id = f"system-{event.name}-{suffix}"
            state = self._maintenance_state
            if state is None:
                self._reject_tts_request(
                    request_id,
                    utterance_id,
                    "voice maintenance state has not been observed",
                )
            elif state.active:
                self._reject_tts_request(
                    request_id,
                    utterance_id,
                    "voice maintenance is active "
                    f"(operation_id={state.operation_id}, "
                    f"reason={state.reason})",
                )
            else:
                self._say_pub.publish(
                    TtsRequest(
                        kind=TtsRequest.SYSTEM,
                        request_id=request_id,
                        utterance_id=utterance_id,
                        text=event.say,
                    )
                )

    def _reject_tts_request(
            self,
            request_id: str,
            utterance_id: str,
            error: str,
    ) -> None:
        self._tts_result_pub.publish(
            TtsResult(
                kind=TtsResult.SYSTEM,
                status=TtsResult.REJECTED,
                request_id=request_id,
                generation_valid=False,
                generation_id="",
                utterance_id=utterance_id,
                error=error,
            )
        )
        self.get_logger().warning(
            f"soundboard TTS request rejected: {error}"
        )


def _validate_voice_maintenance(msg: VoiceMaintenanceState) -> None:
    if msg.version != 1:
        raise ValueError("voice maintenance state version must be 1")
    if msg.active:
        try:
            operation_id = uuid.UUID(msg.operation_id)
        except ValueError as exc:
            raise ValueError(
                "active voice maintenance operation_id must be UUIDv4"
            ) from exc
        if operation_id.version != 4 or str(operation_id) != msg.operation_id:
            raise ValueError(
                "active voice maintenance operation_id must be UUIDv4"
            )
        if (
            not msg.reason
            or msg.reason != msg.reason.strip()
            or len(msg.reason) > 128
            or any(
                ord(character) < 0x20 or ord(character) == 0x7F
                for character in msg.reason
            )
        ):
            raise ValueError(
                "active voice maintenance reason is invalid"
            )
    elif msg.operation_id or msg.reason:
        raise ValueError(
            "inactive voice maintenance must not retain operation data"
        )


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
