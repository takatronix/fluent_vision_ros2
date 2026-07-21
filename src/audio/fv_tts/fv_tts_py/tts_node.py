#!/usr/bin/env python3
from __future__ import annotations

from array import array
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from fluent_dialogue_dora_interfaces.msg import AudioFrame

from aspa_audio.contracts import PlaybackControl

from .contracts import SayRequest, TtsResult
from .synthesis_scheduler import SynthesisScheduler
from .voicevox_backend import SynthesizedAudio, VoicevoxCoreBackend


class FvTtsNode(Node):
    def __init__(self) -> None:
        super().__init__("fv_tts")
        self.declare_parameter("onnxruntime_filename", "")
        self.declare_parameter("open_jtalk_dict_dir", "")
        self.declare_parameter("voice_model_path", "")
        self.declare_parameter("acceleration_mode", "auto")
        self.declare_parameter("cpu_num_threads", 0)
        self.declare_parameter("style_id", 3)

        self._backend = VoicevoxCoreBackend(
            onnxruntime_filename=str(self.get_parameter("onnxruntime_filename").value),
            open_jtalk_dict_dir=str(self.get_parameter("open_jtalk_dict_dir").value),
            voice_model_path=str(self.get_parameter("voice_model_path").value),
            acceleration_mode=str(self.get_parameter("acceleration_mode").value),
            cpu_num_threads=int(self.get_parameter("cpu_num_threads").value),
            style_id=int(self.get_parameter("style_id").value),
        )
        # Load success alone does not prove that the selected style can synthesize.
        # Exercise the native path once without publishing the resulting PCM.
        self._backend.synthesize("起動確認")
        self._scheduler = SynthesisScheduler(
            self._backend.synthesize,
            self._publish_audio,
            self._on_synthesis_failed,
            self._on_synthesis_cancelled,
        )
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._agent_pub = self.create_publisher(AudioFrame, "/audio/agent/frame", qos)
        self._system_pub = self.create_publisher(AudioFrame, "/audio/system/frame", qos)
        self._result_pub = self.create_publisher(String, "/aspa/tts/result", qos)
        self.create_subscription(String, "/aspa/tts/say", self._on_say, qos)
        self.create_subscription(
            String, "/audio/playback/control", self._on_playback_control, qos
        )
        self._ready_service = self.create_service(
            Trigger, "/aspa/tts/ready", self._on_ready
        )

    @staticmethod
    def _on_ready(_request, response):
        response.success = True
        response.message = "voicevox_core synthesis smoke passed"
        return response

    def _on_say(self, msg: String) -> None:
        try:
            request = SayRequest.from_json(msg.data)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        try:
            status = self._scheduler.submit(request)
        except ValueError as exc:
            self._on_synthesis_failed(request, exc)
            return
        if status != "accepted":
            self.get_logger().warn(
                f"dropping {status} TTS request {request.utterance_id!r}"
            )

    def _on_playback_control(self, msg: String) -> None:
        try:
            control = PlaybackControl.from_json(msg.data)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        if control.action == "discard":
            removed = self._scheduler.advance_agent_floor(int(control.floor_epoch))
            if removed:
                self.get_logger().info(
                    f"cancelled {len(removed)} stale queued agent synthesis request(s)"
                )
        elif control.action == "system_abort":
            self._scheduler.cancel_system(control.utterance_id)

    def _publish_audio(self, request: SayRequest, audio: SynthesizedAudio) -> None:
        message = AudioFrame()
        now = self.get_clock().now()
        message.header.stamp = now.to_msg()
        message.source_id = "fv_tts"
        message.stream_id = request.utterance_id
        message.seq = 0
        message.sample_index = 0
        message.capture_time_ns = time.time_ns()
        message.frame_count = len(audio.pcm) // (audio.channels * 2)
        message.encoding = "PCM16LE"
        message.sample_rate_hz = audio.sample_rate_hz
        message.channels = audio.channels
        message.bit_depth = audio.bit_depth
        message.layout = "interleaved"
        message.data = array("B", audio.pcm)
        message.final = True
        publisher = self._agent_pub if request.kind == "agent" else self._system_pub
        publisher.publish(message)
        self._publish_result(TtsResult(request.kind, request.utterance_id, "completed"))

    def _on_synthesis_failed(self, request: SayRequest, exc: Exception) -> None:
        error = str(exc).strip() or type(exc).__name__
        self.get_logger().error(
            f"VOICEVOX synthesis failed for {request.utterance_id}: {error}"
        )
        self._publish_result(TtsResult(request.kind, request.utterance_id, "failed", error))

    def _on_synthesis_cancelled(self, request: SayRequest) -> None:
        self._publish_result(TtsResult(request.kind, request.utterance_id, "cancelled"))

    def _publish_result(self, result: TtsResult) -> None:
        self._result_pub.publish(String(data=result.to_json()))

    def destroy_node(self):
        self._scheduler.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FvTtsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
