#!/usr/bin/env python3
from __future__ import annotations

from array import array
from concurrent.futures import ThreadPoolExecutor
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from fluent_dialogue_dora_interfaces.msg import AudioFrame

from .contracts import SayRequest
from .voicevox_backend import VoicevoxCoreBackend


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
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="voicevox-core")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._agent_pub = self.create_publisher(AudioFrame, "/audio/agent/frame", qos)
        self._system_pub = self.create_publisher(AudioFrame, "/audio/system/frame", qos)
        self.create_subscription(String, "/aspa/tts/say", self._on_say, qos)

    def _on_say(self, msg: String) -> None:
        try:
            request = SayRequest.from_json(msg.data)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        future = self._worker.submit(self._backend.synthesize, request.text)

        def completed(done) -> None:
            try:
                audio = done.result()
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
            except Exception as exc:  # Native synthesis boundary.
                self.get_logger().error(
                    f"VOICEVOX synthesis failed for {request.utterance_id}: {exc}"
                )

        future.add_done_callback(completed)

    def destroy_node(self):
        self._worker.shutdown(wait=True, cancel_futures=True)
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
