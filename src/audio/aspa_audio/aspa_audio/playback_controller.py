"""ROS 2 playback controller owning the robot's single playback stream."""

from __future__ import annotations

from array import array
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from fluent_dialogue_dora_interfaces.msg import AudioFrame

from .contracts import OutputDrained, PlaybackControl, PlaybackEvent
from .gstreamer_pcm import GStreamerPcmConverter
from .playback_state import (
    DeviceBufferTracker,
    PlaybackMachine,
    Utterance,
    should_flush_output,
)


@dataclass
class _Assembly:
    sample_rate_hz: int
    channels: int
    bit_depth: int
    encoding: str
    next_seq: int
    data: bytearray


_OutputKey = tuple[str, int]


@dataclass
class _PendingOutput:
    expected: OutputDrained
    kind: str
    accepted: bool = False


@dataclass
class _FlushRequest:
    target_kind: str
    target_utterance_id: str | None
    keys: set[_OutputKey]
    partial_seen: bool = False
    service_started: bool = False
    service_done: bool = False


class PlaybackControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("playback_controller")
        self.declare_parameter("output_sample_rate_hz", 48000)
        self.declare_parameter("output_channels", 2)
        self.declare_parameter("chunk_ms", 20)
        self.declare_parameter("volume", 1.0)

        rate = int(self.get_parameter("output_sample_rate_hz").value)
        channels = int(self.get_parameter("output_channels").value)
        chunk_ms = int(self.get_parameter("chunk_ms").value)
        if chunk_ms <= 0 or rate * chunk_ms % 1000:
            raise RuntimeError("chunk_ms must produce an integral output frame count")
        self._machine = PlaybackMachine(rate * chunk_ms // 1000)
        self._converter = GStreamerPcmConverter(
            rate,
            channels,
            float(self.get_parameter("volume").value),
        )
        self._assemblies: dict[tuple[str, str], _Assembly] = {}
        self._minimum_agent_epoch = 0
        self._output_inflight: OutputDrained | None = None
        self._output_inflight_kind: str | None = None
        self._output_pending: dict[_OutputKey, _PendingOutput] = {}
        self._suppressed_output_acks: set[_OutputKey] = set()
        self._device_buffer = DeviceBufferTracker()
        self._flush_pending = False
        self._flush_request: _FlushRequest | None = None

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._output_pub = self.create_publisher(AudioFrame, "/audio/output/frame", qos)
        self._event_pub = self.create_publisher(String, "/audio/playback/event", qos)
        self._flush = self.create_client(Trigger, "/audio/output/flush")
        self.create_subscription(
            AudioFrame, "/audio/agent/frame", lambda msg: self._on_frame("agent", msg), qos
        )
        self.create_subscription(
            AudioFrame, "/audio/system/frame", lambda msg: self._on_frame("system", msg), qos
        )
        self.create_subscription(
            AudioFrame, "/audio/cue/frame", lambda msg: self._on_frame("cue", msg), qos
        )
        self.create_subscription(String, "/audio/playback/control", self._on_control, qos)
        self.create_subscription(String, "/audio/output/drained", self._on_output_drained, qos)
        self.create_timer(chunk_ms / 1000.0, self._pump)

    def _on_frame(self, kind: str, msg: AudioFrame) -> None:
        try:
            if kind == "agent" and self._agent_epoch(msg.stream_id) < self._minimum_agent_epoch:
                self.get_logger().warn(
                    f"dropping stale agent PCM {msg.stream_id!r}; "
                    f"minimum floor is {self._minimum_agent_epoch}"
                )
                return
            item = self._assemble(kind, msg)
            if item is None:
                return
            converted = self._converter.convert(
                item.pcm,
                sample_rate_hz=item.sample_rate_hz,
                channels=item.channels,
                bit_depth=item.bit_depth,
                encoding="PCM16LE",
            )
            ready = Utterance(
                kind=item.kind,
                utterance_id=item.utterance_id,
                pcm=converted,
                sample_rate_hz=self._converter.output_rate_hz,
                channels=self._converter.output_channels,
                bit_depth=16,
            )
            events = self._machine.enqueue(ready)
            if kind == "system" and any(event.event == "agent_paused" for event in events):
                self._flush_device(account_ack=True, target_kind="agent")
            self._publish_events(events)
        except Exception as exc:  # ROS boundary: reject malformed audio visibly.
            self.get_logger().error(f"rejecting {kind} audio frame: {exc}")

    def _assemble(self, kind: str, msg: AudioFrame) -> Utterance | None:
        if msg.encoding != "PCM16LE" or msg.layout != "interleaved":
            raise ValueError("audio must be PCM16LE/interleaved")
        if not msg.stream_id:
            raise ValueError("stream_id is the required utterance_id")
        if msg.frame_count * msg.channels * (msg.bit_depth // 8) != len(msg.data):
            raise ValueError("AudioFrame frame_count does not match payload")
        key = (kind, msg.stream_id)
        assembly = self._assemblies.get(key)
        if assembly is None:
            if msg.seq != 0:
                raise ValueError("utterance must begin at seq=0")
            assembly = _Assembly(
                sample_rate_hz=msg.sample_rate_hz,
                channels=msg.channels,
                bit_depth=msg.bit_depth,
                encoding=msg.encoding,
                next_seq=0,
                data=bytearray(),
            )
            self._assemblies[key] = assembly
        if msg.seq != assembly.next_seq:
            raise ValueError(f"non-contiguous audio seq: {msg.seq} != {assembly.next_seq}")
        if (
            msg.sample_rate_hz,
            msg.channels,
            msg.bit_depth,
            msg.encoding,
        ) != (
            assembly.sample_rate_hz,
            assembly.channels,
            assembly.bit_depth,
            assembly.encoding,
        ):
            raise ValueError("audio format changed inside an utterance")
        assembly.data.extend(msg.data)
        assembly.next_seq += 1
        if not msg.final:
            return None
        del self._assemblies[key]
        return Utterance(
            kind=kind,
            utterance_id=msg.stream_id,
            pcm=bytes(assembly.data),
            sample_rate_hz=assembly.sample_rate_hz,
            channels=assembly.channels,
            bit_depth=assembly.bit_depth,
        )

    def _on_control(self, msg: String) -> None:
        try:
            control = PlaybackControl.from_json(msg.data)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        if control.action == "pause":
            events = self._machine.pause_agent()
            self._flush_device(account_ack=True, target_kind="agent")
        elif control.action == "resume":
            events = self._machine.resume_agent()
        elif control.action == "discard":
            self._minimum_agent_epoch = int(control.floor_epoch)
            events = self._machine.discard_agent(
                control.release_hold, control.utterance_id
            )
            self._flush_device(account_ack=False, target_kind="agent")
        else:
            events = self._machine.abort_system(
                control.utterance_id,
                release_hold=control.release_hold == "system",
            )
            self._flush_device(
                account_ack=False,
                target_kind="system",
                target_utterance_id=control.utterance_id,
            )
        self._publish_events(events)

    def _on_output_drained(self, msg: String) -> None:
        try:
            ack = OutputDrained.from_json(msg.data)
            key = (ack.utterance_id, ack.seq)
            pending = self._output_pending.get(key)
            if pending is None:
                raise ValueError("output ack does not match a pending chunk")
            expected = pending.expected
            if ack.status == "accepted":
                if pending.accepted:
                    raise ValueError("duplicate output accepted ack")
                if (
                    ack.frame_count != expected.frame_count
                    or ack.final != expected.final
                ):
                    raise ValueError("accepted ack does not match the complete chunk")
                pending.accepted = True
                if not expected.final and self._inflight_key() == key:
                    self._output_inflight = None
                    self._output_inflight_kind = None
                self._maybe_start_flush()
                return
            if not pending.accepted:
                raise ValueError("terminal output ack arrived before accepted ack")
            if ack.status == "drained" and ack != expected:
                raise ValueError("drained ack does not match the complete pending chunk")
            if ack.status == "flushed" and not (
                0 <= ack.frame_count < expected.frame_count and not ack.final
            ):
                raise ValueError("flushed ack does not describe a partial pending chunk")
            self._device_buffer.note_ack(
                ack.utterance_id,
                final=ack.final,
                status=ack.status,
            )
            del self._output_pending[key]
            if self._inflight_key() == key:
                self._output_inflight = None
                self._output_inflight_kind = None

            flush = self._flush_request
            belongs_to_flush = flush is not None and key in flush.keys
            account_progress = key not in self._suppressed_output_acks
            if belongs_to_flush and flush.partial_seen:
                account_progress = False
            if account_progress:
                self._publish_events(
                    self._machine.output_drained(
                        ack.utterance_id,
                        ack.seq,
                        ack.frame_count,
                        ack.final,
                        ack.status,
                    )
                )
            if belongs_to_flush:
                if ack.status == "flushed":
                    flush.partial_seen = True
                flush.keys.discard(key)
            self._suppressed_output_acks.discard(key)
            self._maybe_finish_flush()
        except ValueError as exc:
            self.get_logger().error(str(exc))

    def _inflight_key(self) -> _OutputKey | None:
        if self._output_inflight is None:
            return None
        return (
            self._output_inflight.utterance_id,
            self._output_inflight.seq,
        )

    @staticmethod
    def _agent_epoch(utterance_id: str) -> int:
        parts = utterance_id.split("-", 2)
        if len(parts) != 3 or parts[0] != "agent":
            raise ValueError("agent utterance_id must be agent-<floor_epoch>-<id>")
        try:
            epoch = int(parts[1])
        except ValueError as exc:
            raise ValueError("agent utterance_id floor epoch is invalid") from exc
        if epoch < 0:
            raise ValueError("agent utterance_id floor epoch is negative")
        return epoch

    def _pump(self) -> None:
        if self._output_inflight is not None or self._flush_pending:
            return
        chunk, events = self._machine.next_chunk()
        if chunk is not None:
            msg = AudioFrame()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.source_id = "aspa_playback_controller"
            msg.stream_id = chunk.utterance_id
            msg.seq = chunk.seq
            msg.sample_index = chunk.sample_index
            msg.capture_time_ns = 0
            msg.frame_count = chunk.frame_count
            msg.encoding = "PCM16LE"
            msg.sample_rate_hz = chunk.sample_rate_hz
            msg.channels = chunk.channels
            msg.bit_depth = chunk.bit_depth
            msg.layout = "interleaved"
            msg.data = array("B", chunk.data)
            msg.final = chunk.final
            expected = OutputDrained(
                utterance_id=chunk.utterance_id,
                seq=chunk.seq,
                frame_count=chunk.frame_count,
                final=chunk.final,
                status="drained",
            )
            key = (chunk.utterance_id, chunk.seq)
            if key in self._output_pending:
                raise RuntimeError(f"duplicate pending output chunk: {key!r}")
            self._output_pending[key] = _PendingOutput(expected, chunk.kind)
            self._output_inflight = expected
            self._output_inflight_kind = chunk.kind
            self._device_buffer.note_chunk(chunk.kind, chunk.utterance_id)
            self._output_pub.publish(msg)
        self._publish_events(events)

    def _flush_device(
        self,
        *,
        account_ack: bool,
        target_kind: str,
        target_utterance_id: str | None = None,
    ) -> None:
        matching_keys = {
            key
            for key, pending in self._output_pending.items()
            if should_flush_output(pending.kind, target_kind)
            and (
                target_utterance_id is None
                or pending.expected.utterance_id == target_utterance_id
            )
        }
        device_matches = self._device_buffer.matches(
            target_kind, target_utterance_id
        )
        if not matching_keys and not device_matches:
            return
        if not account_ack:
            self._suppressed_output_acks.update(matching_keys)
        if self._flush_pending:
            return
        if not self._flush.service_is_ready():
            self.get_logger().error("/audio/output/flush is unavailable")
            return
        self._flush_pending = True
        self._flush_request = _FlushRequest(
            target_kind=target_kind,
            target_utterance_id=target_utterance_id,
            keys=matching_keys,
        )
        self._maybe_start_flush()

    def _maybe_start_flush(self) -> None:
        request = self._flush_request
        if request is None or request.service_started:
            return
        if any(
            not self._output_pending[key].accepted
            for key in request.keys
            if key in self._output_pending
        ):
            return
        request.service_started = True
        future = self._flush.call_async(Trigger.Request())

        def finish_flush(done) -> None:
            try:
                response = done.result()
                if response is None or not response.success:
                    message = "no response" if response is None else response.message
                    raise RuntimeError(message)
                self._device_buffer.clear_matching(
                    request.target_kind, request.target_utterance_id
                )
            except Exception as exc:  # ROS service boundary.
                self.get_logger().error(f"/audio/output/flush failed: {exc}")
            finally:
                request.service_done = True
                self._maybe_finish_flush()

        future.add_done_callback(finish_flush)

    def _maybe_finish_flush(self) -> None:
        request = self._flush_request
        if request is None or not request.service_done or request.keys:
            return
        self._flush_request = None
        self._flush_pending = False

    def _publish_events(self, events: tuple[PlaybackEvent, ...]) -> None:
        for event in events:
            self._event_pub.publish(String(data=event.to_json()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlaybackControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
