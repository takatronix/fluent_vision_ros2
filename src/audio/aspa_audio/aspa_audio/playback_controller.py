"""ROS 2 playback controller owning the robot's single playback stream."""

from __future__ import annotations

from array import array
import base64
from dataclasses import dataclass
import json
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from fluent_dialogue_dora_interfaces.msg import AudioFrame
from fluent_dialogue_dora_interfaces.srv import FlushAudio

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
        self._fatal_output_error: RuntimeError | None = None
        self._browser_boot_id = str(uuid.uuid4())
        self._browser_serial = 0
        self._browser_playout_epoch = 0
        self._browser_invalidated_kinds: tuple[str, ...] = ()

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._output_pub = self.create_publisher(AudioFrame, "/audio/output/frame", qos)
        self._event_pub = self.create_publisher(String, "/audio/playback/event", qos)
        browser_qos = QoSProfile(
            depth=256,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._browser_pub = self.create_publisher(
            String, "/audio/browser/projection", browser_qos
        )
        self._flush = self.create_client(FlushAudio, "/audio/output/flush")
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
            if kind == "system" and self._machine.system_is_aborted(msg.stream_id):
                self._assemblies.pop((kind, msg.stream_id), None)
                self.get_logger().warn(
                    f"dropping PCM for aborted SYSTEM utterance {msg.stream_id!r}"
                )
                return
            first_system_frame = (
                kind == "system"
                and msg.seq == 0
                and (kind, msg.stream_id) not in self._assemblies
            )
            item = self._assemble(kind, msg)
            if first_system_frame:
                reserve_events = self._machine.reserve_system(msg.stream_id)
                if any(
                    event.event == "agent_paused" for event in reserve_events
                ):
                    self._flush_device(account_ack=True, target_kind="agent")
                self._publish_events(reserve_events)
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
            self._publish_events(self._machine.enqueue(ready))
        except Exception as exc:  # ROS boundary: reject malformed audio visibly.
            self.get_logger().error(f"rejecting {kind} audio frame: {exc}")
            if kind == "system" and msg.stream_id:
                self._assemblies.pop((kind, msg.stream_id), None)
                self._publish_events(
                    self._machine.abort_system(msg.stream_id, release_hold=True),
                    fallback_event=PlaybackEvent(
                        "system_aborted", "system", msg.stream_id, 0, 0
                    ),
                    force_invalidated_kind="system",
                )

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
            fallback = PlaybackEvent("agent_paused", "agent", "", 0, 0)
            invalidated_kind = "agent"
        elif control.action == "resume":
            events = self._machine.resume_agent()
            fallback = PlaybackEvent("agent_resumed", "agent", "", 0, 0)
            invalidated_kind = None
        elif control.action == "discard":
            self._minimum_agent_epoch = int(control.floor_epoch)
            for key in tuple(self._assemblies):
                if (
                    key[0] == "agent"
                    and self._agent_epoch(key[1]) < self._minimum_agent_epoch
                ):
                    del self._assemblies[key]
            events = self._machine.discard_agent(
                control.release_hold,
                control.utterance_id,
                minimum_agent_epoch=self._minimum_agent_epoch,
            )
            self._flush_device(
                account_ack=False,
                target_kind="agent",
                minimum_agent_epoch=self._minimum_agent_epoch,
            )
            fallback = PlaybackEvent("agent_discarded", "agent", "", 0, 0)
            invalidated_kind = "agent"
        else:
            self._assemblies.pop(("system", control.utterance_id), None)
            events = self._machine.abort_system(
                control.utterance_id,
                release_hold=control.release_hold == "system",
            )
            self._flush_device(
                account_ack=False,
                target_kind="system",
                target_utterance_id=control.utterance_id,
            )
            fallback = PlaybackEvent(
                "system_aborted", "system", control.utterance_id, 0, 0
            )
            invalidated_kind = "system"
        self._publish_events(
            events,
            fallback_event=fallback,
            force_invalidated_kind=invalidated_kind,
        )

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
                if ack.seq == 0 and key not in self._suppressed_output_acks:
                    self._publish_events(
                        self._machine.output_accepted(ack.utterance_id, ack.seq)
                    )
                if not expected.final and self._inflight_key() == key:
                    self._output_inflight = None
                    self._output_inflight_kind = None
                self._maybe_start_flush()
                return
            terminal_before_start = (
                not pending.accepted
                and ack.status == "flushed"
                and ack.frame_count == 0
                and not ack.final
            )
            if not pending.accepted and not terminal_before_start:
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
            self._publish_browser_frame(chunk)
        self._publish_events(events)

    def _flush_device(
        self,
        *,
        account_ack: bool,
        target_kind: str,
        target_utterance_id: str | None = None,
        minimum_agent_epoch: int | None = None,
    ) -> None:
        def is_target(utterance_id: str) -> bool:
            if target_utterance_id is not None and utterance_id != target_utterance_id:
                return False
            if target_kind == "agent" and minimum_agent_epoch is not None:
                return self._agent_epoch(utterance_id) < minimum_agent_epoch
            return True

        matching_keys = {
            key
            for key, pending in self._output_pending.items()
            if should_flush_output(pending.kind, target_kind)
            and is_target(pending.expected.utterance_id)
        }
        device_matches = self._device_buffer.matches(
            target_kind, target_utterance_id
        ) and (
            self._device_buffer.utterance_id is not None
            and is_target(self._device_buffer.utterance_id)
        )
        if not matching_keys and not device_matches:
            return
        if not account_ack:
            self._suppressed_output_acks.update(matching_keys)
        if self._flush_pending:
            return
        if not self._flush.service_is_ready():
            raise self._fail_output_path("/audio/output/flush is unavailable")
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
        request.service_started = True
        ordered_keys = sorted(request.keys)
        flush_request = FlushAudio.Request()
        flush_request.utterance_ids = [key[0] for key in ordered_keys]
        flush_request.seqs = [key[1] for key in ordered_keys]
        future = self._flush.call_async(flush_request)

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
                self._fail_output_path(f"/audio/output/flush failed: {exc}")
            finally:
                request.service_done = True
                self._maybe_finish_flush()

        future.add_done_callback(finish_flush)

    def _fail_output_path(self, message: str) -> RuntimeError:
        error = RuntimeError(message)
        if self._fatal_output_error is None:
            self._fatal_output_error = error
            self.get_logger().fatal(message)
            self.context.try_shutdown()
        return self._fatal_output_error

    def _maybe_finish_flush(self) -> None:
        request = self._flush_request
        if request is None or not request.service_done or request.keys:
            return
        self._flush_request = None
        self._flush_pending = False

    def _publish_events(
        self,
        events: tuple[PlaybackEvent, ...],
        *,
        fallback_event: PlaybackEvent | None = None,
        force_invalidated_kind: str | None = None,
    ) -> None:
        invalidated_kinds = {
            "agent" if event.event in {"agent_paused", "agent_discarded"}
            else "system"
            for event in events
            if event.event in {"agent_paused", "agent_discarded", "system_aborted"}
        }
        if force_invalidated_kind is not None:
            invalidated_kinds.add(force_invalidated_kind)
        if invalidated_kinds:
            self._browser_playout_epoch += 1
            self._browser_invalidated_kinds = tuple(sorted(invalidated_kinds))
        for event in events:
            self._event_pub.publish(String(data=event.to_json()))
            self._publish_browser_event(event)
        if (
            fallback_event is not None
            and not any(event.event == fallback_event.event for event in events)
        ):
            self._publish_browser_event(fallback_event)

    def _publish_browser_event(self, event: PlaybackEvent) -> None:
        self._publish_browser_projection(
            {
                "packet_type": "state",
                "event": event.event,
                "kind": event.kind,
                "utterance_id": event.utterance_id,
                "played_frames": event.played_frames,
                "total_frames": event.total_frames,
            }
        )

    def _publish_browser_frame(self, chunk) -> None:
        self._publish_browser_projection(
            {
                "packet_type": "frame",
                "kind": chunk.kind,
                "utterance_id": chunk.utterance_id,
                "seq": chunk.seq,
                "sample_index": chunk.sample_index,
                "frame_count": chunk.frame_count,
                "encoding": "PCM16LE",
                "sample_rate_hz": chunk.sample_rate_hz,
                "channels": chunk.channels,
                "bit_depth": chunk.bit_depth,
                "layout": "interleaved",
                "final": chunk.final,
                "data": base64.b64encode(chunk.data).decode("ascii"),
            }
        )

    def _publish_browser_projection(self, packet: dict) -> None:
        self._browser_serial += 1
        payload = {
            "type": "audio_projection",
            "protocol_version": 1,
            "controller_boot_id": self._browser_boot_id,
            "serial": self._browser_serial,
            "playout_epoch": self._browser_playout_epoch,
            "invalidated_kinds": list(self._browser_invalidated_kinds),
            "agent_paused": self._machine.agent_pause_requested,
            **packet,
        }
        self._browser_pub.publish(
            String(data=json.dumps(payload, separators=(",", ":"), sort_keys=True))
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlaybackControllerNode()
    try:
        rclpy.spin(node)
    finally:
        fatal_output_error = node._fatal_output_error
        node.destroy_node()
        node.context.try_shutdown()
    if fatal_output_error is not None:
        raise fatal_output_error
