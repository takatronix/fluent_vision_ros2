"""Deterministic single-stream playback state machine."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Literal

from .contracts import PlaybackEvent, PlaybackHold, PlaybackKind


def should_flush_output(inflight_kind: str | None, target_kind: str) -> bool:
    if target_kind not in {"agent", "system"}:
        raise ValueError("flush target must be agent or system")
    return inflight_kind == target_kind


@dataclass
class Utterance:
    kind: PlaybackKind
    utterance_id: str
    pcm: bytes
    sample_rate_hz: int
    channels: int
    bit_depth: int
    offset_bytes: int = 0
    played_offset_frames: int = 0
    started: bool = False

    def __post_init__(self) -> None:
        if not self.utterance_id:
            raise ValueError("utterance_id must not be empty")
        if self.sample_rate_hz <= 0 or self.channels <= 0 or self.bit_depth != 16:
            raise ValueError("playback requires positive rate/channels and PCM16")
        if len(self.pcm) % self.bytes_per_frame:
            raise ValueError("PCM payload is not aligned to audio frames")

    @property
    def bytes_per_frame(self) -> int:
        return self.channels * (self.bit_depth // 8)

    @property
    def total_frames(self) -> int:
        return len(self.pcm) // self.bytes_per_frame

    @property
    def played_frames(self) -> int:
        return self.played_offset_frames


@dataclass(frozen=True)
class PlaybackChunk:
    utterance_id: str
    kind: PlaybackKind
    seq: int
    sample_index: int
    frame_count: int
    sample_rate_hz: int
    channels: int
    bit_depth: int
    data: bytes
    final: bool


@dataclass
class DeviceBufferTracker:
    """Tracks audio that can still be audible after its pacing acknowledgement."""

    kind: PlaybackKind | None = None
    utterance_id: str | None = None

    def note_chunk(self, kind: PlaybackKind, utterance_id: str) -> None:
        self.kind = kind
        self.utterance_id = utterance_id

    def note_ack(
        self,
        utterance_id: str,
        *,
        final: bool,
        status: Literal["drained", "flushed"],
    ) -> None:
        if self.utterance_id != utterance_id:
            return
        if final or status == "flushed":
            self.kind = None
            self.utterance_id = None

    def matches(
        self,
        target_kind: str,
        target_utterance_id: str | None = None,
    ) -> bool:
        return should_flush_output(self.kind, target_kind) and (
            target_utterance_id is None or self.utterance_id == target_utterance_id
        )

    def clear_matching(
        self,
        target_kind: str,
        target_utterance_id: str | None = None,
    ) -> None:
        if self.matches(target_kind, target_utterance_id):
            self.kind = None
            self.utterance_id = None


class PlaybackMachine:
    """Owns priority, long queues, pause/resume, and discard semantics."""

    def __init__(self, chunk_frames: int) -> None:
        if chunk_frames <= 0:
            raise ValueError("chunk_frames must be positive")
        self.chunk_frames = chunk_frames
        self.agent_queue: Deque[Utterance] = deque()
        self.system_queue: Deque[Utterance] = deque()
        self.cue_queue: Deque[Utterance] = deque()
        self.active: Utterance | None = None
        self.paused_agent: Utterance | None = None
        self._user_hold = False
        self._system_holds: set[str] = set()
        self.awaiting_drain: dict[str, Utterance] = {}

    @property
    def agent_pause_requested(self) -> bool:
        return self._user_hold or bool(self._system_holds)

    def enqueue(self, utterance: Utterance) -> tuple[PlaybackEvent, ...]:
        events: list[PlaybackEvent] = []
        if utterance.kind == "system":
            self._system_holds.add(utterance.utterance_id)
            events.extend(self._pause_active_agent())
            self.system_queue.append(utterance)
        elif utterance.kind == "cue":
            self.cue_queue.append(utterance)
        else:
            self.agent_queue.append(utterance)
        return tuple(events)

    def pause_agent(self) -> tuple[PlaybackEvent, ...]:
        self._user_hold = True
        return tuple(self._pause_active_agent())

    def resume_agent(self) -> tuple[PlaybackEvent, ...]:
        self._user_hold = False
        if self.agent_pause_requested or self.paused_agent is None:
            return ()
        paused = self.paused_agent
        self.paused_agent = None
        self.agent_queue.appendleft(paused)
        return (
            self._event("agent_resumed", paused),
        )

    def discard_agent(
        self,
        release_hold: PlaybackHold,
        utterance_id: str | None = None,
    ) -> tuple[PlaybackEvent, ...]:
        if release_hold not in {"user", "system"}:
            raise ValueError("release_hold must be user or system")
        if release_hold == "system" and not utterance_id:
            raise ValueError("system hold release requires utterance_id")
        if release_hold == "user" and utterance_id is not None:
            raise ValueError("user hold release must not specify utterance_id")
        discarded: list[Utterance] = []
        if self.active is not None and self.active.kind == "agent":
            discarded.append(self.active)
            self.active = None
        if self.paused_agent is not None:
            discarded.append(self.paused_agent)
            self.paused_agent = None
        discarded.extend(self.agent_queue)
        self.agent_queue.clear()
        for drain_utterance_id, item in tuple(self.awaiting_drain.items()):
            if item.kind == "agent":
                discarded.append(item)
                del self.awaiting_drain[drain_utterance_id]
        if release_hold == "user":
            self._user_hold = False
        else:
            self._system_holds.discard(utterance_id)
        return tuple(self._event("agent_discarded", item) for item in discarded)

    def abort_system(
        self, utterance_id: str, *, release_hold: bool = False
    ) -> tuple[PlaybackEvent, ...]:
        if not utterance_id:
            raise ValueError("system abort requires utterance_id")
        events: list[PlaybackEvent] = []
        if (
            self.active is not None
            and self.active.kind == "system"
            and self.active.utterance_id == utterance_id
        ):
            events.append(self._event("system_aborted", self.active))
            self.active = None
        queued = deque()
        while self.system_queue:
            item = self.system_queue.popleft()
            if item.utterance_id == utterance_id:
                events.append(self._event("system_aborted", item))
            else:
                queued.append(item)
        self.system_queue = queued
        draining = self.awaiting_drain.pop(utterance_id, None)
        if draining is not None and draining.kind == "system":
            events.append(self._event("system_aborted", draining))
        elif draining is not None:
            self.awaiting_drain[utterance_id] = draining
        if release_hold:
            self._system_holds.discard(utterance_id)
        if not self.agent_pause_requested and self.paused_agent is not None:
            paused = self.paused_agent
            self.paused_agent = None
            self.agent_queue.appendleft(paused)
            events.append(self._event("agent_resumed", paused))
        return tuple(events)

    def next_chunk(self) -> tuple[PlaybackChunk | None, tuple[PlaybackEvent, ...]]:
        events: list[PlaybackEvent] = []
        if self.active is None:
            self.active = self._select_next()
        item = self.active
        if item is None:
            return None, ()
        if not item.started:
            item.started = True
            events.append(self._event(f"{item.kind}_started", item))

        start = item.offset_bytes
        chunk_bytes = self.chunk_frames * item.bytes_per_frame
        end = min(len(item.pcm), start + chunk_bytes)
        data = item.pcm[start:end]
        sample_index = item.offset_bytes // item.bytes_per_frame
        item.offset_bytes = end
        final = end == len(item.pcm)
        chunk = PlaybackChunk(
            utterance_id=item.utterance_id,
            kind=item.kind,
            seq=sample_index // self.chunk_frames,
            sample_index=sample_index,
            frame_count=len(data) // item.bytes_per_frame,
            sample_rate_hz=item.sample_rate_hz,
            channels=item.channels,
            bit_depth=item.bit_depth,
            data=data,
            final=final,
        )
        if final:
            self.awaiting_drain[item.utterance_id] = item
            self.active = None
        return chunk, tuple(events)

    def output_drained(
        self,
        utterance_id: str,
        seq: int,
        frame_count: int,
        final: bool,
        status: Literal["drained", "flushed"] = "drained",
    ) -> tuple[PlaybackEvent, ...]:
        item = self._find_item(utterance_id)
        expected_seq = item.played_offset_frames // self.chunk_frames
        if seq != expected_seq:
            raise ValueError(f"non-contiguous output ack: {seq} != {expected_seq}")
        item.played_offset_frames += frame_count
        if item.played_offset_frames > item.total_frames:
            raise ValueError("output ack exceeds utterance length")
        if self.paused_agent is item:
            item.offset_bytes = item.played_offset_frames * item.bytes_per_frame
        expected_final = item.played_offset_frames == item.total_frames
        if status == "drained" and final != expected_final:
            raise ValueError("output ack final flag does not match drained length")
        if status == "flushed" and (final or expected_final):
            raise ValueError("flushed output ack must describe a partial chunk")
        if status == "flushed" and self.paused_agent is item:
            return (self._event("agent_paused", item),)
        if not final:
            return ()
        self.awaiting_drain.pop(utterance_id, None)
        if self.paused_agent is item:
            self.paused_agent = None
        return (self._event(f"{item.kind}_completed", item),)

    def _select_next(self) -> Utterance | None:
        if self.system_queue:
            return self.system_queue.popleft()
        if self.cue_queue:
            return self.cue_queue.popleft()
        if not self.agent_pause_requested and self.agent_queue:
            return self.agent_queue.popleft()
        return None

    def _pause_active_agent(self) -> list[PlaybackEvent]:
        paused = self.active if self.active is not None and self.active.kind == "agent" else None
        if paused is None:
            for utterance_id, item in tuple(self.awaiting_drain.items()):
                if item.kind == "agent":
                    paused = item
                    del self.awaiting_drain[utterance_id]
                    break
        if paused is None:
            return []
        if self.active is paused:
            self.active = None
        paused.offset_bytes = paused.played_offset_frames * paused.bytes_per_frame
        self.paused_agent = paused
        return [self._event("agent_paused", paused)]

    def _find_item(self, utterance_id: str) -> Utterance:
        candidates = [self.active, self.paused_agent, self.awaiting_drain.get(utterance_id)]
        candidates.extend(self.agent_queue)
        candidates.extend(self.system_queue)
        candidates.extend(self.cue_queue)
        for item in candidates:
            if item is not None and item.utterance_id == utterance_id:
                return item
        raise ValueError(f"output ack references unknown utterance: {utterance_id}")

    @staticmethod
    def _event(name: str, item: Utterance) -> PlaybackEvent:
        return PlaybackEvent(
            event=name,
            kind=item.kind,
            utterance_id=item.utterance_id,
            played_frames=item.played_frames,
            total_frames=item.total_frames,
        )
