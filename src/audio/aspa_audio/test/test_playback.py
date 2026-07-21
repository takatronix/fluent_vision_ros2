from pathlib import Path
from types import SimpleNamespace

import pytest

from aspa_audio.contracts import OutputDrained, PlaybackControl, PlaybackEvent
from aspa_audio.playback_state import (
    DeviceBufferTracker,
    PlaybackMachine,
    Utterance,
    should_flush_output,
)


def utterance(kind: str, name: str, frames: int = 8) -> Utterance:
    return Utterance(
        kind=kind,
        utterance_id=name,
        pcm=b"\x00\x00" * frames,
        sample_rate_hz=16_000,
        channels=1,
        bit_depth=16,
    )


def test_barge_in_pauses_then_resumes_at_same_offset():
    machine = PlaybackMachine(chunk_frames=2)
    machine.enqueue(utterance("agent", "a"))
    first, events = machine.next_chunk()
    assert first.sample_index == 0
    assert events == ()
    assert [event.event for event in machine.output_accepted("a", 0)] == [
        "agent_started"
    ]
    machine.output_drained("a", 0, 2, False)
    paused = machine.pause_agent()
    assert paused[0].played_frames == 2
    assert machine.next_chunk()[0] is None
    machine.resume_agent()
    resumed, _ = machine.next_chunk()
    assert resumed.sample_index == 2


def test_confirmed_barge_in_discards_paused_and_queued_agent_audio():
    machine = PlaybackMachine(chunk_frames=2)
    machine.enqueue(utterance("agent", "active"))
    machine.enqueue(utterance("agent", "queued"))
    machine.next_chunk()
    machine.output_drained("active", 0, 2, False)
    machine.pause_agent()
    events = machine.discard_agent("user")
    assert {event.utterance_id for event in events} == {"active", "queued"}
    assert machine.next_chunk()[0] is None


def test_floor_discard_preserves_new_agent_pcm_that_arrived_first():
    machine = PlaybackMachine(chunk_frames=2)
    machine.enqueue(utterance("agent", "agent-0-old"))
    machine.enqueue(utterance("agent", "agent-1-new"))
    machine.next_chunk()

    events = machine.discard_agent("user", minimum_agent_epoch=1)

    assert [event.utterance_id for event in events] == ["agent-0-old"]
    next_chunk, _ = machine.next_chunk()
    assert next_chunk.utterance_id == "agent-1-new"


def test_system_preempts_agent_but_only_abort_resumes_old_pcm():
    machine = PlaybackMachine(chunk_frames=8)
    machine.enqueue(utterance("agent", "agent"))
    chunk, _ = machine.next_chunk()
    assert chunk.final
    completed = machine.output_drained("agent", 0, 8, True)
    assert completed[0].event == "agent_completed"
    events = machine.enqueue(utterance("system", "system"))
    assert [event.event for event in events] == []  # agent already completed

    machine = PlaybackMachine(chunk_frames=2)
    machine.enqueue(utterance("agent", "agent"))
    machine.next_chunk()
    events = machine.enqueue(utterance("system", "system"))
    assert [event.event for event in events] == ["agent_paused"]
    chunk, events = machine.next_chunk()
    assert chunk.kind == "system"
    assert events == ()
    assert machine.output_accepted("system", 0)[0].event == "system_started"
    abort_events = machine.abort_system("system", release_hold=True)
    assert [event.event for event in abort_events] == ["system_aborted", "agent_resumed"]
    chunk, _ = machine.next_chunk()
    assert chunk.kind == "agent"


def test_system_reservation_pauses_before_pcm_is_ready_and_is_idempotent():
    machine = PlaybackMachine(chunk_frames=2)
    machine.enqueue(utterance("agent", "agent"))
    machine.next_chunk()

    events = machine.reserve_system("system")

    assert [event.event for event in events] == ["agent_paused"]
    assert machine.reserve_system("system") == ()
    assert machine.next_chunk()[0] is None
    assert [
        event.event
        for event in machine.abort_system("system", release_hold=True)
    ] == ["agent_resumed"]


def test_system_success_then_discard_never_resumes_old_agent_pcm():
    machine = PlaybackMachine(chunk_frames=2)
    machine.enqueue(utterance("agent", "agent"))
    machine.next_chunk()
    machine.output_drained("agent", 0, 2, False)
    machine.enqueue(utterance("system", "system", frames=2))
    machine.discard_agent("system", "system")
    chunk, _ = machine.next_chunk()
    assert chunk.kind == "system"
    completed = machine.output_drained("system", 0, 2, True)
    assert completed[0].event == "system_completed"
    assert machine.next_chunk()[0] is None


def test_system_holds_queued_agent_until_semantic_result():
    machine = PlaybackMachine(chunk_frames=2)
    machine.enqueue(utterance("agent", "queued"))
    machine.enqueue(utterance("system", "system", frames=2))
    system, _ = machine.next_chunk()
    assert system.kind == "system"
    machine.output_drained("system", 0, 2, True)
    assert machine.next_chunk()[0] is None
    machine.abort_system("system", release_hold=True)
    agent, _ = machine.next_chunk()
    assert agent.kind == "agent"


def test_successful_system_interruption_discards_queued_agent():
    machine = PlaybackMachine(chunk_frames=2)
    machine.enqueue(utterance("agent", "queued"))
    machine.enqueue(utterance("system", "system", frames=2))
    machine.next_chunk()
    machine.output_drained("system", 0, 2, True)
    discarded = machine.discard_agent("system", "system")
    assert [event.utterance_id for event in discarded] == ["queued"]
    assert machine.next_chunk()[0] is None


def test_system_discard_releases_requested_hold_with_agent_awaiting_drain():
    machine = PlaybackMachine(chunk_frames=2)
    machine.enqueue(utterance("agent", "agent-1", frames=2))
    machine.enqueue(utterance("agent", "agent-2", frames=2))
    machine.next_chunk()
    machine.next_chunk()

    machine.enqueue(utterance("system", "system", frames=2))
    system, _ = machine.next_chunk()
    machine.output_drained(system.utterance_id, system.seq, system.frame_count, system.final)

    discarded = machine.discard_agent("system", "system")

    assert {event.utterance_id for event in discarded} == {"agent-1", "agent-2"}
    assert not machine.agent_pause_requested


def test_json_contracts_are_strict():
    assert PlaybackControl.from_json('{"action":"pause"}').action == "pause"
    discard = PlaybackControl.from_json(
        '{"action":"discard","floor_epoch":3,"release_hold":"user"}'
    )
    assert discard.floor_epoch == 3
    assert discard.release_hold == "user"
    system_discard = PlaybackControl.from_json(
        '{"action":"discard","floor_epoch":4,"release_hold":"system",'
        '"utterance_id":"system-1"}'
    )
    assert system_discard.utterance_id == "system-1"
    abort = PlaybackControl.from_json(
        '{"action":"system_abort","utterance_id":"system-2"}'
    )
    assert abort.utterance_id == "system-2"
    releasing_abort = PlaybackControl.from_json(
        '{"action":"system_abort","release_hold":"system",'
        '"utterance_id":"system-3"}'
    )
    assert releasing_abort.release_hold == "system"
    event = PlaybackEvent("agent_paused", "agent", "u", 2, 4)
    assert PlaybackEvent.from_json(event.to_json()) == event
    ack = OutputDrained("u", 0, 20, True, "drained")
    assert OutputDrained.from_json(ack.to_json()) == ack
    accepted = OutputDrained("u", 0, 20, False, "accepted")
    assert OutputDrained.from_json(accepted.to_json()) == accepted


def test_ack_arriving_during_pause_advances_resume_position():
    machine = PlaybackMachine(chunk_frames=2)
    machine.enqueue(utterance("agent", "agent"))
    machine.next_chunk()
    machine.pause_agent()
    machine.output_drained("agent", 0, 2, False)
    machine.resume_agent()
    resumed, _ = machine.next_chunk()
    assert resumed.sample_index == 2


def test_final_ack_arriving_during_pause_completes_without_resume():
    machine = PlaybackMachine(chunk_frames=8)
    machine.enqueue(utterance("agent", "agent"))
    machine.next_chunk()
    machine.pause_agent()
    events = machine.output_drained("agent", 0, 8, True)
    assert events[0].event == "agent_completed"
    assert machine.resume_agent() == ()


def test_flush_before_ack_accounts_partial_heard_audio_then_resumes_there():
    machine = PlaybackMachine(chunk_frames=4)
    machine.enqueue(utterance("agent", "agent", frames=8))
    machine.next_chunk()
    machine.pause_agent()
    events = machine.output_drained("agent", 0, 1, False, "flushed")
    assert events[0].event == "agent_paused"
    assert events[0].played_frames == 1
    machine.resume_agent()
    resumed, _ = machine.next_chunk()
    assert resumed.sample_index == 1


def test_resume_before_flush_ack_uses_the_late_physical_playback_offset():
    machine = PlaybackMachine(chunk_frames=4)
    machine.enqueue(utterance("agent", "agent", frames=8))
    machine.next_chunk()
    machine.pause_agent()
    machine.resume_agent()

    machine.output_drained("agent", 0, 2, False, "flushed")

    resumed, _ = machine.next_chunk()
    assert resumed.sample_index == 2


def test_ack_before_flush_and_flush_before_ack_produce_same_resume_offset():
    ack_first = PlaybackMachine(chunk_frames=4)
    ack_first.enqueue(utterance("agent", "agent", frames=8))
    ack_first.next_chunk()
    ack_first.output_drained("agent", 0, 4, False, "drained")
    ack_first.pause_agent()
    ack_first.resume_agent()
    first_chunk, _ = ack_first.next_chunk()

    flush_first = PlaybackMachine(chunk_frames=4)
    flush_first.enqueue(utterance("agent", "agent", frames=8))
    flush_first.next_chunk()
    flush_first.pause_agent()
    flush_first.output_drained("agent", 0, 4, False, "drained")
    flush_first.resume_agent()
    second_chunk, _ = flush_first.next_chunk()
    assert first_chunk.sample_index == second_chunk.sample_index == 4


def test_system_abort_does_not_release_user_barge_hold():
    machine = PlaybackMachine(chunk_frames=2)
    machine.enqueue(utterance("agent", "agent"))
    machine.next_chunk()
    machine.pause_agent()
    machine.enqueue(utterance("system", "system", frames=2))
    events = machine.abort_system("system", release_hold=True)
    assert [event.event for event in events] == ["system_aborted"]
    assert machine.next_chunk()[0] is None
    assert [event.event for event in machine.resume_agent()] == ["agent_resumed"]


def test_user_false_start_does_not_release_system_hold():
    machine = PlaybackMachine(chunk_frames=2)
    machine.enqueue(utterance("agent", "agent"))
    machine.next_chunk()
    machine.enqueue(utterance("system", "system", frames=2))
    machine.pause_agent()
    assert machine.resume_agent() == ()
    machine.abort_system("system", release_hold=True)
    resumed, _ = machine.next_chunk()
    assert resumed.kind == "agent"


def test_system_discard_does_not_release_user_barge_hold():
    machine = PlaybackMachine(chunk_frames=2)
    machine.enqueue(utterance("agent", "old-agent"))
    machine.next_chunk()
    machine.enqueue(utterance("system", "system", frames=2))
    machine.pause_agent()

    machine.discard_agent("system", "system")
    machine.enqueue(utterance("agent", "new-agent"))
    system, _ = machine.next_chunk()
    assert system.kind == "system"
    machine.output_drained("system", 0, 2, True)
    assert machine.next_chunk()[0] is None

    machine.resume_agent()
    resumed, _ = machine.next_chunk()
    assert resumed.utterance_id == "new-agent"


def test_user_discard_does_not_release_system_hold():
    machine = PlaybackMachine(chunk_frames=2)
    machine.enqueue(utterance("agent", "old-agent"))
    machine.next_chunk()
    machine.enqueue(utterance("system", "system", frames=2))
    machine.pause_agent()

    machine.discard_agent("user")
    machine.enqueue(utterance("agent", "new-agent"))
    system, _ = machine.next_chunk()
    assert system.kind == "system"
    machine.output_drained("system", 0, 2, True)
    assert machine.next_chunk()[0] is None

    machine.abort_system("system", release_hold=True)
    resumed, _ = machine.next_chunk()
    assert resumed.utterance_id == "new-agent"


def test_agent_discard_never_flushes_inflight_system_pcm():
    assert not should_flush_output("system", "agent")
    assert should_flush_output("agent", "agent")
    assert should_flush_output("system", "system")


def test_system_audio_abort_retains_hold_until_semantic_result():
    machine = PlaybackMachine(chunk_frames=2)
    machine.enqueue(utterance("agent", "agent"))
    machine.next_chunk()
    machine.enqueue(utterance("system", "system", frames=2))

    events = machine.abort_system("system")

    assert [event.event for event in events] == ["system_aborted"]
    assert machine.agent_pause_requested
    assert machine.next_chunk()[0] is None

    released = machine.abort_system("system", release_hold=True)
    assert [event.event for event in released] == ["agent_resumed"]
    assert machine.next_chunk()[0].kind == "agent"


def test_device_buffer_remains_flushable_after_nonfinal_pacing_ack():
    tracker = DeviceBufferTracker()
    tracker.note_chunk("agent", "agent")
    tracker.note_ack("agent", final=False, status="drained")

    assert tracker.matches("agent")
    assert not tracker.matches("system")

    tracker.clear_matching("agent")
    assert not tracker.matches("agent")


def test_final_or_flushed_ack_clears_device_buffer_identity():
    tracker = DeviceBufferTracker()
    tracker.note_chunk("agent", "agent")
    tracker.note_ack("agent", final=False, status="flushed")
    assert not tracker.matches("agent")

    tracker.note_chunk("system", "system")
    tracker.note_ack("system", final=True, status="drained")
    assert not tracker.matches("system")


def test_agent_flush_never_matches_system_audio_left_on_device():
    tracker = DeviceBufferTracker()
    tracker.note_chunk("system", "system")

    assert not tracker.matches("agent")
    assert tracker.matches("system", "system")
    assert not tracker.matches("system", "other-system")


def test_audio_launch_treats_all_three_nodes_as_one_failure_domain():
    launch_file = Path(__file__).resolve().parents[1] / "launch" / "aspa_audio.launch.py"
    source = launch_file.read_text(encoding="utf-8")

    assert source.count("on_exit=EmitEvent(") == 3
    assert source.count("event=Shutdown(") == 3
    for executable in ("audio_capture", "playback_controller", "audio_output"):
        assert f'executable="{executable}"' in source


def test_controller_flushes_acknowledged_agent_device_buffer_but_not_system():
    pytest.importorskip("rclpy")
    from aspa_audio.playback_controller import PlaybackControllerNode

    class FakeFuture:
        def add_done_callback(self, callback):
            self.callback = callback

    class FakeFlushClient:
        def __init__(self):
            self.calls = 0

        def service_is_ready(self):
            return True

        def call_async(self, _request):
            self.calls += 1
            return FakeFuture()

    def controller_with_device(kind: str):
        device = DeviceBufferTracker()
        device.note_chunk(kind, kind)
        client = FakeFlushClient()
        controller = SimpleNamespace(
            _output_inflight=None,
            _output_inflight_kind=None,
            _output_pending={},
            _suppressed_output_acks=set(),
            _device_buffer=device,
            _flush_pending=False,
            _flush_request=None,
            _flush=client,
        )
        controller._maybe_start_flush = lambda: (
            PlaybackControllerNode._maybe_start_flush(controller)
        )
        return controller, client

    agent_controller, agent_client = controller_with_device("agent")
    PlaybackControllerNode._flush_device(
        agent_controller,
        account_ack=True,
        target_kind="agent",
    )
    assert agent_client.calls == 1

    system_controller, system_client = controller_with_device("system")
    PlaybackControllerNode._flush_device(
        system_controller,
        account_ack=False,
        target_kind="agent",
    )
    assert system_client.calls == 0


def test_controller_fails_closed_when_output_flush_service_disappears():
    pytest.importorskip("rclpy")
    from aspa_audio.playback_controller import PlaybackControllerNode

    class FlushClient:
        @staticmethod
        def service_is_ready():
            return False

    class Context:
        stopped = False

        def try_shutdown(self):
            self.stopped = True

    class Logger:
        messages = []

        def fatal(self, message):
            self.messages.append(message)

    class Harness:
        _fail_output_path = PlaybackControllerNode._fail_output_path

    controller = Harness()
    controller._output_pending = {}
    controller._suppressed_output_acks = set()
    controller._device_buffer = DeviceBufferTracker()
    controller._device_buffer.note_chunk("agent", "agent-0-old")
    controller._flush_pending = False
    controller._flush_request = None
    controller._flush = FlushClient()
    controller._fatal_output_error = None
    controller.context = Context()
    controller.get_logger = lambda: Logger()

    with pytest.raises(RuntimeError, match="flush is unavailable"):
        PlaybackControllerNode._flush_device(
            controller,
            account_ack=False,
            target_kind="agent",
        )

    assert controller.context.stopped
    assert controller._fatal_output_error is not None


def test_controller_fails_closed_when_output_flush_request_fails():
    pytest.importorskip("rclpy")
    from aspa_audio.playback_controller import (
        PlaybackControllerNode,
        _FlushRequest,
    )

    class FailedFuture:
        @staticmethod
        def result():
            raise RuntimeError("device output stopped")

        def add_done_callback(self, callback):
            callback(self)

    class FlushClient:
        @staticmethod
        def call_async(_request):
            return FailedFuture()

    class Context:
        stopped = False

        def try_shutdown(self):
            self.stopped = True

    class Logger:
        def fatal(self, _message):
            pass

    class Harness:
        _fail_output_path = PlaybackControllerNode._fail_output_path
        _maybe_finish_flush = PlaybackControllerNode._maybe_finish_flush

    controller = Harness()
    controller._flush_request = _FlushRequest(
        target_kind="agent",
        target_utterance_id=None,
        keys=set(),
    )
    controller._flush_pending = True
    controller._flush = FlushClient()
    controller._device_buffer = DeviceBufferTracker()
    controller._fatal_output_error = None
    controller.context = Context()
    controller.get_logger = lambda: Logger()

    PlaybackControllerNode._maybe_start_flush(controller)

    assert controller.context.stopped
    assert str(controller._fatal_output_error).endswith("device output stopped")


def test_accepted_ack_paces_output_without_advancing_played_position():
    pytest.importorskip("rclpy")
    from aspa_audio.playback_controller import (
        PlaybackControllerNode,
        _PendingOutput,
    )

    class Machine:
        def __init__(self):
            self.calls = []
            self.accepted_calls = []

        def output_accepted(self, *args):
            self.accepted_calls.append(args)
            return ()

        def output_drained(self, *args):
            self.calls.append(args)
            return ()

    class Logger:
        def error(self, message):
            raise AssertionError(message)

    class Harness:
        _inflight_key = PlaybackControllerNode._inflight_key
        _maybe_start_flush = PlaybackControllerNode._maybe_start_flush
        _maybe_finish_flush = PlaybackControllerNode._maybe_finish_flush

    expected = OutputDrained("agent-0-test", 0, 960, False, "drained")
    controller = Harness()
    controller._output_pending = {
        (expected.utterance_id, expected.seq): _PendingOutput(expected, "agent")
    }
    controller._output_inflight = expected
    controller._output_inflight_kind = "agent"
    controller._suppressed_output_acks = set()
    controller._device_buffer = DeviceBufferTracker()
    controller._device_buffer.note_chunk("agent", expected.utterance_id)
    controller._flush_request = None
    controller._flush_pending = False
    controller._machine = Machine()
    controller._publish_events = lambda _events: None
    controller.get_logger = lambda: Logger()

    PlaybackControllerNode._on_output_drained(
        controller,
        SimpleNamespace(
            data=OutputDrained(
                expected.utterance_id,
                expected.seq,
                expected.frame_count,
                expected.final,
                "accepted",
            ).to_json()
        ),
    )

    assert controller._output_inflight is None
    assert controller._output_pending[(expected.utterance_id, expected.seq)].accepted
    assert controller._machine.accepted_calls == [(expected.utterance_id, 0)]
    assert controller._machine.calls == []

    PlaybackControllerNode._on_output_drained(
        controller, SimpleNamespace(data=expected.to_json())
    )
    assert controller._output_pending == {}
    assert controller._machine.calls == [
        (expected.utterance_id, 0, 960, False, "drained")
    ]


def test_flush_accounts_only_played_prefix_across_resident_chunks():
    pytest.importorskip("rclpy")
    from aspa_audio.playback_controller import (
        PlaybackControllerNode,
        _FlushRequest,
        _PendingOutput,
    )

    class Logger:
        def error(self, message):
            raise AssertionError(message)

    class Harness:
        _inflight_key = PlaybackControllerNode._inflight_key
        _maybe_start_flush = PlaybackControllerNode._maybe_start_flush
        _maybe_finish_flush = PlaybackControllerNode._maybe_finish_flush

    machine = PlaybackMachine(chunk_frames=4)
    machine.enqueue(utterance("agent", "agent-0-test", frames=12))
    chunks = [machine.next_chunk()[0] for _ in range(3)]
    machine.pause_agent()
    expected = [
        OutputDrained(
            chunk.utterance_id,
            chunk.seq,
            chunk.frame_count,
            chunk.final,
            "drained",
        )
        for chunk in chunks
    ]
    keys = {(ack.utterance_id, ack.seq) for ack in expected}

    controller = Harness()
    controller._output_pending = {
        (ack.utterance_id, ack.seq): _PendingOutput(ack, "agent", accepted=True)
        for ack in expected
    }
    controller._output_inflight = expected[-1]
    controller._output_inflight_kind = "agent"
    controller._suppressed_output_acks = set()
    controller._device_buffer = DeviceBufferTracker()
    controller._device_buffer.note_chunk("agent", "agent-0-test")
    controller._flush_request = _FlushRequest(
        target_kind="agent",
        target_utterance_id=None,
        keys=keys,
        service_started=True,
        service_done=True,
    )
    controller._flush_pending = True
    controller._machine = machine
    controller._publish_events = lambda _events: None
    controller.get_logger = lambda: Logger()

    terminal = [
        expected[0],
        OutputDrained("agent-0-test", 1, 2, False, "flushed"),
        OutputDrained("agent-0-test", 2, 0, False, "flushed"),
    ]
    for ack in terminal:
        PlaybackControllerNode._on_output_drained(
            controller, SimpleNamespace(data=ack.to_json())
        )

    assert controller._output_pending == {}
    assert not controller._flush_pending
    machine.resume_agent()
    resumed, _ = machine.next_chunk()
    assert resumed.sample_index == 6


def test_overlapping_systems_each_hold_agent_until_their_semantic_result():
    machine = PlaybackMachine(chunk_frames=2)
    machine.enqueue(utterance("agent", "agent-old"))
    machine.next_chunk()
    machine.output_drained("agent-old", 0, 2, False)
    machine.enqueue(utterance("system", "system-1", frames=2))
    machine.enqueue(utterance("system", "system-2", frames=2))

    first, _ = machine.next_chunk()
    machine.output_drained(first.utterance_id, first.seq, first.frame_count, first.final)
    machine.discard_agent("system", "system-1")
    machine.enqueue(utterance("agent", "agent-after-system-1", frames=2))

    second, _ = machine.next_chunk()
    assert second.utterance_id == "system-2"
    machine.output_drained(second.utterance_id, second.seq, second.frame_count, second.final)
    assert machine.agent_pause_requested
    assert machine.next_chunk()[0] is None

    machine.discard_agent("system", "system-2")
    machine.enqueue(utterance("agent", "agent-after-system-2", frames=2))
    resumed, _ = machine.next_chunk()
    assert resumed.utterance_id == "agent-after-system-2"


def test_targeted_system_abort_preserves_other_system_and_its_hold():
    machine = PlaybackMachine(chunk_frames=2)
    machine.enqueue(utterance("agent", "agent"))
    machine.next_chunk()
    machine.enqueue(utterance("system", "system-1", frames=2))
    machine.enqueue(utterance("system", "system-2", frames=2))

    first, _ = machine.next_chunk()
    assert first.utterance_id == "system-1"
    aborted = machine.abort_system("system-2", release_hold=True)

    assert [event.utterance_id for event in aborted] == ["system-2"]
    assert machine.agent_pause_requested
    machine.output_drained(first.utterance_id, first.seq, first.frame_count, first.final)
    assert machine.next_chunk()[0] is None
    assert [
        event.event
        for event in machine.abort_system("system-1", release_hold=True)
    ] == [
        "agent_resumed"
    ]


def test_system_abort_tombstone_drops_cross_topic_late_pcm():
    machine = PlaybackMachine(chunk_frames=2)

    assert machine.abort_system("system-late", release_hold=True) == ()
    events = machine.enqueue(utterance("system", "system-late", frames=2))

    assert [event.event for event in events] == ["system_aborted"]
    assert not machine.agent_pause_requested
    assert machine.next_chunk()[0] is None


def test_browser_projection_orders_frames_and_invalidating_state() -> None:
    pytest.importorskip("rclpy")
    import json

    from aspa_audio.playback_controller import PlaybackControllerNode

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class Harness:
        _publish_browser_projection = (
            PlaybackControllerNode._publish_browser_projection
        )
        _publish_browser_frame = PlaybackControllerNode._publish_browser_frame

    controller = Harness()
    controller._browser_boot_id = "boot-a"
    controller._browser_serial = 0
    controller._browser_playout_epoch = 0
    controller._browser_invalidated_kinds = ()
    controller._machine = SimpleNamespace(agent_pause_requested=False)
    controller._browser_pub = Publisher()
    controller._event_pub = Publisher()

    controller._publish_browser_frame(SimpleNamespace(
        kind="agent",
        utterance_id="agent-0-test",
        seq=0,
        sample_index=0,
        frame_count=2,
        sample_rate_hz=16_000,
        channels=1,
        bit_depth=16,
        final=False,
        data=b"\x00\x00" * 2,
    ))
    PlaybackControllerNode._publish_events(
        controller,
        (
            PlaybackEvent("agent_paused", "agent", "agent-0-test", 2, 8),
            PlaybackEvent("agent_resumed", "agent", "agent-0-test", 2, 8),
        ),
    )
    controller._publish_browser_frame(SimpleNamespace(
        kind="agent",
        utterance_id="agent-0-test",
        seq=1,
        sample_index=2,
        frame_count=2,
        sample_rate_hz=16_000,
        channels=1,
        bit_depth=16,
        final=False,
        data=b"\x00\x00" * 2,
    ))

    packets = [json.loads(message.data) for message in controller._browser_pub.messages]
    assert [packet["serial"] for packet in packets] == [1, 2, 3, 4]
    assert [packet["playout_epoch"] for packet in packets] == [0, 1, 1, 1]
    assert [packet["packet_type"] for packet in packets] == [
        "frame", "state", "state", "frame"
    ]
    assert packets[0]["kind"] == "agent"
    assert packets[0]["controller_boot_id"] == "boot-a"
    assert packets[0]["protocol_version"] == 1
    assert packets[1]["invalidated_kinds"] == ["agent"]
    assert packets[1]["agent_paused"] is False


def test_browser_projection_invalidates_even_when_playback_control_is_a_noop() -> None:
    pytest.importorskip("rclpy")
    import json

    from aspa_audio.playback_controller import PlaybackControllerNode

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class Harness:
        _publish_browser_projection = (
            PlaybackControllerNode._publish_browser_projection
        )
        _publish_browser_event = PlaybackControllerNode._publish_browser_event

    controller = Harness()
    controller._browser_boot_id = "boot-a"
    controller._browser_serial = 0
    controller._browser_playout_epoch = 0
    controller._browser_invalidated_kinds = ()
    controller._machine = SimpleNamespace(agent_pause_requested=True)
    controller._browser_pub = Publisher()
    controller._event_pub = Publisher()

    PlaybackControllerNode._publish_events(
        controller,
        (),
        fallback_event=PlaybackEvent("agent_paused", "agent", "", 0, 0),
        force_invalidated_kind="agent",
    )

    assert controller._event_pub.messages == []
    packet = json.loads(controller._browser_pub.messages[0].data)
    assert packet["event"] == "agent_paused"
    assert packet["playout_epoch"] == 1
    assert packet["invalidated_kinds"] == ["agent"]
    assert packet["agent_paused"] is True
