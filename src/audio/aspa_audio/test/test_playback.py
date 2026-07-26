from pathlib import Path
from types import SimpleNamespace

import pytest

from aspa_audio.contracts import (
    MAX_SAFE_PLAYOUT_GENERATION,
    OutputDrained,
    PlaybackControl,
    PlaybackEvent,
    advance_playout_generation,
    playout_generation_seed,
)
from aspa_audio.playback_state import (
    DeviceBufferTracker,
    PlaybackMachine,
    Utterance,
    should_flush_output,
)

class MutableMessage:
    pass


def drained_message(ack: OutputDrained) -> SimpleNamespace:
    return SimpleNamespace(
        utterance_id=ack.utterance_id,
        seq=ack.seq,
        frame_count=ack.frame_count,
        final=ack.final,
        status={"accepted": 1, "drained": 2, "flushed": 3}[ack.status],
    )


def test_playout_generation_seed_survives_controller_restart_and_is_js_safe():
    first = playout_generation_seed(1_750_000_000_123_456_789)
    restarted = playout_generation_seed(1_750_000_001_123_456_789)

    assert first == 1_750_000_000_123
    assert restarted > first
    assert advance_playout_generation(restarted) == restarted + 1
    assert restarted < MAX_SAFE_PLAYOUT_GENERATION


def test_playout_generation_rejects_js_unsafe_values():
    with pytest.raises(ValueError, match="JavaScript safe integer"):
        advance_playout_generation(MAX_SAFE_PLAYOUT_GENERATION)
    with pytest.raises(ValueError, match="JavaScript safe integer"):
        playout_generation_seed((MAX_SAFE_PLAYOUT_GENERATION + 1) * 1_000_000)


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


def test_typed_contracts_are_strict():
    pause = PlaybackControl.from_message(
        SimpleNamespace(action=1, release_hold=0, minimum_agent_epoch=0, utterance_id="")
    )
    assert pause.action == "pause"
    discard = PlaybackControl.from_message(
        SimpleNamespace(action=3, release_hold=1, minimum_agent_epoch=3, utterance_id="")
    )
    assert discard.minimum_agent_epoch == 3
    assert discard.release_hold == "user"
    system_discard = PlaybackControl.from_message(
        SimpleNamespace(
            action=3,
            release_hold=2,
            minimum_agent_epoch=4,
            utterance_id="system-1",
        )
    )
    assert system_discard.utterance_id == "system-1"
    abort = PlaybackControl.from_message(
        SimpleNamespace(
            action=4,
            release_hold=0,
            minimum_agent_epoch=0,
            utterance_id="system-2",
        )
    )
    assert abort.utterance_id == "system-2"
    releasing_abort = PlaybackControl.from_message(
        SimpleNamespace(
            action=4,
            release_hold=2,
            minimum_agent_epoch=0,
            utterance_id="system-3",
        )
    )
    assert releasing_abort.release_hold == "system"
    event = PlaybackEvent("agent_paused", "agent", "u", 2, 4, 7)
    assert PlaybackEvent.from_message(event.to_message(MutableMessage)) == event
    ack = OutputDrained("u", 0, 20, True, "drained")
    assert OutputDrained.from_message(drained_message(ack)) == ack
    accepted = OutputDrained("u", 0, 20, False, "accepted")
    assert OutputDrained.from_message(drained_message(accepted)) == accepted


@pytest.mark.parametrize(
    ("factory", "match"),
    [
        (
            lambda: PlaybackControl.from_message(
                SimpleNamespace(
                    action=0,
                    release_hold=0,
                    minimum_agent_epoch=0,
                    utterance_id="",
                )
            ),
            "playback action",
        ),
        (
            lambda: PlaybackEvent.from_message(
                SimpleNamespace(
                    event=4,
                    kind=0,
                    utterance_id="u",
                    played_frames=0,
                    total_frames=0,
                    playout_generation=0,
                )
            ),
            "playback kind",
        ),
        (
            lambda: OutputDrained.from_message(
                SimpleNamespace(
                    utterance_id="u",
                    seq=0,
                    frame_count=0,
                    final=False,
                    status=0,
                )
            ),
            "output drained status",
        ),
    ],
)
def test_typed_contracts_reject_unknown_numeric_enums(factory, match):
    with pytest.raises(ValueError, match=match):
        factory()


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


def test_audio_launch_treats_capture_aec_playback_and_output_as_one_failure_domain():
    launch_file = Path(__file__).resolve().parents[1] / "launch" / "aspa_audio.launch.py"
    source = launch_file.read_text(encoding="utf-8")

    assert source.count("on_exit=EmitEvent(") == 4
    assert source.count("event=Shutdown(") == 4
    for executable in (
        "audio_capture",
        "audio_aec",
        "playback_controller",
        "audio_output",
    ):
        assert f'executable="{executable}"' in source


def test_legacy_python_playback_controller_is_not_a_production_executable():
    package_dir = Path(__file__).resolve().parents[1]
    setup_source = (package_dir / "setup.py").read_text(encoding="utf-8")
    cmake_source = (package_dir / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "playback_controller = aspa_audio.playback_controller:main" not in setup_source
    assert "scripts/playback_controller" not in cmake_source
    assert "release/playback_controller" in cmake_source


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
        drained_message(
            OutputDrained(
                expected.utterance_id,
                expected.seq,
                expected.frame_count,
                expected.final,
                "accepted",
            )
        ),
    )

    assert controller._output_inflight is None
    assert controller._output_pending[(expected.utterance_id, expected.seq)].accepted
    assert controller._machine.accepted_calls == [(expected.utterance_id, 0)]
    assert controller._machine.calls == []

    PlaybackControllerNode._on_output_drained(
        controller, drained_message(expected)
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
            controller, drained_message(ack)
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


def test_typed_playback_events_carry_monotonic_playout_generation() -> None:
    pytest.importorskip("rclpy")
    from aspa_audio.playback_controller import PlaybackControllerNode

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    controller = SimpleNamespace()
    controller._playout_generation = 0
    controller._event_pub = Publisher()

    PlaybackControllerNode._publish_events(
        controller,
        (
            PlaybackEvent("agent_paused", "agent", "agent-0-test", 2, 8),
            PlaybackEvent("agent_resumed", "agent", "agent-0-test", 2, 8),
        ),
    )
    PlaybackControllerNode._publish_events(
        controller,
        (PlaybackEvent("system_aborted", "system", "system-1", 0, 8),),
    )

    assert [message.playout_generation for message in controller._event_pub.messages] == [
        1,
        1,
        2,
    ]
    assert [message.event for message in controller._event_pub.messages] == [4, 5, 10]


def test_typed_fallback_event_invalidates_when_playback_control_is_a_noop() -> None:
    pytest.importorskip("rclpy")
    from aspa_audio.playback_controller import PlaybackControllerNode

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    controller = SimpleNamespace()
    controller._playout_generation = 0
    controller._event_pub = Publisher()

    PlaybackControllerNode._publish_events(
        controller,
        (),
        fallback_event=PlaybackEvent("agent_paused", "agent", "", 0, 0),
        force_invalidated_kind="agent",
    )

    assert len(controller._event_pub.messages) == 1
    event = controller._event_pub.messages[0]
    assert event.event == 4
    assert event.playout_generation == 1


def test_controller_publishes_unscaled_typed_playback_frame() -> None:
    pytest.importorskip("rclpy")
    from aspa_audio.playback_controller import PlaybackControllerNode

    source = b"\xe8\x03\x18\xfc"
    chunk = SimpleNamespace(
        kind="cue",
        utterance_id="cue-ready",
        seq=0,
        sample_index=0,
        frame_count=2,
        sample_rate_hz=48_000,
        channels=1,
        bit_depth=16,
        final=True,
        data=source,
    )

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    controller = SimpleNamespace(
        _output_inflight=None,
        _flush_pending=False,
        _machine=SimpleNamespace(next_chunk=lambda: (chunk, ())),
        _playout_generation=7,
        _output_pending={},
        _output_inflight_kind=None,
        _device_buffer=DeviceBufferTracker(),
        _output_pub=Publisher(),
        _publish_events=lambda _events: None,
    )

    PlaybackControllerNode._pump(controller)

    message = controller._output_pub.messages[0]
    assert bytes(message.pcm_s16le) == source
    assert message.kind == 3
    assert message.playout_generation == 7
