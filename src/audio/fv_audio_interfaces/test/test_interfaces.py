from pathlib import Path
from xml.etree import ElementTree


PACKAGE_DIR = Path(__file__).resolve().parents[1]
MSG_DIR = PACKAGE_DIR / "msg"
SRV_DIR = PACKAGE_DIR / "srv"

EXPECTED = {
    "msg/AudioDevice.msg": (
        "uint8 INPUT=1",
        "uint8 OUTPUT=2",
        "uint8 direction",
        "string id",
        "string name",
        "string host",
        "bool is_default",
        "string manufacturer",
        "string driver",
        "string device_type",
        "string interface_type",
        "string address",
        "string[] details",
    ),
    "msg/PlaybackFrame.msg": (
        "uint8 AGENT=1",
        "uint8 SYSTEM=2",
        "uint8 CUE=3",
        "uint8 kind",
        "string request_id",
        "string generation_id",
        "string utterance_id",
        "uint64 seq",
        "uint64 sample_index",
        "uint32 frame_count",
        "uint32 sample_rate_hz",
        "uint32 channels",
        "uint8[] pcm_s16le",
        "bool final",
        "uint64 playout_generation",
    ),
    "msg/PlaybackControl.msg": (
        "uint8 PAUSE=1",
        "uint8 RESUME=2",
        "uint8 DISCARD=3",
        "uint8 ABORT_SYSTEM=4",
        "uint8 ABORT_AGENT=5",
        "uint8 NONE=0",
        "uint8 USER=1",
        "uint8 SYSTEM=2",
        "uint8 action",
        "string pause_id",
        "uint8 release_hold",
        "uint64 minimum_agent_epoch",
        "string utterance_id",
    ),
    "msg/PlaybackEvent.msg": (
        "uint8 AGENT=1",
        "uint8 SYSTEM=2",
        "uint8 CUE=3",
        "uint8 AGENT_STARTED=1",
        "uint8 SYSTEM_STARTED=2",
        "uint8 CUE_STARTED=3",
        "uint8 AGENT_PAUSED=4",
        "uint8 AGENT_RESUMED=5",
        "uint8 AGENT_DISCARDED=6",
        "uint8 AGENT_COMPLETED=7",
        "uint8 SYSTEM_COMPLETED=8",
        "uint8 CUE_COMPLETED=9",
        "uint8 SYSTEM_ABORTED=10",
        "uint8 AGENT_ABORTED=11",
        "uint8 AGENT_PAUSE_COMMITTED=12",
        "uint8 event",
        "uint8 kind",
        "string pause_id",
        "string request_id",
        "string generation_id",
        "string utterance_id",
        "uint64 played_frames",
        "bool total_frames_valid",
        "uint64 total_frames",
        "bool source_position_valid",
        "uint64 played_source_characters",
        "uint64 total_source_characters",
        "uint64 playout_generation",
    ),
    "msg/TtsAlignmentEvent.msg": (
        "uint64 sample_index",
        "uint64 committed_text_tokens",
    ),
    "msg/TtsSourceProgress.msg": (
        "uint64 committed_text_tokens",
        "uint64 source_char_end",
        "uint64 source_utf8_end",
    ),
    "msg/SynthesizedSpeechChunk.msg": (
        "uint8 AGENT=1",
        "uint8 SYSTEM=2",
        "uint8 AUDIO=1",
        "uint8 COMPLETE=2",
        "uint8 ABORT=3",
        "uint8 kind",
        "uint8 event",
        "string request_id",
        "string generation_id",
        "string utterance_id",
        "uint64 sequence",
        "uint64 first_sample_index",
        "uint32 sample_rate_hz",
        "uint32 channels",
        "float32[] pcm_f32",
        "bool final_chunk",
        "uint64 committed_text_tokens",
        "fv_audio_interfaces/TtsAlignmentEvent[] alignment_events",
        "string source_text",
        "fv_audio_interfaces/TtsSourceProgress[] source_progress",
    ),
    "msg/TtsRequest.msg": (
        "uint8 AGENT=1",
        "uint8 SYSTEM=2",
        "uint8 kind",
        "string request_id",
        "string utterance_id",
        "string text",
    ),
    "msg/TtsSay.msg": (
        "uint8 AGENT=1",
        "uint8 SYSTEM=2",
        "uint8 kind",
        "string request_id",
        "string generation_id",
        "string utterance_id",
        "string text",
    ),
    "msg/TtsResult.msg": (
        "uint8 AGENT=1",
        "uint8 SYSTEM=2",
        "uint8 COMPLETED=1",
        "uint8 FAILED=2",
        "uint8 CANCELLED=3",
        "uint8 REJECTED=4",
        "uint8 kind",
        "uint8 status",
        "string request_id",
        "bool generation_valid",
        "string generation_id",
        "string utterance_id",
        "string error",
    ),
    "msg/TtsRuntimeState.msg": (
        "uint8 STARTING=1",
        "uint8 READY=2",
        "uint8 QUIESCING=3",
        "uint8 STOPPED=4",
        "uint8 FAILED=5",
        "uint32 version",
        "string generation_id",
        "uint64 heartbeat_sequence",
        "uint8 state",
        "bool ready",
        "uint32 process_id",
        "string voice",
        "string engine",
        "string bundle_manifest_sha256",
        "string tokenizer_identity_sha256",
        "int32 cuda_device_index",
        "uint32 sample_rate_hz",
        "uint32 channels",
        "uint32 codec_frame_samples",
        "uint32 initial_frames",
        "uint32 steady_frames",
        "uint32 tail_min_frames",
        "uint32 tail_max_frames",
        "bool last_result_valid",
        "fv_audio_interfaces/TtsResult last_result",
        "string error",
    ),
    "msg/TtsTimingEvent.msg": (
        "uint32 TTS_TIMING_SCHEMA_VERSION=2",
        "uint8 AGENT=1",
        "uint8 SYSTEM=2",
        "uint8 REQUEST_ACCEPTED=1",
        "uint8 FRONTEND_COMPLETED=2",
        "uint8 NATIVE_REQUEST_STARTED=3",
        "uint8 FIRST_NATIVE_AUDIO=4",
        "uint8 FIRST_ROS_AUDIO_PUBLISHED=5",
        "uint8 FIRST_PLAYBACK_FRAME_PUBLISHED=6",
        "uint8 PHYSICAL_PLAYBACK_STARTED=7",
        "uint8 PHYSICAL_PLAYBACK_ENDED=8",
        "uint8 PLAYBACK_UNDERRUN=9",
        "uint32 version",
        "uint8 stage",
        "uint8 kind",
        "string request_id",
        "string generation_id",
        "string utterance_id",
        "uint64 monotonic_time_ns",
        "uint64 underrun_frames",
    ),
    "msg/TtsTimingReceipt.msg": (
        "uint32 TTS_TIMING_RECEIPT_SCHEMA_VERSION=1",
        "uint8 AGENT=1",
        "uint8 SYSTEM=2",
        "uint32 version",
        "uint8 kind",
        "string request_id",
        "string generation_id",
        "string utterance_id",
        "uint64 request_accepted_ns",
        "uint64 frontend_completed_ns",
        "uint64 native_request_started_ns",
        "uint64 first_native_audio_ns",
        "uint64 first_ros_audio_published_ns",
        "uint64 first_playback_frame_published_ns",
        "uint64 physical_playback_started_ns",
        "uint64 physical_playback_ended_ns",
        "uint64 total_underrun_frames",
    ),
    "msg/TtsAdmissionState.msg": (
        "uint32 TTS_ADMISSION_SCHEMA_VERSION=1",
        "uint32 version",
        "string controller_generation_id",
        "uint64 heartbeat_sequence",
        "bool generation_valid",
        "string generation_id",
        "bool admission_open",
        "string reason",
    ),
    "msg/VoiceMaintenanceState.msg": (
        "uint32 version",
        "bool active",
        "string operation_id",
        "string reason",
    ),
    "msg/OutputDrained.msg": (
        "uint8 ACCEPTED=1",
        "uint8 DRAINED=2",
        "uint8 FLUSHED=3",
        "string utterance_id",
        "uint64 seq",
        "uint32 frame_count",
        "bool final",
        "uint8 status",
    ),
    "srv/FlushAudio.srv": (
        "string[] utterance_ids",
        "uint64[] seqs",
        "---",
        "bool success",
        "string message",
    ),
    "srv/ListAudioDevices.srv": (
        "uint8 INPUT=1",
        "uint8 OUTPUT=2",
        "uint8 direction",
        "---",
        "bool success",
        "string message",
        "fv_audio_interfaces/AudioDevice[] devices",
    ),
    "srv/QuiesceTts.srv": (
        "string generation_id",
        "---",
        "bool success",
        "string generation_id",
        "uint32 cancelled_requests",
        "string error",
    ),
    "srv/SetTtsAdmission.srv": (
        "uint8 CLOSE=1",
        "uint8 OPEN=2",
        "uint8 action",
        "string generation_id",
        "---",
        "bool success",
        "bool admission_open",
        "string generation_id",
        "string error",
    ),
    "srv/SetVoiceMaintenance.srv": (
        "uint8 ENTER=1",
        "uint8 EXIT=2",
        "uint8 action",
        "string operation_id",
        "string reason",
        "---",
        "bool success",
        "fv_audio_interfaces/VoiceMaintenanceState state",
        "string error",
    ),
}


def declarations(path: Path) -> tuple[str, ...]:
    lines = []
    for line in path.read_text(encoding="utf-8").splitlines():
        declaration = line.split("#", 1)[0].strip()
        if declaration:
            lines.append(declaration)
    return tuple(lines)


def test_interface_files_and_declarations_are_exact():
    actual_files = {
        f"msg/{path.name}" for path in MSG_DIR.glob("*.msg")
    } | {
        f"srv/{path.name}" for path in SRV_DIR.glob("*.srv")
    }
    assert actual_files == set(EXPECTED)
    for relative_path, expected in EXPECTED.items():
        assert declarations(PACKAGE_DIR / relative_path) == expected


def test_build_metadata_declares_an_interface_only_package():
    cmake = (PACKAGE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    for relative_path in EXPECTED:
        assert relative_path in cmake
    assert "rosidl_generate_interfaces(${PROJECT_NAME}" in cmake
    assert "add_executable(" not in cmake
    assert "ament_python_install_package(" not in cmake

    root = ElementTree.parse(PACKAGE_DIR / "package.xml").getroot()
    assert root.findtext("name") == "fv_audio_interfaces"
    assert root.findtext("member_of_group") == "rosidl_interface_packages"
    assert root.find("export/build_type").text == "ament_cmake"
