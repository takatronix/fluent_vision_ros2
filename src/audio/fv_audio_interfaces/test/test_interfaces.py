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
        "uint8 NONE=0",
        "uint8 USER=1",
        "uint8 SYSTEM=2",
        "uint8 action",
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
        "uint8 event",
        "uint8 kind",
        "string utterance_id",
        "uint64 played_frames",
        "uint64 total_frames",
        "bool source_position_valid",
        "uint64 played_source_characters",
        "uint64 total_source_characters",
        "uint64 playout_generation",
    ),
    "msg/SpeechMark.msg": (
        "uint64 source_start",
        "uint64 source_end",
        "string surface",
        "string pronunciation",
        "uint64 mora_start",
        "uint64 mora_end",
        "uint64 start_frame",
        "uint64 end_frame",
    ),
    "msg/SynthesizedSpeech.msg": (
        "uint8 AGENT=1",
        "uint8 SYSTEM=2",
        "uint8 kind",
        "string utterance_id",
        "uint32 sample_rate_hz",
        "uint32 channels",
        "uint32 bit_depth",
        "uint8[] pcm_s16le",
        "fv_audio_interfaces/SpeechMark[] marks",
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
