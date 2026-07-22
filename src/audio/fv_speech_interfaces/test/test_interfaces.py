from pathlib import Path
from xml.etree import ElementTree


PACKAGE_DIR = Path(__file__).resolve().parents[1]

EXPECTED = {
    "AsrControl.msg": (
        "std_msgs/Header header",
        "string action",
        "string session_id",
        "string user_turn_id",
        "string stream_id",
        "uint64 seq",
        "uint64 start_sample_index",
        "uint64 stop_sample_index",
        "string reason",
    ),
    "AudioFrame.msg": (
        "std_msgs/Header header",
        "string source_id",
        "string stream_id",
        "uint64 seq",
        "uint64 sample_index",
        "uint64 capture_time_ns",
        "uint32 frame_count",
        "string encoding",
        "uint32 sample_rate_hz",
        "uint32 channels",
        "uint32 bit_depth",
        "string layout",
        "uint8[] data",
        "bool final",
    ),
    "Transcript.msg": (
        "std_msgs/Header header",
        "string kind",
        "string session_id",
        "string user_turn_id",
        "string stream_id",
        "uint64 seq",
        "string text",
        "uint64 start_sample_index",
        "uint64 end_sample_index",
    ),
    "TurnEvent.msg": (
        "std_msgs/Header header",
        "string session_id",
        "string user_turn_id",
        "string stream_id",
        "uint64 seq",
        "uint64 sample_index",
        "string state",
        "bool confidence_present",
        "float32 confidence",
        "bool final",
    ),
    "VoiceActivity.msg": (
        "std_msgs/Header header",
        "string source_id",
        "string stream_id",
        "uint64 seq",
        "uint64 sample_index",
        "uint32 frame_count",
        "string state",
        "float32 speech_probability",
        "bool final",
    ),
}


def declarations(path: Path) -> tuple[str, ...]:
    return tuple(
        declaration
        for line in path.read_text(encoding="utf-8").splitlines()
        if (declaration := line.split("#", 1)[0].strip())
    )


def test_interface_surface_is_exact():
    actual = {path.name for path in (PACKAGE_DIR / "msg").glob("*.msg")}
    assert actual == set(EXPECTED)
    assert not (PACKAGE_DIR / "srv").exists()
    for filename, expected in EXPECTED.items():
        assert declarations(PACKAGE_DIR / "msg" / filename) == expected


def test_build_metadata_is_interface_only():
    cmake = (PACKAGE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "project(fv_speech_interfaces)" in cmake
    assert "rosidl_generate_interfaces(${PROJECT_NAME}" in cmake
    for filename in EXPECTED:
        assert f'"msg/{filename}"' in cmake
    assert "add_executable(" not in cmake
    assert "ament_python_install_package(" not in cmake

    root = ElementTree.parse(PACKAGE_DIR / "package.xml").getroot()
    assert root.findtext("name") == "fv_speech_interfaces"
    assert root.findtext("member_of_group") == "rosidl_interface_packages"
    assert root.find("export/build_type").text == "ament_cmake"
