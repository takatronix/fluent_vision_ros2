from hashlib import sha256
from pathlib import Path
import tomllib
from xml.etree import ElementTree


PACKAGE_DIR = Path(__file__).resolve().parents[1]
EXPECTED_MODEL_SHA256 = (
    "7ed98ddbad84ccac4cd0aeb3099049280713df825c610a8ed34543318f1b2c49"
)


def test_package_exports_three_speech_nodes_through_verified_runtime_wrapper():
    cargo = tomllib.loads(
        (PACKAGE_DIR / "rust" / "Cargo.toml").read_text(encoding="utf-8")
    )
    assert cargo["package"]["name"] == "fv-speech-ros2"
    assert {entry["name"] for entry in cargo["bin"]} == {
        "silero_vad",
        "turn_detector",
        "parakeet_asr",
        "fv_speech_runtime_exec",
    }
    assert cargo["features"] == {"default": [], "ros2": ["dep:r2r"]}
    ros_nodes = {
        entry["name"]: entry.get("required-features") for entry in cargo["bin"]
    }
    assert ros_nodes == {
        "silero_vad": ["ros2"],
        "turn_detector": ["ros2"],
        "parakeet_asr": ["ros2"],
        "fv_speech_runtime_exec": None,
    }
    for dependency in ("ort", "parakeet-rs"):
        features = cargo["dependencies"][dependency]["features"]
        assert "cuda" in features
        assert "tensorrt" not in features

    cmake = (PACKAGE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "find_package(fv_speech_interfaces REQUIRED)" in cmake
    assert "build --locked --release --features ros2" in cmake
    assert "LICENSE.SILERO-VAD" in cmake
    assert "THIRD_PARTY_NOTICES.md" in cmake
    for executable in (
        "fv_speech_runtime_exec",
        "silero_vad",
        "turn_detector",
        "parakeet_asr",
    ):
        assert f"release/{executable}" in cmake

    root = ElementTree.parse(PACKAGE_DIR / "package.xml").getroot()
    assert root.findtext("name") == "fv_speech"
    assert root.find("export/build_type").text == "ament_cmake"


def test_launch_owns_three_nodes_and_packaged_model():
    launch = (PACKAGE_DIR / "launch" / "fv_speech.launch.py").read_text(
        encoding="utf-8"
    )
    assert launch.count('package="fv_speech"') == 3
    assert launch.count('executable="fv_speech_runtime_exec"') == 2
    assert 'arguments=["parakeet_asr"]' in launch
    assert 'arguments=["silero_vad"]' in launch
    assert 'executable="turn_detector"' in launch
    assert '"models", "silero_vad_16k_op15.onnx"' in launch
    assert "OpaqueFunction" in launch
    assert '"ASPA_PARAKEET_RUNTIME_MANIFEST"' in launch
    assert '"CUDA_VISIBLE_DEVICES"' in launch
    assert "runtime_library_dirs" not in launch
    assert "ORT_DYLIB_PATH" not in launch
    assert "LD_LIBRARY_PATH" not in launch

    wrapper = (
        PACKAGE_DIR / "rust" / "src" / "bin" / "fv_speech_runtime_exec.rs"
    ).read_text(encoding="utf-8")
    assert "RuntimeManifest::load_verified" in wrapper
    assert '.env("ORT_DYLIB_PATH"' in wrapper
    assert '.env("LD_LIBRARY_PATH"' in wrapper
    assert ".exec()" in wrapper
    assert launch.count("respawn=True") == 3
    assert launch.count("respawn_delay=2.0") == 3
    assert "on_exit=EmitEvent(" not in launch

    model = PACKAGE_DIR / "models" / "silero_vad_16k_op15.onnx"
    assert sha256(model.read_bytes()).hexdigest() == EXPECTED_MODEL_SHA256
    assert model.stat().st_size == 1_289_603

    license_text = (PACKAGE_DIR / "LICENSE.SILERO-VAD").read_text(encoding="utf-8")
    notice = (PACKAGE_DIR / "THIRD_PARTY_NOTICES.md").read_text(encoding="utf-8")
    assert "Copyright (c) 2020-present Silero Team" in license_text
    assert "MIT License" in license_text
    assert EXPECTED_MODEL_SHA256 in notice
    assert "https://github.com/snakers4/silero-vad" in notice


def test_rust_ros_types_use_fv_speech_interfaces_only():
    library = (PACKAGE_DIR / "rust" / "src" / "lib.rs").read_text(
        encoding="utf-8"
    )
    rust_sources = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (PACKAGE_DIR / "rust" / "src").rglob("*.rs")
    )
    assert 'INPUT_SOURCE_ID: &str = "fv_audio_aec"' in library
    assert 'INPUT_STREAM_ID: &str = "audio/mic/main"' in library
    assert "r2r::fv_speech_interfaces::msg" in rust_sources
    assert "fluent_dialogue_dora_interfaces" not in rust_sources
    assert "aspa_dialogue_ros2" not in rust_sources
    assert "ort::ep::CUDA" in rust_sources
    assert ".with_tf32(false)" in rust_sources
    assert "RuntimeManifest::load_verified" in rust_sources
    assert "ort::ep::TensorRT" not in rust_sources
    assert 'provider != "cuda"' in rust_sources


def test_silero_vad_queue_matches_the_audio_publisher_history():
    source = (
        PACKAGE_DIR / "rust" / "src" / "bin" / "silero_vad.rs"
    ).read_text(encoding="utf-8")

    assert "speech_qos(256)" in source
