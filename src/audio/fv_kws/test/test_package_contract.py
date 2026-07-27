from pathlib import Path
import tomllib
from xml.etree import ElementTree


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def test_package_is_a_single_rust_kws_node():
    cargo = tomllib.loads(
        (PACKAGE_DIR / "rust" / "Cargo.toml").read_text(encoding="utf-8")
    )
    assert cargo["package"]["name"] == "fv-kws-ros2"
    assert cargo["dependencies"]["vosk"] == "=0.3.1"
    assert "sherpa-onnx" not in cargo["dependencies"]
    assert cargo["bin"] == [
        {
            "name": "fv_kws",
            "path": "src/bin/fv_kws.rs",
            "required-features": ["ros2"],
        },
        {
            "name": "fv_kws_model_check",
            "path": "src/bin/fv_kws_model_check.rs",
        },
        {
            "name": "fv_kws_benchmark",
            "path": "src/bin/fv_kws_benchmark.rs",
        },
    ]

    cmake = (PACKAGE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "FV_KWS_LIBRARY_DIR" in cmake
    assert "0e9df29f060a93cf3df3263a4d3635e1b75688a5fd84e86ade1599372e3c9597" in cmake
    assert "scripts/setup-vosk-kws --install" in cmake
    assert "build --locked --release --features ros2" in cmake
    assert "fv_kws_benchmark" in cmake

    root = ElementTree.parse(PACKAGE_DIR / "package.xml").getroot()
    assert root.findtext("name") == "fv_kws"
    assert root.find("export/build_type").text == "ament_cmake"


def test_kws_uses_ttl_gated_vad_windows_and_one_resettable_recognizer():
    source = (
        PACKAGE_DIR / "rust" / "src" / "bin" / "fv_kws.rs"
    ).read_text(encoding="utf-8")
    library = (PACKAGE_DIR / "rust" / "src" / "lib.rs").read_text(
        encoding="utf-8"
    )
    assert 'subscribe::<AudioFrame>("/audio/mic/frame"' in source
    assert 'subscribe::<VoiceActivity>("/dialogue/vad/activity"' in source
    assert '"/aspa/dialogue/session_state"' in source
    assert 'INPUT_SOURCE_ID: &str = "fv_audio_aec"' in library
    assert 'INPUT_STREAM_ID: &str = "audio/mic/main"' in library
    spotter = (PACKAGE_DIR / "rust" / "src" / "spotter.rs").read_text(
        encoding="utf-8"
    )
    segmenter = (PACKAGE_DIR / "rust" / "src" / "segmenter.rs").read_text(
        encoding="utf-8"
    )
    assert "Recognizer::new_with_grammar" in spotter
    assert "self.recognizer.reset()" in spotter
    assert "final_result()" in spotter
    assert "DEFAULT_END_SILENCE_SAMPLES: usize = 3_072" in source
    assert "detected_sample_index" in segmenter
    assert "resample" not in source.lower()
    assert "KWS microphone sample discontinuity" in source
    assert "PcmActivityJoiner" in source
    assert "wake_sent" in source


def test_launch_respawns_the_fail_closed_node():
    launch = (PACKAGE_DIR / "launch" / "fv_kws.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'package="fv_kws"' in launch
    assert 'executable="fv_kws"' in launch
    assert "respawn=True" in launch
    assert 'DeclareLaunchArgument(\n            "kws_config"' in launch
    assert 'LaunchConfiguration("kws_config")' in launch
    assert '"ASPA_VOSK_KWS_RUNTIME_MANIFEST"' in launch
    assert '"LD_LIBRARY_PATH"' in launch
