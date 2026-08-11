from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def test_timing_collector_failure_shuts_down_the_audio_group() -> None:
    source = (PACKAGE_DIR / "launch" / "fv_audio.launch.py").read_text(
        encoding="utf-8",
    )
    collector_start = source.index("timing_collector = Node(")
    collector_end = source.index("\n    )", collector_start)
    collector = source[collector_start:collector_end]

    assert 'executable="tts_timing_collector"' in collector
    assert "respawn=False" in collector
    assert "on_exit=[" in collector
    assert "Shutdown(" in collector
    assert "cannot continue without timing evidence" in collector


def test_timing_receipt_store_configuration_belongs_only_to_collector() -> None:
    source = (PACKAGE_DIR / "launch" / "fv_audio.launch.py").read_text(
        encoding="utf-8",
    )

    for setting, default in (
        ("tts_timing_receipt_limit", "256"),
        ("tts_timing_timeout_seconds", "180"),
    ):
        assert (
            f'DeclareLaunchArgument("{setting}", default_value="{default}")'
            in source
        )
        assert f"{setting} = LaunchConfiguration(" in source
    assert "~/.aspa/tts-timing-receipts.json" in source
    assert source.count("FV_AUDIO_TTS_TIMING_RECEIPT_PATH") == 1
    assert source.count("FV_AUDIO_TTS_TIMING_RECEIPT_LIMIT") == 1
    assert source.count("FV_AUDIO_TTS_TIMING_TIMEOUT_SECONDS") == 1


def test_timing_receipts_retain_multiple_completed_requests() -> None:
    source = (
        PACKAGE_DIR / "rust" / "src" / "bin" / "tts_timing_collector.rs"
    ).read_text(encoding="utf-8")

    assert "const RECEIPT_QOS_DEPTH: usize = 64;" in source
    assert "depth: RECEIPT_QOS_DEPTH," in source
    assert "durability: DurabilityPolicy::TransientLocal," in source


def test_rust_codegen_is_limited_to_the_declared_audio_interfaces() -> None:
    source = (PACKAGE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    expected = (
        r"IDL_PACKAGE_FILTER=fv_audio_interfaces\;"
        r"fv_speech_interfaces\;std_msgs"
    )

    assert source.count(expected) == 2
