from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def test_launch_shuts_down_when_native_tts_exits():
    source = (PACKAGE_DIR / "launch" / "fv_tts.launch.py").read_text(
        encoding="utf-8"
    )

    assert source.count('package="fv_tts"') == 1
    assert source.count('executable="fv_tts_node"') == 1
    assert source.count("on_exit=EmitEvent(") == 1
    assert 'event=Shutdown(reason="VOICEVOX TTS exited")' in source


def test_synthesis_timeout_is_configured_and_forwarded_as_float():
    launch_source = (PACKAGE_DIR / "launch" / "fv_tts.launch.py").read_text(
        encoding="utf-8"
    )
    config_source = (PACKAGE_DIR / "config" / "default.yaml").read_text(
        encoding="utf-8"
    )

    assert (
        'DeclareLaunchArgument("synthesis_timeout_seconds", default_value="60.0")'
        in launch_source
    )
    assert 'LaunchConfiguration("synthesis_timeout_seconds")' in launch_source
    assert "value_type=float" in launch_source
    assert "synthesis_timeout_seconds: 60.0" in config_source


def test_native_startup_and_request_calls_are_both_guarded():
    node_source = (PACKAGE_DIR / "src" / "tts_node.cpp").read_text(
        encoding="utf-8"
    )
    scheduler_source = (
        PACKAGE_DIR / "src" / "synthesis_scheduler.cpp"
    ).read_text(encoding="utf-8")

    assert (
        'declare_parameter<double>("synthesis_timeout_seconds", 60.0)'
        in node_source
    )
    assert '"startup smoke synthesis"' in node_source
    assert "synthesis_timeout_);" in node_source
    assert "SynthesisWatchdog watchdog(synthesis_timeout_" in scheduler_source
    assert "request_synthesis_context(request)" in scheduler_source
