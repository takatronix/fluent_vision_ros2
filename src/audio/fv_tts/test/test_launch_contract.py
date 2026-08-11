from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parents[1]


def test_launch_respawns_magpie_tts_as_its_own_recovery_domain():
    source = (PACKAGE_DIR / "launch" / "fv_tts.launch.py").read_text(
        encoding="utf-8"
    )

    assert source.count('package="fv_tts"') == 1
    assert source.count('executable="fv_tts_node"') == 1
    assert source.count("respawn=True") == 1
    assert source.count("respawn_delay=2.0") == 1
    assert "on_exit=EmitEvent(" not in source


def test_all_native_frontend_and_watchdog_timeouts_are_explicit():
    launch_source = (PACKAGE_DIR / "launch" / "fv_tts.launch.py").read_text(
        encoding="utf-8"
    )
    config_source = (PACKAGE_DIR / "config" / "default.yaml").read_text(
        encoding="utf-8"
    )
    compact_launch_source = "".join(launch_source.split())

    for name, default in (
        ("frontend_timeout_seconds", "2.0"),
        ("request_progress_timeout_seconds", "30.0"),
        ("cancellation_timeout_seconds", "2.0"),
        ("startup_timeout_seconds", "300.0"),
        ("timing_collector_discovery_timeout_seconds", "10.0"),
        ("scheduler_watchdog_seconds", "5.0"),
    ):
        assert (
            f'DeclareLaunchArgument("{name}",default_value="{default}")'
            in compact_launch_source
        )
        assert f'LaunchConfiguration("{name}")' in compact_launch_source
        assert f"{name}: {default}" in config_source


def test_runtime_and_frontend_assets_are_explicit_environment_inputs():
    launch_source = (PACKAGE_DIR / "launch" / "fv_tts.launch.py").read_text(
        encoding="utf-8"
    )
    config_source = (PACKAGE_DIR / "config" / "default.yaml").read_text(
        encoding="utf-8"
    )
    for parameter, environment in (
        ("native_library", "MAGPIE_TTS_RT_NATIVE_LIBRARY"),
        ("bundle_path", "MAGPIE_TTS_RT_BUNDLE"),
        ("manifest_sha256", "MAGPIE_TTS_RT_MANIFEST_SHA256"),
        ("frontend_python", "MAGPIE_TTS_RT_FRONTEND_PYTHON"),
        ("frontend_server", "MAGPIE_TTS_RT_FRONTEND_SERVER"),
        ("frontend_lock", "MAGPIE_TTS_RT_FRONTEND_LOCK"),
        ("frontend_contract", "MAGPIE_TTS_RT_FRONTEND_CONTRACT"),
    ):
        assert f'"{parameter}"' in launch_source
        assert f'"{environment}"' in launch_source
        assert f'{parameter}: ""' in config_source


def test_voicevox_and_mutable_voice_apis_are_removed():
    package_files = [
        path
        for path in PACKAGE_DIR.rglob("*")
        if path.is_file()
        and "__pycache__" not in path.parts
        and ".cache" not in path.parts
        and path != Path(__file__).resolve()
        and path.name != "README.md"
    ]
    source = "\n".join(
        path.read_text(encoding="utf-8", errors="ignore") for path in package_files
    )
    assert "VOICEVOX" not in source
    assert "/aspa/tts/settings" not in source
    assert "/aspa/tts/voices" not in source
    assert "style_id" not in source
    assert "voicevox_backend" not in source


def test_rust_codegen_is_limited_to_the_declared_tts_interfaces():
    source = (PACKAGE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")

    assert source.count(
        r"IDL_PACKAGE_FILTER=fv_audio_interfaces\;std_srvs"
    ) == 2
