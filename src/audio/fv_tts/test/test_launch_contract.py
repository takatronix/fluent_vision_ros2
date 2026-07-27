from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def test_launch_respawns_native_tts_as_its_own_recovery_domain():
    source = (PACKAGE_DIR / "launch" / "fv_tts.launch.py").read_text(
        encoding="utf-8"
    )

    assert source.count('package="fv_tts"') == 1
    assert source.count('executable="fv_tts_node"') == 1
    assert source.count("respawn=True") == 1
    assert source.count("respawn_delay=2.0") == 1
    assert "on_exit=EmitEvent(" not in source


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


def test_synthesis_cache_is_configurable_and_off_by_default():
    launch_source = (PACKAGE_DIR / "launch" / "fv_tts.launch.py").read_text(
        encoding="utf-8"
    )
    config_source = (PACKAGE_DIR / "config" / "default.yaml").read_text(
        encoding="utf-8"
    )
    node_source = (PACKAGE_DIR / "src" / "tts_node.cpp").read_text(
        encoding="utf-8"
    )

    # 置き場所を決めるのは配備側。パッケージ既定では永続化しない。
    assert (
        'DeclareLaunchArgument("cache_directory", default_value="")'
        in launch_source
    )
    assert (
        'DeclareLaunchArgument("cache_warmup_file", default_value="")'
        in launch_source
    )
    assert 'cache_directory: ""' in config_source
    assert 'cache_warmup_file: ""' in config_source
    for name in ("cache_memory_entries", "cache_disk_budget_mb"):
        assert f'DeclareLaunchArgument("{name}"' in launch_source
        assert f'LaunchConfiguration("{name}"), value_type=int' in launch_source

    # 合成の入口は必ずキャッシュを通る
    assert "return synthesize_cached(text);" in node_source
    assert "cache_->lookup(style_id, text)" in node_source
    # 話者が変わった合成結果は残さない
    assert "if (backend_->style_id() == style_id) {" in node_source


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
