from pathlib import Path
import subprocess

import pytest

from fv_episode_recorder.moss_runtime import (
    PreflightError,
    RuntimeContract,
    _check_official_revision,
    _resolve_checkpoint,
    _resolve_official_script,
    build_command,
)


def test_resolves_only_official_realtime_layout(tmp_path: Path) -> None:
    repo = tmp_path / "MOSS-VL"
    runtime = repo / "realtime_inference"
    runtime.mkdir(parents=True)
    (runtime / "run_online_inference.py").write_text("# official entry\n")
    (runtime / "video_sources.py").write_text("# official source\n")

    assert _resolve_official_script(str(repo)) == (
        runtime / "run_online_inference.py"
    ).resolve()


def test_missing_runtime_fails_closed(tmp_path: Path) -> None:
    with pytest.raises(PreflightError):
        _resolve_official_script(str(tmp_path / "missing"))


def test_repository_revision_is_pinned(tmp_path: Path) -> None:
    repository = tmp_path / "MOSS-VL"
    repository.mkdir()
    subprocess.run(["git", "init", "-q", str(repository)], check=True)
    (repository / "README.md").write_text("MOSS-VL\n")
    subprocess.run(["git", "-C", str(repository), "add", "README.md"], check=True)
    subprocess.run(
        [
            "git", "-C", str(repository),
            "-c", "user.name=test",
            "-c", "user.email=test@example.com",
            "commit", "-q", "-m", "fixture",
        ],
        check=True,
    )
    revision = subprocess.check_output(
        ["git", "-C", str(repository), "rev-parse", "HEAD"],
        text=True,
    ).strip()

    _check_official_revision(str(repository), revision)
    with pytest.raises(PreflightError):
        _check_official_revision(str(repository), "0" * 40)


def test_local_checkpoint_requires_config(tmp_path: Path) -> None:
    checkpoint = tmp_path / "checkpoint"
    checkpoint.mkdir()
    with pytest.raises(PreflightError):
        _resolve_checkpoint(str(checkpoint), allow_download=False)

    (checkpoint / "config.json").write_text("{}\n")
    assert _resolve_checkpoint(str(checkpoint), allow_download=False) == str(
        checkpoint.resolve()
    )


def test_runtime_command_is_the_official_websocket_service(tmp_path: Path) -> None:
    contract = RuntimeContract(
        script=tmp_path / "run_online_inference.py",
        checkpoint="/models/MOSS-VL-Realtime",
        host="127.0.0.1",
        port=18081,
        attention_backend="flash_attention_2",
    )

    command = build_command(contract)
    assert command[1] == str(contract.script)
    assert "--serve" in command
    assert command[command.index("--checkpoint") + 1] == contract.checkpoint
    assert command[command.index("--dtype") + 1] == "bfloat16"
    assert command[command.index("--attention-backend") + 1] == "flash_attention_2"
