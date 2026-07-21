import hashlib
from pathlib import Path
import subprocess

import pytest

from fv_episode_recorder.moss_runtime import (
    NATIVE_FINGERPRINT,
    THOR_FINGERPRINT,
    PreflightError,
    RuntimeContract,
    SnapshotFile,
    _check_official_revision,
    _resolve_checkpoint,
    _resolve_official_script,
    _validate_runtime_fingerprint,
    build_command,
)


def _checkpoint_fixture(tmp_path: Path) -> tuple[Path, dict[str, SnapshotFile]]:
    checkpoint = tmp_path / "checkpoint"
    checkpoint.mkdir()
    (checkpoint / "config.json").write_text("{}\n")
    (checkpoint / "model.safetensors.index.json").write_text("{}\n")
    (checkpoint / ".aspa-moss-vl-revision").write_text("model-revision\n")
    payload = b"verified model fixture"
    (checkpoint / "fixture.bin").write_bytes(payload)
    expected = {
        "fixture.bin": SnapshotFile(
            hashlib.sha256(payload).hexdigest(),
            len(payload),
        )
    }
    return checkpoint, expected


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


def test_repository_requires_exact_revision_and_clean_worktree(tmp_path: Path) -> None:
    repository = tmp_path / "MOSS-VL"
    repository.mkdir()
    subprocess.run(["git", "init", "-q", str(repository)], check=True)
    (repository / "README.md").write_text("MOSS-VL\n")
    subprocess.run(["git", "-C", str(repository), "add", "README.md"], check=True)
    subprocess.run(
        [
            "git",
            "-C",
            str(repository),
            "-c",
            "user.name=test",
            "-c",
            "user.email=test@example.com",
            "commit",
            "-q",
            "-m",
            "fixture",
        ],
        check=True,
    )
    revision = subprocess.check_output(
        ["git", "-C", str(repository), "rev-parse", "HEAD"],
        text=True,
    ).strip()

    _check_official_revision(str(repository), revision)
    with pytest.raises(PreflightError, match="revision mismatch"):
        _check_official_revision(str(repository), "0" * 40)
    (repository / "untracked.py").write_text("raise RuntimeError\n")
    with pytest.raises(PreflightError, match="not clean"):
        _check_official_revision(str(repository), revision)


def test_local_checkpoint_requires_verified_snapshot(tmp_path: Path) -> None:
    checkpoint, expected = _checkpoint_fixture(tmp_path)
    assert _resolve_checkpoint(
        str(checkpoint),
        allow_download=False,
        expected_revision="model-revision",
        expected_files=expected,
    ) == str(checkpoint.resolve())

    (checkpoint / "fixture.bin").write_bytes(b"tampered")
    with pytest.raises(PreflightError, match="size mismatch"):
        _resolve_checkpoint(
            str(checkpoint),
            allow_download=False,
            expected_revision="model-revision",
            expected_files=expected,
        )

    expected_size = expected["fixture.bin"].size
    assert expected_size is not None
    (checkpoint / "fixture.bin").write_bytes(b"x" * expected_size)
    with pytest.raises(PreflightError, match="SHA256 mismatch"):
        _resolve_checkpoint(
            str(checkpoint),
            allow_download=False,
            expected_revision="model-revision",
            expected_files=expected,
        )

    (checkpoint / "fixture.bin").write_bytes(b"verified model fixture")
    (checkpoint / "unverified.py").write_text("raise RuntimeError\n")
    with pytest.raises(PreflightError, match="unverified Python"):
        _resolve_checkpoint(
            str(checkpoint),
            allow_download=False,
            expected_revision="model-revision",
            expected_files=expected,
        )


def test_checkpoint_revision_and_runtime_downloads_fail_closed(tmp_path: Path) -> None:
    checkpoint, expected = _checkpoint_fixture(tmp_path)
    with pytest.raises(PreflightError, match="downloads are disabled"):
        _resolve_checkpoint(
            str(checkpoint),
            allow_download=True,
            expected_revision="model-revision",
            expected_files=expected,
        )
    with pytest.raises(PreflightError, match="revision mismatch"):
        _resolve_checkpoint(
            str(checkpoint),
            allow_download=False,
            expected_revision="another-revision",
            expected_files=expected,
        )


def test_thor_runtime_fingerprint_requires_sdpa_and_sm110() -> None:
    _validate_runtime_fingerprint(
        runtime_profile="thor",
        attention_backend="sdpa",
        versions=THOR_FINGERPRINT,
        cuda_version="13.0",
        machine="aarch64",
        device_name="NVIDIA Thor",
        device_capability=(11, 0),
    )
    with pytest.raises(PreflightError, match="attention_backend=sdpa"):
        _validate_runtime_fingerprint(
            runtime_profile="thor",
            attention_backend="flash_attention_2",
            versions=THOR_FINGERPRINT,
            cuda_version="13.0",
            machine="aarch64",
            device_name="NVIDIA Thor",
            device_capability=(11, 0),
        )
    with pytest.raises(PreflightError, match="GPU fingerprint mismatch"):
        _validate_runtime_fingerprint(
            runtime_profile="thor",
            attention_backend="sdpa",
            versions=THOR_FINGERPRINT,
            cuda_version="13.0",
            machine="aarch64",
            device_name="NVIDIA Thor",
            device_capability=(10, 0),
        )


def test_native_runtime_fingerprint_remains_exact() -> None:
    _validate_runtime_fingerprint(
        runtime_profile="native",
        attention_backend="flash_attention_2",
        versions=NATIVE_FINGERPRINT,
        cuda_version="12.8",
        machine="x86_64",
        device_name="fixture",
        device_capability=(9, 0),
    )
    wrong = {**NATIVE_FINGERPRINT, "flash_attn": "2.7.4"}
    with pytest.raises(PreflightError, match="flash_attn version mismatch"):
        _validate_runtime_fingerprint(
            runtime_profile="native",
            attention_backend="flash_attention_2",
            versions=wrong,
            cuda_version="12.8",
            machine="x86_64",
            device_name="fixture",
            device_capability=(9, 0),
        )
    with pytest.raises(PreflightError, match="requires x86_64"):
        _validate_runtime_fingerprint(
            runtime_profile="native",
            attention_backend="flash_attention_2",
            versions=NATIVE_FINGERPRINT,
            cuda_version="12.8",
            machine="aarch64",
            device_name="NVIDIA Thor",
            device_capability=(11, 0),
        )


def test_runtime_command_uses_official_websocket_service_directly(tmp_path: Path) -> None:
    contract = RuntimeContract(
        script=tmp_path / "run_online_inference.py",
        checkpoint="/models/MOSS-VL-Realtime",
        host="127.0.0.1",
        port=18081,
        attention_backend="sdpa",
        runtime_profile="thor",
    )

    command = build_command(contract)
    assert command[1] == str(contract.script)
    assert "--serve" in command
    assert command[command.index("--checkpoint") + 1] == contract.checkpoint
    assert command[command.index("--dtype") + 1] == "bfloat16"
    assert command[command.index("--attention-backend") + 1] == "sdpa"
