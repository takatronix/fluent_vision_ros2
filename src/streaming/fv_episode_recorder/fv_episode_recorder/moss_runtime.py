"""Fail-closed launcher for the pinned standalone MOSS realtime service."""

from __future__ import annotations

import argparse
import hashlib
import importlib
import importlib.util
import os
import platform
import socket
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Optional


DEFAULT_CHECKPOINT = "OpenMOSS-Team/MOSS-VL-Realtime"
DEFAULT_REPOSITORY_REVISION = "5cf6753156df2c8e7f38e36b55d28687e1df425e"
DEFAULT_MODEL_REVISION = "ca541ad7e41229fedbae0fec082d6bfe0c2ac9ff"
MODEL_REVISION_MARKER = ".aspa-moss-vl-revision"

NATIVE_FINGERPRINT = {
    "torch": "2.8.0+cu128",
    "torchvision": "0.23.0+cu128",
    "transformers": "4.57.1",
    "flash_attn": "2.8.1",
    "torchcodec": "0.7.0+cu128",
    "cuda": "12.8",
}
THOR_FINGERPRINT = {
    "torch": "2.8.0a0+34c6371d24.nv25.08",
    "torchvision": "0.23.0a0+428a54c9",
    "transformers": "4.57.1",
    "torchcodec": "0.7.0+7dd6092",
    "cuda": "13.0",
}


@dataclass(frozen=True)
class SnapshotFile:
    sha256: str
    size: Optional[int] = None


MODEL_FILES: Mapping[str, SnapshotFile] = {
    "chat_template.json": SnapshotFile(
        "d272c68bd115dca51b7e918dabac29b23510e3a25c4c5d9bd410b96eb11bf26e"
    ),
    "config.json": SnapshotFile(
        "ebab52596af175380fa83ca991c6c6ed7a9ac314fb044d13855e8b5eb2b5580a"
    ),
    "configuration_moss_vl.py": SnapshotFile(
        "0603337d493ec67d6a7838264dab6c92269f8f6f8db2285e5b420765562dbb40"
    ),
    "generation_config.json": SnapshotFile(
        "8b9afb365eaab1c8fc2395f618067ee2307b9821e207f12c5c52cf371a14b0e0"
    ),
    "model.safetensors.index.json": SnapshotFile(
        "25dc84c05625b956cfdb5fb5dd571ed4c725d9f42f42affd223d220d23bd0726"
    ),
    "model-00001-of-00005.safetensors": SnapshotFile(
        "3fe2d46a92e3c036e8dfbb2519f65415f8f1ab4f731ffdcb9c8bbff4e66fc3d6",
        5_274_500_800,
    ),
    "model-00002-of-00005.safetensors": SnapshotFile(
        "18f3579907933bad721053b3ac405c51045db4539fedcc7ee3c8db79e79f70e5",
        5_360_568_508,
    ),
    "model-00003-of-00005.safetensors": SnapshotFile(
        "cecf1e02afe5c7250bb43d6b6e99c7cb430a02afc07a2d697ab1dc4b5f784567",
        5_360_577_920,
    ),
    "model-00004-of-00005.safetensors": SnapshotFile(
        "57d794a266a1283d209d054114d9888a6a6b36a0af87fbf314c92b76c89dc294",
        5_366_957_460,
    ),
    "model-00005-of-00005.safetensors": SnapshotFile(
        "34009172ba732eb447caf1293126bba35eb934a622677ac7c24bb2d6af462b0d",
        1_310_247_928,
    ),
    "modeling_moss_vl.py": SnapshotFile(
        "4e356362c7438e684f031f2c763591674fec663e63e435e58d6a65d5dbe1e33b"
    ),
    "preprocessor_config.json": SnapshotFile(
        "94f62f9fd2d76b66f0a388d688c26ba639d555d4fa69c18efaa483a7f9ed62a7"
    ),
    "processing_moss_vl.py": SnapshotFile(
        "c03bc5dc41bde534a2d3513381a02151c277e3afbf2eace51e063e4dca46d653"
    ),
    "tokenizer.json": SnapshotFile(
        "e7bbd0f9784004df51aca562befc3c7a8f294b4045aa8685536c35804c9aa493"
    ),
    "tokenizer_config.json": SnapshotFile(
        "e23084208a8e96d3d54a5d96d586b1879d7c9de6046e88af5ac8d79dfa989512"
    ),
    "video_preprocessor_config.json": SnapshotFile(
        "f6828441758b19187559988fabac496c478fd3a6b684b1dd674f9f01fd5fcba4"
    ),
    "video_processing_moss_vl.py": SnapshotFile(
        "f1f3302f6792f3709bd4b05382ac2b0767dad74564b787231abba8daad656bdf"
    ),
    "vocab.json": SnapshotFile(
        "ca10d7e9fb3ed18575dd1e277a2579c16d108e32f27439684afa0e10b1440910"
    ),
}


class PreflightError(RuntimeError):
    pass


@dataclass(frozen=True)
class RuntimeContract:
    script: Path
    checkpoint: str
    host: str
    port: int
    attention_backend: str
    runtime_profile: str


def _env_flag(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in {"1", "true", "yes", "on"}


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Start the pinned MOSS-VL-Realtime FastAPI WebSocket service."
    )
    parser.add_argument("--repo", default=os.environ.get("MOSS_VL_REPO", "/opt/MOSS-VL"))
    parser.add_argument(
        "--revision",
        default=os.environ.get("MOSS_VL_REVISION", DEFAULT_REPOSITORY_REVISION),
    )
    parser.add_argument(
        "--checkpoint",
        default=os.environ.get("MOSS_VL_CHECKPOINT", DEFAULT_CHECKPOINT),
    )
    parser.add_argument(
        "--model-revision",
        default=os.environ.get("MOSS_VL_MODEL_REVISION", DEFAULT_MODEL_REVISION),
    )
    parser.add_argument("--host", default=os.environ.get("MOSS_VL_HOST", "127.0.0.1"))
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("MOSS_VL_PORT", "18081")),
    )
    parser.add_argument(
        "--runtime-profile",
        choices=("native", "thor"),
        default=os.environ.get("MOSS_VL_RUNTIME_PROFILE", "native"),
    )
    parser.add_argument(
        "--attention-backend",
        default=os.environ.get("MOSS_VL_ATTENTION_BACKEND", "flash_attention_2"),
    )
    parser.add_argument(
        "--allow-download",
        action="store_true",
        default=_env_flag("MOSS_VL_ALLOW_DOWNLOAD"),
        help="Retained for CLI compatibility; runtime downloads are rejected.",
    )
    parser.add_argument("--preflight-only", action="store_true")
    return parser.parse_args(argv)


def _module_version(module_name: str) -> str:
    try:
        module = importlib.import_module(module_name)
    except ImportError as exc:
        raise PreflightError(f"required Python module is missing: {module_name}") from exc
    version = str(getattr(module, "__version__", ""))
    if not version:
        raise PreflightError(f"cannot determine {module_name} version")
    return version


def _validate_runtime_fingerprint(
    runtime_profile: str,
    attention_backend: str,
    versions: Mapping[str, str],
    cuda_version: str,
    machine: str,
    device_name: str,
    device_capability: tuple[int, int],
) -> None:
    if runtime_profile == "thor":
        if attention_backend != "sdpa":
            raise PreflightError("Thor runtime requires attention_backend=sdpa")
        expected = THOR_FINGERPRINT
        if machine not in {"aarch64", "arm64"}:
            raise PreflightError(f"Thor runtime requires aarch64, got {machine}")
        if "NVIDIA Thor" not in device_name or device_capability != (11, 0):
            raise PreflightError(
                "Thor GPU fingerprint mismatch: "
                f"device={device_name!r} capability={device_capability}"
            )
    elif runtime_profile == "native":
        if attention_backend != "flash_attention_2":
            raise PreflightError(
                "native runtime requires attention_backend=flash_attention_2"
            )
        if machine != "x86_64":
            raise PreflightError(f"native runtime requires x86_64, got {machine}")
        expected = NATIVE_FINGERPRINT
    else:
        raise PreflightError(f"unsupported runtime profile: {runtime_profile}")

    for module_name in ("torch", "torchvision", "transformers", "torchcodec"):
        actual = versions.get(module_name, "")
        required = expected[module_name]
        if actual != required:
            raise PreflightError(
                f"{module_name} version mismatch: required={required} actual={actual}"
            )
    if runtime_profile == "native":
        actual_flash = versions.get("flash_attn", "")
        if actual_flash != expected["flash_attn"]:
            raise PreflightError(
                "flash_attn version mismatch: "
                f"required={expected['flash_attn']} actual={actual_flash}"
            )
    if cuda_version != expected["cuda"]:
        raise PreflightError(
            f"CUDA version mismatch: required={expected['cuda']} actual={cuda_version}"
        )


def _check_runtime_dependencies(runtime_profile: str, attention_backend: str) -> None:
    versions = {
        name: _module_version(name)
        for name in ("torch", "torchvision", "transformers", "torchcodec")
    }
    if runtime_profile == "native":
        versions["flash_attn"] = _module_version("flash_attn")

    torch = importlib.import_module("torch")
    if not torch.cuda.is_available():
        raise PreflightError("CUDA is not available to PyTorch")
    _validate_runtime_fingerprint(
        runtime_profile=runtime_profile,
        attention_backend=attention_backend,
        versions=versions,
        cuda_version=str(getattr(torch.version, "cuda", "")),
        machine=platform.machine(),
        device_name=str(torch.cuda.get_device_name(0)),
        device_capability=tuple(torch.cuda.get_device_capability(0)),
    )

    for module_name in ("fastapi", "uvicorn"):
        if importlib.util.find_spec(module_name) is None:
            raise PreflightError(f"service dependency is missing: {module_name}")
    if not any(importlib.util.find_spec(name) for name in ("websockets", "wsproto")):
        raise PreflightError("WebSocket transport is missing: install websockets or wsproto")


def _resolve_official_script(repo: str) -> Path:
    repository = Path(repo).expanduser().resolve()
    script = repository / "realtime_inference" / "run_online_inference.py"
    sibling = repository / "realtime_inference" / "video_sources.py"
    if not script.is_file() or not sibling.is_file():
        raise PreflightError(
            "official MOSS-VL realtime runtime was not found under "
            f"{repository}/realtime_inference"
        )
    return script


def _check_official_revision(repo: str, expected_revision: str) -> None:
    repository = Path(repo).expanduser().resolve()
    try:
        actual = subprocess.check_output(
            ["git", "-C", str(repository), "rev-parse", "HEAD"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
        dirty = subprocess.check_output(
            [
                "git",
                "-C",
                str(repository),
                "status",
                "--porcelain",
                "--untracked-files=all",
            ],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        raise PreflightError(
            f"MOSS-VL repository revision cannot be verified: {repository}"
        ) from exc
    if actual != expected_revision:
        raise PreflightError(
            f"MOSS-VL revision mismatch: required={expected_revision} actual={actual}"
        )
    if dirty:
        raise PreflightError(f"MOSS-VL repository worktree is not clean: {repository}")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(8 * 1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _verify_checkpoint_files(
    checkpoint: Path,
    expected_files: Optional[Mapping[str, SnapshotFile]] = None,
) -> None:
    files = MODEL_FILES if expected_files is None else expected_files
    unexpected_python = sorted(
        path.name for path in checkpoint.glob("*.py") if path.name not in files
    )
    if unexpected_python:
        raise PreflightError(
            "checkpoint contains unverified Python: " + ", ".join(unexpected_python)
        )
    for filename, expected in files.items():
        path = checkpoint / filename
        if not path.is_file():
            raise PreflightError(f"checkpoint file is missing: {path}")
        if expected.size is not None and path.stat().st_size != expected.size:
            raise PreflightError(
                "checkpoint file size mismatch: "
                f"file={filename} required={expected.size} actual={path.stat().st_size}"
            )
        actual_sha256 = _sha256(path)
        if actual_sha256 != expected.sha256:
            raise PreflightError(
                "checkpoint file SHA256 mismatch: "
                f"file={filename} required={expected.sha256} actual={actual_sha256}"
            )


def _resolve_checkpoint(
    checkpoint: str,
    allow_download: bool,
    expected_revision: str = DEFAULT_MODEL_REVISION,
    expected_files: Optional[Mapping[str, SnapshotFile]] = None,
) -> str:
    if allow_download:
        raise PreflightError(
            "runtime downloads are disabled; preload the pinned snapshot during setup"
        )
    local_path = Path(checkpoint).expanduser()
    if not local_path.is_dir():
        raise PreflightError(f"checkpoint must be a local directory: {checkpoint}")
    local_path = local_path.resolve()
    for required_file in ("config.json", "model.safetensors.index.json"):
        if not (local_path / required_file).is_file():
            raise PreflightError(f"checkpoint file is missing: {local_path / required_file}")
    marker = local_path / MODEL_REVISION_MARKER
    try:
        actual_revision = marker.read_text(encoding="ascii").strip()
    except OSError as exc:
        raise PreflightError(f"checkpoint revision marker is missing: {marker}") from exc
    if actual_revision != expected_revision:
        raise PreflightError(
            "checkpoint revision mismatch: "
            f"required={expected_revision} actual={actual_revision}"
        )
    _verify_checkpoint_files(local_path, expected_files)
    return str(local_path)


def _check_port(host: str, port: int) -> None:
    if not 1 <= port <= 65535:
        raise PreflightError(f"invalid port: {port}")
    probe_host = "" if host == "0.0.0.0" else host
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.bind((probe_host, port))
    except OSError as exc:
        raise PreflightError(f"cannot bind MOSS runtime to {host}:{port}: {exc}") from exc


def preflight(args: argparse.Namespace) -> RuntimeContract:
    if args.revision != DEFAULT_REPOSITORY_REVISION:
        raise PreflightError(
            f"source revision is fixed to {DEFAULT_REPOSITORY_REVISION}"
        )
    if args.model_revision != DEFAULT_MODEL_REVISION:
        raise PreflightError(f"model revision is fixed to {DEFAULT_MODEL_REVISION}")
    script = _resolve_official_script(args.repo)
    _check_official_revision(args.repo, args.revision)
    _check_runtime_dependencies(args.runtime_profile, args.attention_backend)
    checkpoint = _resolve_checkpoint(
        args.checkpoint,
        args.allow_download,
        expected_revision=args.model_revision,
    )
    _check_port(args.host, args.port)
    return RuntimeContract(
        script=script,
        checkpoint=checkpoint,
        host=args.host,
        port=args.port,
        attention_backend=args.attention_backend,
        runtime_profile=args.runtime_profile,
    )


def build_command(contract: RuntimeContract) -> list[str]:
    return [
        sys.executable,
        str(contract.script),
        "--serve",
        "--checkpoint",
        contract.checkpoint,
        "--host",
        contract.host,
        "--port",
        str(contract.port),
        "--dtype",
        "bfloat16",
        "--attention-backend",
        contract.attention_backend,
    ]


def main(argv: Optional[list[str]] = None) -> None:
    args = parse_args(argv)
    try:
        contract = preflight(args)
    except PreflightError as exc:
        print(f"MOSS-VL-Realtime preflight failed: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc

    command = build_command(contract)
    if args.preflight_only:
        print(
            "MOSS-VL-Realtime preflight passed: "
            f"profile={contract.runtime_profile} "
            f"checkpoint={contract.checkpoint} "
            f"endpoint=ws://{contract.host}:{contract.port}/v1/realtime"
        )
        return
    os.execv(command[0], command)


if __name__ == "__main__":
    main()
