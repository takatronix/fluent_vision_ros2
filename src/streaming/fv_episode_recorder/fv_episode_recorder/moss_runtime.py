"""Fail-closed launcher for the official standalone MOSS realtime service."""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import os
import socket
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


DEFAULT_CHECKPOINT = "OpenMOSS-Team/MOSS-VL-Realtime"
DEFAULT_REPOSITORY_REVISION = "5cf6753156df2c8e7f38e36b55d28687e1df425e"
REQUIRED_VERSIONS = {
    "torch": "2.8.0",
    "transformers": "4.57.1",
    "flash_attn": "2.8.1",
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


def _env_flag(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in {"1", "true", "yes", "on"}


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Start the official MOSS-VL-Realtime FastAPI WebSocket service."
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
    parser.add_argument("--host", default=os.environ.get("MOSS_VL_HOST", "127.0.0.1"))
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("MOSS_VL_PORT", "18081")),
    )
    parser.add_argument(
        "--attention-backend",
        default=os.environ.get("MOSS_VL_ATTENTION_BACKEND", "flash_attention_2"),
    )
    parser.add_argument(
        "--allow-download",
        action="store_true",
        default=_env_flag("MOSS_VL_ALLOW_DOWNLOAD"),
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


def _require_version(module_name: str, required: str) -> None:
    actual = _module_version(module_name)
    matches = actual == required or actual.startswith(required + "+")
    if module_name == "torch":
        matches = matches or actual.startswith(required)
    if not matches:
        raise PreflightError(
            f"{module_name} version mismatch: required={required} actual={actual}"
        )


def _check_runtime_dependencies(attention_backend: str) -> None:
    _require_version("torch", REQUIRED_VERSIONS["torch"])
    _require_version("transformers", REQUIRED_VERSIONS["transformers"])
    if attention_backend == "flash_attention_2":
        _require_version("flash_attn", REQUIRED_VERSIONS["flash_attn"])

    torch = importlib.import_module("torch")
    if not torch.cuda.is_available():
        raise PreflightError("CUDA is not available to PyTorch")
    cuda_version = str(getattr(torch.version, "cuda", ""))
    if not cuda_version.startswith("12.8"):
        raise PreflightError(f"CUDA version mismatch: required=12.8 actual={cuda_version}")

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
    except (OSError, subprocess.CalledProcessError) as exc:
        raise PreflightError(
            f"MOSS-VL repository revision cannot be verified: {repository}"
        ) from exc
    if actual != expected_revision:
        raise PreflightError(
            f"MOSS-VL revision mismatch: required={expected_revision} actual={actual}"
        )


def _resolve_checkpoint(checkpoint: str, allow_download: bool) -> str:
    local_path = Path(checkpoint).expanduser()
    if local_path.exists():
        config = local_path / "config.json" if local_path.is_dir() else None
        if config is not None and not config.is_file():
            raise PreflightError(f"checkpoint config.json is missing: {local_path}")
        return str(local_path.resolve())
    if "/" not in checkpoint:
        raise PreflightError(f"checkpoint does not exist: {checkpoint}")
    if allow_download:
        return checkpoint

    try:
        from huggingface_hub import snapshot_download

        cached = snapshot_download(repo_id=checkpoint, local_files_only=True)
    except Exception as exc:
        raise PreflightError(
            f"checkpoint is not cached: {checkpoint}; preload it or explicitly allow download"
        ) from exc
    return str(Path(cached).resolve())


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
    script = _resolve_official_script(args.repo)
    _check_official_revision(args.repo, args.revision)
    _check_runtime_dependencies(args.attention_backend)
    checkpoint = _resolve_checkpoint(args.checkpoint, args.allow_download)
    _check_port(args.host, args.port)
    return RuntimeContract(
        script=script,
        checkpoint=checkpoint,
        host=args.host,
        port=args.port,
        attention_backend=args.attention_backend,
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
            f"checkpoint={contract.checkpoint} "
            f"endpoint=ws://{contract.host}:{contract.port}/v1/realtime"
        )
        return
    os.execv(command[0], command)


if __name__ == "__main__":
    main()
