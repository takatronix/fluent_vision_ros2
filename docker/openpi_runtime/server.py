#!/usr/bin/env python3
from __future__ import annotations

import base64
import io
import json
import os
import time
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional
from urllib.parse import urlparse

import numpy as np
from PIL import Image

from openpi.policies import policy_config
from openpi.training import config as training_config


def _runtime_info() -> dict:
    info = {}
    try:
        import jax

        devices = [str(d) for d in jax.devices()]
        info["jax"] = {
            "version": getattr(jax, "__version__", ""),
            "backend": jax.default_backend(),
            "devices": devices,
        }
    except Exception as exc:  # pragma: no cover - diagnostics only
        info["jax"] = {"error": str(exc)}

    try:
        import torch

        info["torch"] = {
            "version": getattr(torch, "__version__", ""),
            "cuda_available": bool(torch.cuda.is_available()),
            "device_count": int(torch.cuda.device_count()) if torch.cuda.is_available() else 0,
        }
    except Exception as exc:  # pragma: no cover - diagnostics only
        info["torch"] = {"error": str(exc)}
    return info


@dataclass
class PolicyBundle:
    policy_id: str
    config_name: str
    checkpoint_dir: str
    default_prompt: str
    policy: object | None = None
    error: str = ""

    def load(self) -> None:
        try:
            cfg = training_config.get_config(self.config_name)
            self.policy = policy_config.create_trained_policy(
                cfg,
                self.checkpoint_dir,
                default_prompt=self.default_prompt or None,
            )
            self.error = ""
        except Exception as exc:  # pragma: no cover - runtime init path
            self.policy = None
            self.error = str(exc)

    @property
    def ready(self) -> bool:
        return self.policy is not None and not self.error


def _parse_policy_specs() -> dict[str, PolicyBundle]:
    raw = os.environ.get("OPENPI_POLICY_SPECS", "").strip()
    specs: dict[str, PolicyBundle] = {}
    if raw:
        for item in raw.split(";"):
            item = item.strip()
            if not item:
                continue
            parts = item.split(":", 2)
            if len(parts) < 3:
                continue
            policy_id, config_name, remainder = parts
            checkpoint_dir, default_prompt = remainder, ""
            if "|" in remainder:
                checkpoint_dir, default_prompt = remainder.split("|", 1)
            specs[policy_id] = PolicyBundle(
                policy_id=policy_id,
                config_name=config_name,
                checkpoint_dir=checkpoint_dir,
                default_prompt=default_prompt,
            )
    if specs:
        return specs

    policy_id = os.environ.get("OPENPI_POLICY_ID", "pi0").strip() or "pi0"
    mode = os.environ.get("OPENPI_POLICY_MODE", "default").strip().lower()
    if mode == "checkpoint":
        config_name = os.environ.get("OPENPI_POLICY_CONFIG", "pi0_aloha_sim")
        checkpoint_dir = os.environ.get("OPENPI_POLICY_DIR", "gs://openpi-assets/checkpoints/pi0_aloha_sim")
    else:
        env_mode = os.environ.get("OPENPI_ENV", "aloha_sim").strip().lower()
        if env_mode == "droid":
            config_name = "pi05_droid"
            checkpoint_dir = "gs://openpi-assets/checkpoints/pi05_droid"
        elif env_mode == "libero":
            config_name = "pi05_libero"
            checkpoint_dir = "gs://openpi-assets/checkpoints/pi05_libero"
        else:
            config_name = "pi0_aloha_sim"
            checkpoint_dir = "gs://openpi-assets/checkpoints/pi0_aloha_sim"
    return {
        policy_id: PolicyBundle(
            policy_id=policy_id,
            config_name=config_name,
            checkpoint_dir=checkpoint_dir,
            default_prompt=os.environ.get("OPENPI_DEFAULT_PROMPT", ""),
        )
    }


POLICIES = _parse_policy_specs()
for _bundle in POLICIES.values():
    _bundle.load()


def _decode_image(encoded: str) -> np.ndarray:
    raw = base64.b64decode(encoded.encode("ascii"))
    with Image.open(io.BytesIO(raw)) as image:
        return np.asarray(image.convert("RGB"))


def _to_aloha_obs(payload: dict, bundle: PolicyBundle) -> dict:
    state = np.asarray(payload.get("state", []), dtype=np.float32)
    images = {}
    for entry in payload.get("images", []):
        name = str(entry.get("name", "")).strip()
        if not name:
            continue
        image_hwc = _decode_image(entry["data"])
        images[name] = np.transpose(image_hwc, (2, 0, 1))
    obs = {
        "state": state,
        "images": images,
        "prompt": payload.get("prompt") or bundle.default_prompt or "do something",
    }
    return obs


def _build_observation(payload: dict, bundle: PolicyBundle) -> dict:
    fmt = str(payload.get("observation_format", "aloha")).strip().lower()
    if fmt == "aloha":
        return _to_aloha_obs(payload, bundle)
    raise ValueError(f"unsupported observation_format='{fmt}'")


class OpenPiHandler(BaseHTTPRequestHandler):
    server_version = "openpi-runtime/0.1"

    def _write_json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path != "/healthz":
            self._write_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})
            return
        bundles = {
            key: {
                "ready": bundle.ready,
                "config_name": bundle.config_name,
                "checkpoint_dir": bundle.checkpoint_dir,
                "error": bundle.error,
            }
            for key, bundle in POLICIES.items()
        }
        status = HTTPStatus.OK if any(bundle.ready for bundle in POLICIES.values()) else HTTPStatus.SERVICE_UNAVAILABLE
        self._write_json(
            status,
            {
                "ok": status == HTTPStatus.OK,
                "policies": bundles,
                "runtime": _runtime_info(),
            },
        )

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path != "/infer":
            self._write_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})
            return
        try:
            content_len = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(content_len)
            payload = json.loads(raw.decode("utf-8"))
            policy_id = str(payload.get("policy_id") or next(iter(POLICIES.keys()))).strip()
            bundle = POLICIES.get(policy_id)
            if bundle is None:
                raise ValueError(f"unknown policy_id='{policy_id}'")
            if not bundle.ready:
                raise RuntimeError(bundle.error or f"policy '{policy_id}' is not ready")

            obs = _build_observation(payload, bundle)
            start = time.perf_counter()
            result = bundle.policy.infer(obs)
            infer_ms = (time.perf_counter() - start) * 1000.0

            actions = np.asarray(result["actions"], dtype=np.float32)
            response = {
                "ok": True,
                "policy_id": policy_id,
                "actions": actions.tolist(),
                "action_horizon": int(actions.shape[0]),
                "action_dim": int(actions.shape[1]) if actions.ndim == 2 else 0,
                "timing": {
                    "infer_ms": infer_ms,
                },
            }
            if "server_timing" in result:
                response["server_timing"] = result["server_timing"]
            self._write_json(HTTPStatus.OK, response)
        except Exception as exc:  # pragma: no cover - runtime path
            self._write_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})

    def log_message(self, fmt: str, *args) -> None:  # noqa: A003
        print(self.address_string(), "-", fmt % args, flush=True)


def main() -> int:
    host = os.environ.get("OPENPI_LISTEN_HOST", "0.0.0.0")
    port = int(os.environ.get("OPENPI_PORT", "8000"))
    server = ThreadingHTTPServer((host, port), OpenPiHandler)
    print(f"openpi-runtime listening on {host}:{port} policies={list(POLICIES)}", flush=True)
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
