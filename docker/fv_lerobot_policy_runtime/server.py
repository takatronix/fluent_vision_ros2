#!/usr/bin/env python3
from __future__ import annotations

import base64
import json
import os
import shutil
import time
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import cv2
import numpy as np
import torch
import yaml
from lerobot.policies.factory import get_policy_class, make_pre_post_processors


def _runtime_info() -> dict[str, Any]:
    info: dict[str, Any] = {}
    info["torch"] = {
        "version": getattr(torch, "__version__", ""),
        "cuda_available": bool(torch.cuda.is_available()),
        "device_count": int(torch.cuda.device_count()) if torch.cuda.is_available() else 0,
    }
    try:
        import transformers

        info["transformers"] = {"version": getattr(transformers, "__version__", "")}
    except Exception as exc:
        info["transformers"] = {"error": str(exc)}
    return info


def _parse_camera_aliases() -> dict[str, str]:
    aliases: dict[str, str] = {}
    raw = os.environ.get("DPEX_CAMERA_ALIASES", "").strip()
    for item in raw.split(","):
        if "=" not in item:
            continue
        src, dst = item.split("=", 1)
        src = src.strip()
        dst = dst.strip()
        if src and dst:
            aliases[src] = dst
    return aliases


CAMERA_ALIASES = _parse_camera_aliases()


def _ensure_local_tokenizer() -> str | None:
    tokenizer_name = os.environ.get("DPEX_TOKENIZER_NAME", "").strip()
    if tokenizer_name:
        return tokenizer_name

    src = Path(os.environ.get("DPEX_PALIGEMMA_TOKENIZER_MODEL", "/data/.cache/openpi/big_vision/paligemma_tokenizer.model"))
    if not src.exists():
        return None

    dst = Path(os.environ.get("DPEX_LOCAL_TOKENIZER_DIR", "/tmp/paligemma-tokenizer"))
    dst.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst / "tokenizer.model")
    (dst / "tokenizer_config.json").write_text(
        json.dumps(
            {
                "tokenizer_class": "GemmaTokenizer",
                "model_input_names": ["input_ids", "attention_mask"],
                "padding_side": "right",
            }
        )
    )
    (dst / "special_tokens_map.json").write_text(
        json.dumps(
            {
                "bos_token": "<bos>",
                "eos_token": "<eos>",
                "unk_token": "<unk>",
                "pad_token": "<pad>",
            }
        )
    )
    return str(dst)


@dataclass
class PolicyBundle:
    policy_id: str
    policy_type: str
    model_path: str
    device: str
    default_prompt: str = ""
    denoising_steps: int | None = None
    policy: Any = None
    preprocessor: Any = None
    postprocessor: Any = None
    image_shapes: dict[str, tuple[int, int, int]] = field(default_factory=dict)
    state_dim: int | None = None
    error: str = ""

    def load(self) -> None:
        try:
            if not Path(self.model_path).exists():
                raise FileNotFoundError(self.model_path)
            policy_cls = get_policy_class(self.policy_type)
            policy = policy_cls.from_pretrained(self.model_path)
            policy.to(self.device)
            policy.eval()
            preprocessor_overrides = {"device_processor": {"device": self.device}}
            local_tokenizer = _ensure_local_tokenizer()
            if local_tokenizer:
                preprocessor_overrides["tokenizer_processor"] = {"tokenizer_name": local_tokenizer}

            preprocessor, postprocessor = make_pre_post_processors(
                policy.config,
                pretrained_path=self.model_path,
                preprocessor_overrides=preprocessor_overrides,
                postprocessor_overrides={"device_processor": {"device": self.device}},
            )

            image_shapes: dict[str, tuple[int, int, int]] = {}
            input_features = getattr(policy.config, "input_features", {}) or {}
            for key, feature in input_features.items():
                if not isinstance(key, str) or not key.startswith("observation.images."):
                    continue
                shape = getattr(feature, "shape", None)
                if isinstance(shape, (list, tuple)) and len(shape) == 3:
                    image_shapes[key.replace("observation.images.", "", 1)] = tuple(int(v) for v in shape)

            state_dim = None
            state_feature = input_features.get("observation.state")
            raw_shape = getattr(state_feature, "shape", None) if state_feature is not None else None
            if isinstance(raw_shape, (list, tuple)) and raw_shape:
                state_dim = int(raw_shape[0])

            self.policy = policy
            self.preprocessor = preprocessor
            self.postprocessor = postprocessor
            self.image_shapes = image_shapes
            self.state_dim = state_dim
            self.error = ""
        except Exception as exc:  # pragma: no cover - runtime init path
            self.policy = None
            self.preprocessor = None
            self.postprocessor = None
            self.error = str(exc)

    @property
    def ready(self) -> bool:
        return self.policy is not None and not self.error


def _parse_policy_specs() -> dict[str, PolicyBundle]:
    device = os.environ.get("DPEX_DEVICE", "cuda" if torch.cuda.is_available() else "cpu").strip()
    specs: dict[str, PolicyBundle] = {}
    raw = os.environ.get("DPEX_POLICY_SPECS", "").strip()
    if raw:
        for item in raw.split(";"):
            item = item.strip()
            if not item:
                continue
            parts = item.split(":", 2)
            if len(parts) != 3:
                continue
            policy_id, policy_type, remainder = parts
            model_path, default_prompt = remainder, ""
            if "|" in remainder:
                model_path, default_prompt = remainder.split("|", 1)
            specs[policy_id.strip()] = PolicyBundle(
                policy_id=policy_id.strip(),
                policy_type=policy_type.strip(),
                model_path=model_path.strip(),
                device=device,
                default_prompt=default_prompt.strip(),
                denoising_steps=_optional_int(os.environ.get("DPEX_DENOISING_STEPS")),
            )
    if specs:
        return specs

    model_path = os.environ.get("DPEX_MODEL_PATH", "").strip()
    if not model_path:
        # 既定ではモデルを積まない。DPEX_POLICY_SPECS か DPEX_MODEL_PATH で明示する。
        return {}
    policy_id = os.environ.get("DPEX_DEFAULT_POLICY_ID", "default")
    policy_type = os.environ.get("DPEX_POLICY_TYPE", "pi05")
    return {
        policy_id: PolicyBundle(
            policy_id=policy_id,
            policy_type=policy_type,
            model_path=model_path,
            device=device,
            default_prompt=os.environ.get("DPEX_DEFAULT_PROMPT", ""),
            denoising_steps=_optional_int(os.environ.get("DPEX_DENOISING_STEPS")),
        )
    }


def _optional_int(value: str | None) -> int | None:
    if value is None or not str(value).strip():
        return None
    return int(value)


def _load_model_index() -> list[dict[str, Any]]:
    path = Path(os.environ.get("DPEX_POLICY_MODEL_INDEX", "/opt/fv-lerobot-policy-runtime/policy_models.yaml"))
    if not path.exists():
        return []
    data = yaml.safe_load(path.read_text()) or {}
    models = data.get("models", [])
    return models if isinstance(models, list) else []


POLICIES = _parse_policy_specs()
if os.environ.get("DPEX_LOAD_ON_START", "true").strip().lower() not in {"0", "false", "no"}:
    for _bundle in POLICIES.values():
        _bundle.load()


def _decode_image(encoded: str, model_key: str, bundle: PolicyBundle) -> torch.Tensor:
    raw = base64.b64decode(encoded.encode("ascii"))
    arr = np.frombuffer(raw, dtype=np.uint8)
    image_bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise ValueError(f"failed to decode image for {model_key}")
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    target_shape = bundle.image_shapes.get(model_key)
    if target_shape:
        _, height, width = target_shape
        if image_rgb.shape[0] != height or image_rgb.shape[1] != width:
            image_rgb = cv2.resize(image_rgb, (width, height), interpolation=cv2.INTER_LINEAR)
    tensor = torch.from_numpy(image_rgb).permute(2, 0, 1).contiguous().float() / 255.0
    return tensor.unsqueeze(0)


def _build_batch(payload: dict[str, Any], bundle: PolicyBundle) -> dict[str, Any]:
    state = payload.get("state", [])
    if bundle.state_dim is not None and len(state) != bundle.state_dim:
        raise ValueError(f"state dim mismatch: expected {bundle.state_dim}, got {len(state)}")

    batch: dict[str, Any] = {
        "observation.state": torch.tensor([state], dtype=torch.float32),
    }

    images_by_name: dict[str, dict[str, Any]] = {}
    for entry in payload.get("images", []):
        if not isinstance(entry, dict):
            continue
        src = str(entry.get("name", "")).strip()
        dst = CAMERA_ALIASES.get(src, src)
        if src:
            images_by_name[dst] = entry

    for model_key in bundle.image_shapes:
        entry = images_by_name.get(model_key)
        if entry is None:
            available = sorted(images_by_name)
            raise ValueError(f"missing image key: {model_key}; available: {available}")
        batch[f"observation.images.{model_key}"] = _decode_image(str(entry.get("data", "")), model_key, bundle)

    prompt = str(payload.get("prompt") or bundle.default_prompt or "").strip()
    if prompt:
        batch["task"] = [prompt]
    return batch


def _infer(payload: dict[str, Any], bundle: PolicyBundle) -> dict[str, Any]:
    if not bundle.ready:
        bundle.load()
    if not bundle.ready:
        raise RuntimeError(bundle.error or f"policy '{bundle.policy_id}' is not ready")

    assert bundle.preprocessor is not None
    assert bundle.postprocessor is not None
    assert bundle.policy is not None

    batch = _build_batch(payload, bundle)
    policy_batch = bundle.preprocessor(batch)
    start = time.perf_counter()
    with torch.no_grad():
        kwargs: dict[str, Any] = {}
        if bundle.policy_type.lower() in {"pi0", "pi05"} and bundle.denoising_steps is not None:
            kwargs["num_steps"] = bundle.denoising_steps
        actions = bundle.policy.predict_action_chunk(policy_batch, **kwargs)
    infer_ms = (time.perf_counter() - start) * 1000.0

    if actions.ndim == 2:
        actions = actions.unsqueeze(0)
    result: list[list[float]] = []
    for idx in range(actions.shape[1]):
        action_at_idx = actions[:, idx, :]
        postprocessed = bundle.postprocessor(action_at_idx).squeeze(0)
        result.append([float(v) for v in postprocessed.detach().cpu().float().tolist()])
    return {
        "ok": True,
        "policy_id": bundle.policy_id,
        "actions": result,
        "action_horizon": len(result),
        "action_dim": len(result[0]) if result else 0,
        "timing": {"infer_ms": infer_ms},
    }


class DpexPolicyHandler(BaseHTTPRequestHandler):
    server_version = "fv-lerobot-policy-runtime/0.1"

    def _write_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path not in {"/healthz", "/models"}:
            self._write_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})
            return
        bundles = {
            key: {
                "ready": bundle.ready,
                "policy_type": bundle.policy_type,
                "model_path": bundle.model_path,
                "device": bundle.device,
                "image_keys": list(bundle.image_shapes),
                "state_dim": bundle.state_dim,
                "error": bundle.error,
            }
            for key, bundle in POLICIES.items()
        }
        ok = any(bundle.ready for bundle in POLICIES.values())
        if parsed.path == "/models":
            ok = True
        status = HTTPStatus.OK if ok else HTTPStatus.SERVICE_UNAVAILABLE
        self._write_json(
            status,
            {
                "ok": ok,
                "policies": bundles,
                "model_index": _load_model_index(),
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
            payload = json.loads(self.rfile.read(content_len).decode("utf-8"))
            default_policy_id = os.environ.get("DPEX_DEFAULT_POLICY_ID") or next(iter(POLICIES.keys()), "")
            policy_id = str(payload.get("policy_id") or default_policy_id).strip()
            bundle = POLICIES.get(policy_id)
            if bundle is None:
                raise ValueError(f"unknown policy_id='{policy_id}' (loaded: {sorted(POLICIES)})")
            self._write_json(HTTPStatus.OK, _infer(payload, bundle))
        except Exception as exc:  # pragma: no cover - runtime path
            self._write_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})

    def log_message(self, fmt: str, *args: Any) -> None:
        print(self.address_string(), "-", fmt % args, flush=True)


def main() -> int:
    host = os.environ.get("DPEX_LISTEN_HOST", "0.0.0.0")
    port = int(os.environ.get("DPEX_PORT", "8010"))
    print(f"fv-lerobot-policy-runtime listening on {host}:{port} policies={list(POLICIES)}", flush=True)
    ThreadingHTTPServer((host, port), DpexPolicyHandler).serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
