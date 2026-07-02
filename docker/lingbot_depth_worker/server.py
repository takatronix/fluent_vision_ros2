#!/usr/bin/env python3
from __future__ import annotations

import io
import os
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import numpy as np


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() not in ("0", "false", "no", "off")


def _ensure_xformers_free_attention() -> None:
    """Allow inference without xFormers (e.g. Jetson/ARM64).

    The DINOv2-RGBD encoder feeds the transformer blocks a *list* of per-sample
    token sequences; upstream NestedTensorBlock handles that path with xFormers
    nested tensors and asserts otherwise. In eval mode the nested path is pure
    ragged batching (block-diagonal attention mask), so running plain
    Block.forward per element is mathematically identical — sequences never
    attend across each other. The non-xFormers attention fallback already uses
    PyTorch-native SDPA.
    """
    try:
        import xformers  # noqa: F401

        return
    except ImportError:
        pass

    from mdm.model.dinov2_rgbd.layers.block import Block, NestedTensorBlock

    def _forward_per_sample(self, x_or_x_list):
        if isinstance(x_or_x_list, list):
            return [Block.forward(self, x) for x in x_or_x_list]
        return Block.forward(self, x_or_x_list)

    NestedTensorBlock.forward = _forward_per_sample
    print("xformers unavailable: patched NestedTensorBlock for per-sample attention", flush=True)


class LingBotDepthWorker:
    def __init__(self) -> None:
        self.model_id = os.environ.get(
            "MODEL_ID", "robbyant/lingbot-depth-pretrain-vitl-14-v0.5"
        )
        self.local_model_path = os.environ.get("LOCAL_MODEL_PATH", "").strip()
        self.device_mode = os.environ.get("DEVICE", "auto").strip().lower()
        self.use_fp16 = _env_bool("USE_FP16", True)
        self.apply_mask = _env_bool("APPLY_MASK", True)
        self.resolution_level = int(os.environ.get("RESOLUTION_LEVEL", "9"))
        self.fallback_passthrough = _env_bool("FALLBACK_PASSTHROUGH", True)

        self._torch = None
        self._model = None
        self._device = None
        self._error = ""
        self._load_model()

    @property
    def ready(self) -> bool:
        return self._model is not None

    @property
    def status_text(self) -> str:
        if self.ready:
            return f"ok device={self._device}"
        if self.fallback_passthrough:
            return f"degraded fallback=true error={self._error}"
        return f"error fallback=false error={self._error}"

    def _load_model(self) -> None:
        try:
            import torch
            from mdm.model.v2 import MDMModel
        except Exception as exc:
            self._error = f"import failed: {exc}"
            return

        _ensure_xformers_free_attention()

        try:
            if self.device_mode == "cuda":
                device = "cuda"
            elif self.device_mode == "cpu":
                device = "cpu"
            else:
                device = "cuda" if torch.cuda.is_available() else "cpu"

            model_source = self.local_model_path or self.model_id
            model = MDMModel.from_pretrained(model_source).to(device).eval()

            self._torch = torch
            self._device = device
            self._model = model
            self._error = ""
        except Exception as exc:
            self._error = f"load failed: {exc}"

    def infer(
        self,
        color_bgr: np.ndarray,
        depth_m: np.ndarray,
        intrinsics_norm: np.ndarray,
        use_fp16: bool,
        apply_mask: bool,
        resolution_level: int,
        return_points: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]:
        if self.ready:
            try:
                return self._infer_model(
                    color_bgr=color_bgr,
                    depth_m=depth_m,
                    intrinsics_norm=intrinsics_norm,
                    use_fp16=use_fp16,
                    apply_mask=apply_mask,
                    resolution_level=resolution_level,
                    return_points=return_points,
                )
            except Exception as exc:
                self._error = f"infer failed: {exc}"
                if not self.fallback_passthrough:
                    raise

        if not self.fallback_passthrough:
            raise RuntimeError(self._error or "worker not ready")

        mask = np.isfinite(depth_m) & (depth_m > 0.0)
        points = self._depth_to_points(depth_m, intrinsics_norm, mask) if return_points else None
        return depth_m.astype(np.float32), mask.astype(bool), points

    def _infer_model(
        self,
        color_bgr: np.ndarray,
        depth_m: np.ndarray,
        intrinsics_norm: np.ndarray,
        use_fp16: bool,
        apply_mask: bool,
        resolution_level: int,
        return_points: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]:
        torch = self._torch
        if torch is None or self._model is None or self._device is None:
            raise RuntimeError("model is not loaded")

        color_rgb = color_bgr[..., ::-1].astype(np.float32) / 255.0
        image_t = torch.from_numpy(color_rgb).to(device=self._device, dtype=torch.float32)
        image_t = image_t.permute(2, 0, 1).unsqueeze(0)
        depth_t = torch.from_numpy(depth_m.astype(np.float32)).to(device=self._device).unsqueeze(0)
        intr_t = torch.from_numpy(intrinsics_norm.astype(np.float32)).to(device=self._device).unsqueeze(0)

        with torch.inference_mode():
            out = self._model.infer(
                image_t,
                depth_in=depth_t,
                intrinsics=intr_t if return_points else None,
                use_fp16=use_fp16,
                apply_mask=apply_mask,
                resolution_level=resolution_level,
            )

        depth_out = out.get("depth")
        if depth_out is None:
            raise RuntimeError("worker output missing depth")
        depth_np = depth_out.detach().float().cpu().numpy().squeeze().astype(np.float32)

        mask_out = out.get("mask")
        if mask_out is not None:
            mask_np = mask_out.detach().cpu().numpy().squeeze().astype(bool)
        else:
            mask_np = np.isfinite(depth_np) & (depth_np > 0.0)

        if not return_points:
            return depth_np, mask_np, None

        points_out = out.get("points")
        if points_out is not None:
            points_np = points_out.detach().float().cpu().numpy().squeeze().astype(np.float32)
        else:
            points_np = self._depth_to_points(depth_np, intrinsics_norm, mask_np)

        return depth_np, mask_np, points_np

    def _depth_to_points(
        self, depth_m: np.ndarray, intrinsics_norm: np.ndarray, mask: Optional[np.ndarray]
    ) -> np.ndarray:
        height, width = depth_m.shape
        fx = float(intrinsics_norm[0, 0]) * float(width)
        fy = float(intrinsics_norm[1, 1]) * float(height)
        cx = float(intrinsics_norm[0, 2]) * float(width)
        cy = float(intrinsics_norm[1, 2]) * float(height)

        u, v = np.meshgrid(
            np.arange(width, dtype=np.float32),
            np.arange(height, dtype=np.float32),
        )
        z = depth_m.astype(np.float32)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        points = np.stack([x, y, z], axis=-1)

        valid = np.isfinite(z) & (z > 0.0)
        if mask is not None:
            valid &= mask
        points[~valid] = np.inf
        return points.astype(np.float32)


WORKER = LingBotDepthWorker()


class Handler(BaseHTTPRequestHandler):
    server_version = "LingBotDepthWorker/0.1"

    def log_message(self, fmt: str, *args) -> None:
        print(fmt % args, flush=True)

    def do_GET(self) -> None:
        if self.path != "/healthz":
            self.send_error(HTTPStatus.NOT_FOUND, "not found")
            return
        body = WORKER.status_text.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:
        if self.path != "/infer":
            self.send_error(HTTPStatus.NOT_FOUND, "not found")
            return

        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length)

        try:
            with np.load(io.BytesIO(body), allow_pickle=False) as payload:
                color_bgr = payload["color_bgr"].astype(np.uint8, copy=False)
                depth_m = payload["depth_m"].astype(np.float32, copy=False)
                if "depth_scale" in payload:
                    depth_m = depth_m * float(payload["depth_scale"].item())
                intrinsics_norm = payload["intrinsics_norm"].astype(np.float32, copy=False)
                use_fp16 = bool(payload["use_fp16"].item()) if "use_fp16" in payload else WORKER.use_fp16
                apply_mask = bool(payload["apply_mask"].item()) if "apply_mask" in payload else WORKER.apply_mask
                resolution_level = (
                    int(payload["resolution_level"].item())
                    if "resolution_level" in payload
                    else WORKER.resolution_level
                )
                return_points = bool(payload["return_points"].item()) if "return_points" in payload else True
                compress = bool(payload["compress"].item()) if "compress" in payload else True
        except Exception as exc:
            self.send_error(HTTPStatus.BAD_REQUEST, f"invalid request: {exc}")
            return

        try:
            depth_out, mask_out, points_out = WORKER.infer(
                color_bgr=color_bgr,
                depth_m=depth_m,
                intrinsics_norm=intrinsics_norm,
                use_fp16=use_fp16,
                apply_mask=apply_mask,
                resolution_level=resolution_level,
                return_points=return_points,
            )
        except Exception as exc:
            self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, str(exc))
            return

        arrays = {
            "depth": depth_out.astype(np.float32, copy=False),
            "mask": mask_out.astype(np.uint8, copy=False),
        }
        if points_out is not None:
            arrays["points"] = points_out.astype(np.float32, copy=False)

        resp = io.BytesIO()
        # Compression costs ~100ms+ per frame on Jetson; skip it for
        # localhost transport unless the client asks for it.
        if compress:
            np.savez_compressed(resp, **arrays)
        else:
            np.savez(resp, **arrays)
        data = resp.getvalue()

        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def main() -> None:
    host = os.environ.get("LISTEN_HOST", "0.0.0.0")
    port = int(os.environ.get("LISTEN_PORT", "5540"))
    server = ThreadingHTTPServer((host, port), Handler)
    print(f"lingbot-depth-worker listening on {host}:{port} status={WORKER.status_text}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
