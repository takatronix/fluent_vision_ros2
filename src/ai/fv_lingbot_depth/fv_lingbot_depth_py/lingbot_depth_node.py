#!/usr/bin/env python3
from __future__ import annotations

import io
import json
import threading
import time
from typing import Optional, Tuple
import urllib.error
import urllib.request

import cv2
import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger

from fv_lingbot_depth_py.demand_gate import InputSubscriptionGate


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


class FvLingbotDepthNode(Node):
    def __init__(self) -> None:
        super().__init__("fv_lingbot_depth")

        # I/O
        self.declare_parameter("color_topic", "~/color/image_raw")
        self.declare_parameter("depth_topic", "~/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "~/camera_info")
        self.declare_parameter("refined_depth_topic", "depth_refined/image_rect_raw")
        self.declare_parameter("mask_topic", "depth_refined/mask")
        self.declare_parameter("pointcloud_topic", "depth_refined/points")
        self.declare_parameter("frame_id_override", "")

        # Model/runtime
        self.declare_parameter("backend", "direct")  # direct|http
        self.declare_parameter("worker_endpoint", "http://127.0.0.1:5540/infer")
        self.declare_parameter("worker_timeout_sec", 5.0)
        self.declare_parameter("model_id", "robbyant/lingbot-depth-pretrain-vitl-14")
        self.declare_parameter("local_model_path", "")
        self.declare_parameter("device", "auto")  # auto|cuda|cpu
        self.declare_parameter("use_fp16", True)
        self.declare_parameter("apply_mask", True)
        self.declare_parameter("resolution_level", 9)
        self.declare_parameter("fallback_passthrough", True)

        # Depth/point settings
        self.declare_parameter("depth_scale_16uc1", 0.001)
        # 16UC1 (mm) keeps the refined stream the same size/encoding as the
        # raw sensor depth. 32FC1 doubles the message to ~1.2MB, whose
        # fragmented BEST_EFFORT samples starve on busy subscriber nodes
        # (observed: the dashboard received ~0 of them).
        self.declare_parameter("output_encoding", "16UC1")  # 16UC1|32FC1
        self.declare_parameter("min_depth_m", 0.05)
        self.declare_parameter("max_depth_m", 3.0)
        self.declare_parameter("point_stride", 2)

        # Sync/QoS
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_sec", 0.03)
        self.declare_parameter("qos_reliability", "best_effort")

        # Consumer-side gate. When set (e.g. /vlabor/d405_depth_source),
        # inference only runs while the latched selector picks the refined
        # stream ("default"/"raw" = pause). The model stays loaded so
        # switching back is instant; while paused the GPU does no work for
        # this node. Empty = no gate (always-on, pre-gate behaviour) for
        # deployments without the dashboard selector.
        self.declare_parameter("depth_source_topic", "")

        # On-demand capture (~/capture, std_srvs/Trigger): while PAUSED,
        # one service call refines exactly the next synced frame at
        # capture_resolution_level (default = max quality) and re-pauses.
        # This is the "robot stops -> take one clean cloud" path; continuous
        # streaming stays as-is via the source selector.
        self.declare_parameter("capture_resolution_level", 9)
        self.declare_parameter("capture_timeout_sec", 10.0)

        # Cameras that publish no camera_info topic (the D555 serves its
        # streams straight from its on-board DDS server) can supply
        # [fx, fy, cx, cy] in depth-pixel units here; the info subscriber is
        # then dropped and the sync runs on color+depth only. All-zero =
        # disabled (camera_info required, original behaviour).
        self.declare_parameter("static_intrinsics", [0.0, 0.0, 0.0, 0.0])

        # Logging
        self.declare_parameter("log_every_n_frames", 30)

        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.refined_depth_topic = str(self.get_parameter("refined_depth_topic").value)
        self.mask_topic = str(self.get_parameter("mask_topic").value)
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.frame_id_override = str(self.get_parameter("frame_id_override").value)

        self.backend = str(self.get_parameter("backend").value).strip().lower()
        self.worker_endpoint = str(self.get_parameter("worker_endpoint").value).strip()
        self.worker_timeout_sec = float(self.get_parameter("worker_timeout_sec").value)
        self.model_id = str(self.get_parameter("model_id").value)
        self.local_model_path = str(self.get_parameter("local_model_path").value)
        self.device_mode = str(self.get_parameter("device").value)
        self.use_fp16 = bool(self.get_parameter("use_fp16").value)
        self.apply_mask = bool(self.get_parameter("apply_mask").value)
        self.resolution_level = int(self.get_parameter("resolution_level").value)
        self.fallback_passthrough = bool(self.get_parameter("fallback_passthrough").value)

        self.depth_scale_16uc1 = float(self.get_parameter("depth_scale_16uc1").value)
        self.output_encoding = str(self.get_parameter("output_encoding").value).strip().upper()
        if self.output_encoding not in ("16UC1", "32FC1"):
            self.get_logger().warning(
                f"Unknown output_encoding='{self.output_encoding}', falling back to 16UC1"
            )
            self.output_encoding = "16UC1"
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.point_stride = max(1, int(self.get_parameter("point_stride").value))

        self.sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        self.sync_slop_sec = float(self.get_parameter("sync_slop_sec").value)
        self.qos_reliability = str(self.get_parameter("qos_reliability").value).strip().lower()
        self.depth_source_topic = str(self.get_parameter("depth_source_topic").value).strip()
        self.capture_resolution_level = max(0, min(9, int(
            self.get_parameter("capture_resolution_level").value)))
        self.capture_timeout_sec = float(
            self.get_parameter("capture_timeout_sec").value)
        intr_vals = [float(v) for v in self.get_parameter("static_intrinsics").value]
        self._static_intr: Optional[Tuple[float, float, float, float]] = None
        if any(v != 0.0 for v in intr_vals):
            if len(intr_vals) != 4 or intr_vals[0] <= 0.0 or intr_vals[1] <= 0.0:
                raise ValueError(
                    f"static_intrinsics must be [fx, fy, cx, cy] with fx,fy > 0, got {intr_vals}")
            self._static_intr = tuple(intr_vals)

        self.log_every_n_frames = max(1, int(self.get_parameter("log_every_n_frames").value))

        self.depth_pub = self.create_publisher(Image, self.refined_depth_topic, 10)
        self.mask_pub = self.create_publisher(Image, self.mask_topic, 10)
        # Empty pointcloud_topic disables the publisher entirely. Consumers
        # build clouds from the refined depth image themselves; advertising
        # an (on-demand, otherwise silent) topic here just confuses topic
        # listings.
        self.points_pub = (
            self.create_publisher(PointCloud2, self.pointcloud_topic, 10)
            if self.pointcloud_topic else None
        )

        if self.backend not in ("direct", "http"):
            self.get_logger().warning(
                f"Unknown backend='{self.backend}', falling back to direct"
            )
            self.backend = "direct"

        self._model = None
        self._torch = None
        self._device = None
        self._model_error = ""
        if self.backend == "direct":
            self._try_load_model()
        else:
            self.get_logger().info(
                f"LingBot-Depth worker backend enabled endpoint={self.worker_endpoint}"
            )

        # Gate state. With a selector topic configured, start PAUSED until
        # the latched value arrives (the dashboard announces the current
        # selection with transient_local, so a running system converges
        # within one delivery; a missing publisher means nobody is
        # consuming the refined stream anyway).
        self._source_selected = self.depth_source_topic == ""
        if self.depth_source_topic:
            latched = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                String, self.depth_source_topic, self._on_depth_source, latched)
            self.get_logger().info(
                f"inference gated by {self.depth_source_topic} "
                "(paused until it selects the refined stream)")

        # 入力購読は**需要駆動**にする。出力を誰も読んでいない間は購読ごと
        # 落とす。推論だけ止めても購読が生きていると、DDS は購読者ごとに
        # 別コピーを送るので線に乗る量は変わらない (2026-08-23 実測:
        # 生color 573 + 生depth 387 = 960 Mbps を受信して捨て、CPU 62%)。
        self.color_sub = None
        self.depth_sub = None
        self.info_sub = None
        self.sync = None
        self._input_gate = InputSubscriptionGate(
            start=self._start_input_subs,
            stop=self._stop_input_subs,
            log=lambda m: self.get_logger().info(m))

        # On-demand capture: its own callback group so the (blocking) service
        # handler can wait while the sync callback runs on the default group
        # (needs the MultiThreadedExecutor in main()).
        self._capture_lock = threading.Lock()
        self._capture_pending = 0
        self._capture_event = threading.Event()
        self._capture_result: dict = {}
        self._capture_srv = self.create_service(
            Trigger, "~/capture", self._on_capture,
            callback_group=MutuallyExclusiveCallbackGroup())

        # 需要の追従周期。短いほど反応が良いが、出力購読者が現れては消える
        # 使い方だと購読がばたつく。1秒は「UIでレイヤを入れてから絵が出る
        # までの待ち」として許容できる上限。
        self.declare_parameter("demand_poll_sec", 1.0)
        self._demand_poll_sec = max(
            0.2, float(self.get_parameter("demand_poll_sec").value))
        self._demand_timer = self.create_timer(
            self._demand_poll_sec, self._reconcile_input)
        # 起動直後に一度合わせる (既に購読者がいるなら待たせない)。
        self._reconcile_input()

        self._frame_counter = 0
        self._proc_ms_acc = 0.0

        # Runtime tuning: `ros2 param set` (or the dashboard quality button)
        # can adjust the inference resolution without a node restart — the
        # level is sent to the worker per request anyway.
        self.add_on_set_parameters_callback(self._on_param_update)

        model_source = self.local_model_path if self.local_model_path else self.model_id
        self.get_logger().info(
            f"fv_lingbot_depth started color={self.color_topic} depth={self.depth_topic} "
            f"info={self.camera_info_topic} out_depth={self.refined_depth_topic} "
            f"out_points={self.pointcloud_topic} backend={self.backend} model={model_source} "
            f"input=demand-driven({self._demand_poll_sec:g}s)"
        )

    # ---- 入力購読の需要駆動 (demand_gate.InputSubscriptionGate) ----------

    def _start_input_subs(self) -> None:
        """color/depth(/camera_info) を購読し、同期器を張る。"""
        qos = self._sensor_qos_profile()
        self.color_sub = Subscriber(self, Image, self.color_topic,
                                    qos_profile=qos)
        self.depth_sub = Subscriber(self, Image, self.depth_topic,
                                    qos_profile=qos)
        if self._static_intr is not None:
            # No camera_info stream: sync color+depth only, intrinsics come
            # from the static_intrinsics parameter.
            self.info_sub = None
            self.sync = ApproximateTimeSynchronizer(
                [self.color_sub, self.depth_sub],
                queue_size=self.sync_queue_size,
                slop=self.sync_slop_sec,
                allow_headerless=False,
            )
            self.sync.registerCallback(self._on_synced_static_intr)
        else:
            self.info_sub = Subscriber(
                self, CameraInfo, self.camera_info_topic, qos_profile=qos)
            self.sync = ApproximateTimeSynchronizer(
                [self.color_sub, self.depth_sub, self.info_sub],
                queue_size=self.sync_queue_size,
                slop=self.sync_slop_sec,
                allow_headerless=False,
            )
            self.sync.registerCallback(self._on_synced)

    def _stop_input_subs(self) -> None:
        """購読を捨てる。同期器の参照も切って溜まったバッファを解放する。"""
        # 先に同期器を切る。購読だけ消して同期器を残すと、次に張った購読が
        # 古いキューの上に乗って「時刻の合わないペア」を作る。
        self.sync = None
        for name in ("color_sub", "depth_sub", "info_sub"):
            sub = getattr(self, name, None)
            setattr(self, name, None)
            if sub is None:
                continue
            # message_filters.Subscriber は rclpy の購読を .sub に持つ
            # (Jazzy には unregister() が無い)。
            inner = getattr(sub, "sub", None)
            if inner is not None:
                self.destroy_subscription(inner)

    def _input_demanded(self) -> bool:
        """入力を受け取る理由があるか。

        出力に購読者がいるか、capture 要求が来ているか。capture は購読が
        無い状態から1枚だけ処理する口なので、ここに含めないと永遠に
        タイムアウトする。
        """
        if self._demanded():
            return True
        with self._capture_lock:
            return self._capture_pending > 0

    def _reconcile_input(self) -> None:
        self._input_gate.reconcile(self._input_demanded())

    def _on_param_update(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "resolution_level":
                try:
                    self.resolution_level = max(0, min(9, int(p.value)))
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False, reason="resolution_level must be an int 0..9")
                self.get_logger().info(f"resolution_level -> {self.resolution_level}")
        return SetParametersResult(successful=True)

    def _sensor_qos_profile(self) -> QoSProfile:
        reliable = self.qos_reliability == "reliable"
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

    def _try_load_model(self) -> None:
        try:
            import torch
            from mdm.model.v2 import MDMModel
        except Exception as exc:
            self._model = None
            self._torch = None
            self._device = None
            self._model_error = str(exc)
            self.get_logger().error(
                f"LingBot-Depth import failed. Install lingbot-depth deps. error={self._model_error}"
            )
            return

        _ensure_xformers_free_attention()

        try:
            if self.device_mode == "cuda":
                device = "cuda"
            elif self.device_mode == "cpu":
                device = "cpu"
            else:
                device = "cuda" if torch.cuda.is_available() else "cpu"

            model_source = self.local_model_path.strip() if self.local_model_path.strip() else self.model_id
            model = MDMModel.from_pretrained(model_source).to(device).eval()

            self._torch = torch
            self._device = device
            self._model = model
            self._model_error = ""
            self.get_logger().info(f"LingBot-Depth model loaded on {device} ({model_source})")
        except Exception as exc:
            self._model = None
            self._torch = None
            self._device = None
            self._model_error = str(exc)
            self.get_logger().error(f"LingBot-Depth model load failed: {self._model_error}")

    def _on_depth_source(self, msg: String) -> None:
        selected = msg.data not in ("default", "raw")
        if selected == self._source_selected:
            return
        self._source_selected = selected
        if selected:
            self.get_logger().info(f"inference resumed (source={msg.data})")
        else:
            self.get_logger().info(
                f"inference paused (source={msg.data}) - model stays loaded, no GPU work")

    def _on_capture(self, request: Trigger.Request,
                    response: Trigger.Response) -> Trigger.Response:
        """One-shot refine: process the next synced frame, then re-pause.

        Runs at capture_resolution_level (max quality by default) regardless
        of the streaming resolution_level. While the continuous mode is
        active the refined stream is already live, so the call just reports
        that. No passthrough: if the worker/model fails, the call times out
        rather than returning raw depth dressed up as refined.
        """
        if self._source_selected:
            response.success = True
            response.message = json.dumps(
                {"note": "continuous mode active; refined stream already live"})
            return response
        with self._capture_lock:
            if self._capture_pending <= 0:
                self._capture_event.clear()
                self._capture_pending = 1
        # 購読が落ちている状態からの capture。タイマ (既定1秒) を待たずに
        # ここで上げる — 待つと capture_timeout_sec を無駄に食う。
        self._reconcile_input()
        if self._capture_event.wait(self.capture_timeout_sec):
            response.success = True
            response.message = json.dumps(self._capture_result)
        else:
            with self._capture_lock:
                self._capture_pending = 0
            response.success = False
            response.message = (
                f"timeout after {self.capture_timeout_sec:.1f}s "
                "(no synced frame, or worker/model failure -- check the log)")
        return response

    def _on_synced_static_intr(self, color_msg: Image, depth_msg: Image) -> None:
        self._on_synced(color_msg, depth_msg, None)

    def _demanded(self) -> bool:
        """Is anyone actually consuming the refined stream?

        Demand-driven filter semantics (ASPA 2026-07-22): the node and the
        loaded model stay resident so switching on is instant, but inference
        only runs while something subscribes to an output. Wire it into the
        pipeline and it takes effect; unwire it and the GPU work stops.

        Without this the node burns a ViT-L inference on every D405 frame
        forever, even when no subscriber exists -- which is what it did when
        the selector gate was disabled to make it run at all.

        2026-08-23: this gate covered the GPU but **not the subscriptions**.
        The node kept receiving 960 Mbps of raw color+depth and threw it
        away (62% of a core, measured on aspa1 over 8 days). The input
        subscriptions are now gated too -- see `_reconcile_input` and
        `demand_gate.InputSubscriptionGate`. This predicate stays the
        source of truth for "does anyone want the refined stream"; the
        capture service adds its own demand on top.
        """
        pubs = [self.depth_pub, self.mask_pub]
        if self.points_pub is not None:
            pubs.append(self.points_pub)
        for pub in pubs:
            try:
                if pub.get_subscription_count() > 0:
                    return True
            except Exception:       # noqa: BLE001 - never let a probe stop work
                return True
        return False

    def _on_synced(self, color_msg: Image, depth_msg: Image,
                   info_msg: Optional[CameraInfo]) -> None:
        capture = False
        if not self._source_selected:
            with self._capture_lock:
                if self._capture_pending <= 0:
                    return
                capture = True
        # A pending /capture request is an explicit one-shot demand, so it
        # bypasses the subscriber check (the caller wants a single frame
        # even with nothing subscribed).
        if not capture and not self._demanded():
            self.get_logger().debug(
                'no subscriber on refined outputs - skipping inference',
                throttle_duration_sec=30.0)
            return
        start = time.perf_counter()

        try:
            color_bgr = self._color_to_bgr(color_msg)
            depth_m = self._depth_to_meters(depth_msg)
        except Exception as exc:
            self.get_logger().warning(f"Input conversion failed: {exc}")
            return

        if color_bgr is None or depth_m is None:
            return

        if color_bgr.shape[:2] != depth_m.shape[:2]:
            color_bgr = cv2.resize(
                color_bgr,
                (depth_m.shape[1], depth_m.shape[0]),
                interpolation=cv2.INTER_LINEAR,
            )

        if info_msg is not None:
            intr_norm = self._normalized_intrinsics(
                info_msg, depth_m.shape[1], depth_m.shape[0])
        else:
            intr_norm = self._static_intrinsics_norm(
                depth_m.shape[1], depth_m.shape[0])
        if intr_norm is None:
            return

        # PointCloud2 generation and transfer are expensive; only produce
        # points when someone is actually listening.
        want_points = (
            self.points_pub is not None
            and self.points_pub.get_subscription_count() > 0
        )

        # Captures never fall back to passthrough: raw depth dressed up as
        # refined would defeat the point of the capture. The pending flag
        # stays set so the next frame retries until the service times out.
        level = self.capture_resolution_level if capture else self.resolution_level
        allow_passthrough = self.fallback_passthrough and not capture

        if self.backend == "http":
            refined_depth, mask, points = self._run_remote_model(
                color_bgr, depth_m, intr_norm, want_points=want_points,
                resolution_level=level,
            )
            if refined_depth is None:
                if not allow_passthrough:
                    return
                refined_depth = depth_m
                mask = np.isfinite(refined_depth) & (refined_depth > 0.0)
                points = None
        elif self._model is not None:
            refined_depth, mask, points = self._run_model(
                color_bgr, depth_m, intr_norm, resolution_level=level)
            if refined_depth is None:
                if not allow_passthrough:
                    return
                refined_depth = depth_m
                mask = np.isfinite(refined_depth) & (refined_depth > 0.0)
                points = None
        else:
            if not allow_passthrough:
                return
            refined_depth = depth_m
            mask = np.isfinite(refined_depth) & (refined_depth > 0.0)
            points = None

        if want_points and points is None:
            points = self._depth_to_points(refined_depth, intr_norm, mask)

        header = depth_msg.header
        if self.frame_id_override:
            header.frame_id = self.frame_id_override

        depth_out = np.nan_to_num(refined_depth.astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
        if self.output_encoding == "16UC1":
            depth_mm = np.clip(depth_out / self.depth_scale_16uc1, 0, 65535).astype(np.uint16)
            depth_image_msg = self._to_image_msg(depth_mm, "16UC1", header)
        else:
            depth_image_msg = self._to_image_msg(depth_out, "32FC1", header)
        self.depth_pub.publish(depth_image_msg)

        if mask is None:
            mask = np.isfinite(depth_out) & (depth_out > 0.0)
        mask_u8 = np.where(mask, 255, 0).astype(np.uint8)
        mask_msg = self._to_image_msg(mask_u8, "mono8", header)
        self.mask_pub.publish(mask_msg)

        if want_points:
            cloud_msg = self._to_pointcloud2(points, header)
            self.points_pub.publish(cloud_msg)

        self._frame_counter += 1
        proc_ms = (time.perf_counter() - start) * 1000.0
        if capture:
            stamp = float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9
            with self._capture_lock:
                self._capture_pending = 0
                self._capture_result = {
                    "stamp": stamp,
                    "frame_id": header.frame_id,
                    "proc_ms": round(proc_ms, 1),
                    "resolution_level": level,
                }
            self._capture_event.set()
            self.get_logger().info(
                f"capture done stamp={stamp:.3f} proc_ms={proc_ms:.0f} level={level}")
        self._proc_ms_acc += proc_ms
        if self._frame_counter % self.log_every_n_frames == 0:
            avg = self._proc_ms_acc / float(self.log_every_n_frames)
            self._proc_ms_acc = 0.0
            if self.backend == "http":
                mode = "http"
            elif self._model is not None:
                mode = "direct"
            else:
                mode = "fallback"
            self.get_logger().info(
                f"processed={self._frame_counter} avg_proc_ms={avg:.1f} model={mode}"
            )

    def _run_remote_model(
        self, color_bgr: np.ndarray, depth_m: np.ndarray, intr_norm: np.ndarray,
        want_points: bool = True, resolution_level: Optional[int] = None,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        if resolution_level is None:
            resolution_level = self.resolution_level
        payload = io.BytesIO()
        # Uncompressed: zlib costs ~100ms+/frame on Jetson and the worker is
        # local, so raw arrays are much faster end-to-end.
        np.savez(
            payload,
            color_bgr=color_bgr.astype(np.uint8, copy=False),
            depth_m=depth_m.astype(np.float32, copy=False),
            intrinsics_norm=intr_norm.astype(np.float32, copy=False),
            use_fp16=np.asarray(self.use_fp16, dtype=np.uint8),
            apply_mask=np.asarray(self.apply_mask, dtype=np.uint8),
            resolution_level=np.asarray(resolution_level, dtype=np.int32),
            return_points=np.asarray(want_points, dtype=np.uint8),
            compress=np.asarray(0, dtype=np.uint8),
        )

        req = urllib.request.Request(
            self.worker_endpoint,
            data=payload.getvalue(),
            headers={"Content-Type": "application/octet-stream"},
            method="POST",
        )

        try:
            with urllib.request.urlopen(req, timeout=self.worker_timeout_sec) as resp:
                body = resp.read()
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace").strip()
            if detail:
                self.get_logger().error(
                    f"worker request failed status={exc.code} detail={detail}",
                    throttle_duration_sec=5.0,
                )
            else:
                self.get_logger().error(
                    f"worker request failed status={exc.code}", throttle_duration_sec=5.0
                )
            return None, None, None
        except Exception as exc:
            self.get_logger().error(f"worker request failed: {exc}", throttle_duration_sec=5.0)
            return None, None, None

        try:
            with np.load(io.BytesIO(body), allow_pickle=False) as result:
                depth_np = None
                points_np = None
                mask_np = None

                if "depth" in result.files:
                    depth_np = result["depth"].astype(np.float32, copy=False)
                if "mask" in result.files:
                    mask_np = result["mask"].astype(bool, copy=False)
                if "points" in result.files:
                    points_np = result["points"].astype(np.float32, copy=False)

                return depth_np, mask_np, points_np
        except Exception as exc:
            self.get_logger().error(f"invalid worker response: {exc}")
            return None, None, None

    # Manual Image<->numpy conversion: the vlabor backend container has no
    # Python cv_bridge (C++ only), and the encodings involved are trivial.
    def _color_to_bgr(self, msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * msg.step
        if buf.size < expected:
            raise RuntimeError(f"color buffer too small: {buf.size} < {expected}")
        if encoding in ("yuv422_yuy2", "yuyv"):
            # The D555 streams its color natively as YUY2.
            rows = buf[:expected].reshape(msg.height, msg.step)
            pix = rows[:, : msg.width * 2].reshape(msg.height, msg.width, 2)
            return cv2.cvtColor(pix, cv2.COLOR_YUV2BGR_YUY2)
        if encoding not in ("bgr8", "rgb8"):
            raise RuntimeError(f"unsupported color encoding: {msg.encoding}")
        img = buf[:expected].reshape(msg.height, msg.step // 3, 3)[:, : msg.width, :]
        if encoding == "rgb8":
            img = img[:, :, ::-1]
        return np.ascontiguousarray(img)

    def _to_image_msg(self, arr: np.ndarray, encoding: str, header) -> Image:
        msg = Image()
        msg.header = header
        msg.height = int(arr.shape[0])
        msg.width = int(arr.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = int(arr.shape[1] * arr.itemsize)
        msg.data = arr.tobytes()
        return msg

    def _depth_to_meters(self, msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        if encoding in ("16uc1", "mono16"):
            depth = np.frombuffer(msg.data, dtype=np.uint16)
            scale = self.depth_scale_16uc1
        elif encoding == "32fc1":
            depth = np.frombuffer(msg.data, dtype=np.float32)
            scale = 1.0
        else:
            raise RuntimeError(f"unsupported depth encoding: {msg.encoding}")
        row_elems = msg.step // depth.itemsize
        expected = msg.height * row_elems
        if depth.size < expected:
            raise RuntimeError(f"depth buffer too small: {depth.size} < {expected}")
        depth_m = (
            depth[:expected].reshape(msg.height, row_elems)[:, : msg.width].astype(np.float32) * scale
        )
        return np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)

    def _normalized_intrinsics(self, info: CameraInfo, width: int, height: int) -> Optional[np.ndarray]:
        k = np.asarray(info.k, dtype=np.float32).reshape(3, 3)
        fx = float(k[0, 0])
        fy = float(k[1, 1])
        cx = float(k[0, 2])
        cy = float(k[1, 2])
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warning(f"invalid camera intrinsics fx={fx:.3f} fy={fy:.3f}")
            return None

        out = np.eye(3, dtype=np.float32)
        out[0, 0] = fx / float(width)
        out[1, 1] = fy / float(height)
        out[0, 2] = cx / float(width)
        out[1, 2] = cy / float(height)
        return out

    def _static_intrinsics_norm(self, width: int, height: int) -> Optional[np.ndarray]:
        if self._static_intr is None:
            return None
        fx, fy, cx, cy = self._static_intr
        out = np.eye(3, dtype=np.float32)
        out[0, 0] = fx / float(width)
        out[1, 1] = fy / float(height)
        out[0, 2] = cx / float(width)
        out[1, 2] = cy / float(height)
        return out

    def _run_model(
        self, color_bgr: np.ndarray, depth_m: np.ndarray, intr_norm: np.ndarray,
        resolution_level: Optional[int] = None,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        if self._model is None or self._torch is None:
            return None, None, None
        if resolution_level is None:
            resolution_level = self.resolution_level

        try:
            torch = self._torch
            color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

            image_t = torch.from_numpy(color_rgb).to(device=self._device, dtype=torch.float32)
            image_t = image_t.permute(2, 0, 1).unsqueeze(0)
            depth_t = torch.from_numpy(depth_m.astype(np.float32)).to(device=self._device).unsqueeze(0)
            intr_t = torch.from_numpy(intr_norm).to(device=self._device).unsqueeze(0)

            with torch.inference_mode():
                out = self._model.infer(
                    image_t,
                    depth_in=depth_t,
                    intrinsics=intr_t,
                    use_fp16=self.use_fp16,
                    apply_mask=self.apply_mask,
                    resolution_level=resolution_level,
                )

            depth_out = out.get("depth", None)
            points_out = out.get("points", None)
            mask_out = out.get("mask", None)

            depth_np = None
            points_np = None
            mask_np = None
            if depth_out is not None:
                depth_np = depth_out.detach().float().cpu().numpy().squeeze().astype(np.float32)
            if points_out is not None:
                points_np = points_out.detach().float().cpu().numpy().squeeze().astype(np.float32)
            if mask_out is not None:
                mask_np = mask_out.detach().cpu().numpy().squeeze().astype(bool)

            return depth_np, mask_np, points_np
        except Exception as exc:
            self.get_logger().error(f"inference failed: {exc}")
            return None, None, None

    def _depth_to_points(
        self, depth_m: np.ndarray, intr_norm: np.ndarray, mask: Optional[np.ndarray]
    ) -> np.ndarray:
        h, w = depth_m.shape
        fx = float(intr_norm[0, 0]) * float(w)
        fy = float(intr_norm[1, 1]) * float(h)
        cx = float(intr_norm[0, 2]) * float(w)
        cy = float(intr_norm[1, 2]) * float(h)

        u, v = np.meshgrid(
            np.arange(w, dtype=np.float32),
            np.arange(h, dtype=np.float32),
        )
        z = depth_m.astype(np.float32)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        pts = np.stack([x, y, z], axis=-1)

        valid = np.isfinite(z) & (z > 0.0)
        if mask is not None:
            valid &= mask
        pts[~valid] = np.inf
        return pts

    def _to_pointcloud2(self, points: Optional[np.ndarray], header) -> PointCloud2:
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.is_bigendian = False
        msg.is_dense = False
        msg.point_step = 12
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        if points is None or points.ndim != 3 or points.shape[2] != 3:
            msg.width = 0
            msg.row_step = 0
            msg.data = b""
            return msg

        sampled = points[:: self.point_stride, :: self.point_stride, :].reshape(-1, 3)
        if sampled.size == 0:
            msg.width = 0
            msg.row_step = 0
            msg.data = b""
            return msg

        valid = (
            np.isfinite(sampled[:, 0])
            & np.isfinite(sampled[:, 1])
            & np.isfinite(sampled[:, 2])
            & (sampled[:, 2] >= self.min_depth_m)
            & (sampled[:, 2] <= self.max_depth_m)
        )
        xyz = sampled[valid].astype(np.float32, copy=False)
        msg.width = int(xyz.shape[0])
        msg.row_step = msg.point_step * msg.width
        msg.data = xyz.tobytes()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FvLingbotDepthNode()
    # Two threads: the blocking ~/capture handler (own callback group) waits
    # while the sync callback processes the frame on the default group.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
