#!/usr/bin/env python3
"""YOLOE open-vocabulary detection & segmentation ROS2 node.

Text prompt で任意の物体をリアルタイム検出。
プロンプトは ROS2 パラメータまたはサービスで動的に変更可能。
"""
from __future__ import annotations

import time
import json
import os
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
try:
    from fv_msgs.msg import Detection2D, DetectionArray
    _HAS_FV_MSGS = True
except ImportError:
    _HAS_FV_MSGS = False


class FvYoloeNode(Node):
    def __init__(self) -> None:
        super().__init__("fv_yoloe")

        # --- Parameters ---
        self.declare_parameter("input_image_topic", "~/color/image_raw")
        self.declare_parameter("overlay_topic", "yoloe/overlay")
        self.declare_parameter("overlay_compressed_topic", "yoloe/overlay/compressed")
        self.declare_parameter("detections_topic", "yoloe/detections")
        self.declare_parameter("mask_topic", "yoloe/mask")

        # Model
        self.declare_parameter("model_name", "yoloe-11s-seg.pt")
        self.declare_parameter("device", "auto")  # auto|cuda|cpu
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.5)

        # Text prompt (comma-separated class names)
        self.declare_parameter("text_prompt", "cube")
        # Topic to receive prompt updates (publish String to change classes)
        self.declare_parameter("prompt_topic", "~/set_prompt")
        # Topic to receive visual prompt updates from the dashboard registry.
        self.declare_parameter("visual_prompt_topic", "~/set_visual_prompt")

        # Performance
        self.declare_parameter("processing_frequency", 10.0)
        self.declare_parameter("qos_reliability", "best_effort")
        self.declare_parameter("log_every_n_frames", 30)
        self.declare_parameter("jpeg_quality", 80)
        # Runtime on/off gate (dashboard toggle). While false the image
        # callback returns before decode, so the node does no GPU/CPU work
        # at all; the model stays loaded and switching back on is instant.
        # Same pattern as the fv_lingbot_depth depth-source gate.
        self.declare_parameter("enabled", True)

        # Read params
        self.input_topic = str(self.get_parameter("input_image_topic").value)
        self.overlay_topic = str(self.get_parameter("overlay_topic").value)
        self.overlay_compressed_topic = str(self.get_parameter("overlay_compressed_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.mask_topic = str(self.get_parameter("mask_topic").value)

        self.model_name = str(self.get_parameter("model_name").value)
        self.device_mode = str(self.get_parameter("device").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)

        self.text_prompt = str(self.get_parameter("text_prompt").value)
        self.prompt_topic = str(self.get_parameter("prompt_topic").value)
        self.visual_prompt_topic = str(self.get_parameter("visual_prompt_topic").value)

        self.processing_frequency = float(self.get_parameter("processing_frequency").value)
        self.qos_reliability = str(self.get_parameter("qos_reliability").value).strip().lower()
        self.log_every_n_frames = max(1, int(self.get_parameter("log_every_n_frames").value))
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.enabled = bool(self.get_parameter("enabled").value)
        self.add_on_set_parameters_callback(self._on_param_update)

        # Publishers (use sensor QoS = BEST_EFFORT for streaming topics)
        qos_pub = self._sensor_qos()
        self.overlay_pub = self.create_publisher(Image, self.overlay_topic, qos_pub)
        self.overlay_compressed_pub = self.create_publisher(
            CompressedImage, self.overlay_compressed_topic, qos_pub
        )
        # Two publishers, distinct topics so Foxglove/rqt don't see the same
        # name with two different schemas:
        #   <detections_topic>       -> fv_msgs/DetectionArray (primary, for ROS nodes)
        #   <detections_topic>/json  -> std_msgs/String        (JSON debug dump)
        self.detections_pub = self.create_publisher(
            String, f'{self.detections_topic}/json', 10
        )
        if _HAS_FV_MSGS:
            self.detections_fv_pub = self.create_publisher(
                DetectionArray, self.detections_topic, 10
            )
        else:
            self.detections_fv_pub = None
        self.mask_pub = self.create_publisher(Image, self.mask_topic, qos_pub)

        # Load model
        self._model = None
        self._current_classes: List[str] = []
        self._class_aliases = {}
        self._prompt_mode = "text"
        self._load_model()

        # Subscriber
        qos = self._sensor_qos()
        self.image_sub = self.create_subscription(
            Image, self.input_topic, self._on_image, qos
        )

        # Prompt update subscriber
        self.prompt_sub = self.create_subscription(
            String, self.prompt_topic, self._on_prompt, 10
        )
        self.visual_prompt_sub = self.create_subscription(
            String, self.visual_prompt_topic, self._on_visual_prompt, 10
        )

        # Service to get current prompt
        self.create_service(Trigger, "~/get_prompt", self._srv_get_prompt)

        self._frame_counter = 0
        self._proc_ms_acc = 0.0
        self._last_proc_time = 0.0

        self.get_logger().info(
            f"fv_yoloe started: model={self.model_name} prompt='{self.text_prompt}' "
            f"input={self.input_topic} overlay={self.overlay_compressed_topic}"
        )

    def _sensor_qos(self) -> QoSProfile:
        reliable = self.qos_reliability == "reliable"
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

    def _load_model(self) -> None:
        try:
            from ultralytics import YOLOE
            import torch

            if self.device_mode == "cuda":
                device = "cuda"
            elif self.device_mode == "cpu":
                device = "cpu"
            else:
                device = "cuda" if torch.cuda.is_available() else "cpu"

            self.get_logger().info(f"Loading YOLOE model: {self.model_name} on {device}...")
            self._model = YOLOE(self.model_name)
            self._model.to(device)
            self._device = device

            # Set initial text prompt
            self._set_classes(self.text_prompt)

            self.get_logger().info(f"YOLOE model loaded: {self.model_name} on {device}")
        except Exception as e:
            self.get_logger().error(f"YOLOE model load failed: {e}")
            self._model = None

    @staticmethod
    def _prompt_names(prompt_str: str) -> List[str]:
        return [n.strip() for n in str(prompt_str or "").split(",") if n.strip()]

    def _set_classes(self, prompt_str: str) -> None:
        if self._model is None:
            return

        names = self._prompt_names(prompt_str)
        if not names:
            self._clear_classes()
            return

        if names == self._current_classes:
            return

        try:
            text_pe = self._model.get_text_pe(names)
            self._model.set_classes(names, text_pe)
            self._current_classes = names
            self._class_aliases = {}
            self._prompt_mode = "text"
            self.get_logger().info(f"YOLOE classes set: {names}")
        except Exception as e:
            self.get_logger().error(f"Failed to set classes: {e}")

    def _clear_classes(self) -> None:
        self._current_classes = []
        self._class_aliases = {}
        self._prompt_mode = "none"
        if self._model is not None:
            self._model.predictor = None
        self.get_logger().info("YOLOE classes cleared")

    def _on_prompt(self, msg: String) -> None:
        new_prompt = msg.data.strip()
        if new_prompt in ("__clear__", "__none__"):
            new_prompt = ""
        self.get_logger().info(f"Prompt update: '{new_prompt}'")
        self._set_classes(new_prompt)
        # Keep the text_prompt PARAM in sync so `ros2 param get` (and the
        # perception MCP's get_yoloe_prompt) reflect the live prompt - it
        # used to return the startup value forever after topic updates.
        try:
            from rclpy.parameter import Parameter as RclpyParameter
            self.set_parameters([RclpyParameter(
                "text_prompt", RclpyParameter.Type.STRING, new_prompt)])
        except Exception as e:
            self.get_logger().warning(f"text_prompt param sync failed: {e}")

    def _on_visual_prompt(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data or "{}")
        except json.JSONDecodeError as e:
            self.get_logger().warning(f"Invalid visual prompt JSON: {e}")
            return
        self._set_visual_prompt(payload)

    def _build_visual_pe(self, predictor, label: str, examples: list):
        import torch

        visual_pes = []
        used_examples = []
        skipped = 0
        for candidate in examples:
            if not isinstance(candidate, dict):
                skipped += 1
                continue
            image_path = str(
                candidate.get("image_path_abs") or candidate.get("image_path") or ""
            ).strip()
            if not image_path or not os.path.exists(image_path):
                skipped += 1
                continue

            bbox = candidate.get("bbox") or [
                0, 0, candidate.get("width", 0), candidate.get("height", 0)
            ]
            try:
                bbox = [float(v) for v in bbox[:4]]
            except Exception:
                skipped += 1
                continue
            if len(bbox) != 4 or bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
                skipped += 1
                continue

            try:
                predictor.set_prompts({"bboxes": [bbox], "cls": [0]})
                predictor.setup_model(model=self._model.model, verbose=False)
                visual_pe = predictor.get_vpe(image_path)
                if visual_pe is None:
                    skipped += 1
                    continue
                visual_pes.append(visual_pe)
                used_examples.append({
                    "id": str(candidate.get("id") or ""),
                    "image": image_path,
                    "bbox": bbox,
                })
            except Exception as e:
                skipped += 1
                self.get_logger().warning(
                    f"Visual prompt example skipped for {label}: {e}"
                )

        if not visual_pes:
            return None, [], [], skipped

        if len(visual_pes) == 1:
            return visual_pes[0], [label], used_examples, skipped

        try:
            return torch.cat(visual_pes, dim=1), [label] * len(visual_pes), used_examples, skipped
        except Exception as e:
            self.get_logger().warning(
                f"Visual prompt concat failed for {label}, falling back to mean: {e}"
            )
            base_dtype = visual_pes[0].dtype
            visual_pe = torch.stack(
                [pe.to(dtype=torch.float32) for pe in visual_pes],
                dim=0,
            ).mean(dim=0).to(dtype=base_dtype)
            return visual_pe, [label], used_examples, skipped

    def _set_visual_prompt(self, payload: dict) -> None:
        if self._model is None:
            return

        text_names = self._prompt_names(payload.get("text_prompt") or payload.get("text") or "")
        targets = payload.get("targets")
        if not isinstance(targets, list):
            label = str(payload.get("label") or payload.get("prompt") or payload.get("key") or "").strip()
            examples = payload.get("examples") or []
            if not label:
                self.get_logger().warning("Visual prompt ignored: label is empty")
                return
            targets = [{"label": label, "key": payload.get("key", ""), "examples": examples}]

        try:
            import torch
            from ultralytics.models.yolo.yoloe.predict import (
                YOLOEVPDetectPredictor,
                YOLOEVPSegPredictor,
            )

            class_names = []
            class_aliases = {}
            prompt_pes = []
            used_visual = []
            skipped = 0

            if text_names:
                prompt_pes.append(self._model.get_text_pe(text_names))
                class_names.extend(text_names)

            task = str(getattr(self._model, "task", "") or getattr(self._model.model, "task", ""))
            predictor_cls = YOLOEVPSegPredictor if task == "segment" else YOLOEVPDetectPredictor
            predictor = predictor_cls(
                overrides={
                    "task": task or self._model.model.task,
                    "mode": "predict",
                    "save": False,
                    "verbose": False,
                    "batch": 1,
                    "device": self._device,
                    "half": False,
                    "imgsz": self._model.overrides.get("imgsz", 640),
                },
                _callbacks=self._model.callbacks,
            )

            for target in targets:
                if not isinstance(target, dict):
                    skipped += 1
                    continue
                label = str(target.get("label") or target.get("prompt") or target.get("key") or "").strip()
                examples = target.get("examples") or []
                if not label or not isinstance(examples, list) or not examples:
                    skipped += 1
                    continue
                visual_pe, visual_names, used_examples, skipped_count = self._build_visual_pe(
                    predictor, label, examples
                )
                skipped += skipped_count
                if visual_pe is None:
                    continue
                prompt_pes.append(visual_pe)
                class_names.extend(visual_names)
                for name in visual_names:
                    class_aliases[name] = label
                used_visual.extend({"label": label, **item} for item in used_examples)

            if not prompt_pes or not class_names:
                self.get_logger().warning("YOLOE prompt ignored: no usable text or visual prompts")
                return

            pe = prompt_pes[0] if len(prompt_pes) == 1 else torch.cat(prompt_pes, dim=1)
            self._model.set_classes(class_names, pe)
            self._model.predictor = None
            self._current_classes = list(dict.fromkeys(class_aliases.get(name, name) for name in class_names))
            self._class_aliases = class_aliases
            self._prompt_mode = "mixed" if text_names and used_visual else ("visual" if used_visual else "text")
            self.get_logger().info(
                f"YOLOE prompt set: mode={self._prompt_mode} "
                f"text={text_names} visual_examples={len(used_visual)} "
                f"embeddings={len(class_names)} skipped={skipped}"
            )
            for item in used_visual:
                self.get_logger().debug(
                    f"YOLOE visual example: label={item['label']} id={item['id']} "
                    f"image={item['image']} bbox={item['bbox']}"
                )
        except Exception as e:
            self.get_logger().error(f"Failed to set YOLOE visual/mixed prompt: {e}")

    def _srv_get_prompt(self, request, response):
        response.success = self._model is not None
        response.message = ",".join(self._current_classes)
        return response

    def _on_param_update(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == "enabled":
                new_enabled = bool(p.value)
                if new_enabled != self.enabled:
                    self.enabled = new_enabled
                    if new_enabled:
                        self.get_logger().info("YOLOE inference ENABLED")
                    else:
                        self.get_logger().info(
                            "YOLOE inference DISABLED - model stays loaded, no GPU/CPU work")
            elif p.name == "conf_threshold":
                v = float(p.value)
                if not 0.0 <= v <= 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason="conf_threshold must be within [0.0, 1.0]")
                self.conf_threshold = v
                self.get_logger().info(f"conf_threshold -> {v:.2f}")
            elif p.name == "iou_threshold":
                v = float(p.value)
                if not 0.0 <= v <= 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason="iou_threshold must be within [0.0, 1.0]")
                self.iou_threshold = v
                self.get_logger().info(f"iou_threshold -> {v:.2f}")
            elif p.name == "processing_frequency":
                v = float(p.value)
                if v < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="processing_frequency must be >= 0 (0 = every frame)")
                self.processing_frequency = v
                self.get_logger().info(f"processing_frequency -> {v:.1f}Hz")
            elif p.name == "text_prompt":
                # `ros2 param set` path. _set_classes no-ops on an unchanged
                # list, and this branch never re-sets the param, so the
                # topic-side sync in _on_prompt cannot recurse through here.
                v = str(p.value).strip()
                self.get_logger().info(f"Prompt update via param: '{v}'")
                self._set_classes(v)
        return SetParametersResult(successful=True)

    def _on_image(self, msg: Image) -> None:
        if not self.enabled:
            return
        # Throttle
        if self.processing_frequency > 0:
            now = time.monotonic()
            interval = 1.0 / self.processing_frequency
            if (now - self._last_proc_time) < interval:
                return
            self._last_proc_time = now

        start = time.perf_counter()

        # Decode image
        try:
            img_bgr = self._imgmsg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warning(f"Image decode failed: {e}")
            return

        if self._model is None or not self._current_classes:
            self._publish_passthrough(msg, img_bgr, start)
            return

        # Inference
        try:
            results = self._model.predict(
                img_bgr,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                verbose=False,
            )
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            return

        result = results[0]

        # Draw overlay
        overlay = result.plot()

        # Add stats overlay text at top
        proc_ms_now = (time.perf_counter() - start) * 1000.0
        n_det = len(result.boxes) if result.boxes is not None else 0
        device_str = getattr(self, '_device', '?')
        info_text = f"{self.model_name} | {device_str} | {proc_ms_now:.0f}ms | {n_det} det"
        prompt_text = f"Prompt({self._prompt_mode}): {', '.join(self._current_classes)}"
        w_img = overlay.shape[1]
        # Background bar at top
        cv2.rectangle(overlay, (0, 0), (w_img, 44), (0, 0, 0), -1)
        cv2.putText(overlay, info_text, (8, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 200, 255), 1, cv2.LINE_AA)
        cv2.putText(overlay, prompt_text, (8, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 220, 150), 1, cv2.LINE_AA)

        header = msg.header

        # Publish raw overlay
        overlay_msg = self._cv2_to_imgmsg(overlay, "bgr8", header)
        self.overlay_pub.publish(overlay_msg)

        # Publish compressed for dashboard
        _, jpeg_data = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        comp_msg = CompressedImage()
        comp_msg.header = header
        comp_msg.format = "jpeg"
        comp_msg.data = jpeg_data.tobytes()
        self.overlay_compressed_pub.publish(comp_msg)

        # Publish combined instance mask (for fv_3d_detector input)
        h, w = img_bgr.shape[:2]
        combined_mask = np.zeros((h, w), dtype=np.uint8)
        if result.masks is not None and len(result.masks) > 0:
            for i, mask_data in enumerate(result.masks.data):
                m = mask_data.cpu().numpy()
                if m.shape != (h, w):
                    m = cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST)
                combined_mask[m > 0.5] = i + 1  # instance ID (1-based)
        mask_msg = self._cv2_to_imgmsg(combined_mask, "mono8", header)
        self.mask_pub.publish(mask_msg)

        # Stats
        self._frame_counter += 1
        proc_ms = (time.perf_counter() - start) * 1000.0
        self._proc_ms_acc += proc_ms
        n_det = len(result.boxes) if result.boxes is not None else 0

        # Publish detections + stats as JSON
        dets = []
        if result.boxes is not None:
            for i, box in enumerate(result.boxes):
                cls_id = int(box.cls[0])
                cls_name = result.names.get(cls_id, str(cls_id))
                cls_name = self._class_aliases.get(cls_name, cls_name)
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                dets.append({
                    "class": cls_name, "conf": round(conf, 3),
                    "bbox": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)]
                })

        avg_ms = self._proc_ms_acc / self._frame_counter if self._frame_counter > 0 else proc_ms
        det_msg = String()
        det_msg.data = json.dumps({
            "detections": dets,
            "stats": {
                "model": self.model_name,
                "device": getattr(self, '_device', 'unknown'),
                "inference_ms": round(proc_ms, 1),
                "avg_ms": round(avg_ms, 1),
                "fps": round(1000.0 / avg_ms, 1) if avg_ms > 0 else 0,
                "frame": self._frame_counter,
                "classes": self._current_classes,
                "prompt_mode": self._prompt_mode,
                "num_detections": n_det,
            }
        })
        self.detections_pub.publish(det_msg)

        # Publish fv_msgs/DetectionArray for fv_3d_detector compatibility
        if self.detections_fv_pub is not None and result.boxes is not None:
            from geometry_msgs.msg import Point32
            det_arr = DetectionArray()
            det_arr.header = header
            for i, box in enumerate(result.boxes):
                d = Detection2D()
                d.header = header
                d.id = i
                d.class_id = int(box.cls[0])
                raw_label = result.names.get(d.class_id, str(d.class_id))
                d.label = self._class_aliases.get(raw_label, raw_label)
                d.conf_fused = float(box.conf[0])
                d.conf_object = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                d.bbox_min = Point32(x=float(x1), y=float(y1), z=0.0)
                d.bbox_max = Point32(x=float(x2), y=float(y2), z=0.0)
                d.mask_instance_id = i + 1
                det_arr.detections.append(d)
            self.detections_fv_pub.publish(det_arr)

        if self._frame_counter % self.log_every_n_frames == 0:
            self._proc_ms_acc = 0.0
            self._frame_counter = 0
            self.get_logger().info(
                f"avg={avg_ms:.1f}ms ({1000/avg_ms:.1f}fps) "
                f"detections={n_det} classes={self._current_classes}"
            )

    def _publish_passthrough(self, msg: Image, img_bgr: np.ndarray, start: float) -> None:
        """Publish camera image plus empty detection products when no prompt is active."""
        header = msg.header
        overlay_msg = self._cv2_to_imgmsg(img_bgr, "bgr8", header)
        self.overlay_pub.publish(overlay_msg)

        _, jpeg_data = cv2.imencode(".jpg", img_bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        comp_msg = CompressedImage()
        comp_msg.header = header
        comp_msg.format = "jpeg"
        comp_msg.data = jpeg_data.tobytes()
        self.overlay_compressed_pub.publish(comp_msg)

        h, w = img_bgr.shape[:2]
        mask_msg = self._cv2_to_imgmsg(np.zeros((h, w), dtype=np.uint8), "mono8", header)
        self.mask_pub.publish(mask_msg)

        proc_ms = (time.perf_counter() - start) * 1000.0
        det_msg = String()
        det_msg.data = json.dumps({
            "detections": [],
            "stats": {
                "model": self.model_name,
                "device": getattr(self, '_device', 'unknown'),
                "inference_ms": 0.0,
                "avg_ms": round(proc_ms, 1),
                "fps": 0,
                "frame": self._frame_counter,
                "classes": self._current_classes,
                "prompt_mode": "none" if not self._current_classes else self._prompt_mode,
                "num_detections": 0,
            }
        })
        self.detections_pub.publish(det_msg)

        if self.detections_fv_pub is not None:
            det_arr = DetectionArray()
            det_arr.header = header
            self.detections_fv_pub.publish(det_arr)

    # --- Image conversion (no cv_bridge) ---
    def _imgmsg_to_bgr(self, msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        if encoding in ("bgr8", "8uc3"):
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        elif encoding == "rgb8":
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        elif encoding in ("mono8", "8uc1"):
            gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            raise ValueError(f"Unsupported encoding: {msg.encoding}")

    def _cv2_to_imgmsg(self, img: np.ndarray, encoding: str, header) -> Image:
        msg = Image()
        msg.header = header
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = encoding
        if img.ndim == 3:
            msg.step = img.shape[1] * img.shape[2] * img.dtype.itemsize
        else:
            msg.step = img.shape[1] * img.dtype.itemsize
        msg.is_bigendian = False
        msg.data = img.tobytes()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FvYoloeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
