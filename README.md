<div align="center">

# 🌿 FluentVision ROS 2

### A declarative, YAML-driven **vision & perception stack** for ROS 2.
### Wire RGB-D cameras, AI models, and streaming endpoints together — visually, in a browser.

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](#license)
[![Platform](https://img.shields.io/badge/platform-linux%20%7C%20aarch64%20%7C%20x86__64-lightgrey)](#requirements)
[![Language](https://img.shields.io/badge/C%2B%2B%20%2F%20Python-blue)](#)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](#contributing)

<sub>RealSense · YOLOE · YOLO-seg · Depth Anything V3 · AprilTag · 3D OBB · FoundationPose bridge · MJPEG / WebSocket / RTMP streaming · Episode recording</sub>

<br/>

<!-- Drop a hero GIF or screenshot here -->
<!-- <img src="docs/assets/hero.gif" width="780" alt="FluentVision pipeline editor demo" /> -->

</div>

---

## ✨ Why FluentVision?

Building a real-world robot perception stack usually means: a tangled launch file, hard-coded topic names, model loading code copy-pasted between projects, and zero way to inspect or rewire things at runtime.

**FluentVision flips that around.**

- 🧩 **Pipelines are data, not code** — declare your camera → AI → streaming graph in a YAML file.
- 🖱️ **Edit it in a browser** — drag, drop, wire, preview live frames, hit **Save**. Out comes a runnable pipeline.
- 🧠 **Batteries included** — 18+ ready-made AI / 3D / depth nodes you can mix and match.
- ⚡ **Real-time first** — `BEST_EFFORT` QoS by default, GPU inference (TensorRT / ONNX Runtime), works on Jetson / AGX.
- 🎥 **Stream anywhere** — MJPEG for browsers, WebSocket for dashboards, RTMP for OBS, binary WS for point clouds.

---

## 🌿 Fluent Scene — declarative GPU visual runtime (new)

A framework-neutral rendering core that turns a typed YAML scene (`.fvs`) into a
retained **Vulkan** composite — camera imagery, detection boxes, Japanese text —
with type checking, GPU budget planning, and a ROS 2 adapter. Measured ~26×
faster than CPU compositing at 1080p on Jetson Thor. Lives in
[`core/fluent_scene/`](core/fluent_scene/) (standalone CMake, no ROS/GPU
dependency in the core, `COLCON_IGNORE`d so colcon workflows are unaffected).

| | |
|---|---|
| 🇯🇵 日本語 README（デモページへのリンクあり） | [core/fluent_scene/README.md](core/fluent_scene/README.md) |
| 🇬🇧 English README | [core/fluent_scene/README.en.md](core/fluent_scene/README.en.md) |
| 📐 Design spec (en / ja) | [docs/design/fluent_vision_architecture.md](docs/design/fluent_vision_architecture.md) · [ja](docs/design/fluent_vision_architecture.ja.md) |

---

## 🚀 60-second demo

```bash
# 1. Clone into a ROS 2 workspace
cd ~/ros2_ws/src && git clone https://github.com/takatronix/fluent_vision_ros2.git

# 2. Build (do NOT pass --symlink-install)
cd ~/ros2_ws && colcon build --packages-up-to fv_pipeline_editor fluent_vision_system
source install/setup.bash

# 3. CycloneDDS is required (FastRTPS conflicts with librealsense2)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 4. Launch the visual editor
ros2 run fv_pipeline_editor editor_node
# → open  http://localhost:8095
```

Now plug in an Intel RealSense D405 / D415 / D455, drag a **camera** node and a **YOLOE** node onto the canvas, wire them up, hit **Run**. Live detections in your browser, in under a minute.

---

## 🖼️ The pipeline editor

A browser-based, canvas + WebSocket editor served by `fv_pipeline_editor` on port `8095`.

```
┌────────────────────────────────────────────────────────────────────┐
│  FluentVision Editor                                  [Save] [Run] │
├──────────────┬─────────────────────────────────────────────────────┤
│  Nodes       │   ┌─────────────┐      ┌──────────────┐             │
│  ───────     │   │ RealSense   │─▶── │ YOLOE        │──▶ overlay  │
│  📷 Camera   │   │ D405        │      │ "asparagus"  │             │
│  🧠 YOLOE    │   └─────────────┘      └──────────────┘             │
│  📐 3D Det   │          │                                          │
│  🌊 Depth    │          ▼                                          │
│  🎥 MJPEG    │   ┌─────────────┐                                   │
│  📼 Recorder │   │ 3D Detector │──▶  /fv/objects_3d                │
│  ...         │   └─────────────┘                                   │
├──────────────┴─────────────────────────────────────────────────────┤
│  ▶ Preview (live JPEG over binary WebSocket)                       │
└────────────────────────────────────────────────────────────────────┘
```

- **Load**: read any `pipelines/*.yaml` into the canvas.
- **Save**: serialize the graph back to YAML — version-control it like any artefact.
- **Preview**: every `sensor_msgs/Image` topic is one click away from a live JPEG preview pane.
- **Run**: spawn the whole graph as ROS 2 nodes with one button.

> 💡 *Drop a screenshot at `docs/assets/editor.png` and uncomment the hero image above.*

---

## 🧪 Showcase pipelines

Each of these is a single `ros2 launch` away. They live in [`pipelines/`](pipelines/).

<details>
<summary><b>📦 Open-vocabulary object detection (YOLOE)</b></summary>

```bash
ros2 launch fluent_vision_system run.launch.py \
    pipeline:=pipelines/object_detection.yaml
```

Tell it what to find with a text prompt — no retraining required.

</details>

<details>
<summary><b>🟪 Instance segmentation + 3D bounding boxes</b></summary>

```bash
ros2 launch fluent_vision_system run.launch.py \
    pipeline:=pipelines/instance_segmentation.yaml
```

YOLO-seg masks are fused with the RealSense depth stream and turned into oriented 3D bounding boxes (`fv_msgs/Object3DArray`) ready for grasp planning.

</details>

<details>
<summary><b>🌊 Monocular depth (Depth Anything V3)</b></summary>

```bash
ros2 launch fluent_vision_system run.launch.py \
    pipeline:=pipelines/depth_anything.yaml
```

ONNX Runtime or TensorRT backend, arm64 or x64.

</details>

<details>
<summary><b>🎯 AprilTag cube bundle pose</b></summary>

```bash
ros2 launch fluent_vision_system run.launch.py \
    pipeline:=pipelines/foundationpose_bridge.yaml  # AprilTag pipeline coming soon
```

Detects multi-face AprilTag cubes and publishes a stable pose + per-face debug TFs.

</details>

<details>
<summary><b>📼 Episode recording (rosbag + MP4 + markers)</b></summary>

```bash
ros2 run fv_episode_recorder fv_episode_recorder \
    --ros-args -p output_dir:=/data/episodes
```

Records a synchronized rosbag, per-camera MP4 files, `EpisodeMarker` events, and a sidecar JSON — straight into a LeRobot / Pi0.5-friendly layout.

</details>

<details>
<summary><b>🎥 Stream a camera over MJPEG to your browser</b></summary>

```bash
ros2 launch fluent_vision_system run.launch.py \
    pipeline:=pipelines/mjpeg_streaming.yaml
```

Then open `http://<robot-ip>:8080/stream` from your laptop or phone.

</details>

---

## 📐 Anatomy of a pipeline

Pipelines are plain YAML. Here's the essence:

```yaml
name: "Object Detection"
description: "RealSense D405 → YOLO → overlay + MJPEG"

system:
  camera_start_delay: 2.0

nodes:
  - id: camera_d405
    package: fv_realsense
    exec: fv_realsense_node
    parameters:
      serial: "230322270366"
      enable_color: true
      enable_depth: true

  - id: detector
    package: fv_object_detector
    exec: fv_object_detector_node
    parameters:
      input_topic: /fv/d405/color/image_raw
      output_topic: /fv/objects/detections
      model: yolov10s.onnx

  - id: mjpeg
    package: fv_mjpeg_server
    exec: fv_mjpeg_server_node
    parameters:
      input_topic: /fv/objects/overlay
      port: 8080
```

The pipeline editor reads, writes, and visualises exactly this format — no hidden state.

---

## 🧰 What's in the box

<details open>
<summary><b>🧠 AI & inference (<code>src/ai/</code>)</b></summary>

| Package | Role |
|---|---|
| `fv_yoloe` | YOLOE open-vocabulary detection & segmentation (text prompt) |
| `fv_object_detector` | YOLO-family generic object detection |
| `fv_instance_seg` | YOLO-seg instance segmentation (TensorRT / OpenVINO) |
| `fv_object_mask_generator` | UNet semantic segmentation |
| `fv_3d_detector` | 3D OBB / shape / colour / distance from masks + point clouds |
| `fv_cube_detector` | Cube object detection and pose estimation |
| `fv_color_detector` | HSV colour detection with depth distance (C++) |
| `fv_apriltag` | AprilTag detection + cube-bundle pose (pupil-apriltags) |
| `fv_depth_anything` | Depth Anything V3 monocular depth (ONNX / TensorRT) |
| `fv_depth_features` | HHA encoder, surface normals, RGBD fusion (C++) |
| `fv_lingbot_depth` | LingBot-Depth refinement & point cloud generation |
| `fv_pointcloud_pipeline` | ROI extraction & filtering |
| `fv_detection_fusion` | Multi-source aggregation into `fv_msgs/DetectionArray` |
| `fv_foundationpose_bridge` | Isaac ROS FoundationPose → `Object3DArray` + TF |
| `fv_aspara_analyzer` / `fv_aspara_points` / `fv_stalk_estimator` | Asparagus quality & geometry (original use case) |
| `fv_face_recognizer` | Real-time face recognition |
| `fv_policy_runner` | Watch trigger topics and invoke external policy runtimes |

</details>

`fv_apriltag` resolves each ID's physical black-edge size from
`src/ai/fv_apriltag/config/tag_registry.yaml` (nominal `tag_mm` × 0.8). A
positive `tag_size` parameter is an explicit homogeneous-size override;
unknown or reserved IDs without a size do not produce a pose or TF.
The legacy cube launch profiles set that override to 40 mm because their old
IDs overlap the current field allocations; generic field use does not fall
back to legacy meanings.

<details>
<summary><b>📷 Sensors (<code>src/sensors/</code>)</b></summary>

- **`fv_realsense`** — Intel RealSense driver (D405 / D415 / D455). `BEST_EFFORT` QoS, display-mode switching, pixel→3D distance service, per-frame device-time re-anchoring.
- **`fv_camera`** — generic USB / RPi / network camera driver.

</details>

<details>
<summary><b>📡 Streaming (<code>src/streaming/</code>)</b></summary>

- `fv_episode_recorder` — rosbag + camera MP4 + `EpisodeMarker` + sidecar JSON, with preflight, lock, and retention.
- `fv_image_preview` — lightweight preview bridge with focus telemetry & source stats.
- `fv_mjpeg_server`, `fv_websocket_server`, `fv_pointcloud_ws_server`, `fv_rtmp_server`, `fv_image_distributor`, `fv_recorder`.

</details>

<details>
<summary><b>⚙️ System (<code>src/system/</code>)</b></summary>

- **`fluent_vision_system`** — YAML-driven launcher that resolves nodes, parameters, and wiring from `pipelines/*.yaml`.
- **`fv_pipeline_editor`** — visual pipeline editor (aiohttp + canvas, port `8095`).

</details>

<details>
<summary><b>🔊 Audio (<code>src/audio/</code>)</b></summary>

- `fv_audio` (capture), `fv_audio_output` (ALSA), `fv_audio_vad` (VAD / wakeword), `fv_tts` (Open JTalk / pyopenjtalk).

</details>

<details>
<summary><b>📨 Messages, libraries, utils</b></summary>

- `fv_msgs`, `fv_episode_msgs` — message definitions.
- `fluent_lib` — chainable API around OpenCV / PCL / ROS 2.
- `fv_image_filter` — chainable image filters with live parameter updates.
- `fv_topic_relay`, `fv_aspara_ui_cpp` — utilities.

</details>

---

## 🧱 Requirements

| | |
|---|---|
| ROS 2 | Humble Hawksbill |
| OS / Arch | Linux, `x86_64` or `aarch64` (Jetson Orin / AGX Thor) |
| Cameras | Intel RealSense D405 / D415 / D455 (librealsense2 ≥ 2.56) |
| GPU (optional) | NVIDIA CUDA / cuDNN / TensorRT — required for Depth Anything, YOLOE, FoundationPose |
| RMW | **CycloneDDS** (`rmw_cyclonedds_cpp`) — FastRTPS is not supported in processes that load librealsense2 |

---

## 🐳 External inference runtimes

Heavyweight backends live in their own containers (see [`docker/`](docker/)) and are bridged into ROS 2 by the corresponding pipeline:

| Container | Bridged by | Pipeline |
|---|---|---|
| `foundationpose_runtime` | `fv_foundationpose_bridge` | `foundationpose_bridge.yaml` |
| `lingbot_depth_worker` | `fv_lingbot_depth` | `lingbot_depth_http.yaml` |
| `openpi_runtime` | `fv_policy_runner` | `openpi_policy_runner.yaml` |
| `fv_lerobot_policy_runtime` | `fv_policy_runner` | `fv_lerobot_pi05_morikawa_sample.yaml` |

---

## 🔌 Topic & service conventions

Cameras get their own namespace:

```
/fv/d405/color/image_raw
/fv/d405/depth/image_rect_raw
/fv/d405/depth/points
/fv/d415/...
/fv/d455/...
```

AI outputs follow `/fv/<task>/...`:

```
/fv/<task>/detections     # fv_msgs/DetectionArray
/fv/<task>/objects_3d     # fv_msgs/Object3DArray
/fv/<task>/mask           # sensor_msgs/Image
/fv/<task>/overlay        # sensor_msgs/Image (for UI)
```

> ⚠️ `fv_realsense` publishes with **`BEST_EFFORT`** QoS. Subscribers must match — a `RELIABLE` subscription silently receives nothing.

Key `fv_realsense` services:

| Service | Purpose |
|---|---|
| `/fv_realsense/set_mode` | Display mode (0: none, 1: cursor, 2: cursor + coord + distance) |
| `/fv_realsense/get_distance` | Pixel coord → 3D distance |
| `/fv_realsense/get_camera_info` | Camera spec / configuration |
| `/fv_realsense/get_point_cloud` | One-shot 3D point cloud |

---

## 🛠️ Development

```bash
# What topics are alive?
ros2 topic list | grep ^/fv
ros2 topic hz   /fv/d405/color/image_raw

# Which RealSense devices are connected?
python3 scripts/check_camera_serials.py

# Inspect a node
ros2 node info /fv_realsense
```

Build rule, repeated because it matters:

> ⚠️ **Do not pass `--symlink-install`.** It has been observed to corrupt `install/` on aarch64 / Tegra builds. Stick to the default copy-based build (`colcon build --packages-select <pkg>`).

---

## 🗺️ Roadmap

- [ ] One-click pipeline templates inside the editor
- [ ] Live parameter editing pane (sliders / dropdowns from `param_descriptors`)
- [ ] Headless mode that compiles a pipeline to a `launch.py`
- [ ] More AprilTag pipelines (single tag, bundle, board)
- [ ] Public model zoo manifest

Have an idea? [Open an issue.](https://github.com/takatronix/fluent_vision_ros2/issues)

---

## 🤝 Contributing

PRs and issues are very welcome — bug reports, new AI nodes, pipeline examples, docs, anything.

1. Open an issue first for non-trivial changes so we can talk scope.
2. Follow the existing conventional-commit style (`feat(scope): ...`, `fix(scope): ...`, `docs: ...`, …).
3. Run `colcon build --packages-select <changed>` and any package-local tests before opening the PR.
4. **No silent fallbacks** — don't paper over missing TFs / timestamps / sensor data with stale or zero values. Fail closed and warn.

Coding conventions live in [`CLAUDECODE_RULES.md`](CLAUDECODE_RULES.md) and [`CPP_CODING_RULES.md`](CPP_CODING_RULES.md). Per-package design notes live alongside the package (e.g. [`src/audio/fv_audio/DESIGN.md`](src/audio/fv_audio/DESIGN.md)).

---

## 📜 License

[MIT License](LICENSE). A few packages are individually licensed under Apache-2.0 — see each `package.xml` for authoritative info.

---

## 👤 Author

**Takashi Otsuka** — [@takatronix](https://github.com/takatronix)

If FluentVision is useful to you, a ⭐ on GitHub is the best way to say thanks.
