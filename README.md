<div align="center">

# 🌿 FluentVision ROS 2

### A declarative, YAML-driven **vision & perception stack** for ROS 2.
### Wire RGB-D cameras, AI models, and streaming endpoints together — visually, in a browser.

**English** · [日本語](README.ja.md)

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](#-license)
[![Platform](https://img.shields.io/badge/platform-linux%20%7C%20aarch64%20%7C%20x86__64-lightgrey)](#-requirements)
[![Language](https://img.shields.io/badge/C%2B%2B%20%2F%20Python-blue)](#)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](#-contributing)

<sub>RealSense · YOLOE · YOLO-seg · Depth Anything V3 · AprilTag · 3D OBB · FoundationPose bridge · GPU HUD rendering · Episode recording</sub>

<br/>

<img src="core/fluent_stage/docs/images/hud_basic.png" width="780" alt="A real render from fluent_stage: detection boxes, HUD panel, path and gauge composited on the GPU" />

<sub>*Real output — a `fluent_stage` HUD composited from 13 layers, rendered by the engine in this repo.*</sub>

</div>

---

## 🗺️ Overview

![Architecture: pipelines/*.yaml wire sensors → AI → streaming, with the GPU rendering core and external Docker runtimes below](docs/assets/architecture.svg)

FluentVision is two things that share one repository:

1. **A ROS 2 perception stack** (`src/`) — 40+ packages covering cameras, AI inference, 3D geometry, streaming, audio, and recording, wired together by YAML pipeline files and a browser-based visual editor.
2. **A GPU rendering core** (`core/`) — `fluent_stage` and its predecessor `fluent_scene`: standalone (ROS-free) Vulkan compositors that turn a typed YAML scene into a robot HUD, with type checking *before* execution so an AI can safely edit a live screen.

---

## ✨ Why FluentVision?

Building a real-world robot perception stack usually means: a tangled launch file, hard-coded topic names, model loading code copy-pasted between projects, and zero way to inspect or rewire things at runtime.

**FluentVision flips that around.**

- 🧩 **Pipelines are data, not code** — declare your camera → AI → streaming graph in a YAML file.
- 🖱️ **Edit it in a browser** — drag, drop, wire, preview live frames, hit **Run**. The editor spawns the graph as real ROS 2 nodes.
- 🧠 **Batteries included** — 17 AI / 3D / depth packages you can mix and match.
- ⚡ **Real-time first** — `BEST_EFFORT` QoS by default, GPU inference (TensorRT / ONNX Runtime / OpenVINO), runs on Jetson Orin / AGX Thor.
- 🎥 **Stream anywhere** — HTTP image server for browsers, WebSocket for dashboards, RTMP for OBS, binary WS for point clouds.
- 🎨 **Draw the robot's screen declaratively** — the `fluent_stage` core renders HUDs at 1080p in ~5.6 ms/frame on Vulkan (~17× faster than CPU compositing).

---

## 🚀 60-second demo

```bash
# 1. Clone into a ROS 2 workspace
cd ~/ros2_ws/src && git clone https://github.com/takatronix/fluent_vision_ros2.git

# 2. Build (do NOT pass --symlink-install)
cd ~/ros2_ws && colcon build --packages-up-to fv_pipeline_editor fv_realsense fv_object_detector
source install/setup.bash

# 3. CycloneDDS is required (FastRTPS conflicts with librealsense2)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 4. Launch the visual editor
ros2 run fv_pipeline_editor editor_node
# → open  http://localhost:8095
```

Now plug in an Intel RealSense D405 / D415 / D455, load the built-in **Object Detection** pipeline (or drag a **camera** node and a **detector** node onto the canvas and wire them), hit **Run**. Live detections in your browser, in under a minute.

> The editor stores your pipelines in `~/.fluent_vision/pipelines/`; the ones shipped in [`pipelines/`](pipelines/) appear as built-in templates.

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
│  🎥 Streams  │   ┌─────────────┐                                   │
│  📼 Recorder │   │ 3D Detector │──▶  /fv/objects_3d                │
│  ...         │   └─────────────┘                                   │
├──────────────┴─────────────────────────────────────────────────────┤
│  ▶ Preview (live JPEG over binary WebSocket)                       │
└────────────────────────────────────────────────────────────────────┘
```

- **Load**: read any pipeline YAML into the canvas — your own or the built-in templates.
- **Save**: serialize the graph back to YAML — version-control it like any artefact.
- **Preview**: every `sensor_msgs/Image` topic is one click away from a live JPEG preview pane.
- **Run**: the editor spawns every node in the graph via `ros2 run`, passing its parameters — no launch files involved.

---

## 📐 Anatomy of a pipeline

Pipelines are plain YAML. This is (abridged) the real [`pipelines/object_detection.yaml`](pipelines/object_detection.yaml):

```yaml
name: "Object Detection"
description: "RealSense → YOLOv10 Object Detector"

system:
  camera_start_delay: 2.0

nodes:
  - id: realsense_1
    package: fv_realsense
    exec: fv_realsense_node
    node_name: realsense_1
    namespace: /fv
    parameters:
      camera_selection:
        selection_method: "auto"
      camera:
        color_width: 640
        color_height: 480
        color_fps: 15

  - id: object_detector_1
    package: fv_object_detector
    exec: fv_object_detector_node
    node_name: object_detector_1
    namespace: /fv
    parameters:
      input_image_topic: /fv/realsense_1/color/image_raw
      model:
        device: "auto"
        confidence_threshold: 0.5
      enable_tracking: true

layout:                       # canvas positions — kept by the editor
  realsense_1: { x: 100, y: 200 }
  object_detector_1: { x: 420, y: 200 }
```

The pipeline editor reads, writes, visualises, and **runs** exactly this format — no hidden state.

### Launching without the editor

`fluent_vision_system` is a YAML-driven launcher with its own (different) config schema — nodes grouped under `groups:`, with per-group enable flags. Sample configs live in [`src/system/fluent_vision_system/config/`](src/system/fluent_vision_system/config/):

```bash
ros2 launch fluent_vision_system run.py \
    config:=/abs/path/to/fluent_vision_system.sample.yaml
```

For the fixed dual-camera stack used on the original robot there is also the hardcoded [`launch/start_fv.sh`](launch/start_fv.sh) (`./start_fv.sh real|sim`).

> ⚠️ The editor's `pipelines/*.yaml` (flat `nodes:` list) and the launcher's `groups:` configs are **two different schemas** — a pipeline file passed to `run.py` will launch zero nodes.

---

## 🎨 The GPU rendering core (`core/`)

Standalone CMake projects — **no ROS, GPU, or ML dependency in the core**, `COLCON_IGNORE`d so colcon workflows are unaffected. They render the robot's screen: camera imagery, detection boxes, Japanese text, paths, gauges.

### fluent_stage — current generation

<table>
<tr>
<td width="50%"><img src="core/fluent_stage/docs/images/filters_tour.png" alt="30 image filters rendered by fluent_stage" width="100%"/><br/><sub>30 filters, single-sourced between GLSL and C++</sub></td>
<td width="50%"><img src="core/fluent_stage/docs/images/ui_catalog.png" alt="UI controls: slider, segmented, dropdown, gauge" width="100%"/><br/><sub>6 UI controls — state = attribute overrides, input = pointer injection</sub></td>
</tr>
</table>

A CALayer-style **retained layer tree with SDF rendering**. One chainable C++ line or one declarative YAML block produce the identical picture:

```cpp
Stage stage(1920, 1080);
stage.image(camera);
stage.boxes(detections).color(Color::Teal).smoothing(0.2f);
```

- **Scene documents (`.fvs`, schema `fluent.scene/v1alpha2`)** — type checking, reference resolution, and digest all run *before* execution, so an AI can safely generate or edit a live robot screen.
- **Two backends, bit-identical**: CPU reference renderer and Vulkan (SPIR-V baked at build time). 1080p HUD: **5.6 ms/frame Vulkan vs 94.7 ms CPU (~17×)**, including readback.
- **13 content types** (image, text with Japanese shaping, boxes with temporal smoothing, arc, grid, …), **30 filters**, **implicit animation** (`Transaction`), deterministic time (`render(stage, dt)`).
- **`fvsc` CLI**: `validate` / `preview` / `fmt` / `digest` / `describe --json`.
- **`stage_web`** (port 8790): the Stage streamed to your browser as MJPEG, mouse/touch fed back as pointer injection — zero UI logic in the page.
- **`scene_web`** (port 8791): live `.fvs` editing — every save is validated, compiled, linted, and swapped **atomically at a frame boundary**; broken edits keep the old frame and show a red banner.
- **ROS 2 binding** (`scene_node` + `fluent.binding/v1alpha1` documents): wires topics into `$inputs`, with `/inspect` and `/at` introspection.

📖 Docs (Japanese is the working language; an English README is planned for the OSS release): [README](core/fluent_stage/README.md) · [Getting started](core/fluent_stage/docs/getting-started.ja.md) · [Cookbook](core/fluent_stage/docs/cookbook.ja.md) · [Changelog](core/fluent_stage/CHANGELOG.md) · [Design spec](docs/design/fluent_stage.ja.md)

### fluent_scene — first generation

The node-graph-based predecessor (schema `fluent.scene/v1alpha1`): typed YAML → canonical IR → GPU budget plan → retained Vulkan composite. Measured ~26× faster than CPU compositing at 1080p on Jetson Thor. Ships its own `fvsc`, `fv_render`, benchmarks, and a ROS 2 adapter node.

📖 [日本語 README](core/fluent_scene/README.md) · [English README](core/fluent_scene/README.en.md) · [Architecture spec (en)](docs/design/fluent_vision_architecture.md) · [ja](docs/design/fluent_vision_architecture.ja.md)

> ⚠️ **Both projects use the `.fvs` extension and ship a CLI named `fvsc`, but the schemas are incompatible** (`v1alpha1` node graph vs `v1alpha2` layer tree). New work targets **fluent_stage**.

---

## 🧰 What's in the box

<details open>
<summary><b>🧠 AI & inference (<code>src/ai/</code>)</b></summary>

| Package | Role |
|---|---|
| `fv_yoloe` | YOLOE open-vocabulary detection & segmentation (text prompt) |
| `fv_object_detector` | YOLO-family generic object detection |
| `fv_instance_seg` | YOLO-seg instance segmentation (TensorRT / OpenVINO) |
| `fv_object_mask_generator` | UNet semantic segmentation (OpenVINO) |
| `fv_3d_detector` | 3D OBB / shape / colour / distance from masks + point clouds |
| `fv_color_detector` | HSV colour detection with depth distance (C++) |
| `fv_apriltag` | AprilTag detection + cube-bundle pose (pupil-apriltags) |
| `fv_depth_anything` | Depth Anything V3 monocular depth (ONNX / TensorRT) |
| `fv_depth_features` | HHA encoder, surface normals, RGBD fusion (C++) |
| `fv_lingbot_depth` | LingBot-Depth refinement & point cloud generation |
| `fv_pointcloud_pipeline` | ROI extraction & filtering |
| `fv_detection_fusion` | Multi-source aggregation into `fv_msgs/DetectionArray` |
| `fv_foundationpose_bridge` | Isaac ROS FoundationPose → `Object3DArray` + TF |
| `fv_aspara_points` / `fv_stalk_estimator` | Asparagus geometry — map-frame point accumulation, stalk length estimation |
| `fv_face_recognizer` | Real-time face recognition |
| `fv_policy_runner` | Watch trigger topics and invoke external policy runtimes (OpenPI / LeRobot) |
| `fv_cube_detector` | *Deprecated* — superseded by `fv_instance_seg` |

> `fv_aspara_analyzer` was retired (directory holds only configs, `COLCON_IGNORE`d).

`fv_apriltag` resolves each ID's physical black-edge size from
[`src/ai/fv_apriltag/config/tag_registry.yaml`](src/ai/fv_apriltag/config/tag_registry.yaml) (nominal `tag_mm` × 0.8). A
positive `tag_size` parameter is an explicit homogeneous-size override;
unknown or reserved IDs without a size do not produce a pose or TF.
Printable tag sheets, cube kits, and 3D-printable mounts live in [`docs/印刷物/`](docs/印刷物/).

</details>

<details>
<summary><b>📷 Sensors (<code>src/sensors/</code>)</b></summary>

- **`fv_realsense`** — Intel RealSense driver (D405 / D415 / D455). `BEST_EFFORT` QoS, display-mode switching, pixel→3D distance service, per-frame device-time re-anchoring.
- **`fv_camera`** — generic USB / RPi / network camera driver.
- **`fv_insta360_x3`** — Insta360 X3 in UVC webcam mode.

</details>

<details>
<summary><b>📡 Streaming & recording (<code>src/streaming/</code>)</b></summary>

- `fv_episode_recorder` — rosbag + camera MP4 + `EpisodeMarker` + sidecar JSON, with preflight, lock, and retention. LeRobot / Pi0.5-friendly layout.
- `fv_image_distributor` — HTTP image server for browsers (default port `8080`, `/image.jpg`).
- `fv_websocket_server` — image streaming over WebSocket (default port `8765`).
- `fv_pointcloud_ws_server` — binary WebSocket point-cloud streaming.
- `fv_rtmp_server` — RTMP ↔ ROS image bridge (e.g. DJI Fly).
- `fv_mjpeg_server` — MJPEG **ingest** client: pulls an MJPEG URL (e.g. a network camera) and publishes it as `sensor_msgs/Image`.
- `fv_image_preview` — lightweight preview bridge with focus telemetry & source stats.
- `fv_recorder` — simple record / playback.

</details>

<details>
<summary><b>⚙️ System (<code>src/system/</code>)</b></summary>

- **`fv_pipeline_editor`** — visual pipeline editor (aiohttp + canvas, port `8095`).
- **`fluent_vision_system`** — YAML-driven launcher (`run.py`, `groups:` schema).
- **`fv_machine_calibration`** — hand-eye calibration data collection + offline solver.

</details>

<details>
<summary><b>🔊 Audio (<code>src/audio/</code>)</b></summary>

- `fv_audio` (capture), `fv_audio_output` (ALSA), `fv_audio_vad` (VAD / wakeword), `fv_tts` (Open JTalk / pyopenjtalk), `fv_soundboard` (event → sound / TTS cues).

</details>

<details>
<summary><b>📨 Messages, libraries, UI, utils</b></summary>

- `fv_msgs`, `fv_episode_msgs` — message definitions.
- `fluent_lib` — chainable API around OpenCV / PCL / ROS 2.
- `fv_image_filter` — chainable image filters with live parameter updates.
- `fv_aspara_ui_cpp` — overlay UI node (C++); `fv_episode_ui` — Svelte SPA for browsing recorded episodes.
- `fv_topic_relay` — topic relay utility.
- `src/external/` — vendored `vision_msgs` + RViz plugins.

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
| Rendering core | Vulkan (optional — CPU reference backend runs anywhere), FreeType + HarfBuzz |

---

## 🐳 External inference runtimes

Heavyweight backends live in their own containers (see [`docker/`](docker/)) and are bridged into ROS 2:

| Container | Bridged by | Entry point |
|---|---|---|
| `foundationpose_runtime` | `fv_foundationpose_bridge` | [`pipelines/foundationpose_bridge.yaml`](pipelines/foundationpose_bridge.yaml) |
| `lingbot_depth_worker` | `fv_lingbot_depth` | [`pipelines/lingbot_depth_http.yaml`](pipelines/lingbot_depth_http.yaml) |
| `openpi_runtime` | `fv_policy_runner` | [`pipelines/openpi_policy_runner.yaml`](pipelines/openpi_policy_runner.yaml) |
| `fv_lerobot_policy_runtime` | `fv_policy_runner` | [`src/system/fluent_vision_system/config/fv_policy_runner_pi05_morikawa.yaml`](src/system/fluent_vision_system/config/fv_policy_runner_pi05_morikawa.yaml) (params file) |

---

## 🔌 Topic & service conventions

Topics are scoped by **node name** under the `/fv` namespace (the default since the multi-camera refactor):

```
/fv/realsense_1/color/image_raw
/fv/realsense_1/depth/image_rect_raw
/fv/realsense_1/depth/color/points
/fv/realsense_2/...
```

> Legacy per-model names (`/fv/d405/...`, `/fv/d415/...`) still appear in [`scripts/fv_realsense_d405.yaml`](scripts/)-style override files used by `launch/start_fv.sh` — they are explicit absolute-name overrides, not the default.

AI outputs follow `/fv/<task>/...`:

```
/fv/<task>/detections     # fv_msgs/DetectionArray
/fv/<task>/objects_3d     # fv_msgs/Object3DArray
/fv/<task>/mask           # sensor_msgs/Image
/fv/<task>/overlay        # sensor_msgs/Image (for UI)
```

> ⚠️ `fv_realsense` publishes with **`BEST_EFFORT`** QoS by default. Subscribers must match — a `RELIABLE` subscription silently receives nothing.

`fv_realsense` services are **node-private** (`~/…` → `/fv/<node_name>/…`) and individually switchable via the `services.*` parameters:

| Service | Purpose | Default |
|---|---|---|
| `~/get_distance` | Pixel coord → 3D distance | enabled in code, off in `default_config.yaml` |
| `~/get_camera_info` | Camera spec / configuration | same as above |
| `~/set_mode` | Display mode (0: none, 1: cursor, 2: cursor + coord + distance) | **disabled** |

> The former `get_point_cloud` service was removed — use the point-cloud topics.

---

## 🛠️ Development

```bash
# What topics are alive?
ros2 topic list | grep ^/fv
ros2 topic hz   /fv/realsense_1/color/image_raw

# Which RealSense devices are connected?
python3 scripts/check_camera_serials.py

# Record an episode (rosbag + MP4 + markers)
ros2 run fv_episode_recorder recorder_node --ros-args -p output_dir:=/data/episodes

# Inspect a node
ros2 node info /fv/realsense_1
```

Build rule, repeated because it matters:

> ⚠️ **Do not pass `--symlink-install`.** It has been observed to corrupt `install/` on aarch64 / Tegra builds. Stick to the default copy-based build (`colcon build --packages-select <pkg>`).

More docs:

- [`docs/design/`](docs/design/) — architecture & design specs (en / ja).
- [`docs/印刷物/`](docs/印刷物/) — printable AprilTag sheets, cube kits, 3D-printable mounts.
- [`rtabmap/`](rtabmap/) — RTAB-Map SLAM configs & scripts (ja).
- [`CPP_CODING_RULES.md`](CPP_CODING_RULES.md) — C++ conventions (ja); [`CLAUDECODE_RULES.md`](CLAUDECODE_RULES.md) — AI-agent operating rules (ja).

---

## 🗺️ Roadmap

- [ ] One-click pipeline templates inside the editor
- [ ] Live parameter editing pane (sliders / dropdowns from `param_descriptors`)
- [ ] Headless mode that compiles a pipeline to a `launch.py`
- [ ] `fluent_stage` English documentation (the L3 ROS 2 binding + inspector have landed)
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
