# FoundationPose Runtime

`isaac_ros_foundationpose` を FluentVision から切り離して動かすための CUDA 前提 runtime です。

対象:

- `linux/amd64`: Ubuntu 24.04 + NVIDIA GPU
- `linux/arm64`: Jetson Thor

この runtime は `Isaac ROS FoundationPose` の ROS 2 パッケージと TensorRT 変換ツールを入れたベースコンテナです。  
FluentVision 本体とは分離し、必要な mesh / ONNX / TensorRT plan / launch を mount して使います。

## 方針

- FoundationPose は NVLabs 生実装ではなく `Isaac ROS` 版を使う
- `amd64` と `arm64` は同じ Dockerfile で分岐
- 現在の Thor 環境向け既定値は `Isaac ROS 4.1 + r38.2` にしてある
- 将来 `4.3 + r38.4` に上げる場合は build arg で切り替える

## ビルド

ローカル単一アーキテクチャ:

```bash
cd /home/aspa/ros2_ws/src/fluent_vision_ros2/docker/foundationpose_runtime
./build.sh
```

既定値:

- `ISAAC_ROS_RELEASE=4.1`
- `L4T_SUITE=r38.2`

JetPack 7.1 / Isaac ROS 4.3 に合わせる例:

```bash
ISAAC_ROS_RELEASE=4.3 L4T_SUITE=r38.4 ./build.sh
```

## multi-arch buildx

manifest 付きで registry に push する例:

```bash
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f /home/aspa/ros2_ws/src/fluent_vision_ros2/docker/foundationpose_runtime/Dockerfile \
  -t ghcr.io/your-org/foundationpose-runtime:4.1 \
  --build-arg ISAAC_ROS_RELEASE=4.1 \
  --build-arg L4T_SUITE=r38.2 \
  --push \
  /home/aspa/ros2_ws/src/fluent_vision_ros2
```

注意:

- multi-arch は通常 `--push` が必要
- `amd64` / `arm64` の両方で NVIDIA runtime が必要
- x86 側は `Ampere` 以上、8GB VRAM 以上が現実的

## 実行

対話シェルで入る:

```bash
cd /home/aspa/ros2_ws/src/fluent_vision_ros2/docker/foundationpose_runtime
./run.sh
```

workspace mount を明示する例:

```bash
ISAAC_ROS_WS_HOST=/mnt/nova_ssd/workspaces/isaac_ros-dev ./run.sh
```

## インストール確認

コンテナ内で:

```bash
ros2 pkg prefix isaac_ros_foundationpose
ros2 pkg prefix isaac_ros_examples
which trtexec
```

## TensorRT engine 生成

FoundationPose は ONNX から `.plan` を作っておく前提です。

コンテナ内で:

```bash
/opt/foundationpose-runtime/build_engines.sh
```

既定の入力:

- `${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/refine_model.onnx`
- `${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/score_model.onnx`

出力:

- `${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/refine_trt_engine.plan`
- `${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/score_trt_engine.plan`

## FoundationPose 起動 helper

コンテナ内で公式 quickstart 相当を起動する helper を入れています。

```bash
/opt/foundationpose-runtime/run_foundationpose_example.sh
```

RealSense fragment 付きで起動する例:

```bash
FP_LAUNCH_FRAGMENTS=realsense_mono_rect_depth,foundationpose \
FP_MESH_FILE_PATH=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/Mac_and_cheese_0_1/Mac_and_cheese_0_1.obj \
/opt/foundationpose-runtime/run_foundationpose_example.sh
```

## FluentVision との接続メモ

このコンテナは FoundationPose 本体の実行基盤です。  
FluentVision に実際に組み込むには次が別途必要です。

- `/fv/d405/color/image_raw`
- `/fv/d405/depth/image_rect_raw`
- `/fv/d405/color/camera_info`
- segmentation mask
- object mesh (`.obj`)
- 出力 `Detection3DArray` / TF を FluentVision 側形式に変換する bridge

## 参考

- Isaac ROS Getting Started:
  https://nvidia-isaac-ros.github.io/getting_started/
- Isaac ROS Pose Estimation:
  https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/index.html
- Isaac ROS FoundationPose:
  https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html
