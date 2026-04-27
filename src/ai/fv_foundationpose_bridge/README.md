# FV FoundationPose Bridge

`isaac_ros_foundationpose` の `vision_msgs/Detection3DArray` を FluentVision の `fv_msgs/Object3DArray` と TF に変換するノードです。

## 入力

- `input_topic`:
  - 既定 `"/pose_estimation/output"`
  - FoundationPose tracking を使う場合は `"/tracking/output"` に差し替え

## 出力

- `output_topic`: `fv_msgs/Object3DArray`
- TF:
  - `publish_tf=true` のとき `tf_child_frame_prefix` 配下に object pose を publish

## 主な用途

- FluentVision 既存の `Object3DArray` 前提 UI / downstream 処理につなぐ
- FoundationPose の pose を TF tree に乗せる

## 例

```bash
ros2 run fv_foundationpose_bridge fv_foundationpose_bridge_node \
  --ros-args \
  -p input_topic:=/pose_estimation/output \
  -p output_topic:=/fv/foundationpose/objects_3d \
  -p publish_tf:=true \
  -p tf_parent_frame:=fv/d405/color_optical_frame
```
