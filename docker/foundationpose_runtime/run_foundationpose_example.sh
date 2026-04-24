#!/usr/bin/env bash
set -euo pipefail

ISAAC_ROS_WS="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"
FP_LAUNCH_FRAGMENTS="${FP_LAUNCH_FRAGMENTS:-foundationpose}"
FP_INTERFACE_SPECS_FILE="${FP_INTERFACE_SPECS_FILE:-${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/quickstart_interface_specs.json}"
FP_MESH_FILE_PATH="${FP_MESH_FILE_PATH:-${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_foundationpose/Mustard/textured_simple.obj}"
FP_SCORE_ENGINE_FILE_PATH="${FP_SCORE_ENGINE_FILE_PATH:-${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/score_trt_engine.plan}"
FP_REFINE_ENGINE_FILE_PATH="${FP_REFINE_ENGINE_FILE_PATH:-${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/refine_trt_engine.plan}"
FP_RT_DETR_ENGINE_FILE_PATH="${FP_RT_DETR_ENGINE_FILE_PATH:-${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.plan}"
FP_TEXTURE_PATH="${FP_TEXTURE_PATH:-}"

args=(
  "launch_fragments:=${FP_LAUNCH_FRAGMENTS}"
  "interface_specs_file:=${FP_INTERFACE_SPECS_FILE}"
  "mesh_file_path:=${FP_MESH_FILE_PATH}"
  "score_engine_file_path:=${FP_SCORE_ENGINE_FILE_PATH}"
  "refine_engine_file_path:=${FP_REFINE_ENGINE_FILE_PATH}"
  "rt_detr_engine_file_path:=${FP_RT_DETR_ENGINE_FILE_PATH}"
)

if [ -n "${FP_TEXTURE_PATH}" ]; then
  args+=("texture_path:=${FP_TEXTURE_PATH}")
fi

exec ros2 launch isaac_ros_examples isaac_ros_examples.launch.py "${args[@]}"
