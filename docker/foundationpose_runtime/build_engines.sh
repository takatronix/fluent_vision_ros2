#!/usr/bin/env bash
set -euo pipefail

ISAAC_ROS_WS="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"
MODEL_DIR="${MODEL_DIR:-${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose}"
REFINE_ONNX="${REFINE_ONNX:-${MODEL_DIR}/refine_model.onnx}"
SCORE_ONNX="${SCORE_ONNX:-${MODEL_DIR}/score_model.onnx}"
REFINE_PLAN="${REFINE_PLAN:-${MODEL_DIR}/refine_trt_engine.plan}"
SCORE_PLAN="${SCORE_PLAN:-${MODEL_DIR}/score_trt_engine.plan}"
TRTEXEC_BIN="${TRTEXEC_BIN:-/usr/src/tensorrt/bin/trtexec}"

if [ ! -x "${TRTEXEC_BIN}" ]; then
  echo "trtexec not found: ${TRTEXEC_BIN}" >&2
  exit 1
fi

if [ ! -f "${REFINE_ONNX}" ]; then
  echo "Missing refine ONNX: ${REFINE_ONNX}" >&2
  exit 1
fi

if [ ! -f "${SCORE_ONNX}" ]; then
  echo "Missing score ONNX: ${SCORE_ONNX}" >&2
  exit 1
fi

mkdir -p "${MODEL_DIR}"

"${TRTEXEC_BIN}" \
  --onnx="${REFINE_ONNX}" \
  --saveEngine="${REFINE_PLAN}" \
  --minShapes=input1:1x160x160x6,input2:1x160x160x6 \
  --optShapes=input1:1x160x160x6,input2:1x160x160x6 \
  --maxShapes=input1:42x160x160x6,input2:42x160x160x6

"${TRTEXEC_BIN}" \
  --onnx="${SCORE_ONNX}" \
  --saveEngine="${SCORE_PLAN}" \
  --minShapes=input1:1x160x160x6,input2:1x160x160x6 \
  --optShapes=input1:1x160x160x6,input2:1x160x160x6 \
  --maxShapes=input1:252x160x160x6,input2:252x160x160x6

echo "Generated:"
echo "  ${REFINE_PLAN}"
echo "  ${SCORE_PLAN}"
