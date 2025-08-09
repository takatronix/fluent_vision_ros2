#!/bin/bash

# start_fv.sh — Fluent Vision starter
# Usage:
#   ./start_fv.sh            # real time (default)
#   ./start_fv.sh sim        # simulation time (/clock 必須)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-real}"
USE_SIM=false
if [ "${MODE}" = "sim" ]; then
  USE_SIM=true
fi
SIM_ARG=(--ros-args -p use_sim_time:=${USE_SIM})

# Stop residual processes (no source, fast)
if [ -x "${SCRIPT_DIR}/stop_fv.sh" ]; then
  "${SCRIPT_DIR}/stop_fv.sh" || true
else
  pkill -9 -f "fv_" || true
  pkill -9 -f "depth_image_proc" || true
  pkill -9 -f "foxglove_bridge" || true
fi
sleep 1

echo "[start_fv] Mode=${MODE} use_sim_time=${USE_SIM}"

# 1) RealSense D415
if command -v ros2 >/dev/null 2>&1; then
  echo "[start_fv] Starting fv_realsense_d415..."
  if [ -f "${SCRIPT_DIR}/fv_realsense_d415.yaml" ]; then
    ros2 run fv_realsense fv_realsense_node "${SIM_ARG[@]}" \
      --ros-args --params-file "${SCRIPT_DIR}/fv_realsense_d415.yaml" -r __node:=fv_realsense_d415 &
  else
    echo "[start_fv] (warn) ${SCRIPT_DIR}/fv_realsense_d415.yaml not found. Starting with defaults."
    ros2 run fv_realsense fv_realsense_node "${SIM_ARG[@]}" \
      --ros-args -r __node:=fv_realsense_d415 &
  fi
else
  echo "[start_fv] ros2 command not found."; exit 1
fi

# Wait D415 color topic
for i in {1..15}; do
  if ros2 topic list | grep -q "/fv/d415/color/image_raw"; then
    echo "[start_fv] D415 ready"; break
  fi
  sleep 1
done

# 2) RealSense D405
echo "[start_fv] Starting fv_realsense_d405..."
if [ -f "${SCRIPT_DIR}/fv_realsense_d405.yaml" ]; then
  ros2 run fv_realsense fv_realsense_node "${SIM_ARG[@]}" \
    --ros-args --params-file "${SCRIPT_DIR}/fv_realsense_d405.yaml" -r __node:=fv_realsense_d405 &
else
  echo "[start_fv] (warn) ${SCRIPT_DIR}/fv_realsense_d405.yaml not found. Starting with defaults."
  ros2 run fv_realsense fv_realsense_node "${SIM_ARG[@]}" \
    --ros-args -r __node:=fv_realsense_d405 &
fi

# depth_image_proc は不要（fv_realsenseが /fv/*/registered_points を直接Publish）

# 5) Object detector (D415/D405) — optional, skip if launch file missing
if ros2 pkg prefix fv_object_detector >/dev/null 2>&1; then
  if [ -f "${SCRIPT_DIR}/fv_object_detector_d415.yaml" ]; then
    ros2 launch fv_object_detector fv_object_detector_launch.py ${SIM_ARG[@]} \
      node_name:=fv_object_detector_d415 \
      config_file:="${SCRIPT_DIR}/fv_object_detector_d415.yaml" \
      input_topic:="/fv/d415/color/image_raw" \
      output_image_topic:="/fv/d415/object_detection/annotated_image" \
      output_detections_topic:="/fv/d415/object_detection/detections" &
  fi
  if [ -f "${SCRIPT_DIR}/fv_object_detector_d405.yaml" ]; then
    ros2 launch fv_object_detector fv_object_detector_launch.py ${SIM_ARG[@]} \
      node_name:=fv_object_detector_d405 \
      config_file:="${SCRIPT_DIR}/fv_object_detector_d405.yaml" \
      input_topic:="/fv/d405/color/image_raw" \
      output_image_topic:="/fv/d405/object_detection/annotated_image" \
      output_detections_topic:="/fv/d405/object_detection/detections" &
  fi
fi

# 6) Analyzer (D415/D405) — requires YAML; skip if missing
if [ -f "${SCRIPT_DIR}/fv_aspara_analyzer_d415.yaml" ]; then
  ros2 run fv_aspara_analyzer fv_aspara_analyzer_node ${SIM_ARG[@]} \
    --ros-args --params-file "${SCRIPT_DIR}/fv_aspara_analyzer_d415.yaml" -r __node:=fv_aspara_analyzer_d415 &
else
  echo "[start_fv] (warn) analyzer config missing: ${SCRIPT_DIR}/fv_aspara_analyzer_d415.yaml — skipped"
fi
if [ -f "${SCRIPT_DIR}/fv_aspara_analyzer_d405.yaml" ]; then
  ros2 run fv_aspara_analyzer fv_aspara_analyzer_node ${SIM_ARG[@]} \
    --ros-args --params-file "${SCRIPT_DIR}/fv_aspara_analyzer_d405.yaml" -r __node:=fv_aspara_analyzer_d405 &
else
  echo "[start_fv] (warn) analyzer config missing: ${SCRIPT_DIR}/fv_aspara_analyzer_d405.yaml — skipped"
fi

# 7) Foxglove Bridge
if ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  echo "[start_fv] Starting foxglove_bridge..."
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml ${SIM_ARG[@]} &
fi

echo "[start_fv] All requested nodes started (mode=${MODE})." 

