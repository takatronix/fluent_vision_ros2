#!/bin/bash
# Entry point for the browser-webcam demo container.
# Launches the three demo nodes and serves everything on $PORT (default 7860).
set -e
source /opt/ros/humble/setup.bash
source /app/install/setup.bash

PORT="${PORT:-7860}"
pids=()
trap 'kill "${pids[@]}" 2>/dev/null' EXIT

ros2 run fv_object_detector fv_object_detector_node \
    --ros-args -r __ns:=/fv -r __node:=object_detector_coco \
    --params-file /app/demo/config/detector_coco.yaml &
pids+=($!)

ros2 run fv_yoloe fv_yoloe_node \
    --ros-args -r __ns:=/fv -r __node:=yoloe_1 \
    -p input_image_topic:=/fv/browser_cam/yoloe/image_raw \
    -p overlay_topic:=/fv/browser_demo/yoloe/overlay \
    -p overlay_compressed_topic:=/fv/browser_demo/yoloe/overlay/compressed \
    -p detections_topic:=/fv/browser_demo/yoloe/detections \
    -p model_name:=yoloe-11s-seg.pt \
    -p device:=cpu \
    -p text_prompt:="person, cup, phone" \
    -p processing_frequency:=3.0 &
pids+=($!)

# Foreground: the single public port (page + WS up/down).
exec ros2 run fv_browser_camera browser_camera_node \
    --ros-args -r __ns:=/fv -r __node:=browser_cam \
    -p port:="$PORT" \
    -p yoloe_prompt_topic:=/fv/yoloe_1/set_prompt
