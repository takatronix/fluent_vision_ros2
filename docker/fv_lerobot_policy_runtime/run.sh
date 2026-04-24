#!/usr/bin/env bash
set -euo pipefail

docker rm -f fv-lerobot-policy-runtime fv-dpex-policy-runtime >/dev/null 2>&1 || true
docker run -d --rm \
  --name fv-lerobot-policy-runtime \
  --net=host \
  --gpus all \
  --env-file /home/aspa/daihen-physical-ai/data/.env \
  -e DPEX_PORT=8010 \
  -e DPEX_LOAD_ON_START=true \
  -e DPEX_DEFAULT_POLICY_ID=pi05_week19_morikawa_all_20k \
  -e 'DPEX_POLICY_SPECS=pi05_week19_morikawa_all_20k:pi05:/data/models/669ce87f-8cfa-5836-8f29-b9c3f6e650cf;pi05_week19_morikawa_all_10k:pi05:/data/models/1dc2af59-2d87-52f9-99af-fae7c6335bcf' \
  -e 'DPEX_CAMERA_ALIASES=cam_high=top_camera,top=top_camera,top_camera=top_camera,cam_left_wrist=arm_camera,cam_right_wrist=arm_camera,arm=arm_camera,arm_camera=arm_camera' \
  -e HF_HOME=/data/.cache/huggingface \
  -e TRANSFORMERS_CACHE=/data/.cache/huggingface \
  -e TORCH_HOME=/data/.cache/torch \
  -v /home/aspa/daihen-physical-ai/data:/data \
  -v /home/aspa/ros2_ws/src/fluent_vision_ros2/docker/fv_lerobot_policy_runtime/policy_models.yaml:/opt/fv-lerobot-policy-runtime/policy_models.yaml:ro \
  fv-lerobot-policy-runtime:latest
