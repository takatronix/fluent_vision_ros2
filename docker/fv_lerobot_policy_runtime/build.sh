#!/usr/bin/env bash
set -euo pipefail

docker build \
  -f /home/aspa/ros2_ws/src/fluent_vision_ros2/docker/fv_lerobot_policy_runtime/Dockerfile \
  -t fv-lerobot-policy-runtime:latest \
  /home/aspa
