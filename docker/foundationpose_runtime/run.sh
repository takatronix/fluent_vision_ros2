#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-foundationpose-runtime:latest}"
CONTAINER_NAME="${CONTAINER_NAME:-foundationpose-runtime}"
ENABLE_GPU="${ENABLE_GPU:-1}"
ISAAC_ROS_WS_HOST="${ISAAC_ROS_WS_HOST:-}"
ISAAC_ROS_WS_CONTAINER="${ISAAC_ROS_WS_CONTAINER:-/workspaces/isaac_ros-dev}"

if [ -z "${ISAAC_ROS_WS_HOST}" ]; then
  if [ -d "/mnt/nova_ssd/workspaces/isaac_ros-dev" ]; then
    ISAAC_ROS_WS_HOST="/mnt/nova_ssd/workspaces/isaac_ros-dev"
  else
    ISAAC_ROS_WS_HOST="${HOME}/workspaces/isaac_ros-dev"
  fi
fi

mkdir -p "${ISAAC_ROS_WS_HOST}"

docker_args=(--rm -it --name "${CONTAINER_NAME}" --net=host)

if [ "${ENABLE_GPU}" = "1" ]; then
  docker_args+=(--gpus all)
fi

cmd=(bash)
if [ "$#" -gt 0 ]; then
  cmd=("$@")
fi

exec docker run "${docker_args[@]}" \
  -e ISAAC_ROS_WS="${ISAAC_ROS_WS_CONTAINER}" \
  -v "${ISAAC_ROS_WS_HOST}:${ISAAC_ROS_WS_CONTAINER}" \
  "${IMAGE_NAME}" \
  "${cmd[@]}"
