#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-openpi-runtime:latest}"
OPENPI_PORT="${OPENPI_PORT:-8000}"
OPENPI_ENV="${OPENPI_ENV:-aloha_sim}"
OPENPI_POLICY_MODE="${OPENPI_POLICY_MODE:-default}"
OPENPI_POLICY_CONFIG="${OPENPI_POLICY_CONFIG:-pi0_aloha_sim}"
OPENPI_POLICY_DIR="${OPENPI_POLICY_DIR:-gs://openpi-assets/checkpoints/pi0_aloha_sim}"
ENABLE_GPU="${ENABLE_GPU:-1}"

docker_args=(--rm --net=host)

if [ "${ENABLE_GPU}" = "1" ]; then
  docker_args+=(--gpus all)
fi

exec docker run "${docker_args[@]}" \
  -e OPENPI_PORT="${OPENPI_PORT}" \
  -e OPENPI_ENV="${OPENPI_ENV}" \
  -e OPENPI_POLICY_MODE="${OPENPI_POLICY_MODE}" \
  -e OPENPI_POLICY_CONFIG="${OPENPI_POLICY_CONFIG}" \
  -e OPENPI_POLICY_DIR="${OPENPI_POLICY_DIR}" \
  "${IMAGE_NAME}"
