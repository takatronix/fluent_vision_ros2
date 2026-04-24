#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-lingbot-depth-worker:latest}"
MODEL_ID="${MODEL_ID:-robbyant/lingbot-depth-pretrain-vitl-14-v0.5}"
LOCAL_MODEL_PATH="${LOCAL_MODEL_PATH:-}"
DEVICE="${DEVICE:-auto}"
LISTEN_PORT="${LISTEN_PORT:-5540}"
USE_FP16="${USE_FP16:-1}"
APPLY_MASK="${APPLY_MASK:-1}"
RESOLUTION_LEVEL="${RESOLUTION_LEVEL:-9}"
FALLBACK_PASSTHROUGH="${FALLBACK_PASSTHROUGH:-1}"
HF_CACHE_DIR="${HF_CACHE_DIR:-/tmp/lingbot-hf-cache}"
ENABLE_GPU="${ENABLE_GPU:-1}"

mkdir -p "${HF_CACHE_DIR}"

GPU_ARGS=()
if [ "${ENABLE_GPU}" = "1" ]; then
  GPU_ARGS+=(--gpus all)
fi

docker run --rm \
  --net=host \
  "${GPU_ARGS[@]}" \
  -e MODEL_ID="${MODEL_ID}" \
  -e LOCAL_MODEL_PATH="${LOCAL_MODEL_PATH}" \
  -e DEVICE="${DEVICE}" \
  -e LISTEN_PORT="${LISTEN_PORT}" \
  -e USE_FP16="${USE_FP16}" \
  -e APPLY_MASK="${APPLY_MASK}" \
  -e RESOLUTION_LEVEL="${RESOLUTION_LEVEL}" \
  -e FALLBACK_PASSTHROUGH="${FALLBACK_PASSTHROUGH}" \
  -v "${HF_CACHE_DIR}:/root/.cache/huggingface" \
  "${IMAGE_NAME}"
