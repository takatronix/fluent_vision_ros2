#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ARCH="$(uname -m)"

IMAGE_NAME="${IMAGE_NAME:-lingbot-depth-worker:latest}"
INSTALL_XFORMERS="${INSTALL_XFORMERS:-0}"
if [ -z "${BASE_IMAGE+x}" ]; then
  if [ "${ARCH}" = "aarch64" ]; then
    BASE_IMAGE="nvcr.io/nvidia/pytorch:26.03-py3"
  else
    BASE_IMAGE="python:3.10-slim-bookworm"
  fi
else
  BASE_IMAGE="${BASE_IMAGE}"
fi

if [ -z "${TORCH_INDEX_URL+x}" ]; then
  if [ "${ARCH}" = "x86_64" ]; then
    TORCH_INDEX_URL="https://download.pytorch.org/whl/cu128"
  else
    TORCH_INDEX_URL=""
  fi
else
  TORCH_INDEX_URL="${TORCH_INDEX_URL}"
fi

if [ -z "${TORCH_PACKAGES+x}" ]; then
  if [ "${ARCH}" = "aarch64" ]; then
    TORCH_PACKAGES=""
  else
    TORCH_PACKAGES="torch==2.6.0 torchvision"
  fi
else
  TORCH_PACKAGES="${TORCH_PACKAGES}"
fi

docker build \
  -f "${PROJECT_ROOT}/docker/lingbot_depth_worker/Dockerfile" \
  -t "${IMAGE_NAME}" \
  --build-arg BASE_IMAGE="${BASE_IMAGE}" \
  --build-arg TORCH_INDEX_URL="${TORCH_INDEX_URL}" \
  --build-arg TORCH_PACKAGES="${TORCH_PACKAGES}" \
  --build-arg INSTALL_XFORMERS="${INSTALL_XFORMERS}" \
  "${PROJECT_ROOT}"
