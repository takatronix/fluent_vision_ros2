#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ARCH="$(uname -m)"

IMAGE_NAME="${IMAGE_NAME:-openpi-runtime:latest}"
OPENPI_INSTALL_PROFILE="${OPENPI_INSTALL_PROFILE:-default}"
JETSON_JAX_VERSION="${JETSON_JAX_VERSION:-0.10.0}"
JETSON_JAX_INDEX_URL="${JETSON_JAX_INDEX_URL:-https://pypi.jetson-ai-lab.io/sbsa/cu130}"

if [ -z "${BASE_IMAGE+x}" ]; then
  if [ "${ARCH}" = "aarch64" ]; then
    BASE_IMAGE="nvcr.io/nvidia/pytorch:26.03-py3"
    OPENPI_INSTALL_PROFILE="jetson-thor"
  else
    BASE_IMAGE="python:3.11-slim-bookworm"
  fi
fi

if [ -z "${JAX_SPEC+x}" ]; then
  if [ "${ARCH}" = "aarch64" ]; then
    JAX_SPEC="jax==0.5.3"
  else
    JAX_SPEC="jax[cuda12]==0.5.3"
  fi
fi

if [ -z "${TORCH_SPEC+x}" ]; then
  if [ "${ARCH}" = "aarch64" ]; then
    TORCH_SPEC="__REMOVE_TORCH__"
  else
    TORCH_SPEC="torch==2.7.1"
  fi
fi

docker build \
  -f "${PROJECT_ROOT}/docker/openpi_runtime/Dockerfile" \
  -t "${IMAGE_NAME}" \
  --build-arg BASE_IMAGE="${BASE_IMAGE}" \
  --build-arg OPENPI_INSTALL_PROFILE="${OPENPI_INSTALL_PROFILE}" \
  --build-arg JAX_SPEC="${JAX_SPEC}" \
  --build-arg TORCH_SPEC="${TORCH_SPEC}" \
  --build-arg JETSON_JAX_VERSION="${JETSON_JAX_VERSION}" \
  --build-arg JETSON_JAX_INDEX_URL="${JETSON_JAX_INDEX_URL}" \
  "${PROJECT_ROOT}"
