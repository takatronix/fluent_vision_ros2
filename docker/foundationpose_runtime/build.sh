#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ARCH="$(uname -m)"

IMAGE_NAME="${IMAGE_NAME:-foundationpose-runtime:latest}"
BASE_IMAGE="${BASE_IMAGE:-ubuntu:24.04}"
ISAAC_ROS_RELEASE="${ISAAC_ROS_RELEASE:-4.1}"
INSTALL_EXAMPLES="${INSTALL_EXAMPLES:-1}"

if [ -z "${L4T_SUITE+x}" ]; then
  case "${ISAAC_ROS_RELEASE}" in
    4.1) L4T_SUITE="r38.2" ;;
    4.2|4.3) L4T_SUITE="r38.4" ;;
    *)
      echo "Set L4T_SUITE explicitly for ISAAC_ROS_RELEASE=${ISAAC_ROS_RELEASE}" >&2
      exit 1
      ;;
  esac
fi

if [ "${ARCH}" != "x86_64" ] && [ "${ARCH}" != "aarch64" ]; then
  echo "Unsupported host arch: ${ARCH}" >&2
  exit 1
fi

docker build \
  -f "${PROJECT_ROOT}/docker/foundationpose_runtime/Dockerfile" \
  -t "${IMAGE_NAME}" \
  --build-arg BASE_IMAGE="${BASE_IMAGE}" \
  --build-arg ISAAC_ROS_RELEASE="${ISAAC_ROS_RELEASE}" \
  --build-arg L4T_SUITE="${L4T_SUITE}" \
  --build-arg INSTALL_EXAMPLES="${INSTALL_EXAMPLES}" \
  "${PROJECT_ROOT}"
