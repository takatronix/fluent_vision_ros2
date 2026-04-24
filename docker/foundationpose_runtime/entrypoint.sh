#!/usr/bin/env bash
set -euo pipefail

restore_nounset=0
if [[ $- == *u* ]]; then
  restore_nounset=1
  set +u
fi

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [ -f "${ISAAC_ROS_WS}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${ISAAC_ROS_WS}/install/setup.bash"
fi

if [ "${restore_nounset}" = "1" ]; then
  set -u
fi

exec "$@"
