#!/usr/bin/env bash

set -euo pipefail

# Resolve workspace root (this script lives in launch/)
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
cd "$WS_ROOT"

echo "Building ROS 2 workspace at: $WS_ROOT"

# Default, fast incremental build (no sourcing)
DEFAULT_ARGS=(
  --symlink-install
  --event-handlers console_direct+
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
)

if [[ $# -gt 0 ]]; then
  echo "> colcon build $*"
  colcon build "$@"
else
  echo "> colcon build ${DEFAULT_ARGS[*]}"
  colcon build "${DEFAULT_ARGS[@]}"
fi

echo "Build finished."


