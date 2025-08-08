#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
cd "$WS_DIR"

echo "[build_all] Workspace: $WS_DIR"
echo "[build_all] Clean build artifacts..."
rm -rf build/ install/ log/

echo "[build_all] Step 1: Build fluent_lib first"
colcon build \
  --symlink-install \
  --packages-select fluent_lib \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF

echo "[build_all] Step 2: Build the rest of the workspace"
colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF

echo "[build_all] DONE"


