#!/usr/bin/env bash

set -euo pipefail

# Resolve workspace root (this script lives in launch/)
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
cd "$WS_ROOT"

echo "Cleaning ROS 2 build artifacts at: $WS_ROOT"

targets=(build install log)
for t in "${targets[@]}"; do
  if [[ -e $t ]]; then
    echo "- Removing $t/"
    rm -rf "$t"
  else
    echo "- Skipping $t/ (not found)"
  fi
done

echo "Clean finished."


