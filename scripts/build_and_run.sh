#!/bin/bash
set -euo pipefail

# workspace root
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
cd "$WS_ROOT"

echo "== Fast build =="
./scripts/fast_build.sh

echo "== Stop all FV nodes =="
./scripts/stop_fv

echo "== Start all FV nodes =="
./scripts/start_fv

echo "== System snapshot =="
ros2 node list | sort || true
echo "-- topics (aspara) --"
ros2 topic list | grep aspara || true

echo "== Monitoring (CTRL+C to stop monitors; system keeps running) =="
# Lightweight FPS monitors (background)
ros2 topic hz /fv/d415/aspara_analysis/result 2>/dev/null &
HZ1=$!
ros2 topic hz /fv/d405/aspara_analysis/result 2>/dev/null &
HZ2=$!

trap 'echo; echo "Stopping monitors..."; kill $HZ1 $HZ2 2>/dev/null || true' INT TERM EXIT

# Keep script alive to show hz output
wait


