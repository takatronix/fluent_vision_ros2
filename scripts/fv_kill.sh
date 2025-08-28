#!/bin/bash

set -uo pipefail

echo "🛑 Killing Aspara Vision processes..."

patterns=(
  "aspara_vision_manager"                               # ros2 run name (if used)
  "aspara_vision_ai/aspara_vision_ai/vision_manager.py" # direct python path
  "/ros2_ws/install/aspara_vision_ai/lib/aspara_vision_ai/vision_manager.py" # installed path
  "fv_realsense_node"
  "fv_object_detector_node"
  "fv_aspara_analyzer_node"
  "fv_object_mask_generator_node"
  "stem_detector_node"
)

list_pids() {
  local pattern="$1"
  mapfile -t pids < <(pgrep -f "$pattern" || true)
  # exclude current script and its parent shell
  local self_pids=("$$" "${PPID:-}" )
  for pid in "${self_pids[@]}"; do
    pids=("${pids[@]/$pid}")
  done
  # print remaining
  for pid in "${pids[@]:-}"; do
    if [[ -n "${pid:-}" ]]; then echo "$pid"; fi
  done
}

kill_phase() {
  local signal="$1"; shift
  local label="$1"; shift
  local any_killed=false
  for pattern in "${patterns[@]}"; do
    mapfile -t pids < <(list_pids "$pattern")
    if (( ${#pids[@]} > 0 )); then
      any_killed=true
      echo "[$label] $pattern -> ${#pids[@]} procs"
      # First try pkill (faster), then fallback to per-pid
      pkill -$signal -f "$pattern" 2>/dev/null || true
      for pid in "${pids[@]}"; do
        kill -$signal "$pid" 2>/dev/null || true
      done
    fi
  done
  if $any_killed; then sleep 1; fi
}

# 1) Gentle stop
kill_phase TERM "TERM"

# 2) Wait up to 5s for clean exit
for _ in 1 2 3 4 5; do
  remaining=0
  for pattern in "${patterns[@]}"; do
    mapfile -t pids < <(list_pids "$pattern")
    (( remaining += ${#pids[@]} ))
  done
  if (( remaining == 0 )); then break; fi
  sleep 1
done

# 3) Force kill leftovers
kill_phase KILL "KILL"

# 4) Report remaining (if any)
left=0
for pattern in "${patterns[@]}"; do
  mapfile -t pids < <(list_pids "$pattern")
  (( left += ${#pids[@]} ))
done

if (( left == 0 )); then
  echo "✅ All vision processes terminated."
else
  echo "⚠️ Some processes may still be running ($left). Run again or check with: pgrep -af python | grep vision_manager"
fi


