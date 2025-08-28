#!/usr/bin/env bash
# Fast incremental build + optional run for Aspara Analyzer
# Usage:
#   fast_build_run.sh [--run d405|d415] [--clean] [--cmake Debug|RelWithDebInfo] [--gdb]
#                     [--tail] [--no-kill]

set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

RUN_TARGET=""    # d405 or d415
CMAKE_TYPE="RelWithDebInfo"
DO_CLEAN=0
USE_GDB=0
TAIL_LOG=0
KILL_EXISTING=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --run)
      RUN_TARGET="$2"; shift 2 ;;
    --clean)
      DO_CLEAN=1; shift ;;
    --cmake)
      CMAKE_TYPE="$2"; shift 2 ;;
    --gdb)
      USE_GDB=1; shift ;;
    --tail)
      TAIL_LOG=1; shift ;;
    --no-kill)
      KILL_EXISTING=0; shift ;;
    -h|--help)
      echo "Usage: $0 [--run d405|d415] [--clean] [--cmake Debug|RelWithDebInfo] [--gdb] [--tail] [--no-kill]"; exit 0 ;;
    *)
      echo "Unknown arg: $1" >&2; exit 2 ;;
  esac
done

cd "${WS_ROOT}"

if [[ ${DO_CLEAN} -eq 1 ]]; then
  echo "[fast] cleaning build logs for selected packages..."
  rm -rf build/fv_aspara_analyzer install/fv_aspara_analyzer* log/* || true
fi

echo "[fast] building (packages-up-to fv_aspara_analyzer) with CMAKE_BUILD_TYPE=${CMAKE_TYPE}"
colcon build \
  --symlink-install \
  --packages-up-to fv_aspara_analyzer \
  --cmake-args -DCMAKE_BUILD_TYPE=${CMAKE_TYPE}

echo "[fast] build done."

if [[ -z "${RUN_TARGET}" ]]; then
  echo "[fast] no --run specified. Done."
  exit 0
fi

cd "${SCRIPT_DIR}"

if [[ ${KILL_EXISTING} -eq 1 ]]; then
  echo "[fast] killing existing analyzer processes (if any)"
  pkill -f fv_aspara_analyzer_node || true
  sleep 0.3
fi

LOGFILE="/tmp/aspara_${RUN_TARGET}.log"

case "${RUN_TARGET}" in
  d405)
    if [[ ${USE_GDB} -eq 1 ]]; then
      echo "[fast] launching D405 under gdb"
      bash -lc 'source /opt/ros/humble/setup.bash && source "'"${WS_ROOT}"'/install/setup.bash" && exec gdb -q -ex run --args ros2 run fv_aspara_analyzer fv_aspara_analyzer_node --ros-args --params-file "'"${SCRIPT_DIR}"'/fv_aspara_analyzer_d405.yaml" -r __node:=fv_aspara_analyzer_d405' | tee "${LOGFILE}"
    else
      echo "[fast] launching D405"
      ./d405debug.sh | tee "${LOGFILE}"
    fi
    ;;
  d415)
    if [[ ${USE_GDB} -eq 1 ]]; then
      echo "[fast] launching D415 under gdb"
      bash -lc 'source /opt/ros/humble/setup.bash && source "'"${WS_ROOT}"'/install/setup.bash" && exec gdb -q -ex run --args ros2 run fv_aspara_analyzer fv_aspara_analyzer_node --ros-args --params-file "'"${SCRIPT_DIR}"'/fv_aspara_analyzer_d415.yaml" -r __node:=fv_aspara_analyzer_d415' | tee "${LOGFILE}"
    else
      echo "[fast] launching D415"
      ./d415debug.sh | tee "${LOGFILE}"
    fi
    ;;
  *)
    echo "[fast] unknown --run target: ${RUN_TARGET}" >&2; exit 2 ;;
esac

if [[ ${TAIL_LOG} -eq 1 ]]; then
  echo "[fast] tailing ${LOGFILE}"
  tail -n 200 -f "${LOGFILE}"
fi


