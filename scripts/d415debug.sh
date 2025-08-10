source /opt/ros/humble/setup.bash
source ~/seedbox-r1/fluent_vision_ros2/install/setup.bash
# -----------------------------------------------------------------
# [4] アスパラ分析ノード（重要）
# -----------------------------------------------------------------
#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

source /opt/ros/humble/setup.bash
if [ -d "${WS_ROOT}/install" ]; then source "${WS_ROOT}/install/setup.bash" || true; fi

PARAMS_FILE="${SCRIPT_DIR}/fv_aspara_analyzer_d415.yaml"
if [ ! -f "${PARAMS_FILE}" ]; then
  echo "[d415debug] params file not found: ${PARAMS_FILE}" >&2
  exit 2
fi

echo "🌱 Starting Aspara Analyzer D415 node..."
exec ros2 run fv_aspara_analyzer fv_aspara_analyzer_node \
  --ros-args \
  --params-file "${PARAMS_FILE}" \
  -r __node:=fv_aspara_analyzer_d415