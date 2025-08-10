
#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

source /opt/ros/humble/setup.bash
if [ -d "${WS_ROOT}/install" ]; then source "${WS_ROOT}/install/setup.bash" || true; fi

PARAMS_FILE="${SCRIPT_DIR}/fv_aspara_analyzer_d405.yaml"
if [ ! -f "${PARAMS_FILE}" ]; then
  echo "[d405debug] params file not found: ${PARAMS_FILE}" >&2
  exit 2
fi

echo "🌱 Starting Aspara Analyzer D405 node..."
exec ros2 run fv_aspara_analyzer fv_aspara_analyzer_node \
  --ros-args \
  --params-file "${PARAMS_FILE}" \
  -r __node:=fv_aspara_analyzer_d405