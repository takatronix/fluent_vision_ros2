#!/bin/bash
set -euo pipefail

# Resolve workspace root from this script's location
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
cd "$WS_ROOT"

# Source environment once (fast if already built)
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  # Avoid unbound var errors from set -u during sourcing
  set +u
  # shellcheck disable=SC1090
  source "$WS_ROOT/install/setup.bash"
  set -u
fi

bash ./scripts/stop_fv.sh
bash ./scripts/start_fv415.sh
bash ./scripts/start_fv405.sh
