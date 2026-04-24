#!/usr/bin/env bash
set -euo pipefail

export PYTHONPATH="/opt/percus_ai:/opt/lerobot/src:${PYTHONPATH:-}"

if [[ "$#" -gt 0 ]]; then
  exec "$@"
fi

exec python /opt/fv-lerobot-policy-runtime/server.py
