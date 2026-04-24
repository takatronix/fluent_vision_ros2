#!/usr/bin/env bash
set -euo pipefail

cd /opt/openpi
venv_python="/opt/openpi/.venv/bin/python"

if [ "$#" -gt 0 ]; then
  exec "$@"
fi

server_mode="${OPENPI_SERVER_MODE:-http}"

if [ "${server_mode}" = "http" ]; then
  exec "${venv_python}" /opt/openpi-runtime/server.py
fi

mode="${OPENPI_POLICY_MODE:-default}"
port="${OPENPI_PORT:-8000}"
default_prompt="${OPENPI_DEFAULT_PROMPT:-}"
args=("scripts/serve_policy.py" "--port=${port}")

if [ -n "${default_prompt}" ]; then
  args+=("--default-prompt=${default_prompt}")
fi

if [ "${mode}" = "checkpoint" ]; then
  args+=(
    "policy:checkpoint"
    "--policy.config=${OPENPI_POLICY_CONFIG:-pi0_aloha_sim}"
    "--policy.dir=${OPENPI_POLICY_DIR:-gs://openpi-assets/checkpoints/pi0_aloha_sim}"
  )
else
  args+=("--env=${OPENPI_ENV:-aloha_sim}")
fi

exec "${venv_python}" "${args[@]}"
