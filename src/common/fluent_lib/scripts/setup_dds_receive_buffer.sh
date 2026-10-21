#!/usr/bin/env bash
set -euo pipefail

readonly DDS_RMEM_MIN_BYTES=16777216
readonly DDS_SYSCTL_FILE="/etc/sysctl.d/90-fluent-vision-dds.conf"
readonly DDS_RMEM_MAX_RUNTIME_FILE="/proc/sys/net/core/rmem_max"

die() {
  printf 'setup_dds_receive_buffer: %s\n' "$*" >&2
  return 1
}

ensure_dds_receive_buffer_minimum() {
  local sysctl_file=$1
  local runtime_file=$2
  local minimum_bytes=$3
  if [[ ! $minimum_bytes =~ ^[1-9][0-9]*$ ]]; then
    die "receive buffer minimum must be a positive byte count"
    return 1
  fi

  local current_bytes
  current_bytes=$(tr -d '[:space:]' <"$runtime_file")
  if [[ ! $current_bytes =~ ^[1-9][0-9]*$ ]]; then
    die "current receive buffer is not a positive byte count: $current_bytes"
    return 1
  fi

  local effective_bytes=$minimum_bytes
  if (( current_bytes > minimum_bytes )); then
    effective_bytes=$current_bytes
  fi

  local temporary
  temporary=$(mktemp)
  {
    printf '# Managed by setup_dds_receive_buffer.sh.\n'
    printf 'net.core.rmem_max = %s\n' "$effective_bytes"
  } >"$temporary"
  install -D -m 0644 "$temporary" "$sysctl_file"
  rm -f "$temporary"

  printf '%s\n' "$effective_bytes" >"$runtime_file"
  [[ $(tr -d '[:space:]' <"$runtime_file") == "$effective_bytes" ]] ||
    die "receive buffer verification failed after update"
}

main() {
  (( $# == 0 )) || die "this command takes no arguments"
  (( EUID == 0 )) || die "run this command with sudo"

  ensure_dds_receive_buffer_minimum \
    "$DDS_SYSCTL_FILE" "$DDS_RMEM_MAX_RUNTIME_FILE" "$DDS_RMEM_MIN_BYTES"

  printf 'DDS receive buffer ceiling: %s bytes (minimum: %s bytes)\n' \
    "$(tr -d '[:space:]' <"$DDS_RMEM_MAX_RUNTIME_FILE")" "$DDS_RMEM_MIN_BYTES"
  printf 'Restart DDS processes to use the updated receive buffer ceiling.\n'
}

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
