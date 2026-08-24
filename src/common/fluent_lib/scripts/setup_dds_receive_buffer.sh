#!/usr/bin/env bash
set -euo pipefail

readonly DDS_RMEM_MAX_BYTES=16777216
readonly DDS_SYSCTL_FILE="/etc/sysctl.d/90-fluent-vision-dds.conf"
readonly DDS_RMEM_MAX_RUNTIME_FILE="/proc/sys/net/core/rmem_max"

die() {
  printf 'setup_dds_receive_buffer: %s\n' "$*" >&2
  return 1
}

configure_dds_receive_buffer() {
  local sysctl_file=$1
  local runtime_file=$2
  local bytes=$3
  if [[ ! $bytes =~ ^[1-9][0-9]*$ ]]; then
    die "receive buffer must be a positive byte count"
    return 1
  fi

  local temporary
  temporary=$(mktemp)
  {
    printf '# Managed by setup_dds_receive_buffer.sh.\n'
    printf 'net.core.rmem_max = %s\n' "$bytes"
  } >"$temporary"
  install -D -m 0644 "$temporary" "$sysctl_file"
  rm -f "$temporary"

  printf '%s\n' "$bytes" >"$runtime_file"
  [[ $(tr -d '\n' <"$runtime_file") == "$bytes" ]] ||
    die "receive buffer verification failed after update"
}

main() {
  (( $# == 0 )) || die "this command takes no arguments"
  (( EUID == 0 )) || die "run this command with sudo"

  configure_dds_receive_buffer \
    "$DDS_SYSCTL_FILE" "$DDS_RMEM_MAX_RUNTIME_FILE" "$DDS_RMEM_MAX_BYTES"

  printf 'DDS receive buffer ceiling: %s bytes\n' "$DDS_RMEM_MAX_BYTES"
  printf 'Restart DDS processes to use the updated receive buffer ceiling.\n'
}

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
