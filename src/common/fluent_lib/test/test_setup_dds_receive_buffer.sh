#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/scripts/setup_dds_receive_buffer.sh"

fail() {
  printf 'test_setup_dds_receive_buffer: %s\n' "$*" >&2
  exit 1
}

assert_equal() {
  local expected=$1
  local actual=$2
  [[ $actual == "$expected" ]] ||
    fail "expected '$expected', got '$actual'"
}

temporary=$(mktemp -d)
trap 'rm -rf "$temporary"' EXIT
printf '212992\n' >"$temporary/rmem_max"

configure_dds_receive_buffer \
  "$temporary/sysctl.d/90-fluent-vision-dds.conf" "$temporary/rmem_max" 16777216
assert_equal \
  $'# Managed by setup_dds_receive_buffer.sh.\nnet.core.rmem_max = 16777216' \
  "$(cat "$temporary/sysctl.d/90-fluent-vision-dds.conf")"
assert_equal '16777216' "$(cat "$temporary/rmem_max")"

receive_buffer_config=$(cat "$temporary/sysctl.d/90-fluent-vision-dds.conf")
configure_dds_receive_buffer \
  "$temporary/sysctl.d/90-fluent-vision-dds.conf" "$temporary/rmem_max" 16777216
assert_equal \
  "$receive_buffer_config" \
  "$(cat "$temporary/sysctl.d/90-fluent-vision-dds.conf")"

if configure_dds_receive_buffer \
  "$temporary/sysctl.d/invalid.conf" "$temporary/rmem_max" invalid 2>/dev/null; then
  fail "invalid receive buffer size was accepted"
fi
