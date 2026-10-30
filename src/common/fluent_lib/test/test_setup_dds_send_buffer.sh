#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/scripts/setup_dds_send_buffer.sh"

fail() {
  printf 'test_setup_dds_send_buffer: %s\n' "$*" >&2
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
printf '212992\n' >"$temporary/wmem_max"

ensure_dds_send_buffer_minimum \
  "$temporary/sysctl.d/90-fluent-vision-dds-send.conf" \
  "$temporary/wmem_max" 16777216
assert_equal \
  $'# Managed by setup_dds_send_buffer.sh.\nnet.core.wmem_max = 16777216' \
  "$(cat "$temporary/sysctl.d/90-fluent-vision-dds-send.conf")"
assert_equal '16777216' "$(cat "$temporary/wmem_max")"

send_buffer_config=$(cat "$temporary/sysctl.d/90-fluent-vision-dds-send.conf")
ensure_dds_send_buffer_minimum \
  "$temporary/sysctl.d/90-fluent-vision-dds-send.conf" \
  "$temporary/wmem_max" 16777216
assert_equal \
  "$send_buffer_config" \
  "$(cat "$temporary/sysctl.d/90-fluent-vision-dds-send.conf")"

printf '33554432\n' >"$temporary/wmem_max"
ensure_dds_send_buffer_minimum \
  "$temporary/sysctl.d/90-fluent-vision-dds-send.conf" \
  "$temporary/wmem_max" 16777216
assert_equal \
  $'# Managed by setup_dds_send_buffer.sh.\nnet.core.wmem_max = 33554432' \
  "$(cat "$temporary/sysctl.d/90-fluent-vision-dds-send.conf")"
assert_equal '33554432' "$(cat "$temporary/wmem_max")"

if ensure_dds_send_buffer_minimum \
  "$temporary/sysctl.d/invalid.conf" "$temporary/wmem_max" invalid 2>/dev/null; then
  fail "invalid send buffer size was accepted"
fi

printf 'invalid\n' >"$temporary/wmem_max"
if ensure_dds_send_buffer_minimum \
  "$temporary/sysctl.d/invalid-current.conf" \
  "$temporary/wmem_max" 16777216 2>/dev/null; then
  fail "invalid current send buffer size was accepted"
fi
