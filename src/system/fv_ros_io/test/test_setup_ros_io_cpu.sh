#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)/scripts/setup_ros_io_cpu.sh"

fail() {
  printf 'test_setup_ros_io_cpu: %s\n' "$*" >&2
  exit 1
}

assert_equal() {
  local expected=$1
  local actual=$2
  [[ $actual == "$expected" ]] ||
    fail "expected '$expected', got '$actual'"
}

assert_equal "0 1 2 5 7 8" "$(expand_cpu_list '0-2,5,7-8' | paste -sd' ' -)"
assert_equal "0-2,5,7-8" "$(printf '0\n1\n2\n5\n7\n8\n' | compress_cpu_list)"
assert_equal "0 2 10 11" "$(subtract_cpu_lists '0-3,10-13' '1,3,12-13' | paste -sd' ' -)"
assert_equal "8-10" "$(extract_isolated_cpu_list 'quiet isolcpus=domain,managed_irq,8-10 splash')"

updated=$(replace_managed_kernel_args \
  'quiet splash isolcpus=domain,8 irqaffinity=0-7 fv_ros_io.cpu=8' \
  13 13 0-12)
assert_equal \
  'quiet splash fv_ros_io.cpu=13 isolcpus=domain,managed_irq,13 irqaffinity=0-12 kthread_cpus=0-12' \
  "$updated"
assert_equal "$updated" "$(replace_managed_kernel_args "$updated" 13 13 0-12)"

temporary=$(mktemp -d)
trap 'rm -rf "$temporary"' EXIT
cat >"$temporary/extlinux.conf" <<'EOF'
TIMEOUT 30
DEFAULT primary

LABEL primary
      MENU LABEL primary kernel
      LINUX /boot/Image
      APPEND ${cbootargs} quiet

LABEL backup
      APPEND ${cbootargs} backup
EOF
assert_equal \
  '${cbootargs} quiet' \
  "$(read_extlinux_command_line "$temporary/extlinux.conf")"
update_extlinux "$temporary/extlinux.conf" "$updated"
extlinux_once=$(cat "$temporary/extlinux.conf")
update_extlinux "$temporary/extlinux.conf" "$updated"
assert_equal "$extlinux_once" "$(cat "$temporary/extlinux.conf")"
assert_equal \
  "      APPEND $updated" \
  "$(awk '$1 == "APPEND" { print; exit }' "$temporary/extlinux.conf")"
assert_equal \
  '      APPEND ${cbootargs} backup' \
  "$(awk '$1 == "LABEL" && $2 == "backup" { backup=1; next } backup && $1 == "APPEND" { print; exit }' "$temporary/extlinux.conf")"

cat >"$temporary/grub" <<'EOF'
GRUB_TIMEOUT=5
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"
EOF
assert_equal 'quiet splash' "$(read_grub_command_line "$temporary/grub")"
update_grub_file "$temporary/grub" "$updated"
assert_equal "$updated" "$(read_grub_command_line "$temporary/grub")"
grub_once=$(cat "$temporary/grub")
update_grub_file "$temporary/grub" "$updated"
assert_equal "$grub_once" "$(cat "$temporary/grub")"

printf '212992\n' >"$temporary/rmem_max"
configure_dds_receive_buffer \
  "$temporary/sysctl.d/90-fv-ros-io.conf" "$temporary/rmem_max" 16777216
assert_equal \
  $'# Managed by setup_ros_io_cpu.sh.\nnet.core.rmem_max = 16777216' \
  "$(cat "$temporary/sysctl.d/90-fv-ros-io.conf")"
assert_equal '16777216' "$(cat "$temporary/rmem_max")"
receive_buffer_config=$(cat "$temporary/sysctl.d/90-fv-ros-io.conf")
configure_dds_receive_buffer \
  "$temporary/sysctl.d/90-fv-ros-io.conf" "$temporary/rmem_max" 16777216
assert_equal \
  "$receive_buffer_config" \
  "$(cat "$temporary/sysctl.d/90-fv-ros-io.conf")"

mkdir -p "$temporary/sys/cpu" "$temporary/proc"
printf '0-3\n' >"$temporary/sys/cpu/online"
for cpu in 0 1 2 3; do
  mkdir -p "$temporary/sys/cpu/cpu$cpu/topology"
  printf '1024\n' >"$temporary/sys/cpu/cpu$cpu/cpu_capacity"
  printf '%s\n' "$cpu" >"$temporary/sys/cpu/cpu$cpu/topology/thread_siblings_list"
done
cat >"$temporary/proc/stat" <<'EOF'
cpu  40 0 40 400 0 0 0 0 0 0
cpu0 10 0 10 100 0 0 0 0 0 0
cpu1 10 0 10 100 0 0 0 0 0 0
cpu2 10 0 10 100 0 0 0 0 0 0
cpu3 10 0 10 100 0 0 0 0 0 0
EOF
cat >"$temporary/proc/interrupts" <<'EOF'
           CPU0       CPU1       CPU2       CPU3
  1:          1          1          1          1  test
EOF
read -r selected_cpu selected_siblings selected_irq selected_busy < <(
  select_ros_io_core "$temporary/sys/cpu" "$temporary/proc" 0
)
assert_equal '3' "$selected_cpu"
assert_equal '3' "$selected_siblings"
assert_equal '0' "$selected_irq"
assert_equal '0' "$selected_busy"

assert_equal \
  '2 2' \
  "$(configured_ros_io_core "$temporary/sys/cpu" 'quiet fv_ros_io.cpu=2' '0-3')"

printf '2-3\n' >"$temporary/sys/cpu/cpu2/topology/thread_siblings_list"
assert_equal \
  '2-3' \
  "$(configured_ros_io_cpus "$temporary/sys/cpu" 'quiet fv_ros_io.cpu=2')"
assert_equal \
  '0-1,4-5' \
  "$(subtract_cpu_lists '0-5' '2-3' | compress_cpu_list)"

cat >"$temporary/proc/stat" <<'EOF'
cpu  7600000001 2 2 2 2 2 2 2 2 2
cpu0 3800000000 1 1 1 1 1 1 1 1 1
cpu1 3800000001 1 1 1 1 1 1 1 1 1
EOF
cat >"$temporary/proc/interrupts" <<'EOF'
           CPU0       CPU1
  1: 3800000000 3800000001  test
EOF
read_cpu_snapshot "$temporary/proc/stat" "$temporary/proc/interrupts" "$temporary/large"
assert_equal '0 3800000007' "$(sed -n '1p' "$temporary/large.busy")"
assert_equal '1 3800000008' "$(sed -n '2p' "$temporary/large.busy")"
assert_equal '0 3800000000' "$(sed -n '1p' "$temporary/large.irq")"
assert_equal '1 3800000001' "$(sed -n '2p' "$temporary/large.irq")"
