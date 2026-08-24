#!/usr/bin/env bash
set -euo pipefail

readonly FV_ROS_IO_KEY="fv_ros_io.cpu"
readonly SAMPLE_SECONDS=2
readonly DDS_RMEM_MAX_BYTES=16777216
readonly DDS_SYSCTL_FILE="/etc/sysctl.d/90-fv-ros-io.conf"
readonly DDS_RMEM_MAX_RUNTIME_FILE="/proc/sys/net/core/rmem_max"

die() {
  printf 'setup_ros_io_cpu: %s\n' "$*" >&2
  exit 1
}

expand_cpu_list() {
  local value=${1//$'\n'/}
  awk -v list="$value" '
    BEGIN {
      count = split(list, fields, ",")
      for (i = 1; i <= count; ++i) {
        if (fields[i] ~ /^[0-9]+$/) {
          print fields[i] + 0
        } else if (fields[i] ~ /^[0-9]+-[0-9]+$/) {
          split(fields[i], range, "-")
          if (range[1] > range[2]) exit 2
          for (cpu = range[1]; cpu <= range[2]; ++cpu) print cpu
        } else if (fields[i] != "") {
          exit 2
        }
      }
    }
  ' | sort -n -u
}

compress_cpu_list() {
  sort -n -u | awk '
    function emit(first, last) {
      if (output != "") output = output ","
      output = output first
      if (last != first) output = output "-" last
    }
    NR == 1 { first = previous = $1; next }
    $1 == previous + 1 { previous = $1; next }
    { emit(first, previous); first = previous = $1 }
    END { if (NR > 0) emit(first, previous); print output }
  '
}

cpu_list_contains() {
  local list=$1
  local expected=$2
  expand_cpu_list "$list" | grep -qx "$expected"
}

subtract_cpu_lists() {
  local complete=$1
  local removed=$2
  awk '
    NR == FNR { removed[$1] = 1; next }
    !($1 in removed) { print }
  ' <(expand_cpu_list "$removed") <(expand_cpu_list "$complete")
}

read_kernel_arg() {
  local key=$1
  local command_line=$2
  local token
  for token in $command_line; do
    if [[ $token == "$key="* ]]; then
      printf '%s\n' "${token#*=}"
    fi
  done
}

extract_isolated_cpu_list() {
  local command_line=$1
  local value
  while read -r value; do
    [[ -n $value ]] || continue
    tr ',' '\n' <<<"$value" |
      awk '/^[0-9]+$/ || /^[0-9]+-[0-9]+$/'
  done < <(read_kernel_arg isolcpus "$command_line") |
    paste -sd, -
}

replace_managed_kernel_args() {
  local command_line=$1
  local reserved_cpu=$2
  local isolated_cpus=$3
  local housekeeping_cpus=$4
  local token
  local -a preserved=()

  for token in $command_line; do
    case "$token" in
      "$FV_ROS_IO_KEY="*|isolcpus=*|irqaffinity=*|kthread_cpus=*) ;;
      *) preserved+=("$token") ;;
    esac
  done

  preserved+=(
    "$FV_ROS_IO_KEY=$reserved_cpu"
    "isolcpus=domain,managed_irq,$isolated_cpus"
    "irqaffinity=$housekeeping_cpus"
    "kthread_cpus=$housekeeping_cpus"
  )
  printf '%s\n' "${preserved[*]}"
}

read_cpu_snapshot() {
  local stat_file=$1
  local interrupts_file=$2
  local output=$3

  awk '
    /^cpu[0-9]+ / {
      cpu = substr($1, 4)
      idle = $5 + $6
      total = 0
      for (i = 2; i <= NF; ++i) total += $i
      printf "%s %.0f\n", cpu, total - idle
    }
  ' "$stat_file" | sort -n >"$output.busy"

  awk '
    NR == 1 {
      for (i = 1; i <= NF; ++i) {
        if ($i ~ /^CPU[0-9]+$/) cpu_column[i + 1] = substr($i, 4)
      }
      next
    }
    {
      for (column in cpu_column) {
        if ($column ~ /^[0-9]+$/) irq[cpu_column[column]] += $column
      }
    }
    END {
      for (cpu in irq) printf "%s %.0f\n", cpu, irq[cpu]
    }
  ' "$interrupts_file" | sort -n >"$output.irq"
}

cpu_metric_delta() {
  local before_file=$1
  local after_file=$2
  local cpu=$3
  local before after
  before=$(awk -v cpu="$cpu" '$1 == cpu { print $2 }' "$before_file")
  after=$(awk -v cpu="$cpu" '$1 == cpu { print $2 }' "$after_file")
  printf '%s\n' "$(( ${after:-0} - ${before:-0} ))"
}

cpu_capacity() {
  local sys_cpu_root=$1
  local cpu=$2
  local capacity_file="$sys_cpu_root/cpu$cpu/cpu_capacity"
  local frequency_file="$sys_cpu_root/cpu$cpu/cpufreq/cpuinfo_max_freq"
  if [[ -r $capacity_file ]]; then
    cat "$capacity_file"
  elif [[ -r $frequency_file ]]; then
    cat "$frequency_file"
  else
    printf '1\n'
  fi
}

thread_siblings() {
  local sys_cpu_root=$1
  local cpu=$2
  local sibling_file="$sys_cpu_root/cpu$cpu/topology/thread_siblings_list"
  if [[ -r $sibling_file ]]; then
    tr -d '\n' <"$sibling_file"
  else
    printf '%s\n' "$cpu"
  fi
}

configured_ros_io_cpus() {
  local sys_cpu_root=$1
  local command_line=$2
  local cpu
  while read -r cpu; do
    [[ -n $cpu ]] || continue
    [[ $cpu =~ ^[0-9]+$ ]] || die "fv_ros_io.cpu must be a non-negative CPU index"
    thread_siblings "$sys_cpu_root" "$cpu"
  done < <(read_kernel_arg "$FV_ROS_IO_KEY" "$command_line")
}

configured_ros_io_core() {
  local sys_cpu_root=$1
  local command_line=$2
  local online=$3
  local -a configured=()
  mapfile -t configured < <(read_kernel_arg "$FV_ROS_IO_KEY" "$command_line")
  (( ${#configured[@]} == 1 )) || return 1

  local cpu=${configured[0]}
  [[ $cpu =~ ^[0-9]+$ ]] || return 1
  (( cpu != 0 )) || return 1
  cpu_list_contains "$online" "$cpu" || return 1

  local max_capacity=0
  local logical capacity
  while read -r logical; do
    [[ $logical -ne 0 ]] || continue
    capacity=$(cpu_capacity "$sys_cpu_root" "$logical")
    (( capacity > max_capacity )) && max_capacity=$capacity
  done < <(expand_cpu_list "$online")
  (( max_capacity > 0 )) || return 1
  (( $(cpu_capacity "$sys_cpu_root" "$cpu") == max_capacity )) || return 1

  local siblings
  siblings=$(thread_siblings "$sys_cpu_root" "$cpu")
  while read -r logical; do
    cpu_list_contains "$online" "$logical" || return 1
    (( $(cpu_capacity "$sys_cpu_root" "$logical") == max_capacity )) || return 1
  done < <(expand_cpu_list "$siblings")

  printf '%s %s\n' "$cpu" "$siblings"
}

select_ros_io_core() {
  local sys_cpu_root=$1
  local proc_root=$2
  local sample_seconds=$3
  local online
  online=$(tr -d '\n' <"$sys_cpu_root/online")

  local temporary
  temporary=$(mktemp -d)
  trap 'rm -rf "$temporary"' RETURN
  read_cpu_snapshot "$proc_root/stat" "$proc_root/interrupts" "$temporary/before"
  sleep "$sample_seconds"
  read_cpu_snapshot "$proc_root/stat" "$proc_root/interrupts" "$temporary/after"

  local max_capacity=0
  local cpu capacity
  while read -r cpu; do
    [[ $cpu -ne 0 ]] || continue
    capacity=$(cpu_capacity "$sys_cpu_root" "$cpu")
    (( capacity > max_capacity )) && max_capacity=$capacity
  done < <(expand_cpu_list "$online")
  (( max_capacity > 0 )) || die "host must provide an online CPU in addition to CPU 0"

  local best_cpu=-1
  local best_siblings=""
  local best_irq=-1
  local best_busy=-1
  local siblings logical sibling_capacity irq busy
  declare -A visited=()

  while read -r cpu; do
    [[ $cpu -ne 0 ]] || continue
    capacity=$(cpu_capacity "$sys_cpu_root" "$cpu")
    (( capacity == max_capacity )) || continue

    siblings=$(thread_siblings "$sys_cpu_root" "$cpu")
    [[ -z ${visited[$siblings]+x} ]] || continue
    visited[$siblings]=1

    irq=0
    busy=0
    while read -r logical; do
      sibling_capacity=$(cpu_capacity "$sys_cpu_root" "$logical")
      (( sibling_capacity == max_capacity )) || continue 2
      irq=$((irq + $(cpu_metric_delta "$temporary/before.irq" "$temporary/after.irq" "$logical")))
      busy=$((busy + $(cpu_metric_delta "$temporary/before.busy" "$temporary/after.busy" "$logical")))
    done < <(expand_cpu_list "$siblings")

    logical=$(expand_cpu_list "$siblings" | tail -n 1)
    if (( best_cpu < 0 || irq < best_irq ||
          (irq == best_irq && busy < best_busy) ||
          (irq == best_irq && busy == best_busy && logical > best_cpu) )); then
      best_cpu=$logical
      best_siblings=$siblings
      best_irq=$irq
      best_busy=$busy
    fi
  done < <(expand_cpu_list "$online")

  (( best_cpu >= 0 )) || die "no ROS I/O CPU candidate was found"
  printf '%s %s %s %s\n' "$best_cpu" "$best_siblings" "$best_irq" "$best_busy"
}

update_extlinux() {
  local file=$1
  local command_line=$2
  local default_label
  default_label=$(awk '$1 == "DEFAULT" { print $2; exit }' "$file")
  [[ -n $default_label ]] || die "DEFAULT entry is missing from $file"

  local temporary
  temporary=$(mktemp)
  awk -v label="$default_label" -v args="$command_line" '
    $1 == "LABEL" { current = $2 }
    current == label && $1 == "APPEND" {
      match($0, /^[[:space:]]*/)
      print substr($0, 1, RLENGTH) "APPEND " args
      updated = 1
      next
    }
    { print }
    END { if (!updated) exit 3 }
  ' "$file" >"$temporary" || {
    rm -f "$temporary"
    die "APPEND line for DEFAULT entry is missing from $file"
  }
  install -m 0644 "$temporary" "$file"
  rm -f "$temporary"
}

read_extlinux_command_line() {
  local file=$1
  local default_label
  default_label=$(awk '$1 == "DEFAULT" { print $2; exit }' "$file")
  [[ -n $default_label ]] || die "DEFAULT entry is missing from $file"

  awk -v label="$default_label" '
    $1 == "LABEL" { current = $2 }
    current == label && $1 == "APPEND" {
      sub(/^[[:space:]]*APPEND[[:space:]]+/, "")
      print
      found = 1
      exit
    }
    END { if (!found) exit 3 }
  ' "$file" || die "APPEND line for DEFAULT entry is missing from $file"
}

read_grub_command_line() {
  local file=$1
  local assignment
  assignment=$(grep -m1 '^GRUB_CMDLINE_LINUX_DEFAULT=' "$file" || true)
  [[ -n $assignment ]] || {
    printf '\n'
    return
  }

  local value=${assignment#*=}
  if [[ $value =~ ^\"(.*)\"$ || $value =~ ^\'(.*)\'$ ]]; then
    printf '%s\n' "${BASH_REMATCH[1]}"
  else
    die "GRUB_CMDLINE_LINUX_DEFAULT must use matching quotes in $file"
  fi
}

update_grub_file() {
  local file=$1
  local command_line=$2
  local escaped=${command_line//\\/\\\\}
  escaped=${escaped//\"/\\\"}
  local temporary
  temporary=$(mktemp)
  awk -v args="$escaped" '
    /^GRUB_CMDLINE_LINUX_DEFAULT=/ {
      print "GRUB_CMDLINE_LINUX_DEFAULT=\"" args "\""
      updated = 1
      next
    }
    { print }
    END {
      if (!updated) print "GRUB_CMDLINE_LINUX_DEFAULT=\"" args "\""
    }
  ' "$file" >"$temporary"
  install -m 0644 "$temporary" "$file"
  rm -f "$temporary"
}

update_grub() {
  local file=$1
  local command_line=$2
  update_grub_file "$file" "$command_line"
  update-grub
}

configure_dds_receive_buffer() {
  local sysctl_file=$1
  local runtime_file=$2
  local bytes=$3
  [[ $bytes =~ ^[1-9][0-9]*$ ]] || die "DDS receive buffer must be a positive byte count"

  local temporary
  temporary=$(mktemp)
  {
    printf '# Managed by setup_ros_io_cpu.sh.\n'
    printf 'net.core.rmem_max = %s\n' "$bytes"
  } >"$temporary"
  install -D -m 0644 "$temporary" "$sysctl_file"
  rm -f "$temporary"

  printf '%s\n' "$bytes" >"$runtime_file"
  [[ $(tr -d '\n' <"$runtime_file") == "$bytes" ]] ||
    die "DDS receive buffer verification failed after update"
}

main() {
  (( $# == 0 )) || die "this command takes no arguments"
  (( EUID == 0 )) || die "run this command with sudo"
  [[ -r /sys/devices/system/cpu/online ]] || die "Linux CPU sysfs is unavailable"
  [[ -r /proc/stat && -r /proc/interrupts && -r /proc/cmdline ]] ||
    die "Linux procfs CPU metrics are unavailable"

  local current_command_line
  current_command_line=$(cat /proc/cmdline)

  local bootloader boot_file configured_command_line
  if [[ -f /boot/extlinux/extlinux.conf ]]; then
    bootloader=extlinux
    boot_file=/boot/extlinux/extlinux.conf
    configured_command_line=$(read_extlinux_command_line "$boot_file")
  elif [[ -f /etc/default/grub ]]; then
    bootloader=grub
    boot_file=/etc/default/grub
    configured_command_line=$(read_grub_command_line "$boot_file")
  else
    die "supported extlinux or GRUB configuration was not found"
  fi

  local selected_cpu siblings irq_delta=0 busy_delta=0 selection_source
  local online
  online=$(tr -d '\n' </sys/devices/system/cpu/online)
  if read -r selected_cpu siblings < <(
    configured_ros_io_core \
      /sys/devices/system/cpu "$configured_command_line" "$online"
  ); then
    selection_source=existing
  else
    read -r selected_cpu siblings irq_delta busy_delta < <(
      select_ros_io_core /sys/devices/system/cpu /proc "$SAMPLE_SECONDS"
    )
    selection_source=measured
  fi

  local previous_ros_io_cpus existing_isolated retained_isolated
  local combined_isolated housekeeping
  previous_ros_io_cpus=$(
    {
      configured_ros_io_cpus /sys/devices/system/cpu "$current_command_line"
      configured_ros_io_cpus /sys/devices/system/cpu "$configured_command_line"
    } | tr ',' '\n' | awk 'NF' | compress_cpu_list
  )
  existing_isolated=$(
    {
      extract_isolated_cpu_list "$current_command_line"
      extract_isolated_cpu_list "$configured_command_line"
    } | tr ',' '\n' | awk 'NF' | paste -sd, -
  )
  retained_isolated=$(
    subtract_cpu_lists "$existing_isolated" "$previous_ros_io_cpus" |
      compress_cpu_list
  )
  combined_isolated=$(
    {
      expand_cpu_list "$retained_isolated"
      expand_cpu_list "$siblings"
    } | compress_cpu_list
  )
  housekeeping=$(
    subtract_cpu_lists "$online" "$combined_isolated" |
      compress_cpu_list
  )
  [[ -n $housekeeping ]] || die "CPU selection left no housekeeping CPU"

  local updated_command_line
  updated_command_line=$(replace_managed_kernel_args \
    "$configured_command_line" "$selected_cpu" "$combined_isolated" "$housekeeping")

  local changed=false
  if [[ $updated_command_line != "$configured_command_line" ]]; then
    local timestamp
    timestamp=$(date +%Y%m%d%H%M%S)
    cp -a "$boot_file" "$boot_file.fv_ros_io.$timestamp.bak"
    if [[ $bootloader == extlinux ]]; then
      update_extlinux "$boot_file" "$updated_command_line"
      [[ $(read_extlinux_command_line "$boot_file") == "$updated_command_line" ]] ||
        die "extlinux verification failed after update"
    else
      update_grub "$boot_file" "$updated_command_line"
      [[ $(read_grub_command_line "$boot_file") == "$updated_command_line" ]] ||
        die "GRUB verification failed after update"
    fi
    changed=true
  fi

  configure_dds_receive_buffer \
    "$DDS_SYSCTL_FILE" "$DDS_RMEM_MAX_RUNTIME_FILE" "$DDS_RMEM_MAX_BYTES"

  printf 'ROS I/O CPU: %s\n' "$selected_cpu"
  printf 'Isolated CPUs: %s\n' "$combined_isolated"
  printf 'Housekeeping CPUs: %s\n' "$housekeeping"
  printf 'DDS receive buffer ceiling: %s bytes\n' "$DDS_RMEM_MAX_BYTES"
  printf 'DDS processes started after this command use the updated receive buffer ceiling.\n'
  if [[ $selection_source == measured ]]; then
    printf 'Selection sample: irq_delta=%s busy_delta=%s over %ss\n' \
      "$irq_delta" "$busy_delta" "$SAMPLE_SECONDS"
  else
    printf 'Selection source: existing valid assignment\n'
  fi

  local -a active_reserved=()
  mapfile -t active_reserved < <(read_kernel_arg "$FV_ROS_IO_KEY" "$current_command_line")
  if [[ $changed == false && ${#active_reserved[@]} -eq 1 &&
        ${active_reserved[0]} == "$selected_cpu" ]] &&
      cpu_list_contains "$(cat /sys/devices/system/cpu/isolated)" "$selected_cpu"; then
    printf 'The CPU isolation contract is active.\n'
  else
    printf 'The CPU isolation contract becomes active after the next host boot.\n'
  fi
}

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
