#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/scripts/fv-uvcvideo"

fail() {
  printf 'test_fv_uvcvideo: %s\n' "$*" >&2
  exit 1
}

assert_equal() {
  local expected=$1
  local actual=$2
  [[ $actual == "$expected" ]] || fail "expected '$expected', got '$actual'"
}

temporary=$(mktemp -d)
trap 'rm -rf "$temporary"' EXIT

printf '# R38 (release), REVISION: 4.0, BOARD: generic\n' >"$temporary/nv_tegra_release"
printf 'ID=debian\n' >"$temporary/debian-os-release"
printf 'ID=ubuntu\n' >"$temporary/ubuntu-os-release"
assert_equal \
  'jetson-linux-r38.4' \
  "$(detect_profile aarch64 6.8.12-tegra "$temporary/nv_tegra_release" "$temporary/ubuntu-os-release")"
assert_equal \
  'jetson-linux-r38.4' \
  "$(detect_profile aarch64 6.8.12-tegra-custom "$temporary/nv_tegra_release" "$temporary/ubuntu-os-release")"
printf '# R36 (release), REVISION: 4.0, BOARD: generic\n' >"$temporary/other-nv-tegra-release"
assert_equal \
  'unsupported' \
  "$(detect_profile aarch64 5.15.148-tegra "$temporary/other-nv-tegra-release" "$temporary/ubuntu-os-release")"
assert_equal \
  'ubuntu' \
  "$(detect_profile x86_64 6.8.0-138-generic /nonexistent "$temporary/ubuntu-os-release")"
assert_equal \
  'unsupported' \
  "$(detect_profile x86_64 6.8.12-generic /nonexistent "$temporary/debian-os-release")"

dpkg-query() {
  if [[ ${*: -1} == linux-modules-6.8.0-138-generic ]]; then
    printf 'linux\t6.8.0-138.138\n'
    return
  fi
  return 1
}
assert_equal \
  $'linux-source-6.8.0\tlinux\t6.8.0-138.138' \
  "$(ubuntu_kernel_source_spec 6.8.0-138-generic)"
assert_equal \
  'ubuntu' \
  "$(select_install_profile x86_64 6.8.0-138-generic /nonexistent "$temporary/ubuntu-os-release")"
assert_equal \
  'unsupported' \
  "$(select_install_profile x86_64 6.8.0-custom /nonexistent "$temporary/ubuntu-os-release")"
unset -f dpkg-query

mkdir -p "$temporary/build/include/config"
printf '6.8.12-tegra-custom\n' >"$temporary/build/include/config/kernel.release"
assert_equal \
  '6.8.12-tegra-custom' \
  "$(kernel_build_release "$temporary/build")"

mkdir -p "$temporary/uvc"
printf '%s\n' \
  '#define UVC_URBS        5' \
  '#define UVC_MAX_PACKETS 32' >"$temporary/uvc/uvcvideo.h"
printf '%s\n' \
  '#include <linux/module.h>' \
  'MODULE_VERSION("1.1.1");' >"$temporary/uvc/uvc_driver.c"
touch "$temporary/uvc/uvc_video.c"
printf '%s\n' \
  'uvcvideo-objs := uvc_driver.o uvc_video.o' \
  'obj-$(CONFIG_USB_VIDEO_CLASS) += uvcvideo.o' >"$temporary/uvc/Makefile"

patch_uvc_source "$temporary/uvc"
assert_equal \
  '32' \
  "$(sed -nE 's/^#define UVC_URBS[[:space:]]+([0-9]+).*/\1/p' "$temporary/uvc/uvcvideo.h")"
grep -q 'MODULE_INFO(fv_uvc_urbs, "32");' "$temporary/uvc/uvc_driver.c" ||
  fail 'module metadata was not added'
grep -q '^obj-m += uvcvideo.o$' "$temporary/uvc/Makefile" ||
  fail 'external module target was not configured'

patch_uvc_source "$temporary/uvc"
assert_equal \
  '1' \
  "$(grep -c 'MODULE_INFO(fv_uvc_urbs' "$temporary/uvc/uvc_driver.c")"

cp -a "$temporary/uvc" "$temporary/ambiguous"
printf '#define UVC_URBS 7\n' >>"$temporary/ambiguous/uvcvideo.h"
if (patch_uvc_source "$temporary/ambiguous" >/dev/null 2>&1); then
  fail 'ambiguous UVC_URBS definitions were accepted'
fi

if ! printf 'y\n' | confirm_unsupported aarch64 6.8.12-custom >/dev/null 2>&1; then
  fail 'unsupported environment confirmation rejected y'
fi
if printf 'n\n' | confirm_unsupported aarch64 6.8.12-custom >/dev/null 2>&1; then
  fail 'unsupported environment confirmation accepted n'
fi

mkdir -p "$temporary/state"
printf managed-module >"$temporary/uvcvideo.ko"
printf 'module_sha256=%s\n' \
  "$(sha256sum "$temporary/uvcvideo.ko" | awk '{print $1}')" \
  >"$temporary/state/installed.env"
verify_managed_install "$temporary/uvcvideo.ko" "$temporary/state"
printf changed-module >"$temporary/uvcvideo.ko"
if (verify_managed_install "$temporary/uvcvideo.ko" "$temporary/state") >/dev/null 2>&1; then
  fail 'a changed installed module was accepted for removal'
fi

printf 'test_fv_uvcvideo: passed\n'
