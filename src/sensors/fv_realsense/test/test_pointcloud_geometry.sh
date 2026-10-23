#!/usr/bin/env bash
set -euo pipefail
package_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
build_dir=$(mktemp -d)
trap 'rm -rf -- "$build_dir"' EXIT
"${CXX:-g++}" -std=c++17 -O1 -g -Wall -Wextra -Werror \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I "$package_dir/include" "$package_dir/test/test_pointcloud_geometry.cpp" \
  -o "$build_dir/test_pointcloud_geometry"
"$build_dir/test_pointcloud_geometry"
