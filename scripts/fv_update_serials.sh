#!/usr/bin/env bash
set -euo pipefail

# =============================
# RealSense シリアル自動更新ツール
# 対象設定: /ros2_ws/config/fv_realsense_d405.yaml, fv_realsense_d415.yaml
# 使い方:
#   ./fv_update_serials.sh          # 検出と差分確認のみ
#   ./fv_update_serials.sh --update # 自動で設定ファイルを書き換え（バックアップ作成）
# =============================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR" && pwd)"
CONFIG_DIR="$PROJECT_ROOT/config"
D405_CONFIG="$CONFIG_DIR/fv_realsense_d405.yaml"
D415_CONFIG="$CONFIG_DIR/fv_realsense_d415.yaml"
TMP_OUT="/tmp/fv_camera_serials.txt"

print_header() {
  echo "======================================"
  echo "$1"
  echo "======================================"
}

# ツールチェック（どちらか一方があればOK）
require_tools() {
  local has_rs=0
  local has_lsusb=0
  if command -v rs-enumerate-devices >/dev/null 2>&1; then
    has_rs=1
  fi
  if command -v lsusb >/dev/null 2>&1; then
    has_lsusb=1
  fi
  if [[ $has_rs -eq 0 && $has_lsusb -eq 0 ]]; then
    echo "❌ 検出に必要なツールが見つかりません: rs-enumerate-devices または lsusb をインストールしてください。" >&2
    exit 1
  fi
  if [[ $has_rs -eq 0 && $has_lsusb -eq 1 ]]; then
    echo "⚠️  rs-enumerate-devices が見つかりません。lsusb フォールバックのみで検出します。" >&2
  fi
  if [[ $has_rs -eq 1 && $has_lsusb -eq 0 ]]; then
    echo "ℹ️  lsusb が見つかりませんが、rs-enumerate-devices があるため検出可能です。" >&2
  fi
}

# rs-enumerate-devices で検出
detect_with_rs() {
  command -v rs-enumerate-devices >/dev/null 2>&1 || return 1
  local out
  if ! out=$(rs-enumerate-devices 2>/dev/null); then
    return 1
  fi
  local model="" d405="" d415=""
  while IFS= read -r line; do
    if [[ "$line" == *"Name"*"Intel RealSense"* ]]; then
      if [[ "$line" == *"D405"* ]]; then
        model="D405"
      elif [[ "$line" == *"D415"* ]]; then
        model="D415"
      else
        model=""
      fi
    elif [[ "$line" == *"Serial Number"* && -n "$model" ]]; then
      local ser
      ser=$(echo "$line" | awk -F':' '{print $2}' | sed -E 's/^[[:space:]]+//')
      [[ "$model" == "D405" ]] && d405="$ser"
      [[ "$model" == "D415" ]] && d415="$ser"
      model=""
    fi
  done <<< "$out"
  : > "$TMP_OUT"
  [[ -n "$d415" ]] && echo "D415:$d415" >> "$TMP_OUT"
  [[ -n "$d405" ]] && echo "D405:$d405" >> "$TMP_OUT"
  if [[ -s "$TMP_OUT" ]]; then
    return 0
  else
    return 1
  fi
}

# lsusb フォールバック
detect_with_lsusb() {
  local d405="" d415=""
  d405=$(lsusb -v -d 8086:0b5b 2>/dev/null | awk '/iSerial/ {print $3; exit}') || true
  d415=$(lsusb -v -d 8086:0ad3 2>/dev/null | awk '/iSerial/ {print $3; exit}') || true
  : > "$TMP_OUT"
  [[ -n "$d415" ]] && echo "D415:$d415" >> "$TMP_OUT"
  [[ -n "$d405" ]] && echo "D405:$d405" >> "$TMP_OUT"
  if [[ -s "$TMP_OUT" ]]; then
    return 0
  else
    return 1
  fi
}

read_current_serial() {
  # $1: config path
  if [[ -f "$1" ]]; then
    # 引用符付きシリアルに対応
    grep -E "^[[:space:]]*serial_number:[[:space:]]*\".*\"" "$1" | sed -E 's/.*serial_number:[[:space:]]*"(.*)".*/\1/' || true
  else
    echo "" 
  fi
}

update_serial_in_file() {
  # $1: config path, $2: new serial
  local cfg="$1"; local newser="$2"
  [[ -f "$cfg" ]] || { echo "❌ 設定ファイルが見つかりません: $cfg"; return 1; }

  cp -p "$cfg" "${cfg}.bak"
  # serial_number: "..." の行を書き換える（末尾コメント保持）
  # ^(\s*serial_number:\s*")([^\"]*)(".*)$ -> 1 + new + 3
  sed -E -i 's/^([[:space:]]*serial_number:[[:space:]]*")[^"]*(".*)$/\1'"$newser"'\2/' "$cfg"
}

main() {
  local DO_UPDATE="false"
  if [[ "${1:-}" == "--update" || "${1:-}" == "-u" ]]; then
    DO_UPDATE="true"
  fi

  print_header "RealSense カメラ検出"
  require_tools
  rm -f "$TMP_OUT" || true
  if ! detect_with_rs; then
    echo "ℹ️  rs-enumerate-devices による検出に失敗。lsusb で再試行します。" >&2
    if ! detect_with_lsusb; then
      echo "❌ カメラ検出に失敗しました" >&2
    fi
  fi

  local D415_DETECTED="" D405_DETECTED=""
  if [[ -f "$TMP_OUT" ]]; then
    D415_DETECTED=$(grep -m1 "^D415:" "$TMP_OUT" | cut -d':' -f2-)
    D405_DETECTED=$(grep -m1 "^D405:" "$TMP_OUT" | cut -d':' -f2-)
  fi

  print_header "現在の設定確認 (/config)"

  local D415_CURRENT="" D405_CURRENT=""
  if [[ -f "$D415_CONFIG" ]]; then
    D415_CURRENT=$(read_current_serial "$D415_CONFIG")
    echo "D415: $(basename "$D415_CONFIG")"
    echo "  現在: ${D415_CURRENT:-未設定}"
    echo "  検出: ${D415_DETECTED:-未検出}"
    if [[ -n "$D415_DETECTED" && "$D415_CURRENT" == "$D415_DETECTED" ]]; then
      echo "  ✅ 一致"
    elif [[ -n "$D415_DETECTED" ]]; then
      echo "  ⚠️  差分あり (更新候補)"
    else
      echo "  ❌ 検出なし"
    fi
  else
    echo "❌ D415設定ファイルなし: $D415_CONFIG"
  fi
  echo
  if [[ -f "$D405_CONFIG" ]]; then
    D405_CURRENT=$(read_current_serial "$D405_CONFIG")
    echo "D405: $(basename "$D405_CONFIG")"
    echo "  現在: ${D405_CURRENT:-未設定}"
    echo "  検出: ${D405_DETECTED:-未検出}"
    if [[ -n "$D405_DETECTED" && "$D405_CURRENT" == "$D405_DETECTED" ]]; then
      echo "  ✅ 一致"
    elif [[ -n "$D405_DETECTED" ]]; then
      echo "  ⚠️  差分あり (更新候補)"
    else
      echo "  ❌ 検出なし"
    fi
  else
    echo "❌ D405設定ファイルなし: $D405_CONFIG"
  fi

  if [[ "$DO_UPDATE" == "true" ]]; then
    echo
    print_header "自動更新モード"
    if [[ -n "$D415_DETECTED" && "$D415_DETECTED" != "$D415_CURRENT" && -f "$D415_CONFIG" ]]; then
      echo "📝 D415 を更新中..."
      update_serial_in_file "$D415_CONFIG" "$D415_DETECTED"
      echo "  ✅ ${D415_CURRENT:-(未設定)} → $D415_DETECTED"
      echo "  バックアップ: ${D415_CONFIG}.bak"
    fi
    if [[ -n "$D405_DETECTED" && "$D405_DETECTED" != "$D405_CURRENT" && -f "$D405_CONFIG" ]]; then
      echo "📝 D405 を更新中..."
      update_serial_in_file "$D405_CONFIG" "$D405_DETECTED"
      echo "  ✅ ${D405_CURRENT:-(未設定)} → $D405_DETECTED"
      echo "  バックアップ: ${D405_CONFIG}.bak"
    fi
    echo "完了"
  else
    echo
    echo "📌 自動更新するには: $0 --update"
  fi
}

trap 'rm -f "$TMP_OUT"' EXIT
main "$@"

