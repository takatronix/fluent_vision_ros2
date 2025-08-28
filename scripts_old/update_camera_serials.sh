#!/bin/bash

# スクリプトのディレクトリを取得（相対パス対応）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "======================================"
echo "RealSenseカメラのシリアル番号検出ツール"
echo "======================================"
echo ""

# Function to get camera info
get_camera_info() {
    # カメラ検出用の一時的なPythonスクリプトを作成
    cat > /tmp/detect_cameras.py << 'EOF'
#!/usr/bin/env python3
import pyrealsense2 as rs
import sys

try:
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if len(devices) == 0:
        print("カメラが検出されませんでした")
        sys.exit(1)
    
    print(f"検出されたRealSenseカメラ: {len(devices)}台")
    print("")
    
    d415_serial = None
    d405_serial = None
    
    for i, device in enumerate(devices):
        name = device.get_info(rs.camera_info.name)
        serial = device.get_info(rs.camera_info.serial_number)
        firmware = device.get_info(rs.camera_info.firmware_version)
        
        print(f"カメラ {i+1}:")
        print(f"  モデル: {name}")
        print(f"  シリアル番号: {serial}")
        print(f"  ファームウェア: {firmware}")
        print("")
        
        if "D415" in name:
            d415_serial = serial
        elif "D405" in name:
            d405_serial = serial
    
    # 検出結果をファイルに保存（他の処理で使用）
    with open("/tmp/camera_serials.txt", "w") as f:
        if d415_serial:
            f.write(f"D415:{d415_serial}\n")
        if d405_serial:
            f.write(f"D405:{d405_serial}\n")
            
except Exception as e:
    print(f"エラーが発生しました: {e}")
    sys.exit(1)
EOF
}

# カメラ検出を実行
echo "🔍 カメラを検出中..."
echo ""

# pyrealsense2が利用可能かチェック
if command -v python3 >/dev/null 2>&1 && python3 -c "import pyrealsense2" 2>/dev/null; then
    get_camera_info
    python3 /tmp/detect_cameras.py
    DETECTION_RESULT=$?
else
    echo "⚠️  pyrealsense2が利用できません"
    echo "以下のコマンドでインストールしてください:"
    echo "  pip3 install pyrealsense2"
    exit 1
fi

# 検出結果を読み込み
if [ -f /tmp/camera_serials.txt ]; then
    D415_DETECTED=$(grep "D415:" /tmp/camera_serials.txt | cut -d':' -f2)
    D405_DETECTED=$(grep "D405:" /tmp/camera_serials.txt | cut -d':' -f2)
fi

echo "======================================"
echo "現在の設定ファイルの確認"
echo "======================================"
echo ""

# 設定ファイルのパス（相対パス）
D415_CONFIG="$PROJECT_ROOT/scripts/fv_realsense_d415.yaml"
D405_CONFIG="$PROJECT_ROOT/scripts/fv_realsense_d405.yaml"

# 現在の設定値を取得
if [ -f "$D415_CONFIG" ]; then
    D415_CURRENT=$(grep "serial_number:" "$D415_CONFIG" | sed 's/.*serial_number:[[:space:]]*"\(.*\)".*/\1/')
    echo "D415設定ファイル: $(basename "$D415_CONFIG")"
    echo "  現在の設定値: ${D415_CURRENT:-未設定}"
    if [ -n "$D415_DETECTED" ]; then
        echo "  検出された値: $D415_DETECTED"
        if [ "$D415_CURRENT" = "$D415_DETECTED" ]; then
            echo "  ✅ 設定は最新です"
        else
            echo "  ⚠️  設定の更新が必要です"
        fi
    else
        echo "  ❌ D415カメラが検出されませんでした"
    fi
else
    echo "❌ D415設定ファイルが見つかりません: $D415_CONFIG"
fi

echo ""

if [ -f "$D405_CONFIG" ]; then
    D405_CURRENT=$(grep "serial_number:" "$D405_CONFIG" | sed 's/.*serial_number:[[:space:]]*"\(.*\)".*/\1/')
    echo "D405設定ファイル: $(basename "$D405_CONFIG")"
    echo "  現在の設定値: ${D405_CURRENT:-未設定}"
    if [ -n "$D405_DETECTED" ]; then
        echo "  検出された値: $D405_DETECTED"
        if [ "$D405_CURRENT" = "$D405_DETECTED" ]; then
            echo "  ✅ 設定は最新です"
        else
            echo "  ⚠️  設定の更新が必要です"
        fi
    else
        echo "  ❌ D405カメラが検出されませんでした"
    fi
else
    echo "❌ D405設定ファイルが見つかりません: $D405_CONFIG"
fi

echo ""
echo "======================================"

# 自動更新オプション（--updateフラグ）
if [ "$1" = "--update" ] || [ "$1" = "-u" ]; then
    echo "自動更新モード"
    echo "======================================"
    echo ""
    
    # D415の更新
    if [ -n "$D415_DETECTED" ] && [ "$D415_CURRENT" != "$D415_DETECTED" ]; then
        echo "📝 D415設定を更新中..."
        if [ -f "$D415_CONFIG" ]; then
            # バックアップを作成
            cp "$D415_CONFIG" "${D415_CONFIG}.bak"
            # シリアル番号を更新
            sed -i "s/serial_number:.*/serial_number: \"$D415_DETECTED\"/" "$D415_CONFIG"
            echo "  ✅ D415設定を更新しました: $D415_CURRENT → $D415_DETECTED"
            echo "  バックアップ: ${D415_CONFIG}.bak"
        fi
    fi
    
    # D405の更新
    if [ -n "$D405_DETECTED" ] && [ "$D405_CURRENT" != "$D405_DETECTED" ]; then
        echo "📝 D405設定を更新中..."
        if [ -f "$D405_CONFIG" ]; then
            # バックアップを作成
            cp "$D405_CONFIG" "${D405_CONFIG}.bak"
            # シリアル番号を更新
            sed -i "s/serial_number:.*/serial_number: \"$D405_DETECTED\"/" "$D405_CONFIG"
            echo "  ✅ D405設定を更新しました: $D405_CURRENT → $D405_DETECTED"
            echo "  バックアップ: ${D405_CONFIG}.bak"
        fi
    fi
    
    echo ""
    echo "更新が完了しました"
else
    # 手動更新の案内
    echo ""
    echo "📌 設定を更新する方法:"
    echo ""
    
    if [ -n "$D415_DETECTED" ] && [ "$D415_CURRENT" != "$D415_DETECTED" ]; then
        echo "1. D415の設定を更新:"
        echo "   vi $D415_CONFIG"
        echo "   serial_number: \"$D415_DETECTED\""
        echo ""
    fi
    
    if [ -n "$D405_DETECTED" ] && [ "$D405_CURRENT" != "$D405_DETECTED" ]; then
        echo "2. D405の設定を更新:"
        echo "   vi $D405_CONFIG"
        echo "   serial_number: \"$D405_DETECTED\""
        echo ""
    fi
    
    echo "または、このスクリプトで自動更新:"
    echo "  $0 --update"
fi

echo ""
echo "======================================"

# 一時ファイルのクリーンアップ
rm -f /tmp/detect_cameras.py /tmp/camera_serials.txt