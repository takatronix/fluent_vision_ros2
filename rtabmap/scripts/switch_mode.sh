#!/bin/bash
# RTABMap モード切り替えスクリプト
# 用途：マッピング ↔ ローカライゼーション モードの切り替え

set -e  # エラー時終了

echo "=== RTABMap モード切り替え ==="
echo "日時: $(date)"

# RTABMapサービス名
RTABMAP_NODE="/rtabmap"
SET_MODE_MAPPING_SERVICE="${RTABMAP_NODE}/set_mode_mapping"
SET_MODE_LOCALIZATION_SERVICE="${RTABMAP_NODE}/set_mode_localization"
GET_MAP_DATA_SERVICE="${RTABMAP_NODE}/get_map_data"

# 関数定義
check_rtabmap_running() {
    if ! pgrep -x "rtabmap" > /dev/null; then
        echo "エラー: RTABMapが実行されていません"
        echo "先にマッピングまたはローカライゼーションを開始してください"
        exit 1
    fi
}

check_service_available() {
    local service_name=$1
    local timeout=5
    
    echo "サービス確認中: $service_name"
    if timeout $timeout ros2 service list | grep -q "$service_name"; then
        echo "✓ サービス利用可能: $service_name"
        return 0
    else
        echo "✗ サービス利用不可: $service_name"
        return 1
    fi
}

get_current_mode() {
    # パラメータから現在のモードを取得
    echo "現在のモード確認中..."
    if ros2 param get $RTABMAP_NODE Mem/IncrementalMemory 2>/dev/null | grep -q "true"; then
        echo "現在のモード: マッピング"
        return 0  # mapping mode
    elif ros2 param get $RTABMAP_NODE Mem/IncrementalMemory 2>/dev/null | grep -q "false"; then
        echo "現在のモード: ローカライゼーション"
        return 1  # localization mode
    else
        echo "モード取得に失敗しました"
        return 2  # unknown
    fi
}

switch_to_mapping() {
    echo ""
    echo "=== マッピングモードに切り替え中 ==="
    
    if check_service_available "$SET_MODE_MAPPING_SERVICE"; then
        echo "マッピングモードサービス呼び出し中..."
        if ros2 service call "$SET_MODE_MAPPING_SERVICE" std_srvs/srv/Empty; then
            echo "✓ マッピングモードに切り替え完了"
            echo "注意: 新しいマップデータが既存データベースに追加されます"
        else
            echo "✗ マッピングモード切り替えに失敗"
            return 1
        fi
    else
        echo "✗ マッピングモードサービスが利用できません"
        return 1
    fi
}

switch_to_localization() {
    echo ""
    echo "=== ローカライゼーションモードに切り替え中 ==="
    
    if check_service_available "$SET_MODE_LOCALIZATION_SERVICE"; then
        echo "ローカライゼーションモードサービス呼び出し中..."
        if ros2 service call "$SET_MODE_LOCALIZATION_SERVICE" std_srvs/srv/Empty; then
            echo "✓ ローカライゼーションモードに切り替え完了"
            echo "注意: マップの更新は停止されました"
        else
            echo "✗ ローカライゼーションモード切り替えに失敗"
            return 1
        fi
    else
        echo "✗ ローカライゼーションモードサービスが利用できません"
        return 1
    fi
}

show_map_stats() {
    echo ""
    echo "=== マップ統計情報 ==="
    
    if check_service_available "$GET_MAP_DATA_SERVICE"; then
        echo "マップデータ取得中..."
        # マップデータサービスを呼び出し（簡略版）
        if ros2 service call "$GET_MAP_DATA_SERVICE" rtabmap_msgs/srv/GetMapData "{}"; then
            echo "マップデータ取得完了"
        else
            echo "マップデータ取得に失敗"
        fi
    fi
    
    # ノード数とループクロージャ数の表示
    echo "RTABMap統計:"
    ros2 topic echo --once /rtabmap/info 2>/dev/null | head -20 || echo "統計情報の取得に失敗"
}

# メイン処理
case "${1:-interactive}" in
    mapping|map|m)
        check_rtabmap_running
        switch_to_mapping
        ;;
    localization|loc|l)
        check_rtabmap_running
        switch_to_localization
        ;;
    status|s)
        check_rtabmap_running
        get_current_mode
        show_map_stats
        ;;
    toggle|t)
        check_rtabmap_running
        if get_current_mode; then
            # マッピングモード -> ローカライゼーションモードに切り替え
            switch_to_localization
        else
            # ローカライゼーションモード -> マッピングモードに切り替え
            switch_to_mapping
        fi
        ;;
    interactive|i|"")
        check_rtabmap_running
        
        # 現在のモード表示
        get_current_mode
        current_mode=$?
        
        echo ""
        echo "切り替えオプション:"
        echo "1) マッピングモード（新しいマップデータを追加）"
        echo "2) ローカライゼーションモード（位置推定のみ）"
        echo "3) ステータス表示"
        echo "4) 終了"
        echo ""
        read -p "選択してください (1-4): " -n 1 -r choice
        echo
        
        case $choice in
            1)
                switch_to_mapping
                ;;
            2)
                switch_to_localization
                ;;
            3)
                show_map_stats
                ;;
            4)
                echo "終了します"
                exit 0
                ;;
            *)
                echo "無効な選択です"
                exit 1
                ;;
        esac
        ;;
    help|h|--help|-h)
        echo "使用方法: $0 [コマンド]"
        echo ""
        echo "コマンド:"
        echo "  mapping, map, m     マッピングモードに切り替え"
        echo "  localization, loc, l  ローカライゼーションモードに切り替え"
        echo "  toggle, t           現在のモードを切り替え"
        echo "  status, s           現在の状態を表示"
        echo "  interactive, i      対話モード（デフォルト）"
        echo "  help, h             このヘルプを表示"
        echo ""
        echo "注意:"
        echo "- RTABMapが実行中である必要があります"
        echo "- モード切り替え後、しばらく処理時間がかかる場合があります"
        exit 0
        ;;
    *)
        echo "不明なコマンド: $1"
        echo "ヘルプ: $0 help"
        exit 1
        ;;
esac

echo ""
echo "=== モード切り替え完了 ==="
echo "日時: $(date)"