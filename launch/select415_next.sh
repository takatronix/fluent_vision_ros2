#!/bin/bash

# =============================================================================
# Fluent Vision - D415アスパラガス解析「次へ」選択コマンド
# =============================================================================
# 
# 概要：
#   - D415アスパラガス解析ノードに「次のアスパラガス」選択メッセージを送信
#   - キーボードショートカットとして使用可能
#   - リアルタイムでのアスパラガス選択制御
#
# 使用方法：
#   ./select415_next.sh
#   
# 作者：Takashi Otsuka
# 作成日：2024年
# バージョン：1.0
# =============================================================================

# 色付きログ出力関数
log_info() {
    echo -e "\033[32m[INFO]\033[0m $1"
}

log_warn() {
    echo -e "\033[33m[WARN]\033[0m $1"
}

log_error() {
    echo -e "\033[31m[ERROR]\033[0m $1"
}

# ヘルプ表示
show_help() {
    echo "Fluent Vision D415アスパラガス解析「次へ」選択コマンド"
    echo ""
    echo "使用方法:"
    echo "  $0"
    echo ""
    echo "機能:"
    echo "  D415アスパラガス解析ノードに「次のアスパラガス」選択メッセージを送信"
    echo ""
    echo "例:"
    echo "  $0     # D415の次のアスパラガスを選択"
}

# メイン処理
main() {
    # 引数チェック
    if [[ $# -gt 0 ]]; then
        if [[ "$1" == "-h" || "$1" == "--help" ]]; then
            show_help
            exit 0
        else
            log_error "無効な引数: $1"
            show_help
            exit 1
        fi
    fi
    
    log_info "D415アスパラガス解析「次へ」選択を実行中..."
    
    # D415アスパラガス解析ノードに「次へ」メッセージを送信
    # サービス名: /fv_aspara_analyzer_d415/select_next_asparagus
    # メッセージタイプ: std_srvs/srv/Trigger
    
    if ros2 service list | grep -q "/fv_aspara_analyzer_d415/select_next_asparagus"; then
        log_info "D415アスパラガス解析ノードに「次へ」メッセージを送信中..."
        ros2 service call /fv_aspara_analyzer_d415/select_next_asparagus std_srvs/srv/Trigger
        log_info "✅ D415「次へ」選択完了"
    else
        log_error "❌ D415アスパラガス解析ノードのサービスが見つかりません"
        log_warn "ノードが起動しているか確認してください: ros2 node list | grep fv_aspara_analyzer_d415"
        exit 1
    fi
}

# スクリプト実行
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
