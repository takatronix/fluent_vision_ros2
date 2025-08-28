#!/bin/bash

# =============================================================================
# Fluent Vision - AIノード制御スクリプト
# =============================================================================
# 
# 概要：
#   - object_mask_generator（UNet）とobject_detector（YOLO）のON/OFF制御
#   - カメラ別（D415/D405）での個別制御
#   - 重い処理を片方だけ停止してリソース節約
#
# 使用方法：
#   ./ai_control.sh [start|stop|restart] [unet|yolo|both] [d415|d405|both]
#   
#   例：
#   ./ai_control.sh stop unet d415    # D415のUNetを停止
#   ./ai_control.sh start yolo d405   # D405のYOLOを開始
#   ./ai_control.sh restart both both # 全AIノードを再起動
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
    echo "Fluent Vision AIノード制御スクリプト"
    echo ""
    echo "使用方法:"
    echo "  $0 [start|stop|restart] [unet|yolo|both] [d415|d405|both]"
    echo ""
    echo "コマンド:"
    echo "  start    - AIノードを開始"
    echo "  stop     - AIノードを停止"
    echo "  restart  - AIノードを再起動"
    echo ""
    echo "AIタイプ:"
    echo "  unet     - object_mask_generator（セグメンテーション）"
    echo "  yolo     - object_detector（物体検出）"
    echo "  both     - 両方のAIノード"
    echo ""
    echo "カメラ:"
    echo "  d415     - RealSense D415"
    echo "  d405     - RealSense D405"
    echo "  both     - 両方のカメラ"
    echo ""
    echo "例:"
    echo "  $0 stop unet d415     # D415のUNetを停止"
    echo "  $0 start yolo d405    # D405のYOLOを開始"
    echo "  $0 restart both both  # 全AIノードを再起動"
}

# ノード名の取得
get_node_names() {
    local ai_type=$1
    local camera=$2
    local nodes=""
    
    case $ai_type in
        "unet")
            case $camera in
                "d415") nodes="fv_object_mask_generator_d415" ;;
                "d405") nodes="fv_object_mask_generator_d405" ;;
                "both") nodes="fv_object_mask_generator_d415 fv_object_mask_generator_d405" ;;
            esac
            ;;
        "yolo")
            case $camera in
                "d415") nodes="fv_object_detector_d415" ;;
                "d405") nodes="fv_object_detector_d405" ;;
                "both") nodes="fv_object_detector_d415 fv_object_detector_d405" ;;
            esac
            ;;
        "both")
            case $camera in
                "d415") nodes="fv_object_mask_generator_d415 fv_object_detector_d415" ;;
                "d405") nodes="fv_object_mask_generator_d405 fv_object_detector_d405" ;;
                "both") nodes="fv_object_mask_generator_d415 fv_object_mask_generator_d405 fv_object_detector_d415 fv_object_detector_d405" ;;
            esac
            ;;
    esac
    
    echo "$nodes"
}

# ノードの開始
start_nodes() {
    local nodes=$1
    log_info "AIノードを開始中..."
    
    for node in $nodes; do
        log_info "ノード開始: $node"
        ros2 run ${node%_*} ${node%_*}_node --ros-args --params-file ${node}.yaml -r __node:=$node &
        sleep 2
    done
    
    log_info "AIノード開始完了"
}

# ノードの停止
stop_nodes() {
    local nodes=$1
    log_info "AIノードを停止中..."
    
    for node in $nodes; do
        log_info "ノード停止: $node"
        pkill -f "$node" || true
        sleep 1
    done
    
    log_info "AIノード停止完了"
}

# ノードの再起動
restart_nodes() {
    local nodes=$1
    log_info "AIノードを再起動中..."
    
    stop_nodes "$nodes"
    sleep 3
    start_nodes "$nodes"
    
    log_info "AIノード再起動完了"
}

# メイン処理
main() {
    local action=$1
    local ai_type=$2
    local camera=$3
    
    # 引数チェック
    if [[ $# -lt 3 ]]; then
        log_error "引数が不足しています"
        show_help
        exit 1
    fi
    
    # アクションの検証
    case $action in
        "start"|"stop"|"restart") ;;
        *) 
            log_error "無効なアクション: $action"
            show_help
            exit 1
            ;;
    esac
    
    # AIタイプの検証
    case $ai_type in
        "unet"|"yolo"|"both") ;;
        *)
            log_error "無効なAIタイプ: $ai_type"
            show_help
            exit 1
            ;;
    esac
    
    # カメラの検証
    case $camera in
        "d415"|"d405"|"both") ;;
        *)
            log_error "無効なカメラ: $camera"
            show_help
            exit 1
            ;;
    esac
    
    # ノード名の取得
    local nodes=$(get_node_names "$ai_type" "$camera")
    
    if [[ -z "$nodes" ]]; then
        log_error "指定された条件でノードが見つかりません"
        exit 1
    fi
    
    log_info "対象ノード: $nodes"
    
    # アクション実行
    case $action in
        "start")
            start_nodes "$nodes"
            ;;
        "stop")
            stop_nodes "$nodes"
            ;;
        "restart")
            restart_nodes "$nodes"
            ;;
    esac
}

# スクリプト実行
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi 