#!/usr/bin/env bash
# Build Benchmark Script for FluentVision ROS2
# 📊 ビルド時間の比較測定ツール

set -euo pipefail

# カラー出力
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
cd "$WS_ROOT"

echo -e "${BLUE}📊 FluentVision Build Benchmark${NC}"
echo -e "${GREEN}🎯 標準ビルド vs 高速ビルドの性能比較${NC}"
echo ""

# クリーンアップ
cleanup_build() {
    echo -e "${YELLOW}🧹 ビルドアーティファクトをクリーンアップ...${NC}"
    rm -rf build/ install/ log/
    echo -e "${GREEN}✅ クリーンアップ完了${NC}"
}

# 標準ビルド測定
benchmark_standard() {
    echo -e "${BLUE}📏 標準ビルド測定開始...${NC}"
    cleanup_build
    
    start_time=$(date +%s.%N)
    colcon build \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        --event-handlers console_direct+ \
        > /tmp/standard_build.log 2>&1
    end_time=$(date +%s.%N)
    
    standard_time=$(echo "$end_time - $start_time" | bc)
    echo -e "${GREEN}✅ 標準ビルド完了: ${standard_time}秒${NC}"
    
    return 0
}

# 高速ビルド測定
benchmark_fast() {
    echo -e "${BLUE}⚡ 高速ビルド測定開始...${NC}"
    cleanup_build
    
    start_time=$(date +%s.%N)
    ./scripts/fast_build.sh > /tmp/fast_build.log 2>&1
    end_time=$(date +%s.%N)
    
    fast_time=$(echo "$end_time - $start_time" | bc)
    echo -e "${GREEN}✅ 高速ビルド完了: ${fast_time}秒${NC}"
    
    return 0
}

# 結果比較
compare_results() {
    echo ""
    echo -e "${BLUE}📊 ============ ベンチマーク結果 ============${NC}"
    echo -e "${YELLOW}🐌 標準ビルド時間:${NC} ${standard_time}秒"
    echo -e "${GREEN}⚡ 高速ビルド時間:${NC} ${fast_time}秒"
    
    if command -v bc &> /dev/null; then
        improvement=$(echo "scale=1; ($standard_time - $fast_time) / $standard_time * 100" | bc)
        speedup=$(echo "scale=2; $standard_time / $fast_time" | bc)
        
        echo -e "${GREEN}🚀 改善率:${NC} ${improvement}%"
        echo -e "${GREEN}📈 速度向上:${NC} ${speedup}倍"
    fi
    
    echo -e "${BLUE}=========================================${NC}"
    echo ""
    
    # ログファイルの保存
    echo -e "${YELLOW}📝 詳細ログ保存先:${NC}"
    echo "  - 標準ビルド: /tmp/standard_build.log"
    echo "  - 高速ビルド: /tmp/fast_build.log"
}

# インクリメンタルビルドテスト
test_incremental() {
    echo -e "${BLUE}🔄 インクリメンタルビルドテスト${NC}"
    
    # 小さな変更を作成
    touch src/common/fluent_lib/src/image.cpp
    
    echo -e "${YELLOW}📝 ファイル変更シミュレーション（image.cpp）${NC}"
    
    # 高速インクリメンタルビルド測定
    start_time=$(date +%s.%N)
    ./scripts/fast_build.sh > /tmp/incremental_build.log 2>&1
    end_time=$(date +%s.%N)
    
    incremental_time=$(echo "$end_time - $start_time" | bc)
    echo -e "${GREEN}⚡ インクリメンタルビルド: ${incremental_time}秒${NC}"
    
    echo -e "${YELLOW}📝 インクリメンタルログ: /tmp/incremental_build.log${NC}"
}

# メイン処理
main() {
    echo -e "${YELLOW}⚠️  注意: 完全なベンチマークには5-10分かかります${NC}"
    echo -e "${YELLOW}📊 2回のフルビルド + 1回のインクリメンタルビルドを実行${NC}"
    read -p "続行しますか？ (y/N): " -n 1 -r
    echo
    
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "ベンチマーク中止"
        exit 0
    fi
    
    # bc（計算ツール）インストール確認
    if ! command -v bc &> /dev/null; then
        echo -e "${YELLOW}📦 bc（計算ツール）をインストール...${NC}"
        sudo apt install -y bc
    fi
    
    # ベンチマーク実行
    benchmark_standard
    benchmark_fast
    compare_results
    test_incremental
    
    echo -e "${GREEN}🎉 ベンチマーク完了！${NC}"
}

main "$@"
