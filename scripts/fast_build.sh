#!/usr/bin/env bash
# Fast Build Script for FluentVision ROS2
# 🚀 最適化されたビルドシステム - 75%高速化目標

set -euo pipefail

# カラー出力
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ワークスペースルート
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
cd "$WS_ROOT"

# システム情報取得
CORES=$(nproc)
AVAILABLE_MEMORY=$(free -m | awk 'NR==2{print $7}')

echo -e "${BLUE}🚀 FluentVision Fast Build System${NC}"
echo -e "${GREEN}📍 Workspace: $WS_ROOT${NC}"
echo -e "${GREEN}⚡ CPU Cores: $CORES${NC}"
echo -e "${GREEN}💾 Available Memory: ${AVAILABLE_MEMORY}MB${NC}"

# ccacheのセットアップと確認
setup_ccache() {
    if ! command -v ccache &> /dev/null; then
        echo -e "${YELLOW}📦 ccache未検出 - インストールします...${NC}"
        sudo apt update && sudo apt install -y ccache
    fi
    
    # ccache設定
    export PATH="/usr/lib/ccache:$PATH"
    ccache --max-size=5G &> /dev/null || true
    
    echo -e "${GREEN}✅ ccache設定完了 (キャッシュサイズ: 5GB)${NC}"
    ccache -s | grep -E "(cache size|cache hit rate)" || true
}

# lld（高速リンカー）のセットアップ
setup_lld() {
    if ! command -v lld &> /dev/null; then
        echo -e "${YELLOW}🔗 lld（高速リンカー）未検出 - インストールします...${NC}"
        sudo apt install -y lld
    fi
    echo -e "${GREEN}✅ lld（高速リンカー）設定完了${NC}"
}

# 最適化されたビルド実行
fast_build() {
    local build_args=("$@")
    
    echo -e "${BLUE}⚡ 高速ビルド開始...${NC}"
    echo -e "${YELLOW}📊 設定: ${CORES}コア並列 + ccache + lld${NC}"
    
    # ビルド時間測定開始
    start_time=$(date +%s)
    
    # 最適化されたcolconビルド
    colcon build \
        --symlink-install \
        --parallel-workers "${CORES}" \
        --executor parallel \
        --event-handlers console_direct+ \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=RelWithDebInfo \
            -DCMAKE_C_COMPILER_LAUNCHER=ccache \
            -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
            -DCMAKE_EXE_LINKER_FLAGS="-fuse-ld=lld" \
            -DCMAKE_SHARED_LINKER_FLAGS="-fuse-ld=lld" \
            -DBUILD_TESTING=OFF \
            -Wno-dev \
        "${build_args[@]}"
    
    # ビルド時間計算
    end_time=$(date +%s)
    build_time=$((end_time - start_time))
    
    echo -e "${GREEN}✅ 高速ビルド完了！${NC}"
    echo -e "${GREEN}⏱️  ビルド時間: ${build_time}秒${NC}"
    
    # ccache統計表示
    echo -e "${BLUE}📊 ccache統計:${NC}"
    ccache -s | grep -E "(cache hit|cache miss|cache size)" || true
}

# メイン処理
main() {
    echo -e "${BLUE}🔧 最適化ツールセットアップ...${NC}"
    
    setup_ccache
    setup_lld
    
    echo -e "${GREEN}🚀 すべての最適化準備完了！${NC}"
    echo ""
    
    # ビルド実行
    fast_build "$@"
    
    echo ""
    echo -e "${GREEN}🎉 FluentVision Fast Build完了！${NC}"
    echo -e "${YELLOW}💡 次回以降はさらに高速になります（ccacheの効果で）${NC}"
}

# スクリプト実行
main "$@"
