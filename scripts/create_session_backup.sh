#!/usr/bin/env bash
# scripts/create_session_backup.sh
# AIセッション記憶保存・新チャット移行支援スクリプト

set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
cd "$WS_ROOT"

# セッション記録ディレクトリの作成
HISTORY_DIR="_history"
SESSION_DIR="$HISTORY_DIR/sessions"
FAILURE_DIR="$HISTORY_DIR/failures"
KNOWLEDGE_DIR="$HISTORY_DIR/knowledge"

mkdir -p "$SESSION_DIR" "$FAILURE_DIR" "$KNOWLEDGE_DIR"

# 現在日時の取得
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
SESSION_FILE="$SESSION_DIR/${TIMESTAMP}_session_backup.md"

echo "🧠 AIセッション記憶保存開始..."

# セッション記録ファイルの作成
cat > "$SESSION_FILE" << 'EOF'
# AI セッション記録

## 📊 セッション基本情報
- **作成日時**: AUTO_TIMESTAMP
- **作業ディレクトリ**: AUTO_WORKSPACE  
- **主要作業**: [AIが記入する]
- **セッション状態**: [完了/中断/移行準備]

## 🎯 今回の作業目標
- [ ] [目標1]
- [ ] [目標2]  
- [ ] [目標3]

## ✅ 完了した作業
### 成功事例
- **作業1**: [詳細・関連ファイル]
- **作業2**: [詳細・関連ファイル]

### 作成・変更したファイル
- `filepath1`: [変更内容]
- `filepath2`: [変更内容]

## ❌ 失敗・未完了の作業
### 遭遇した問題
- **問題1**: [詳細・エラーメッセージ・試行回数]
- **問題2**: [詳細・エラーメッセージ・試行回数]

### 試行したが失敗した解決策
- [解決策1]: [失敗理由]
- [解決策2]: [失敗理由]

## 🧠 重要な発見・学習事項
### 技術的発見
- [発見1]: [詳細・応用可能性]
- [発見2]: [詳細・応用可能性]

### プロジェクト理解の深化
- [理解1]: [詳細]
- [理解2]: [詳細]

## 🚨 注意事項・制約
### 絶対に避けるべき行動
- [禁止行動1]: [理由]
- [禁止行動2]: [理由]

### 重要な制約・制限
- [制約1]: [詳細]
- [制約2]: [詳細]

## 📋 次セッションへの申し送り

### 🎯 優先的に確認すべき事項
1. **プロジェクト基本ルール**
   - `.cursor/cursor_rules.md`
   - `.cursor/fluentvision_project_rules.md`
   - `.cursor/basic_development_rules.md`

2. **現在の作業状況**
   - `TODO*.md` - 最新の作業リスト
   - `README.md` - プロジェクト概要
   - この記録ファイル

### 💡 推奨される次のアクション
- [アクション1]: [理由・期待効果]
- [アクション2]: [理由・期待効果]

### ⚠️ 回避すべき手法
- [手法1]: [失敗経験・リスク]
- [手法2]: [失敗経験・リスク]

### 🔍 有用な情報源
- [情報源1]: [内容・活用方法]
- [情報源2]: [内容・活用方法]

## 🤖 次AI担当者への特別メッセージ
```
私は以下の理由で記憶を移行します：
[理由: コンテキスト制限/複雑性過多/失敗ループ/etc.]

特に注意してほしいのは：
1. [注意点1]
2. [注意点2]  
3. [注意点3]

成功させるために：
- [成功要因1]
- [成功要因2]

この記録が役立つことを願っています。
FluentVisionの「Flow Like Water」精神で、共に最高のシステムを作りましょう！
```

---
**🌟 記録者**: [AI名/ID]  
**🎯 継続ミッション**: FluentVision ROS2で世界最高の映像配信プラットフォームを構築
EOF

# プレースホルダーの置換
sed -i "s/AUTO_TIMESTAMP/$TIMESTAMP/g" "$SESSION_FILE"
sed -i "s|AUTO_WORKSPACE|$WS_ROOT|g" "$SESSION_FILE"

echo "📝 セッション記録ファイルを作成しました："
echo "   $SESSION_FILE"

# 関連情報の自動収集
echo ""
echo "📊 現在の作業状況を自動収集中..."

# プロジェクト状況のスナップショット
STATUS_FILE="$SESSION_DIR/${TIMESTAMP}_project_status.md"
cat > "$STATUS_FILE" << EOF
# プロジェクト状況スナップショット - $TIMESTAMP

## 📁 ディレクトリ構造
EOF

# 主要ディレクトリの構造を記録
echo '```' >> "$STATUS_FILE"
tree -L 3 -a -I '.git|node_modules|build|install|log' >> "$STATUS_FILE" 2>/dev/null || ls -la >> "$STATUS_FILE"
echo '```' >> "$STATUS_FILE"

# Gitステータス
echo -e "\n## 🔄 Git状況" >> "$STATUS_FILE"
echo '```' >> "$STATUS_FILE"
git status --porcelain 2>/dev/null >> "$STATUS_FILE" || echo "Git情報なし" >> "$STATUS_FILE"
echo '```' >> "$STATUS_FILE"

# 最近の変更ファイル
echo -e "\n## 📝 最近の変更ファイル (24時間以内)" >> "$STATUS_FILE"
echo '```' >> "$STATUS_FILE"
find . -name "*.cpp" -o -name "*.hpp" -o -name "*.py" -o -name "*.md" -o -name "*.sh" | \
  xargs ls -lt 2>/dev/null | head -20 >> "$STATUS_FILE" || echo "変更ファイル情報なし" >> "$STATUS_FILE"
echo '```' >> "$STATUS_FILE"

# TODOファイルの内容
echo -e "\n## 📋 現在のTODOリスト" >> "$STATUS_FILE"
find . -name "TODO*.md" -o -name "TODO*.txt" 2>/dev/null | while read -r todo_file; do
  echo -e "\n### $(basename "$todo_file")" >> "$STATUS_FILE"
  echo '```' >> "$STATUS_FILE"
  cat "$todo_file" >> "$STATUS_FILE" 2>/dev/null || echo "読み込みエラー" >> "$STATUS_FILE"
  echo '```' >> "$STATUS_FILE"
done

echo "📊 プロジェクト状況スナップショットを作成しました："
echo "   $STATUS_FILE"

# 新チャット移行ガイドの作成
HANDOVER_FILE="$SESSION_DIR/${TIMESTAMP}_new_chat_guide.md"
cat > "$HANDOVER_FILE" << 'EOF'
# 🤖 新AI担当者向け クイックスタートガイド

## ⚡ 緊急確認事項 (最初の5分で実行)

### 1️⃣ プロジェクト基本理解
```bash
# FluentVision ROS2の6つのコア価値を確認
cat README.md | head -50

# 統合ルールの確認 
cat .cursor/cursor_rules.md

# プロジェクト専用ルールの確認
cat .cursor/fluentvision_project_rules.md
```

### 2️⃣ 現在の作業状況確認
```bash
# 最新のセッション記録を確認
ls -lt _history/sessions/ | head -3

# 現在のTODOを確認
find . -name "TODO*.md" | xargs cat

# 最近の変更を確認
git log --oneline -10
```

### 3️⃣ 前任AIからの申し送り確認
- このファイルと同じディレクトリの `*_session_backup.md` を**必ず読む**
- 失敗事例があれば `_history/failures/` も確認
- 重要な制約・禁止事項を把握

## 🎯 FluentVision ROS2 の本質 (必須記憶事項)

### プロジェクトミッション
**「いかに便利に、楽に、簡単に、スムースに、高速に映像や情報を美しくさまざまなコンピュータに配信できるか」**

### 6つのコア価値
1. **🛠️ 便利（Convenient）** - ワンライン起動・自動設定
2. **😌 楽（Easy）** - 設定レス・自動回復
3. **🎯 簡単（Simple）** - 直感的API・1行完結
4. **🌊 スムース（Smooth）** - フレーム落ちゼロ・適応的品質
5. **⚡ 高速（Fast）** - C++最適化・GPU加速
6. **🎨 美しい（Beautiful）** - エレガントAPI・美しい映像品質

### 技術スタック
- **ベース**: ROS2 Humble + C++17
- **センサー**: Intel RealSense (D415, D405)
- **AI/ML**: YOLOv10 (物体検出), UNet (セグメンテーション)
- **映像**: OpenCV, PCL
- **配信**: MJPEG, WebSocket, RTMP
- **新技術**: Unity, XREAL

## 🚫 絶対禁止事項 (即座に記憶)

### ファイル管理
- `xxx_final.cpp` などの悪いファイル命名
- メインディレクトリへのテスト/デバッグファイル作成
- 未テストファイルのGitコミット

### コーディング
- Python本番コード使用（C++は100倍速）
- 車輪の再発明・既存ライブラリの再実装
- 設計書なしの「いきなり実装」

### AI行動
- **🔒 ターミナルロック**（長時間コマンドの同期実行）
- 同一失敗の2回以上繰り返し
- コンテキスト制限の無視

## ✅ 必須実行事項

### 作業開始時
- **🧠 記憶再構築・報告** - プロジェクト理解を確認・報告
- **⏰ コマンド実行前の時間予測** - タイムアウト設定・バックグラウンド化
- **💡 能動的最適化提案** - より良い方法の積極的提案

### 開発フロー
- 実装前の多言語検索（英語・日本語・中国語）
- 設計書→仕様確定→協議→実装のフロー
- 1クラス1ファイル・20行以内関数
- 抽象基底クラスによる将来拡張性確保

## 🔄 作業継続のための重要ヒント

### FluentLib統合の確認
```bash
# FluentAPIの確認
cat src/common/fluent_lib/include/fluent.hpp
cat src/common/fluent_lib/include/image.hpp
cat src/common/fluent_lib/include/cloud.hpp
```

### 高速ビルドシステムの活用
```bash
# 高速ビルド（推奨）
./scripts/fast_build.sh

# 性能測定
./scripts/build_benchmark.sh
```

### パッケージ構造の理解
```
src/
├── ai/          # YOLOv10, UNet, アスパラ分析
├── sensors/     # RealSense, カメラ
├── streaming/   # MJPEG, WebSocket, 録画
├── utils/       # フィルタ、リレー
└── common/      # fluent_lib (重要!)
```

## 💪 成功のための心構え

1. **Flow Like Water**: 思考がそのままコードに表現される
2. **短いコードは正義**: 20行以内の関数を心がける
3. **抽象化の美学**: 将来の拡張性を常に考慮
4. **失敗を恐れない**: 迅速な試行錯誤とループ防止
5. **謙虚な協力**: 限界を認識し、適切にエスカレーション

---

**🌟 引き継ぎメッセージ**: FluentVisionの未来は君の手に！6つのコア価値を体現し、世界最高の映像配信プラットフォームを共に築こう。困ったときは記録を確認し、必要なら新しい知性と協力することを恐れずに。

**🎯 今すぐ実行**: まず前任者のセッション記録を読み、プロジェクト理解を報告してください！
EOF

echo ""
echo "🤖 新AI担当者向けガイドを作成しました："
echo "   $HANDOVER_FILE"

# 使用方法の表示
echo ""
echo "🎯 使用方法："
echo ""
echo "📝 AIエージェント向け："
echo "   1. セッション記録ファイルを編集して作業内容を記録"
echo "   2. 新チャット移行時は記録ファイルを参照"
echo "   3. 失敗ループ検知時は _history/failures/ に記録"
echo ""
echo "👤 ユーザー向け："
echo "   1. AIが記憶移行を提案した際にこのスクリプトを実行"
echo "   2. 新AIには記録ファイルの場所を教える"
echo "   3. 重要な制約・禁止事項を伝える"
echo ""
echo "🔄 継続性："
echo "   - 定期的に _history/ ディレクトリをバックアップ"
echo "   - 成功事例は _history/knowledge/ に蓄積"
echo "   - 失敗パターンは _history/failures/ で共有"

echo ""
echo "✅ セッション記憶保存システムの準備完了！"
