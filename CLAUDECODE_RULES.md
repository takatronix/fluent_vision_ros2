# 🤖 ClaudeCode用 FluentVision ROS2 統合開発ルール

## 🎯 プロジェクトの本質
**ミッション**: 「いかに便利に、楽に、簡単に、スムースに、高速に映像や情報を美しくさまざまなコンピュータに配信できるか」

### 6つのコア価値（絶対遵守）
1. **🛠️ 便利** - ワンライン起動、自動設定
2. **😌 楽** - 設定レス、自動回復
3. **🎯 簡単** - 直感的API、1行完結
4. **🌊 スムース** - フレーム落ちゼロ、適応的品質
5. **⚡ 高速** - C++最適化、GPU加速
6. **🎨 美しい** - エレガントAPI、美しい映像品質

## 📋 ClaudeCode作業開始チェックリスト

### 🧠 記憶再構築（必須実行）
```bash
# 1. プロジェクト状況確認
cat /home/aspara/seedbox-r1/fluent_vision_ros2/README.md
find /home/aspara/seedbox-r1/fluent_vision_ros2 -name "TODO*.md" -o -name "TODO*.txt" | xargs cat

# 2. 最新の作業履歴確認
ls -la /home/aspara/seedbox-r1/fluent_vision_ros2/_history/sessions/

# 3. git状況確認
cd /home/aspara/seedbox-r1/fluent_vision_ros2 && git status
```

### 報告テンプレート
```
🧠 **記憶再構築完了**
- ✅ ドキュメント確認: [確認した文書]
- ✅ 履歴確認: [前回作業内容]
- ✅ 現状把握: [現在の状況]
- ⚠️ 不明点: [確認必要事項]
```

## 🚫 絶対禁止事項

### ファイル・ディレクトリ
- `xxx_final.cpp`などの悪い命名
- メインディレクトリへのテスト/デバッグファイル作成
- 車輪の再発明・既存ライブラリの再実装
- 設計書なしの「いきなり実装」
- 未テストファイルのGitコミット
- Python本番コード使用（C++は100倍速）
- src/内への.mdファイル直接配置

### ROS2システム起動
- **単体ノード起動禁止**: `ros2 run`での個別起動は不可
- **必須**: `./launch/start_fv.sh`でシステム全体を起動

### コマンド実行
- **ターミナルロック禁止**: 長時間コマンドの同期実行
- **タイムアウト必須**: 10秒以内、失敗時は30秒まで延長

## ⏰ コマンド実行ルール

### 実行前チェックリスト
```
❓ このコマンドは30秒以上かかる可能性があるか？
❓ ネットワーク・IO・ビルド処理を含むか？
❓ ユーザー入力待ちの可能性があるか？
❓ 無限ループ・デッドロックの危険性があるか？
```

### 実行方式
- **短時間（<30秒）**: 同期実行OK
- **中時間（30秒-5分）**: `timeout 300`必須
- **長時間（>5分）**: バックグラウンド実行必須
- **危険コマンド（ROS2ノード等）**: 事前確認・バックグラウンド必須

### 実行例
```bash
# 短時間
ls -la

# 中時間
timeout 300 colcon build --packages-select package_name

# 長時間
colcon build --symlink-install &
BUILD_PID=$!
echo "ビルド実行中 (PID: $BUILD_PID)"
echo "停止方法: kill $BUILD_PID"

# ROS2ノード（危険）
ros2 run package_name node_name &
NODE_PID=$!
echo "ノード起動完了 (PID: $NODE_PID)"
echo "停止方法: kill $NODE_PID"
```

## 🔄 失敗ループ防止

### 自動検知トリガー
- 同一エラーを2回経験 → 別アプローチ検討
- 同一ファイルを3回連続編集 → 作業停止・再評価
- 進捗なしに20分経過 → 状況報告・方針転換

### ループ検出時の対応
```
🔄 **失敗ループ検出**
1. 即座に作業停止
2. 過去の解決策を検索
3. 代替アプローチの提案
4. 必要なら専門エージェントへ相談
```

## 📊 コンテキスト管理

### 制限検知レベル
- **注意（20回）**: 効率低下の可能性
- **警戒（30回）**: 新チャット移行検討
- **緊急（40回）**: 即座に移行推奨

### 新チャット移行時の申し送り
```markdown
## 申し送り事項
- 目標: [具体的な目標]
- 進捗: [現在の状況]
- 課題: [解決すべき問題]
- 制約: [重要な制約]
- 失敗: [避けるべき手法]
- 推奨: [推奨アプローチ]
```

## 🔧 開発フロー

### 必須ステップ
1. **設計書作成** → docs/design/
2. **仕様確定** → 協議・レビュー
3. **TODO管理** → TodoWriteツール使用
4. **実装前検索** → 英語・日本語・中国語
5. **実装** → 1クラス1ファイル、関数20行以内
6. **テスト** → ビルド・単体・統合テスト
7. **ドキュメント更新** → README.md必須

### ビルドコマンド
```bash
cd /home/aspara/seedbox-r1/fluent_vision_ros2

# 標準ビルド
colcon build --symlink-install

# 特定パッケージ
colcon build --packages-select [package_name]

# fluent_lib更新時（必須）
colcon build --packages-select fluent_lib
source install/setup.bash
```

### システム起動
```bash
# 起動（必須方法）
./script/start_fv

# 停止
./script/stop_fv

# 状態確認
ros2 topic list
ros2 node list
```

## 💡 能動的最適化

### AIの義務
- より効率的な方法を発見したら即座に提案
- 非効率な手法を発見したら代替案を提示
- 最新技術・ツールを積極的に紹介
- リスク・問題の早期警告

### 提案テンプレート
```
💡 **より良い方法を発見**
現在: [現在の方法]
提案: [改善方法]
メリット: 
- ⏰ 時間: X分→Y分
- 🚀 性能: [具体的数値]
試してみますか？
```

## 📝 ドキュメント作成

### 実装ログ（/seedbox-r1/_docs/）
```bash
# 日付確認（必須）
date

# ファイル名形式
yyyymmdd/hhmm_種別_機能名.md

# 種別
- plan: 計画ログ
- fix: 解決ログ
- report: レポート
- memo: 備忘録
- halfway: 途中経過
- [IMPORTANT]: 重要マーカー
```

### 作成前の確認
**必ずユーザーに確認すること**

## 🏗️ FluentLib設計原則

### 美しいAPI例
```cpp
// Image処理
Image::load("input.jpg")
    .resize(640, 480)
    .blur(2.0)
    .bright(1.2)
    .save("output.jpg");

// Cloud処理  
Cloud::fromRGBD(rgb, depth)
    .filter(0.1, 5.0)
    .downsample(0.01)
    .publish("/processed_cloud");

// 配信
UniversalBroadcaster::from(camera)
    .to_web("http://localhost:8080")
    .to_mobile("rtmp://mobile")
    .to_unity("ws://unity")
    .start();
```

## 🚨 トラブルシューティング

### カメラ接続問題
```bash
pkill -f realsense
# デバイス再接続
sudo chmod 666 /dev/video*
```

### ビルドエラー
```bash
# キャッシュクリア
rm -rf build/ install/ log/
colcon build --symlink-install
```

### ROS2デッドロック回避
- CallbackGroupの使用
- MultiThreadedExecutorの使用
- サービス呼び出し時のスピン処理

## 📊 性能目標
- メモリ使用量: 5-10MB（現在10-20MB）
- 処理時間: 50-100ms（現在100-200ms）
- C++ vs Python: 100倍速

## 🔗 重要トピック

### カメラストリーム
- `/fv/d415/color/image_raw`
- `/fv/d405/depth/image_rect_raw`

### AI処理結果
- `/fv/object_detection/result`
- `/fv/aspara_detection/result`
- `/fv/aspara_analysis/result`

### サービス
- `/fv_realsense/set_mode`
- `/fv_realsense/get_distance`
- `/fv_realsense/get_point_cloud`

## 🌟 作業の心得

### Flow Like Water哲学
- 思考がそのままコードに表現される
- 障害に遭遇したら迂回路を見つける
- 必要に応じて器を変える（新チャット移行）
- 水晶のように透明な責任分散

### 品質判断基準
「このコードを読んで美しいと感じるか？」

### 継続的改善
- 月次での技術動向調査
- ベストプラクティスの更新
- プロジェクト固有知識の蓄積

---

**統合ミッション**: FluentVisionの"Flow Like Water"哲学で、世界最高の映像配信プラットフォームを構築する。AIの限界を謙虚に受け入れ、記憶保存・エスカレーション・協力により、継続的に高品質なソリューションを提供する。