# D405 30 FPS 安定化 (PR #14 + PR #15) 統合検証レポート

## 目的

PR #14 (publish専用スレッドのFIFO化とDDS受信バッファ設定) と
PR #15 (D405の30 FPS配信と録画の安定化) を1本に統合したブランチについて、
x86_64 (Intel) と arm64 (Jetson Thor) の両アーキテクチャでビルド・テスト・
コードレビューによる総合検証を行った結果を記録する。

- 統合ブランチ: `integration/d405-stream-stability-pr14-15`
- 統合方法: `main` (PR #14 マージ済み `c262f28`) に
  `fix/d405-stream-stability` (`362294c`) をマージ。コンフリクトなし。
- 統合後のツリーは PR #15 ブランチ先端のツリーと一致する
  (PR #15 は PR #14 の先端に1コミットを積んだ stacked PR のため)。

## 変更内容の要約

| 領域 | 内容 |
|---|---|
| `fluent_lib` | `configure_current_io_thread()` — publish専用スレッドへ `SCHED_FIFO:20` を適用。権限なしは warning を出して `SCHED_OTHER` 継続 (silent fallback ではない) |
| `fluent_lib/scripts` | `setup_dds_receive_buffer.sh` (`net.core.rmem_max` ≥ 16 MiB)、`setup_dds_send_buffer.sh` (`net.core.wmem_max` ≥ 16 MiB)。それぞれ別の `/etc/sysctl.d` ファイルへ永続化 |
| `fv_realsense` | color / depth の processing スレッド分離、color raw / compressed color / depth ごとの容量1 publish queue + 専用 publish スレッド、ストリーム別 drop counter、device timestamp 単調性の stream 別分離 |
| `fv-uvcvideo` | `UVC_URBS=32` の uvcvideo モジュールをビルド・導入する任意ツール。Jetson Linux R38.4 / Ubuntu (x86_64・arm64) 対応、vermagic 検証、override 保存/復元、MOK 署名対応 |
| docs | `ros_io_thread_scheduling.ja.md`、`fv_realsense_d405_streaming.ja.md` (設計・実測値・導入手順) |

## ビルド・テスト結果

### arm64 — Jetson AGX Thor (Jetson Linux R38.4, kernel 6.8.12-tegra, ROS 2 Jazzy)

- `colcon build --packages-up-to fv_realsense` (Release): **成功**
  (警告は既存の未使用変数と、環境由来の OpenCV 4.6/4.8 リンカ警告のみ)
- `ctest`:
  - `test_ros_io_thread` (gtest 2件): **合格**
  - `test_setup_dds_receive_buffer`: **合格**
  - `test_setup_dds_send_buffer`: **合格**
  - `test_fv_uvcvideo`: **合格**

### x86_64 — RTX 4090 マシン (Ubuntu, Docker `ros:jazzy-ros-base`)

- `rosdep install --from-paths src --ignore-src` による依存解決: **成功**
- `colcon build --packages-up-to fv_realsense` (Release): **成功**
- `ctest`:
  - `test_ros_io_thread` (gtest 2件): **合格**
  - `test_setup_dds_receive_buffer`: **合格**
  - `test_setup_dds_send_buffer`: **合格**
  - `test_fv_uvcvideo`: **合格**

両アーキテクチャでビルド・機能テストとも同一結果。アーキテクチャ依存の
コード分岐は `fv-uvcvideo` の profile 検出のみで、C++ / sysctl スクリプトは
POSIX 標準 API・共通 sysctl 名のみを使用しており可搬。

### lint について

`ament_lint` 系 (uncrustify / cpplint / pep257 / copyright / xmllint) は
リポジトリ全体で既存違反 900 件超により従来から失敗しており、本統合はそれを
新たに悪化させる変更ではない (PR #15 本文にも既存違反947件と明記)。
PR #14 で追加された `ros_io_thread.cpp` 等にも copyright ヘッダ欠如などの
lint 違反があるが、これらは既に `main` へマージ済みの内容である。

## コードレビュー (総合調査) の結果

3系統 (スレッド化 / fv-uvcvideo 可搬性 / fluent_lib+スクリプト) の精査を実施。
**[重大] は 0 件**。以下は把握しておくべき指摘。

### 挙動変更 (下流影響の可能性)

- **color/depth の ROS タイムスタンプが独立化** — 旧 mode2 は depth メッセージに
  color の stamp を付けていたが、新実装は各ストリームが自分の device timestamp
  由来 stamp を持つ。より正確になる一方、下流で `message_filters::ExactTime`
  による color/depth 厳密同期をしている場合はマッチしなくなる
  (ApproximateTime なら問題なし)。既知の下流 (VLAbor recorder 等) は
  実機60秒×3回収録で欠損ゼロを確認済み (PR #15 本文)。

### 運用注意 ([中])

- `fv-uvcvideo status` は override ファイル自身の `modinfo` を読むだけで、
  kernel が実際に選択するモジュールとの突合をしない。depmod の `updates/`
  優先が効かない環境では誤表示しうる。
- `fv-uvcvideo` の apt 解決は実行中カーネルと完全一致の版数をピンするため、
  カーネル更新後未再起動などで archive から当該版が消えていると失敗する
  (最新カーネルへ更新・再起動済みが前提)。
- `fv-uvcvideo install` 途中で `depmod -a` が失敗すると、モジュール設置済みだが
  管理記録未生成の状態になり、正規の uninstall が拒否される (発生確率は低い)。
- sysctl スクリプトは他の `/etc/sysctl.d/*.conf` に同一 key の低い値があっても
  実行時警告しない (辞書順後勝ちで負ける可能性は設計 doc に文章で記載済み)。
- 本機 (Thor) は Jetson Linux **R38.2.2** のため `fv-uvcvideo` は
  unsupported 判定になる (対応は R38.4 と generic Ubuntu)。誤って
  generic 経路へ流れない Tegra ガードが効いていることは実機確認済み。

### 軽微

- 処理スレッド停止通知に lost-wakeup の窓があるが、`wait_for` 1000ms で
  必ず再評価されるため最悪でも約1秒の停止遅延に留まる (恒久ハングなし)。
- ストリーム別 time state のリセットが相互リセットのため、直後の1フレームのみ
  単調クランプがスキップされうる。
- gtest が権限ある環境ではテストプロセスのメインスレッドを FIFO:20 に
  昇格させたままにする (テスト内容上、実害なし)。

### 問題なしと確認できた点 (抜粋)

- データ競合・デッドロック・use-after-free なし。bundle は全てコピー済み
  ROS メッセージで rs2::frame の生ポインタはスレッド境界を越えない。
- デストラクタの join 順序 (処理→publisher)、publisher の queue ドレイン、
  初期化失敗時のクリーンアップは正しい。
- 容量1 queue の drop カウンタは latest-wins 意味論として厳密。
- SCHED_FIFO 権限なしは WARN ログ + SCHED_OTHER 継続 (silent fallback でない)。
- sysctl receive/send は別ファイル・別 key で相互上書き事故なし。
- `fv-uvcvideo` は distro モジュール非破壊 (updates/ への override 設置のみ)、
  sha 記録・vermagic 検証・MOK 署名強制検出あり。
- ドキュメント (`ros_io_thread_scheduling.ja.md` / `fv_realsense_d405_streaming.ja.md`)
  とコードの数値・ログ文言・ファイル名は全て一致。

## 運用上の注意

- `SCHED_FIFO` を有効にするには、コンテナは `CAP_SYS_NICE`、
  systemd service は `LimitRTPRIO=20` 以上が必要。なくても起動は継続する。
- sysctl スクリプトは `sudo` で実行し、適用後に DDS プロセス再起動が必要。
- `fv-uvcvideo` は任意導入。導入時はカーネル更新のたびに再ビルドが必要。
