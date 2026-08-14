# Changelog

## 0.9.0 — 2026-08-14 (Phase L3: インスペクター + ROS binding + scene_node)

- **シーンインスペクター**（§13-3、`scene/inspector.hpp`）: 全レイヤーを
  ステージ空間に固定した配置表（resolved bounds・local→stage 変換・実効
  opacity・描画順）。`visibleAt` が「座標 (x,y) に見えているものは何か」を
  最前面から返し、`inspectJson` / `atJson` が JSON で答える。リンターは
  この同じ配置計算の上の検査になった（幾何の定義は一箇所）
- scene_web に `/inspect` と `/at?x=&y=` を追加（配置スナップショットは
  値のみ共有 — HTTP スレッドはライブシーンに触らない）
- **binding 文書**（`fluent.binding/v1alpha1`、`scene/binding.hpp`）:
  「何が要るか」（Scene の inputs）と「この機体でどこから来るか」（topic /
  message_type / qos / converter）の分離。converter カタログは単一表、
  message_type の適合・adapter・QoS 語彙を activation 前に全て検証。
  `validateBindingAgainstScene` が未宣言入力(エラー)・型不適合(エラー)・
  未バインド入力(情報: fallback 提示)を判定
- **scene_node**（本番 ROS 2 アダプター）: scene.fvs + binding.fvb →
  購読(Image / CompressedImage / Detection2DArray / String / Polygon)、
  変換はレンダースレッドで latest-wins、描画(Vulkan、無ければ CPU)、
  出力 Surface を Image / CompressedImage で publish。scene は
  ホットリロード(検証→フレーム境界 swap)、binding は寿命固定(配線変更 =
  再起動、設計どおり)
- 実機 E2E: d405 CompressedImage 30fps + String トピック → 合成 20fps を
  `/fluent_scene/composite` に配信、ライブ文字列と実映像を確認
- **`aspa_json_to_detection2d` converter**: aspa 認識スタックの独自 JSON
  (`class` / `conf` / `box_xyxy` 画像ピクセル座標、`\uXXXX` エスケープの
  日本語ラベル含む)を boxes へ。image_size でステージ論理座標に変換。
  実機 d405 + 検出枠 HUD(角丸・ラベル・スコア・平滑化)を描画確認

## 0.8.1 — 2026-08-14 (scene_web: 編集→保存→原子的差し替え)

- `tools/scene_web` — Scene 文書(.fvs)のライブプレビューサーバー。ファイルを
  保存すると validate → compile → lint を通し、**フレーム境界で原子的に
  差し替え**（§2 の activate）。壊れた編集は旧画面を保持したまま赤バナーで
  エラー行を表示 — 壊れたフレームが表に出る経路は存在しない
- `--image input=topic` で ROS 2 CompressedImage を宣言済み `$inputs.<名前>`
  へ接続（ROS 環境を source してビルド）。未接続の image 入力は
  プレースホルダーパネルのまま = 入力契約がそのままデモになる
- GET /status が digest・リロード回数・エラー・lint 警告を JSON で返す
  （ページの状態表示はこれを映すだけ）

## 0.8.0 — 2026-08-14 (Phase L2: Scene 宣言層)

- **Scene v1alpha2**（`fluent_stage::scene`）: レイヤーツリー YAML（.fvs）
  → Stage 構築。§1.3 の契約を実装 — 型検査・未知キー拒否・`$inputs` /
  `$params` 参照解決・必須/排他フィールド検査を**実行前に全て**行う
  （`parseScene` が ok なら compile は資源上限以外で失敗しない）
- **同一出力の契約**: コンパイラは C++ 著者と同じ Layer API で木を組み、
  明示されたフィールドだけを適用する（既定値の定義は C++ 側に一度だけ）。
  §2 の canonical 例を YAML と C++ の両方で描き**ピクセル一致**を
  scene_tests の golden で検証
- **並べ替え不変 digest + 正準フォーマッタ**（§13-8）: 正準形は一つ
  （表順のキー・明示フィールドのみ・安定数値表記）。digest はその
  SHA-256。マッピング順・コメント・空白の変更では変わらない
- **単一定義のメタデータ表**: `shared/contents_def.h`（content 13種の
  フィールド/型/既定値）と `shared/attributes_def.h`（§6.2 属性15種)を
  新設 — filters_def.h 方式。validation・describe・fmt の語彙は全て
  ここから導出、既定値は C++ 構造体とテストで突き合わせ
- **`fvsc` CLI**: `validate`（型検査+リンター+digest）/ `preview`
  （PPM レンダリング。未接続の image 入力は入力名入りプレースホルダー）/
  `fmt` / `digest` / `describe --json`
- **能力の自己記述**（§13-1）: `describe --json` が content/属性/
  フィルタ30種/入力・パラメータ型/上限を機械可読カタログで返す
- **デザインリンター**（§13-2）: コントラスト比 4.5:1（実レンダリング
  ピクセルに対して測定）・テキストはみ出し・完全遮蔽（protected の
  遮蔽はエラー = §13-4）・画面外配置
- **ランタイム表面**: `CompiledScene` — `setImage/setText/setPoints/
  setBoxes`（宣言型チェック+容量クランプ）と `setParam`（§9 の animate
  宣言 → Transaction、layer transition フォールバック）。image 入力の
  fallback は placeholder / hide / hold
- YAML パーサ（依存ゼロの有界サブセット・fluent_scene から移植）に
  複数行フローコレクション対応を追加（§2 の canonical レイアウト）
- 設計書 §2 の YAML 例を修正: テキスト位置は content の `position`
  フィールドへ（レイヤー属性 position は anchor 配置なので同じ絵に
  ならない — golden が機械検証）

## 0.7.0 — 2026-08-14 (本物の水面屈折波紋)

- `fs_ripple` フィルタ（filters_shared.h、30種目）: 拡大する波面の周りの
  減衰正弦波でサンプリング位置を変位させ、**下の映像を実際に歪ませる**。
  CPU/GPU 同一ソース（GPU parity max_diff=0）
- FilterUnit に `CoordX`/`CoordY` を追加 — フィルタパラメータとして
  レイヤーローカルの論理座標を渡すと、レンダラーがバッファ uv へ変換
  （§5-3 の座標版）
- `fx::Ripple` を屈折駆動に全面書き換え: リングレイヤーを廃止し、対象
  レイヤー（=水面）のフィルタチェーンに波を積む方式へ。既存フィルタは
  base として保持、破棄時に原状復帰。`max_waves`（既定8）で有界
- stage_web: カメラ映像が水面に（クリック=スプラッシュ、ホバー=航跡）。
  デモ映像を畝のある圃場パターンに刷新

## 0.6.0 — 2026-08-14 (stage_web: ブラウザで触れる)

- `tools/stage_web` — Stage の Surface を MJPEG で配信し、ブラウザの
  mouse/touch/pointer イベントを論理座標に正規化して
  `stage.pointerDown/Move/Up` へ逆流させる自己完結 Web アプリ
  （POSIX ソケット + libjpeg のみ。ページ側 UI ロジックはゼロ）
- デモ HUD: 収穫開始/停止・ライトスイッチ+インジケーター・速度スライダー・
  モード segmented・プロファイル dropdown・バッテリーゲージ・検出枠・
  ホバー波紋。Vulkan バックエンドで描画（無ければ CPU にフォールバック）
- E2E 実証: HTTP 経由のポインタ注入だけで全コントロールが反応

## 0.5.0 — 2026-08-14 (Phase L1: Vulkan バックエンド)

- `VulkanRenderer` — CPU リファレンスと同一出力の GPU 本番バックエンド。
  全 golden シーンを GPU で通過（ほぼ全て max_diff 1〜2、閾値クリフを持つ
  フィルタのみ許容差内の残差）。1080p の代表 HUD で 5.6ms/frame
  （CPU 94.7ms、~17倍。毎フレームの CPU 読み戻し込み）
- 単一ソースの完結: shapes_shared.h / filters_shared.h が GLSL として
  SPIR-V にコンパイルされ GPU で実行される（ビルド時 glslc、実行時の
  シェーダーコンパイルはゼロ）
- CPU/GPU 共通のプラン層 src/render_shared.hpp（offscreen 判定・extent・
  dash 分割・フィルタ論理単位スケール）— 両バックエンドが同じ判断で
  ツリーを歩く
- ブレンド4種を premultiplied 固定機能式に統一（CPU 実装 = GPU ブレンド
  ファクタと恒等。Add/Multiply/Screen の意味論を両バックエンドで一致）
- golden_tests --renderer=vulkan（ctest: golden_tests_vulkan、GPU が無い
  環境ではスキップ）と examples/bench（CPU vs GPU 実測）
- 画像/フィルタのサンプリングを CPU とビット同型の texelFetch 実装に
  （pixelate のブロックずれ・sourceRect 境界の1texel差を根治）

## 0.4.0 — 2026-08-14 (波紋エフェクト + Dropdownスクロール)

- `fx::Ripple`（effects.hpp 新設）: ポインタの軌跡に広がって消える
  リング波紋（トレイル + タップの二重スプラッシュ）。円レイヤー +
  Transaction のみ、`max_rings` で有界、dt 駆動で決定的
- Dropdown: `max_visible` 超のリストをドラッグスクロール（6単位で
  タップ/ドラッグ弁別、開いた時に選択行へ自動スクロール）
- テスト: ripple 生成/間引き/上限/消滅、dropdown スクロール+選択。
  golden `ripple_t015`、example `ripple_demo`

## 0.3.0 — 2026-08-14 (UIカタログ完成)

- `ui::Slider`（0..1。ドラッグはポインタ直結・onChange連続、setValueは
  アニメ。fillはクリップ式で角丸を歪めない）
- `ui::Segmented`（2〜5択の排他選択。ピルがスライド、ラベル色が状態表示）
- `ui::Gauge`（表示専用ラジアル。`Layer::setArc` データ更新APIを追加）
- `ui::Dropdown`（root末尾ポップアップ+透明スクリムの外側タップ閉じ+
  上下自動開き+シェブロン回転。初版は max_visible で切りスクロールなし）
- ハンドラ実行中の自己remove（ポップアップが自分を閉じる）を安全化
  （配送時にハンドラをコピーして呼び出す）
- テスト: ui_tests に4コントロール分を追加、golden `ui_catalog`、
  example `ui_catalog`

## 0.2.0 — 2026-08-14 (Phase L4 先行: UIコントロール)

- ポインタ注入（§10-3）: `Stage::pointerDown/Move/Up/Cancel`。Web ビューア
  のクリック・タッチ・VR レイを論理座標の3呼び出しに正規化する統一入力口。
  キャプチャ追跡（UIControl 型）、`Layer::onPointer`、interactive hit-test
  （handler の無いサブツリーはポインタに対して透明 = disabled 素通し）
- `ui::Button`（momentary）と `ui::Switch`（トグル）: プレハブサブツリー +
  状態=属性上書き + Transaction 遷移（§10-1/2）。スタイルは
  ButtonStyle / SwitchStyle で差し替え
- テスト: ui_tests（タップ・スライドオフ・キャンセル・disabled 素通し・
  トグル中間フレーム・ジェスチャ中の remove 安全性）+ golden
  `ui_controls` + example `ui_demo`

## 0.1.0 — 2026-08-14 (Phase L0)

初版。Stage API + CPU リファレンスバックエンド。

- CALayer 準拠のレイヤーツリー（bounds / anchor / position / frame 糖衣、
  左上原点・+y下の単一座標系、zPosition なし）
- content 13種（image / text / line / polyline / polygon / rect / circle /
  circles / arc / arrow / crosshair / grid / boxes）を SDF 描画
- 属性: opacity / hidden / masksToBounds / cornerRadius / shadow / border /
  background / blend(normal, add, multiply, screen) / rotation / scale /
  transform
- フィルタ 29種（GLSL∩C++ 単一ソース + X-macro メタデータ。長さ系
  パラメータは論理単位）
- Transaction による implicit animation（dt 注入・決定的、進行中の
  再変更は現在値から継続）
- 検出ボックスの時間平滑化（id / 最近傍対応、時定数指定）
- テキスト: FreeType + HarfBuzz（日本語、欠落グリフは決定的代替）
- hit-test（CALayer 規則）、StageLimits（構造は throw、データは
  切詰め+診断）
- テスト: stage_tests（幾何・アニメ・limits・フィルタ表・決定性）+
  golden 7シーン + 全 example の CI 実行

既知の制限（cookbook 末尾に記載）: arc の dash / Cap::Butt 未対応、
テキスト1行のみ、sRGB 空間でのフィルタ計算。
