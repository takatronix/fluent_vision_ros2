# Changelog

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
