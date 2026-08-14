# Changelog

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
