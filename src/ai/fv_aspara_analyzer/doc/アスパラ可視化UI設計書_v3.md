### アスパラ可視化 UI 設計書 v3（v2 リファクタ対応）

目的:
- 農家向けにフリッカのない滑らかな可視化を提供（パススルー画像 + オーバーレイ）
- 選択中/非選択の描画差別化、収穫ハイライト（23cm）、情報ウィンドウ（右）
- `fluent_lib` を極力活用し、再利用可能なUI部品をライブラリ化

レイヤ構成（背面→前面）:
1. 背景: カメラ画像（BGR8、無加工）
2. OverlayEngine（補間済み）
   - BBox（選択=緑・太枠・α0.8、非選択=灰・細枠・α0.4）
   - ラベル（全個体: `ID:xx 信頼度:yy.y%`）
   - 収穫ハイライト（選択中のみ、緑/オレンジ、タイトル「アスパラXXcm # ID」）
3. HUD（右側情報ウィンドウ、ステータス等）

主要仕様:
- 単位: 内部=m、表示=cm（タイトルは整数、情報は小数1桁）
- ハイライト: 長さ≥閾値(23cm)→緑、未満→オレンジ。未算出/レンジ外は非表示
- 情報ウィンドウ（右）: ID, 長さ, 距離, 太さ, 曲がり度, グレード, 収穫可否, [debug]点群
- オフスクリーンHUD合成（αブレンド）: `hud.offscreen_enabled: true` で有効

アニメーション:
- `OverlayEngine` が各IDの bbox, alpha を保持し、`tick(dt)` で LERP 補間
- ゲイン: `overlay.smooth_gain`, `overlay.fade_in_gain`, `overlay.fade_out_gain`

YAML（キー仕様・例）:（現在のYAMLを正とする）
- 入力
  - `camera_topic`: カラー画像の購読トピック（例: `/fv/d405/color/image_raw`）
  - `detection_topic`: 検出結果の購読トピック（例: `/fv/d405/object_detection/detections`）
- 出力
  - `output_annotated_image_topic`: 注釈付き画像の出力トピック（例: `/fv/d405/aspara_analysis/result`）
    - 圧縮版は実装側で自動生成（`<output>/compressed`）。個別キーは不要。
- アニメーション/オーバーレイ
  - `overlay.smooth_gain`: 位置サイズのLERPゲイン
  - `overlay.fade_in_gain`, `overlay.fade_out_gain`: αのフェードイン/アウトゲイン
  - `overlay.selected.{alpha, thickness, color_bgr}`
  - `overlay.unselected.{alpha, thickness, color_bgr}`
- ハイライト
  - `harvest.{enabled, selected_only, threshold_cm, title_bg_alpha, title_text_color_bgr}`
- HUD/情報（現行YAML）
  - `info_window_enabled`, `info_window_alpha`, `info_font_scale`, `info_line_height`
  - `info_show_id`, `info_show_length`, `info_show_distance`, `info_show_diameter`, `info_show_straightness`, `info_show_grade`, `info_show_harvestable`, `info_show_debug_points`
  - `info_anchor`, `info_offset_x`, `info_offset_y`
  - `hud.offscreen_enabled`
  - `display_validity.*`

責務分担:
- `AsparaVisualizerV2`: 画像パススルー、OverlayEngine呼び出し、HUD配置
- `OverlayEngineV2`: 補間状態管理とスナップショット提供
- `HUD`（fluent_ui）: 情報ウィンドウ・ステータス描画

エラー/未入力時動作:
- 画像未到着時も黒背景「待機中…」を周期出力（可視化が止まらない）

今後の拡張:
- 点群プレビューHUD（低FPS）、ルート/アプローチ点の視覚強調、距離適応ZOOM


