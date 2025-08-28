# アスパラ ID 管理（設計・仕様）

作成日: 2025-08-16 / 対象: `fv_aspara_analyzer`

## 目的
- 検出結果（YOLO）からアスパラ個体へ一意な ID を安定付番・維持する
- 複数個体・連続フレームでのトラッキング、選択維持、再選択ルールを明確化

## 用語
- 検出: YOLO の `Detection2D`（class_id=0=本体）
- 個体: 1 本のアスパラ（`AsparaInfo` で管理）
- 選択: 操作対象の 1 本（重い点群処理を許可）

## 選択モードとフラグ
- モード定義（将来のために用意）
  - 手動: 外部サービスでのみ選択変更
  - 自動: 毎フレーム「面積→信頼度」優先で自動選択
  - 自動2: 基本は選択維持、消失時のみ自動再選択
- 現在の運用（確定）
  - `aspara_selection.manual_enabled: false`
  - `aspara_selection.auto_enabled: false`
  - `aspara_selection.keep_and_reacquire_enabled: true`  # 自動2

## データ構造（要点）
- `AsparaInfo`
  - `id: int` — 個体 ID（連番）
  - `bounding_box_2d: cv::Rect` — 検出矩形（本体）
  - `confidence: float` — 検出信頼度（0..1）
  - `spike_parts: std::vector<AsparagusPart>` — 穂
  - `approach` — 2D/3D アプローチ点
  - 分析結果（長さ/太さ/真直度/グレード 等）
- `AsparagusPart`
  - `class_id`（0=本体, 1=穂）
  - `bounding_box_2d`, `confidence`, `is_valid`

## ID 付番・更新ルール
1) 新規検出リスト → 既存個体へマッチ
- 指標: IoU（矩形重なり） or 2D 中心距離
- 閾値（既定）:
  - `aspara_selection.reacquire_iou: 0.3`
  - `aspara_selection.reacquire_center_ratio: 0.5`  # 中心距離 <= 0.5 * 対角（対角長 × 比）
  - `aspara_selection.sticky_bbox_expand_ratio: 1.2`    # 前回BBoxの拡張係数（YAMLキー名に合わせる）
- 複数重複→最大 IoU 優先

2) マッチ不可 → 新規 ID 付番
- `next_aspara_id_` を採番して `AsparaInfo` 生成

3) 消失処理
- 一定時間未観測で「消失」→描画から除外
  - `aspara_selection.stale_max_age_sec: 0.5`

4) 選択維持（Sticky）
- YOLO の一時消失に備え、選択領域を保持
  - `aspara_selection.sticky_ms: 800`  # 既定値 OK
- 保持期間内に近傍で再検出した場合、同一個体に再アタッチ
  - 再アタッチ領域: 前回BBoxを `sticky_region_expand` 倍に拡張した矩形内を優先

## 選択ルール
- 通常は現在の選択を維持（自動2）
- 再アタッチ失敗・選択なし時に候補から選ぶ基準:
  - 並び替え: 面積降順（既定）
  - `aspara_selection.sort: "area"`  # 必要なら将来変更可

## 外部操作/通知
- すでに「次へ/前へ」は実装済み（想定）
- ID 直接指定は不要
- 選択中アスパラ情報を配信するトピックを公開
  - カメラごとに名前空間を分ける（例）
    - D405: `selected_asparagus_topic: "/fv/d405/selected_asparagus_info"`
    - D415: `selected_asparagus_topic: "/fv/d415/selected_asparagus_info"`
  - 公開有効/周期: `selected_asparagus_info_enabled`, `selected_asparagus_publish_rate`
  - メッセージ例: `AsparaSelectedInfo`（id, bbox, confidence, length_m 等。内部は m、表示は cm）

## 非選択の軽量 3D 参照
- 非選択個体は軽量に 2D+小窓深度で概略 3D を推定
  - `lightweight_3d_for_unselected.enabled: true`
  - `lightweight_3d_for_unselected.window_size: 11`
  - `lightweight_3d_for_unselected.aggregator: "median"`
- 目的: アプローチ点のみ作成（重い処理は実施しない）

## リセット/上限
- ID 全リセットはしない（無効）
  - `id_tracker.reset_on_idle_sec: -1`  # 無効値
- トラッキング上限
  - `id_tracker.max_tracked_ids: 32`（既定、必要に応じて調整）

## 表示仕様（関連）
- ラベルの信頼度: 小数 1 桁（例: 87.6% → 87.6% と表示）
- 距離・長さなどの単位: cm で表示
- 全個体: `ID:xx 信頼度:yy.y%` を矩形左上に表示
- 選択中: 緑・太枠（2px）/ 透過 80%
- 非選択: グレー・細枠（1px）/ 透過 40%
- スムーズ追従: OverlayEngine が `tick(dt)` 補間（毎フレーム）

## YAML パラメータ（まとめ）
- `aspara_selection.manual_enabled: false`
- `aspara_selection.auto_enabled: false`
- `aspara_selection.keep_and_reacquire_enabled: true`
- `aspara_selection.sticky_ms: 800`
- `aspara_selection.reacquire_iou: 0.3`
- `aspara_selection.reacquire_center_ratio: 0.5`
- `aspara_selection.sticky_bbox_expand_ratio: 1.2`
- `aspara_selection.stale_max_age_sec: 0.5`
- `aspara_selection.sort: "area"`
- `selected_asparagus_info_enabled: true`
- `selected_asparagus_topic: "/fv/<camera>/selected_asparagus_info"`
- `selected_asparagus_publish_rate: 10.0`
- `lightweight_3d_for_unselected.enabled: true`
- `lightweight_3d_for_unselected.window_size: 11`
- `lightweight_3d_for_unselected.aggregator: "median"`
- `id_tracker.reset_on_idle_sec: -1`
- `id_tracker.max_tracked_ids: 32`

## 変更履歴
- 2025-08-16: 初版 → 方針確定事項を反映（sticky_ms=800ms、自動2、ラベル1桁小数、cm 表示 など）
