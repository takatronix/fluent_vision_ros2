# Fluent Scene 2D 描画・フィルタ API 再設計

ステータス: **draft — オーナーレビュー待ち**
前提文書: [fluent_vision_architecture.ja.md](fluent_vision_architecture.ja.md)（規範）

## 0. この文書がある理由

これまでの実装は「頼まれたものを1個ずつ足す」形で進み、結果として
(1) 描画プリミティブが歯抜け、(2) フィルタが画像専用でレイヤーに掛からない、
(3) フィルタ定義が3箇所に分散、という **Fluent Vision の概念に反する形**になった。
本文書は 2D 描画とフィルタの **API 全体を先に確定** し、以後の実装はこのカタログを
埋める作業に限定するための再設計である。

設計原則（親文書の決定に追加）:

- **P-1 レイヤーは第一級**。iOS CALayer と同等に、全 visual ノードの出力は
  位置・変形・不透明度・影・角丸・エフェクトを *属性として持つ* レイヤーである。
- **P-2 フィルタはどこにでも掛かる**。image にも、単一レイヤーにも、レイヤーグループにも、
  同じフィルタノードが同じ意味で適用できる。
- **P-3 定義は一度だけ**。フィルタ・プリミティブの本体/メタデータは単一ソースに置き、
  シェーダー・CPU・レジストリ・ドキュメントはそこから導出する。魔法数は生成物にしか
  現れない。
- **P-4 GPUImage / Core Graphics 水準を下回らない**。カタログの空欄は「未実装」として
  明示し、存在しないことにしない。

---

## 1. レイヤーモデル — シーンはレイヤーツリーである

**旧構造（nodes 列 + composite ノードで配線）は廃止する。** CALayer と同じく、
シーンの本体は「内容 + 属性 + サブレイヤー」を持つレイヤーの木であり、合成順は
木の順序そのものである。composite ノードという配線は存在しない。

```yaml
schema: fluent.scene/v1alpha2
layers:
  - id: camera
    content: { image: { source: $inputs.camera, fit: contain } }
    filters:
      - bilateral: { radius: 4 }          # フィルタはレイヤーにインラインで付く
      - color_transform: { contrast: 1.2 }

  - id: route
    content: { polyline: { points: $inputs.path, thickness: 6, dash: 18,
                           color: [1.0, 0.62, 0.15, 0.9] } }
    shadow: { offset: [0, 2], blur: 4, color: [0, 0, 0, 0.5] }

  - id: hud                               # グループ = sublayers を持つレイヤー
    position: [24, 24]
    opacity: 0.9
    shadow: { offset: [0, 4], blur: 12, color: [0, 0, 0, 0.5] }
    sublayers:
      - id: panel
        content: { rect: { size: [340, 96], corner_radius: 12,
                           color: [0, 0, 0, 0.45] } }
      - id: title
        position: [16, 12]
        content: { text: { source: $inputs.status_text,
                           font: $resources.ui_font, shadow: true } }
```

- `content:` は §2 のプリミティブ1個（`image` / `text` / `rect` / `polyline` / …）。
  content を持たず `sublayers:` だけ持つレイヤーがグループ。
- `filters:` は §3 のフィルタを**宣言順に**適用するインラインチェーン。
  グループに書けばサブレイヤー合成後の1枚に掛かる（グループぼかし・グループ透過）。
- 検証・有界性・予算・digest の規範は親文書のまま（レイヤー数・ネスト深さも静的上限）。
- 出力は暗黙にルートツリーの合成結果（`outputs.frame`）。複数出力が要る場合のみ
  `outputs` で部分木を名指しする。

| 属性 | 型 | 既定 | 意味 |
|---|---|---|---|
| position | vec2f | [0,0] | 平行移動（px） |
| anchor | vec2f | [0.5,0.5] | rotation/scale の基準（レイヤー境界に対する 0..1） |
| rotation | f32 | 0 | 度 |
| scale | vec2f | [1,1] | 拡縮 |
| opacity | f32 | 1 | グループにも適用可（合成後に一括） |
| corner_radius | f32 | 0 | 角丸クリップ |
| shadow | struct | なし | offset/blur/color。シルエット（α）から生成 |
| border | struct | なし | width/color。クリップ形状に沿う |
| background | vec4f | 透明 | レイヤー境界の背景板 |
| blend | enum | normal | 合成モード |
| effects | layer参照列 | なし | フィルタチェーン（§3 と同一ノード） |

実装: 属性/フィルタ/グループを持つレイヤーは透明クリアの中間ターゲットに描画し、
shadow → background → border → 本体（filters 適用後）の順で合成する。属性を
持たない葉レイヤーは合成パスに直接描く（ゼロコスト維持）。

移行: `fluent.scene/v1alpha1`（nodes+composite）は当面パーサーが受理し続けるが、
新機能はツリー形式にのみ追加する。例・ページ・テストはツリー形式へ書き直す。

---

## 2. 描画プリミティブカタログ

方針: **データ駆動**（入力ポートから点列・矩形列を受ける）と
**宣言駆動**（パラメータで1個描く）の両方を揃える。既存の歯抜けを埋める。

| ノード | 状態 | 入力 | 主パラメータ | 用途 |
|---|---|---|---|---|
| visual.image2d | 実装済 | image | fit / **rect**(新: 任意配置) | 映像・画像の配置 |
| visual.line2d | **新** | — | from, to, thickness, dash, cap(butt/round) | 単線・ガイド線 |
| visual.polyline2d | 拡張 | polyline2d | thickness, **dash**, cap | 経路・軌跡 |
| visual.rect2d | **新** | — | rect, corner_radius, thickness(0=塗り), color | パネル・囲み |
| visual.rects2d | **新** | rects2d(点列型追加) | 同上（データ駆動） | 任意の矩形群 |
| visual.circle2d | **新** | — | center, radius, thickness | 単円・照準 |
| visual.circles2d | 実装済 | points2d | radius, thickness(0=塗り=**点**) | 点群・マーカー |
| visual.ellipse2d | **新** | — | center, radii, rotation, thickness | 楕円 |
| visual.arc2d | **新** | — | center, radius, start/end(度), thickness, cap | ゲージ・角度表示 |
| visual.polygon2d | **新** | polyline2d | color（塗り）, thickness(枠) | 領域表示 |
| visual.arrow2d | **新** | — / points | from, to, thickness, head_size | 方向・速度ベクトル |
| visual.crosshair2d | **新** | — | center, size, gap, thickness | 中心/照準マーク |
| visual.grid2d | **新** | — | spacing, thickness, color | キャリブ・デバッグ |
| visual.boxes2d | 実装済 | detections | color, **corner_radius**, smoothing, show_label | 検出枠 |
| text.dynamic | 拡張 | string | font, position, **size**(新), **align**(新), **max_width/wrap**(新), shadow | ラベル・状態 |
| composite.layers | 実装済 | layer列 | color_space | 最終合成 |
| layer.group | **新** | layer列 | （layer属性） | グループ化 |

共通規則:
- 塗り/枠は `thickness`（0 = 塗り）で統一。角は `corner_radius`、線端/継ぎ目は
  `cap: butt|round` で統一。点線は `dash`（px、等間隔）で統一。
- 全て SDF レンダリング（AA 付き・拡大無劣化）。動的個数は bounds 必須（親文書どおり）。
- 座標は出力ピクセル。変換は layer.position/rotation/scale で行う。

---

## 3. フィルタカタログ

適用対象は P-2 により **image / layer / layer.group のすべて**。
定義は P-3 により `filters_shared.h` の一箇所（本体は GLSL∩C++ で一度だけ記述、
CPU/GPU/レジストリ/カタログページは全てそこから導出）。

### 3.1 色調・トーン（実装済み）
grayscale / sepia / invert / hue / exposure / white_balance / levels /
threshold / posterize / solarize / rgb / haze / color_transform（bri/con/sat/gam）/ opacity

### 3.2 平滑化・ノイズ除去
| フィルタ | 状態 |
|---|---|
| blur（分離ガウシアン） | 実装済 |
| bilateral（エッジ保存） | 実装済 |
| median 3×3 | 実装済 |
| box_blur | P0 |
| unsharp_mask | P0 |
| kuwahara（油彩・強ノイズ除去） | P1 |
| tilt_shift（帯状ぼかし） | P1 |

### 3.3 エッジ・様式化（実装済み）
sharpen / emboss / edge_sobel / sketch / toon、＋ P1: canny, crosshatch

### 3.4 変形・サンプリング
| フィルタ | 状態 |
|---|---|
| pixelate / swirl / motion_blur / zoom_blur / halftone | 実装済 |
| **lens_undistort（歪み補正: k1,k2,p1,p2）** | **P0 — ロボット必須** |
| crop / flip / rotate90 | P0 |
| perspective_warp | P1 |
| fisheye / bulge / pinch | P2 |

### 3.5 合成・キー
| フィルタ | 状態 |
|---|---|
| chroma_key | P1 |
| lut（3D LUT ルックアップ、.cube 資産） | P1 |
| blend（2入力: add/multiply/screen/overlay） | P1（2入力フィルタ基盤を含む） |

### 3.6 解析（将来・別カテゴリ）
histogram_equalization (P1) / CLAHE (P2) / auto_white_balance (P2)

---

## 4. アニメーション宣言（fluent_animation の宣言版）

`runtime_mutable` パラメータに **宣言的イージング**を付ける。外部から `setParam` した
とき、即値ではなく宣言された曲線で遷移する。

```yaml
params:
  blur_radius:
    type: f32
    default: 0.0
    runtime_mutable: true
    animate: { duration: 0.4, ease: ease_in_out }   # setParam が 0.4s かけて効く
```

将来: レイヤー遷移（fade/slide in-out）、時間駆動アニメーション（sin波等の宣言）。

---

## 5. 実装アーキテクチャ（P-3 の具体化）

1. **`filters_shared.h`（済・単一定義）** — 各フィルタ = 名前付き引数の関数1個。
   GLSL と CPU は同一本体をコンパイル。メタデータ表も同居し、スロット対応は
   dispatch 関数1箇所のみ。
2. **プリミティブも同方式** — SDF 形状関数（`sdf_rounded_rect`, `sdf_segment`,
   `sdf_arc`…）を `shapes_shared.h` に一度だけ書き、全形状シェーダーと CPU が共有。
   形状ノードのメタデータ（パラメータ名・既定値）も同居。
3. **レイヤーパス** — 属性付きレイヤー/グループは transient ターゲット
   （プランに計上、aliasing 可）へ描画 → shadow/border/effects 付き合成。
4. **2入力フィルタ基盤**（blend/chroma_key 用）— filter パスの入力を最大2に一般化。

---

## 6. 実装順序（レビュー後に確定）

| Tier | 内容 | 受け入れ基準 |
|---|---|---|
| **T0 基盤** | filters_shared 配線完了 / shapes_shared / レイヤー属性（position・opacity・shadow・corner_radius・border・background）/ layer.group | テキスト・枠・任意レイヤーに影が付いた実描画。定義単一化で両バックエンド同一出力 |
| **T1 プリミティブ** | line2d, rect2d/rects2d, circle2d, arc2d, polygon2d, arrow2d, crosshair2d, grid2d, dash/cap 統一, image2d rect 配置, text size/align/wrap | カタログ全形状の実描画サンプル + golden |
| **T2 フィルタ P0** | box_blur, unsharp_mask, lens_undistort, crop/flip/rotate90 | 歪み補正の実カメラ検証を含む |
| **T3 動きと合成** | animate 宣言, blend/chroma_key/lut, layer.effects | ライブデモ |

---

## 7. 未決定（レビューで決めること）

- layer.transform に skew/3D（perspective）まで入れるか
- text の縦書き・ルビの扱い
- blend モードの初期セット（normal/add/multiply/screen で足りるか）
- lens_undistort のモデル（plumb_bob で開始し fisheye equidistant を P1 にするか）
