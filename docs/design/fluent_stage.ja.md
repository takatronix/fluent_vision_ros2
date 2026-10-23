# fluent_stage 設計書 — Stage / Layer / Surface と Scene の関係

ステータス: **v1.0 承認済み（2026-08-14）**。§2・§9・§10・§12 は同日の追補（レビュー待ち）。

**目的**: ロボットのカメラ映像と情報を、**美しく・リアルタイムに・宣言的に**表示する。
将来の AI と人が、ロボットの上でこれを使う。OSS として公開に耐える美しさと正しさを
完成の定義に含める。

**照準**: **AI 時代の GPU グラフィックのデファクトスタンダード**。
人間には1行で書け、AI には安全に生成・検証・監査でき、実行中に差し替えられる —
この3つを同時に満たす描画基盤は現存しない。ロボットの HUD はその最初の実証場であり、
設計は最初からその射程で行う（S-0〜S-7、§13 AI 協調はその布石）。

関連文書:
[fluent_scene_2d_api.ja.md](fluent_scene_2d_api.ja.md)（プリミティブ/フィルタのカタログ）
[fluent_vision_architecture.ja.md](fluent_vision_architecture.ja.md)（型検証・決定性・予算・MCP の規範）

---

## 1. 用語 — 4つの言葉の定義

| 用語 | 一言でいうと | 実体 | 誰が触るか |
|---|---|---|---|
| **Scene** | 画面の**宣言文書**（.fvs / YAML） | 型検査・digest・予算・fallback を持つデータ | AI・ツール・運用者。差分レビューと監査の単位 |
| **Stage** | 実行中の**レイヤーツリーの土台** | C++ の保持型オブジェクト | C++ を書く人間・binding・レンダラー |
| **Layer** | ツリーの1ノード | content（描くもの）+ 属性（合成のしかた）+ sublayers | Stage 経由で生成・変更 |
| **Surface** | **オフスクリーンの絵の実体** | レンダリング結果のピクセル（RGBA） | 画面表示・ROS publish・読み戻し |

関係を1枚で:

```
        （AI・運用はここを編集）                （人間の C++ はここに直接書ける）
┌─────────────────────┐  compile   ┌──────────────────────┐  render  ┌─────────────┐
│  Scene (.fvs, YAML)  │ ────────▶ │  Stage（レイヤーツリー）│ ───────▶ │  Surface     │
│  型検査/digest/予算   │  検証→構築 │  保持型・実行中に可変    │  毎フレーム│  ピクセル     │
└─────────────────────┘            └──────────────────────┘          └─────────────┘
                                       ▲            ▲
                       ROS binding ────┘            └──── setParam / setImage /
                       （スナップショットが                setPoints / opacity() …
                         同じ setter で流れ込む）           （人間も同じ入口）
```

**要点: Stage が本体で、Scene はそれを組み立てる宣言層。** Scene で書けることは
すべて Stage の C++ API で書ける。逆は真ではない — Scene には実行に不要だが運用に
必要な契約（型検証、並べ替え不変 digest、予算ゲート、入力宣言と fallback）が付く。
これが「AI が安全に画面を生成・編集できる」根拠になる。

### 1.1 他システムとの対応（類推で理解するための表）

| ここでの語 | iOS | Flash | Web | After Effects |
|---|---|---|---|---|
| Scene | SwiftUI（宣言） | .fla | HTML | comp のプロジェクト定義 |
| Stage | CALayer ツリー | **Stage**（display list） | DOM | 実行中の comp |
| Layer | CALayer | DisplayObject/Sprite | 要素 | レイヤー |
| Surface | (CARenderer の出力) | BitmapData | canvas バッファ | レンダリング済みフレーム |

### 1.2 命名の検討記録（なぜこの名前か）

- **Canvas は不採用** — java.awt/Android の語感が強く、「ツリーの土台」と「ピクセルの
  置き場」という別概念を1語に混ぜてしまっていた。
- **Stage** — Flash の display list の語。レイヤー毎フィルタ（Blur, DropShadow…）文化の
  元祖で、本ライブラリの概念と最も一致する。短く、舞台の比喩が直感的。
- **Surface** — オフスクリーンバッファの業界共通語（SkSurface / VkSurface /
  EGLSurface / Android Surface / wl_surface）。
- **Layer** — CALayer / Photoshop / After Effects / SurfaceFlinger と万国共通。
- **Scene** は既存の fluent_scene（宣言層）の名を使う。fluent_scene（宣言）と
  fluent_stage（実行）が対。

### 1.3 責務の境界

| 機能 | Scene（宣言層） | Stage（実行層） |
|---|---|---|
| レイヤーツリーの構造 | YAML で宣言 | C++ で生成・保持 |
| 型検査・参照解決 | ここで実施（実行前に全拒否） | C++ の型システムが担う |
| digest（並べ替え不変の指紋） | ○ | × |
| 予算（GPU メモリ・転送量） | 宣言し、コンパイルで拒否 | 上限として受け取る |
| 実行中の変更 | ×（文書編集→再コンパイル→原子的差し替え） | ○ `opacity()` `setImage()` `setParam()` |
| ROS binding / fallback | 入力契約と fallback を宣言 | スナップショットを setter で受ける |
| AI による編集 | ○（差分・digest・検証で安全） | ×（直接は触らせない） |

---

## 2. 同じ画面を、C++ と Scene で

同じデモを両方の書き方で示す。**どちらでも完全に同じ絵が出る**ことが契約。

C++（Stage — アプリに組み込む・拡張する人向け）:

```cpp
Stage stage(1920, 1080);
stage.image(camera);
stage.boxes(detections).color(Color::Teal).smoothing(0.2f);

auto& hud = stage.group("hud").position(24, 24).opacity(0.9f).shadow();
hud.rect({0, 0, 340, 96}).cornerRadius(12).color({0, 0, 0, 0.45f});
hud.text("走行中", {16, 12}).size(28);
```

Scene（.fvs — 現場調整・リモート更新・AI 生成向け）:

```yaml
schema: fluent.scene/v1alpha2
inputs:
  camera:     { type: image.rgba8, update: per_frame }
  detections: { type: "sequence<Detection2D, 128>", update: per_frame }
layers:
  - content: { image: { source: $inputs.camera } }
  - content: { boxes: { source: $inputs.detections,
                        color: [0.1, 0.9, 0.7, 1], smoothing: 0.2 } }
  - id: hud
    position: [24, 24]
    opacity: 0.9
    shadow: {}
    sublayers:
      - content: { rect: { size: [340, 96], corner_radius: 12,
                           color: [0, 0, 0, 0.45] } }
      - content: { text: { text: "走行中", position: [16, 12], size: 28 } }
```

> 補足（L2 実装時の修正）: テキストの位置は content の `position` フィールドに
> 書く。初稿はレイヤー属性 `position: [16, 12]` としていたが、レイヤー属性の
> position は **anchor の親座標配置**（§3.1 の CALayer 準拠）であり、C++ 側の
> `text("走行中", {16, 12})`（内容の基準点）と同じ絵にならない。§2 の
> 「どちらでも完全に同じ絵」契約は scene_tests の YAML↔C++ ピクセル一致
> golden で機械検証されている。

| | C++（Stage） | Scene（.fvs） |
|---|---|---|
| 実行までの手順 | ビルドして実行 | **ビルド不要** — 文書を読み込むだけ |
| 画面の変更 | 再コンパイル・再デプロイ | **実行中に差し替え**（検証 → フレーム境界で原子的に切替） |
| 書く主体 | 人間のエンジニア | 人間 + **AI**。型検査・参照解決・予算ゲートが誤りを実行前に全て拒否し、digest が変更を監査可能にする |
| 向く用途 | アプリ組込み、新ノード開発 | 現場での画面調整、遠隔更新、AI による画面生成・自動改善 |

AI が実行中に画面を変える流れ（親設計書の MCP ライフサイクルに接続）:
**AI が YAML を生成/編集 → validate（誤りは全部ここで拒否）→ compile → 隔離 preview
→ フレーム境界で activate**。壊れた画面が表に出る経路は存在しない。

---

## 3. Stage ライブラリの設計原則

| # | 原則 |
|---|---|
| **S-0** | **基本は iOS（Core Animation / UIKit）を規範とする — ただし良いところだけ**。レイヤーモデル・implicit animation・属性の名前と意味は、文書化された理由なしに iOS から逸脱しない。一方で iOS の**悪癖は継がない**（座標系の分裂と geometryFlipped、二重座標系の露出など）。準拠・改良の別は §3.1 の表に理由付きで登録する |
| S-1 | **1行で描ける**。全引数に良い既定値。装飾はチェーンで任意に足す |
| S-2 | **描いたものはレイヤー**。全描画呼び出しはレイヤーを作り `Layer&` を返す |
| S-3 | **配置と内容の分離**。content（何を描くか）と Layer 属性（どこに/どう合成するか）を混ぜない |
| S-4 | **GPU 用語を API に出さない**。NDC・UV・descriptor はバックエンドの実装詳細 |
| S-5 | **定義は一度だけ**。フィルタ/形状の本体・メタデータは単一ソース（GLSL∩C++ 共有ヘッダ）から全系統に導出。魔法数は生成物にしか現れない |
| S-6 | **有界**。レイヤー数・ネスト深さ・点数・カーネルは静的上限（親規範の継承） |
| S-7 | **属性変更は既定でアニメート可能**（§9）。iOS の implicit animation と同じ思想 |

### 3.1 iOS 対応表 — 同じもの・意図的に変えたもの

**そのまま対応（意味も iOS 準拠）**:

| fluent_stage | iOS | 備考 |
|---|---|---|
| `frame` | UIView.frame | 糖衣（下記の bounds/position から算出） |
| `position` | CALayer.position | **anchor の親座標での位置**（CALayer 準拠に修正。「frame origin の移動」ではない） |
| `anchor` | CALayer.anchorPoint | 既定 (0.5, 0.5) も同じ |
| `opacity` / `hidden` | opacity / isHidden | |
| `cornerRadius` | cornerRadius | |
| `masksToBounds` | masksToBounds | **既定 false**（iOS と同じ: 角丸だけでは中身を切らない） |
| `border` (width/color) | borderWidth / borderColor | |
| `shadow` (offset/radius/color/opacity) | shadowOffset / shadowRadius / shadowColor / shadowOpacity | フィールド名も radius に統一（blur から改称） |
| `transform` | affineTransform | 簡易属性と合成、anchor 基準 |
| `fit` | contentsGravity | contain=resizeAspect / cover=resizeAspectFill / fill=resize |
| `Transaction` | CATransaction | implicit animation の括り |
| `states`（UI） | UIControl state | 属性上書き + 自動遷移 |
| hit-test | hitTest / pointInside | frame とツリー順、transform 考慮 |

**意図的な逸脱 = iOS の悪癖を直す箇所（理由付き）**:

| 項目 | iOS | ここでの決定 | 理由 |
|---|---|---|---|
| 座標系 | UIKit 左上 / Quartz・macOS 左下の**二重系** + geometryFlipped | **左上・y下の1系のみ。反転スイッチは作らない** | 二重系は「画像が上下逆」事故の温床（§4.1 の教訓）。悪癖は継がない |
| `sourceRect` | contentsRect（0–1 正規化） | **論理 px の矩形** | 0–1 系を API に出さない（§4）。名前も変えて混同を防ぐ |
| `zPosition` | あり | **持たない**（順序 = ツリーの前順のみ） | 決定性と可読性。宣言文書の diff で重なりが読めることを優先。必要が実証されたら再検討 |
| 3D transform | CATransform3D | 2D アフィンまで | 2D HUD が目的。perspective は将来判断（§15） |
| 色型 | CGColor | float RGBA 構造体 | GPU/CPU 共有の単純さ。sRGB 前提 |

## 4. 座標系

- **論理座標系**: `Stage(w, h)` が宣言。位置・太さ・角丸・フォントサイズ・ぼかし半径
  など、API に現れる長さは**すべて論理単位**。
- 出力解像度はレンダー時に指定し一様スケール。描画は全て SDF なので、スケールは
  **ビットマップ拡大ではなく数式の再評価** — どの出力解像度でもベクタ品質。
- 論理と出力のアスペクト差のポリシーは Stage 属性: `fit`（レターボックス、既定）/
  `fill` / `stretch`。
- サブレイヤーは**親レイヤーのローカル座標**（position/rotation/scale を継承）。
  `anchor`（0..1）が回転・拡縮の基準点。
- GPU の 0–1 正規化座標は**採用しない**（「太さ 2」「角丸 12」が書けない・アスペクトで
  歪む）。相対配置はレイアウトヘルパー（`Align::Center`, `pct(50)`）で後日応える。

### 4.1 レイヤーの幾何 — 原点・bounds・position・回転（CALayer 準拠）

**原点は常に左上、+x 右、+y 下**（UIKit 準拠）。Stage の (0,0) は画面左上。

> **歴史からの教訓**: Apple 圏は数学的慣習（PostScript/Quartz: 左下・y上 —
> macOS の AppKit/CALayer が今もこれ）と画面の慣習（UIKit: 左上・y下）が同居し、
> CGBitmapContext や macOS↔iOS 移植で「画像が上下逆」という定番事故を生んできた。
> 本ライブラリは **全 API・全データ（bounds / content / hit-test / sourceRect /
> Surface の行順）で左上・y下の1系のみ**とし、`geometryFlipped` に相当する
> スイッチは設けない。GPU バックエンド固有の向き（OpenGL テクスチャ等）は
> レンダラー内部に完全に閉じ込める。

各レイヤーは3つの幾何要素を持つ:

```
親レイヤーの bounds 空間                     子レイヤー自身の bounds 空間
┌────────────────────────────┐              ┌──────────────┐(0,0) 左上原点
│ (0,0)                       │              │  content と   │
│        position ●───────────┼──▶ ここに    │  sublayers は │
│        （= anchor が親空間の │    anchor が │  この空間で   │
│          どこに来るか）      │    置かれる  │  座標を書く   │
└────────────────────────────┘              └──────────────┘(w,h)
```

| 要素 | 意味 |
|---|---|
| `bounds` | レイヤー自身の座標空間（左上 (0,0)〜(w,h)）。content の座標・サブレイヤーの position は**この空間**で書く |
| `anchor` | bounds 内の正規化点（既定 (0.5,0.5) = 中心）。回転・拡縮・transform の軸 |
| `position` | **anchor が親の bounds 空間のどこに置かれるか** |
| `frame` | 糖衣。回転なしのとき frame = {position − anchor×size, size}。設定すると bounds.size と position を逆算 |

**回転の意味論**（`rotation` / `transform`）:

1. 回転は **anchor を軸**に回る（既定=中心。左上を軸にしたければ `anchor(0,0)`）。
2. **サブツリーごと回る**。content もサブレイヤーも、影・border・角丸クリップも
   一体で回転する（iOS と同じ: 変換はレイヤー座標空間全体に掛かる）。
3. `bounds` は回転で**変わらない**。親から見た外接矩形（frame 相当）は膨らむ。
   回転が掛かっているレイヤーに frame を set してはならない（iOS と同じ注意。
   API はこのとき診断を出す）。
4. ヒットテスト（§10）は親→子へ**逆変換**してローカル座標で判定（iOS と同じ）。
5. 実装: 回転・拡縮は SDF の評価座標を逆変換して行うためベクタ品質を保つ。
   フィルタ/影を伴う場合は中間ターゲットに描いてから変換合成（バイリニア）。

```cpp
stage.rect({0, 0, 200, 100})
     .position(960, 540)        // 中心が画面中央に来る（anchor 既定 = 中心）
     .rotation(30);             // その中心の周りに 30° 回る

stage.text("針", {0, 0})
     .anchor(0.5f, 1.0f)        // 下端中央を軸に
     .position(960, 800)
     .rotation(speed_deg);      // メーター針のような回転
```

## 5. 複数解像度の画像の合成

1. image content は自レイヤーの `frame`（論理矩形、省略時は親いっぱい）へ
   `fit: contain | cover | fill` で写像。混在解像度のマージは「各画像が各自の frame に
   収まる」だけで、特別な機構は発生しない。
2. 拡縮のサンプリングはバイリニア。強い縮小（>2×）のミップマップは未実装項目
   （§15-3）。
2b. **部分コピー＆ペースト**: `sourceRect`（元画像の切り出し矩形）+ `frame`（貼り先）
   の組で表現する。同一ソースを複数レイヤーで異なる sourceRect 参照してよい
   （転送は1回、切り出しはサンプリングで行うためコピーコストは増えない）。

```cpp
stage.image(cam).sourceRect({640, 200, 320, 240})   // 元のこの領域だけを
     .frame({40, 40, 640, 480});                    // ここへ拡大して貼る
```
3. **フィルタのパラメータは論理単位**。`blur(4)` は元画像が 640×480 でも 4K でも
   画面上で同じ見た目（ランタイムが層ごとの論理→ソース px 係数を掛けてから
   カーネルへ渡す）。
4. メモリ・予算計上はネイティブ解像度で行う（planner 連携）。

```cpp
Stage stage(1920, 1080);
stage.image(main_cam);                                   // 1280×720 → 全面
stage.image(rear_cam).frame({1560, 40, 320, 180});       // 640×480 → PiP
stage.image(depth_vis).frame({1560, 240, 320, 180}).opacity(0.8f);
```

## 6. Layer モデル

### 6.1 構造

- Layer = **content（0 or 1個）+ 属性 + sublayers（0..n）**。
- content を持たず sublayers を持つものがグループ。グループの属性・フィルタは
  サブツリー合成後の1枚に掛かる（グループ透過・グループ影・グループぼかし）。
- 合成順 = ツリーの前順（後に追加したものが上）。`zIndex` は導入しない。

### 6.2 属性（すべて既定値つき・実行中変更可・§9 により補間可能）

| 属性 | 型 / 既定 | 意味 |
|---|---|---|
| `frame` | Rect / 親いっぱい | 親ローカル座標での配置矩形（UIView.frame 相当の糖衣） |
| `position` | Vec2 | **anchor の親座標での位置**（CALayer.position 準拠） |
| `anchor` | Vec2 / (0.5,0.5) | 回転・拡縮・transform の基準点 |
| `masksToBounds` | bool / false | true でサブレイヤー/内容を境界（角丸含む）でクリップ |
| `rotation` | 度 / 0 | |
| `scale` | Vec2 / (1,1) | 反転は負値（`scale(-1, 1)` で左右反転） |
| `transform` | 2×3 行列 / 単位 | **完全なアフィン**（shear 含む）。簡易属性と合成（iOS と同型: 内部的には全レイヤーが1つのアフィン行列に解決される） |
| `opacity` | 0..1 / 1 | グループなら合成後に一括適用 |
| `shadow` | offset/blur/color | シルエット（α）から生成。**テキストにも枠にも掛かる** |
| `border` | width/color | クリップ形状（角丸含む）に沿う |
| `background` | Color / 透明 | レイヤー矩形の背景板 |
| `cornerRadius` | f32 / 0 | レイヤークリップと background/border の角丸 |
| `blend` | normal/add/multiply/screen | 親への合成モード |
| `hidden` | bool / false | 描画スキップ（ツリーからは消さない） |
| `filters` | Filter 列 / 空 | §8。宣言順に適用 |

### 6.3 ライフサイクル

- 生成: 描画メソッドがレイヤーを作り `Layer&` を返す。id は自動採番、後から触るもの
  だけ `id("hud")` で命名。
- 更新: 属性 setter はいつでも可（保持型・次フレーム反映）。データ系 content は
  `setImage / setPoints / setBoxes / setText` で差し替え。変更のないレイヤーは
  転送・再構築ともゼロコスト。
- 削除: `remove()`（子ごと）。一時退避は `hidden(true)`。
- スレッド: Stage は単一スレッド所有。レンダラーとの受け渡しはフレーム境界。

## 7. content カタログ

共通スタイル: `color`（既定 白）/ `thickness`（0=塗り）/ `dash`（0=実線）/
`cap`（round|butt）/ `cornerRadius`。全て SDF 描画（AA 付き・拡大無劣化）。

| content | 生成呼び出し | 備考 |
|---|---|---|
| image | `image(view, fit)` | view は借用（render まで有効）。`sourceRect`（CALayer contentsRect 相当）で**元画像の部分矩形だけを切り出して貼れる** — 画像の部分コピー＆ペーストはこれと frame の組で書く |
| text | `text(utf8, pos)` | `size`/`align`。HarfBuzz 整形、欠落グリフは決定的代替 |
| line | `line(from, to)` | dash/cap 対応の単線 |
| polyline | `polyline(pts)` | 経路。カプセル SDF、丸継ぎ目 |
| polygon | `polygon(pts)` | 閉路の塗り（thickness>0 で枠線） |
| rect | `rect(r)` | cornerRadius、塗り/枠 |
| circle | `circle(c, r)` | 塗り/リング |
| circles | `circles(pts, r)` | 点群マーカー。thickness 0 が「点」 |
| arc | `arc(c, r, a0, a1)` | ゲージ・角度。度指定 |
| arrow | `arrow(from, to)` | 方向・速度ベクトル |
| crosshair | `crosshair(c, size)` | 照準・中心マーク |
| grid | `grid(spacing)` | デバッグ・キャリブ格子 |
| boxes | `boxes(dets)` | 検出枠。cornerRadius / smoothing（EMA+最近傍）対応 |

## 8. フィルタ

- 本体は `filters_shared.h`（GLSL∩C++ の単一定義、現行 28 種 + gaussian blur）。
  カタログと優先度（lens_undistort 等 P0 含む）は
  [fluent_scene_2d_api.ja.md §3](fluent_scene_2d_api.ja.md) が正。
- **適用対象は任意の Layer**。グループに掛ければ合成後の1枚に掛かる。
- API は3段:
  1. よく使うものはメソッド — `.blur(4)` `.contrast(1.2f)`
  2. 全種は型付き構造体 — `.filter(Bilateral().radius(4).sigma_color(0.2))`
     （定義テーブルから生成。文字列キー・魔法数なし）
  3. Scene はインライン宣言 — `filters: [ {blur: {radius: 4}} ]`
- パラメータは論理単位（§5-3）。`$params` 連携（setParam でのライブ変更）は
  Scene 層の機能。

## 9. アニメーション — iOS の implicit animation を踏襲する

目標体験: **属性を変えるだけで、滑らかに動く**。

C++（Transaction — CATransaction と同型）:

```cpp
{
    Transaction t(0.3f, Ease::InOut);        // このスコープ内の属性変更は補間される
    stage.find("hud")->opacity(0.0f).position(24, -120);   // 0.3s でフェード+スライド
}
stage.find("hud")->hidden(true);             // スコープ外の変更は即時
```

Scene（属性の transition 宣言 + params の animate 宣言）:

```yaml
- id: hud
  opacity: $params.hud_alpha
  transition: { duration: 0.3, ease: ease_in_out }   # この層の属性変更は補間される
params:
  hud_alpha: { type: f32, default: 1.0, runtime_mutable: true,
               animate: { duration: 0.3, ease: ease_in_out } }  # setParam が曲線で効く
```

- 補間対象（L0）: opacity / position / scale / rotation / frame。色・フィルタ
  パラメータは L1。イージング: linear / ease_in / ease_out / ease_in_out
  （spring は将来）。
- 実装原理: 属性は「目標値 + 現在値 + 曲線」を持ち、レンダラーがフレーム dt で
  現在値を進める。データ側の補間（boxes の smoothing）は別系統として維持。
- 進行中の再変更は現在値から新目標へ繋ぐ（iOS と同じ。ジャンプしない）。

## 10. UI コントロール — レイヤーとしての設計（Phase L4）

ボタン・スイッチ等は**新しい描画機構ではなく、レイヤーの合成体**として定義する。

1. **コントロール = プレハブのサブツリー**。`ui.button` は「background rect
   （cornerRadius付き）+ text ラベル」のグループに、**状態機械**と**イベント契約**を
   足したもの。描画は既存の content/属性を使い、新しい描画プリミティブを増やさない。
2. **状態は属性の上書きとして宣言する**（iOS の UIControl state / CSS 擬似クラスと
   同型）。状態遷移は §9 の implicit animation で自動的に滑らかになる。

```yaml
- id: start_button
  frame: [24, 900, 220, 64]
  content: { button: { label: "収穫開始" } }
  states:
    pressed:  { scale: [0.96, 0.96], background: [0.1, 0.7, 0.55, 1] }
    disabled: { opacity: 0.4 }
- id: light_switch
  frame: [270, 900, 96, 48]
  content: { switch: { on: $params.light_on } }   # 状態はパラメータに束縛
```

3. **入力はポインタイベントに正規化して注入する**。Stage 自身は入力デバイスを
   持たない。Web ビューアのクリック・タッチ・VR コントローラのレイは、すべて
   論理座標のポインタイベントとして `stage.pointerDown/Move/Up(pos)` に入る。
   Stage が CALayer と同じ規則で hit-test（frame とツリー順、transform 考慮）する。
4. **イベントの出口は2系統**。C++ は `.onTap([]{...})` コールバック。Scene/ROS 世界
   では宣言した出力（例: `/ui/events` トピックに control id と新しい値）として
   publish — binding の outputs と同じ機構に乗せる。
5. 初期カタログ: `ui.button`（momentary）/ `ui.switch`（トグル）/
   `ui.slider`（値 0..1）/ `ui.gauge`（表示専用）。
   **実装状況（2026-08-14）**: §10-3 のポインタ注入と、button / switch /
   slider / segmented / gauge / dropdown の6種を C++ 先行実装済み
   （`fluent_stage/ui.hpp`）。segmented と dropdown は同日のオーナー指示で
   カタログに追加（2〜5択は segmented、多数択は dropdown）。dropdown の
   ポップアップは root 末尾のグループ + 全面透明スクリム（外側タップで
   閉じる）+ 上下自動開き + ドラッグスクロール（6単位でタップ/ドラッグ
   弁別）で、既存機構のみで構成。disabled はポインタを下へ素通し、状態遷移
   は Transaction 経由で自動アニメ。ポインタ波紋（同日オーナー要望）は
   `fx::Ripple`（effects.hpp）としてリング式を実装済み — 円レイヤー +
   Transaction のみ、dt 駆動で決定的、`max_rings` で有界。
   残: slider のキーボード/ステップ操作、フォーカスモデル（§10-6）、
   レイヤー単位のホバー状態、屈折式波紋（L1 の ripple フィルタ）。
6. 未決定として明示: 入力ソースの構成（配備ごとに web / タッチ / VR）、フォーカス
   モデル、長押し・ドラッグの扱い、アクセシビリティ。

## 11. レンダリング契約

- `Surface frame = renderer.render(stage /*, output_w, output_h */)`。バックエンドは
  CPU（リファレンス）と Vulkan（本番）。同一ツリーから同一の絵（許容差内）を出す
  ことをテストで保証。
- 保持型: パイプライン・アトラス・バッファはロード時のみ構築。フレーム毎は
  「変更データの転送 + 記録済み処理の再生」。シェーダーのフレーム毎コンパイルは
  ゼロ（カウンターで証明）。
- 属性/グループ/フィルタ付き Layer は transient 中間ターゲットへ描画して合成
  （予算計上は planner 連携）。無属性の葉レイヤーは直接合成でゼロオーバーヘッド。

## 12. ドキュメント体制 — OSS の当たり前を完成の定義に含める

**push の前提条件**（欠けていたら push しない）:

1. **全公開 API にドキュメンテーションコメント**（Doxygen 形式）。ヘッダが一次
   ソースで、全クラス・全メソッド・全パラメータ・全既定値に説明が付く。
2. docs 構成:
   - `getting-started`（5分で最初の絵が出るチュートリアル、C++ と Scene の両方）
   - `api/`（リファレンス。ヘッダから生成し、手書きの補足を持つ）
   - `cookbook/`（やりたいこと別: PiP、経路表示、HUD パネル、影付きテキスト、
     フィルタチェーン、ライブパラメータ…）
   - `design/`（本文書群 — なぜこうなっているか）
3. 例（examples/）は**全部ビルド・実行可能**で、CI（テスト）に含める。
4. README: 開発中の正は日本語。**OSS 公開時は英語 README を正面**にし日本語版を併設
   （GitHub 検索で世界の開発者と AI に見つけてもらうため。世界中のロボットで使われる
   未来がこのプロジェクトのゴール）。CHANGELOG を維持。
5. **トップページは L0/L2 完成時に全面刷新**（現行はテンプレ的で刷新対象と
   オーナー判定済み）。新構成: 冒頭に実レンダリング画像1枚 + 一行ピッチ →
   C++ 3行と YAML の対比（スクロールなしで価値が伝わる）→ 5分クイックスタート →
   カタログ → AI 協調。バッジは CI/license/version の3つまで、絵文字見出しは不使用。
5. カタログ系ページ（デモ・ノードカタログ）は実装と同期して更新する。

## 13. AI 協調 — AI が書き手・運用者になるための機構

Scene が「AI が安全に書ける形式」であることを、次の8つの機構で具体化する。

1. **能力の自己記述（capability discovery）** — ノード・フィルタ・属性のスキーマを
   単一定義テーブルから **JSON カタログとして自動生成**し、`fvsc describe --json` と
   MCP `node_types.list` で返す。AI はロボットごとの機能差に追従し、一発で valid な
   Scene を書ける。*L2 の完了条件。*
2. **デザインリンター** — 型検査の先の「良い」を警告する: テキスト/背景のコントラスト比
   （WCAG 4.5:1）、テキストの frame はみ出し、完全遮蔽レイヤー、画面外配置。
   `validate` の warning として返し、AI の生成ループをそのまま品質ループにする。
   **「美しく」を機械化する仕組み。** *L2 の完了条件。*
3. **シーンインスペクター** — 計算済み frame・実効 opacity・遮蔽関係・
   「座標 (x,y) に見えているものは何か」を問える API（DOM インスペクター相当）。
   preview 画像と組で、AI が「なぜ見えない」を自己解決できる。*L2〜L3。*
4. **保護レイヤー** — E-STOP 表示等の安全上重要なレイヤーに `protected: true`。
   通常権限のパッチでは非表示・削除・遮蔽（上への不透明配置）ができない。
   「AI が画面を自由に編集できる」と「安全表示は絶対に消えない」を両立する。
   検証はコンパイル時（遮蔽検査はリンターと同機構）。*設計は今、強制は MCP 実装時。*
5. **意味タグ** — レイヤーに `role: speed-indicator` 等の自由メタデータ。
   AI が「速度表示を大きくして」を id 探索なしで解決できる。*小、随時。*
6. **決定的アニメーションテスト** — Transaction/補間にも時刻注入を通し、
   「t=0.15s の画面」を再現可能にする（binding と同じ原則）。*L0 の設計に含める。*
7. **生成来歴** — Scene メタデータに `generated_by`（モデル・日時・プロンプトの
   ハッシュ）。digest と組で「この画面は誰がいつ作ったか」を監査できる。*小、随時。*
8. **正準フォーマッタ（`fvsc fmt`）** — AI 編集 → 正規化出力 → 最小 diff。
   人間レビューと digest 運用に噛み合う。*小、L2 併設。*

## 14. 実装フェーズ（承認後に着手）

| Phase | 範囲 | 完了条件 |
|---|---|---|
| L0 | Stage API + CPU バックエンド + Transaction（§9 の基盤、時刻注入 §13-6） | 全 content・全属性・フィルタ・グループ・補間が C++ 単体で動き golden 通過。**§12 のドキュメント同時完成** |
| L1 | Vulkan バックエンド | CPU と同一出力・per-frame compile 0 |
| L2 | Scene v1alpha2（レイヤーツリー YAML）→ Stage 構築 | YAML と C++ で同一出力。旧 nodes+composite 形式は廃棄。**§13-1 describe --json と §13-2 デザインリンターを含む** |
| L3 | binding / ROS / ページ・テスト移行 + インスペクター（§13-3） | 既存 E2E の復元 |
| L4 | UI コントロール（§10）+ 保護レイヤー強制（§13-4、MCP と共に） | button/switch がポインタ注入で動き、イベントが topic/コールバックに出る |

## 15. 未決定事項（承認前に潰す）

1. サイズを持たない content（text/line 等）の frame 省略時の境界
   — 提案: content の外接矩形を自動採用し、shadow/clip はそれ基準
2. blend の初期セットは normal/add/multiply/screen の4つで開始してよいか
3. 縮小時ミップマップの優先度（P1 でよいか）
4. レイヤーの動的増減と予算 — 提案: Stage 生成時に上限（max_layers 等）を宣言し、
   超過は即エラー（暗黙の再確保をしない）
5. Surface の所有権 — 提案: レンダラーが保持しビューを貸す（`frame.pixels()` は
   次の render まで有効）
6. §9 Transaction のネスト規則と、進行中アニメーションの query API
7. §10 の入力ソース抽象（web ビューア/タッチ/VR）の最小共通形
