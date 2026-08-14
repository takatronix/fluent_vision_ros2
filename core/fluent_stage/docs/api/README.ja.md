# API リファレンス

**一次ソースはヘッダです。** 全公開クラス・全メソッド・全パラメータ・全既定値に
Doxygen コメントが付いており、リファレンスはそこから生成します:

```bash
sudo apt install doxygen graphviz
doxygen docs/api/Doxyfile      # → build/docs/api/html/index.html
```

## ヘッダ地図（読む順）

| ヘッダ | 内容 |
|---|---|
| [fluent_stage.hpp](../../include/fluent_stage/fluent_stage.hpp) | 傘ヘッダ。これだけincludeすればよい |
| [stage.hpp](../../include/fluent_stage/stage.hpp) | `Stage` — 論理座標系の宣言、ツリーの根、`advance(dt)`、`hitTest` |
| [layer.hpp](../../include/fluent_stage/layer.hpp) | `Layer` — 描画呼び出し、幾何(bounds/anchor/position/frame)、属性、フィルタ、データ更新 |
| [content.hpp](../../include/fluent_stage/content.hpp) | content 13種の定義（何を描くか） |
| [types.hpp](../../include/fluent_stage/types.hpp) | Shadow / Border / ImageView / Box / 各enum / StageLimits |
| [geometry.hpp](../../include/fluent_stage/geometry.hpp) | Vec2 / Rect / Mat23 / Color |
| [filters.hpp](../../include/fluent_stage/filters.hpp) | `Filter` 値と型付き29種、`filterTable()` |
| [transaction.hpp](../../include/fluent_stage/transaction.hpp) | `Transaction` — implicit animation のスコープ |
| [animation.hpp](../../include/fluent_stage/animation.hpp) | `Animated<T>` とイージング（内部機構だが公開） |
| [renderer.hpp](../../include/fluent_stage/renderer.hpp) | `Renderer` 契約（dt注入・Surface貸出） |
| [cpu_renderer.hpp](../../include/fluent_stage/cpu_renderer.hpp) | リファレンスバックエンド |
| [vulkan_renderer.hpp](../../include/fluent_stage/vulkan_renderer.hpp) | GPU本番バックエンド（CPUと同一出力・実行時シェーダーコンパイルゼロ） |
| [surface.hpp](../../include/fluent_stage/surface.hpp) | `Surface` — RGBA8出力ビュー |

## 単一ソース（shared/）

[shared/](../../include/fluent_stage/shared/) は GLSL∩C++ の共通部分集合で
書かれた**定義の一次ソース**です。CPU も GPU（L1）も同じ本体をコンパイル
します。

- `filters_shared.h` — 全フィルタ本体 + mode id + ディスパッチ
- `filters_def.h` — フィルタのメタデータ（名前・パラメータ・既定値・単位）。
  X-macro で型付きstruct・既定値・specテーブルを導出
- `shapes_shared.h` — 全形状の SDF
- `glsl_compat.hpp` — CPU 用の GLSL 互換シム

バックエンド間で判断が揃う仕組みは `src/render_shared.hpp`（共有プラン層:
offscreen判定・extent・dash分割）と `backend/vulkan/shaders/`（同じ共有ヘッダを
GLSLとしてSPIR-V化）にあります。

フィルタを1つ足す = `filters_shared.h` に関数1個+ディスパッチ1行、
`filters_def.h` にブロック1個。それだけで型付きAPI・カタログ・
`filters_tour` example・（L2で）Scene スキーマに現れます。
