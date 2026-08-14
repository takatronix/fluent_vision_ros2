// cpu_renderer.cpp — the reference compositor (§11, Phase L0).
//
// Correctness first, speed second: this backend defines what every frame
// must look like, and the Vulkan backend (L1) is validated against it.
//
// Pipeline per layer:
//   direct path   — plain leaves evaluate their SDF straight into the
//                   target through the full local→output transform
//                   (rotation stays vector-crisp);
//   offscreen path — layers with filters, shadows, masking, group opacity,
//                   or non-normal group blending render their subtree into
//                   a transient premultiplied-float buffer at output
//                   density, filter it, derive the shadow from its alpha,
//                   and composite with bilinear sampling.
//
// Internal pixel format: RGBA float32, premultiplied alpha, sRGB-encoded
// (compositing in encoded space, like the platform compositors this
// library's look is calibrated against). Surface output converts to
// straight-alpha RGBA8.

#include "fluent_stage/cpu_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <variant>
#include <vector>

#include "fluent_stage/shared/glsl_compat.hpp"

namespace fluent_stage {
namespace shapes {
using namespace glsl;
#include "fluent_stage/shared/shapes_shared.h"
}  // namespace shapes
}  // namespace fluent_stage

#include "fluent_stage/stage.hpp"
#include "render_shared.hpp"
#include "text_atlas.hpp"

namespace fluent_stage {

namespace {

using glsl::vec2;
using glsl::vec3;
using glsl::vec4;
using plan::dashSegments;
using plan::IRect;
using plan::pathSegments;
using plan::rectIntersect;
using plan::rectPad;
using plan::rectUnion;
using plan::scaleOf;
using plan::Seg;
using plan::segBounds;
using plan::targetBBox;

vec2 gv(Vec2 v) { return {v.x, v.y}; }

constexpr float kPi = 3.14159265358979f;

// ---------------------------------------------------------------------------
// Buf — premultiplied RGBA float32 pixels
// ---------------------------------------------------------------------------

struct Buf {
    int w = 0;
    int h = 0;
    std::vector<float> px;  // w * h * 4

    void reset(int width, int height) {
        w = width;
        h = height;
        px.assign(static_cast<size_t>(w) * h * 4, 0.0f);
    }
    float* at(int x, int y) { return px.data() + (static_cast<size_t>(y) * w + x) * 4; }
    const float* at(int x, int y) const {
        return px.data() + (static_cast<size_t>(y) * w + x) * 4;
    }
};

// Bilinear sample of a premultiplied buffer; outside is transparent.
void sampleBilinear(const Buf& b, float x, float y, float out[4]) {
    const float fx = x - 0.5f;
    const float fy = y - 0.5f;
    const int x0 = static_cast<int>(std::floor(fx));
    const int y0 = static_cast<int>(std::floor(fy));
    const float tx = fx - x0;
    const float ty = fy - y0;
    out[0] = out[1] = out[2] = out[3] = 0;
    for (int dy = 0; dy <= 1; ++dy) {
        for (int dx = 0; dx <= 1; ++dx) {
            const int xi = x0 + dx;
            const int yi = y0 + dy;
            if (xi < 0 || yi < 0 || xi >= b.w || yi >= b.h) {
                continue;
            }
            const float wgt = (dx ? tx : 1 - tx) * (dy ? ty : 1 - ty);
            const float* p = b.at(xi, yi);
            for (int c = 0; c < 4; ++c) {
                out[c] += p[c] * wgt;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Blending — THE definition of the four modes, shared with the GPU.
//
// All buffers are premultiplied. Each mode is a fixed-function blend
// equation (cs/cd premultiplied source/dest, sa source alpha), so the
// Vulkan backend realizes the identical math as blend factors:
//
//   Normal:   co = cs           + cd*(1-sa)     (ONE, ONE_MINUS_SRC_ALPHA)
//   Add:      co = cs           + cd            (ONE, ONE)
//   Multiply: co = cs*cd        + cd*(1-sa)     (DST_COLOR, ONE_MINUS_SRC_ALPHA)
//   Screen:   co = cs*(1-cd)    + cd            (ONE_MINUS_DST_COLOR, ONE)
//   ao = sa + da*(1-sa)                          (all modes)
//
// Values may exceed 1 mid-composite (Add); the final surface conversion
// clamps, exactly like a float16 render target read back through unpremul.
// ---------------------------------------------------------------------------

// Composites a premultiplied source sample over dst.
void blendPremul(float* dst, const float src[4], Blend mode) {
    const float sa = src[3];
    if (sa <= 0) {
        return;
    }
    for (int c = 0; c < 3; ++c) {
        switch (mode) {
            case Blend::Normal:   dst[c] = src[c] + dst[c] * (1 - sa); break;
            case Blend::Add:      dst[c] = src[c] + dst[c]; break;
            case Blend::Multiply: dst[c] = src[c] * dst[c] + dst[c] * (1 - sa); break;
            case Blend::Screen:   dst[c] = src[c] * (1 - dst[c]) + dst[c]; break;
        }
    }
    dst[3] = sa + dst[3] * (1 - sa);
}

// Composites one straight-alpha source sample (color + coverage) over dst.
void blendPixel(float* dst, Color src, float alpha, Blend mode) {
    const float sa = src.a * alpha;
    if (sa <= 0) {
        return;
    }
    const float premul[4] = {src.r * sa, src.g * sa, src.b * sa, sa};
    blendPremul(dst, premul, mode);
}

// ---------------------------------------------------------------------------
// Coverage mask — correct antialiased union of overlapping strokes
// ---------------------------------------------------------------------------

// Multi-part content (polyline joints, dashes, markers, grids) must not
// double-blend where parts overlap, so parts accumulate max-coverage into a
// mask over the content's target bbox, and the mask composites once.
struct Mask {
    IRect box;
    std::vector<float> cov;

    void reset(IRect b) {
        box = b;
        cov.assign(static_cast<size_t>(std::max(0, b.x1 - b.x0)) *
                       static_cast<size_t>(std::max(0, b.y1 - b.y0)),
                   0.0f);
    }
    float* at(int x, int y) {
        return cov.data() + static_cast<size_t>(y - box.y0) * (box.x1 - box.x0) + (x - box.x0);
    }
};

// Rasterizes one part: evaluates `coverage(local)` over the target bbox of
// `local_bbox` and max-accumulates into the mask.
template <typename CoverageFn>
void rasterizePart(Mask& mask, const Mat23& inv, const Mat23& fwd, Rect local_bbox, float pad,
                   int w, int h, CoverageFn coverage) {
    IRect r = targetBBox(fwd, local_bbox, pad, w, h);
    r.x0 = std::max(r.x0, mask.box.x0);
    r.y0 = std::max(r.y0, mask.box.y0);
    r.x1 = std::min(r.x1, mask.box.x1);
    r.y1 = std::min(r.y1, mask.box.y1);
    if (r.empty()) {
        return;
    }
    for (int y = r.y0; y < r.y1; ++y) {
        for (int x = r.x0; x < r.x1; ++x) {
            const Vec2 local = inv.apply({x + 0.5f, y + 0.5f});
            const float c = coverage(gv(local));
            if (c > 0) {
                float* m = mask.at(x, y);
                *m = std::max(*m, c);
            }
        }
    }
}

void compositeMask(Buf& target, const Mask& mask, Color color, float alpha, Blend mode) {
    for (int y = mask.box.y0; y < mask.box.y1; ++y) {
        for (int x = mask.box.x0; x < mask.box.x1; ++x) {
            const float c =
                mask.cov[static_cast<size_t>(y - mask.box.y0) * (mask.box.x1 - mask.box.x0) +
                         (x - mask.box.x0)];
            if (c > 0) {
                blendPixel(target.at(x, y), color, alpha * c, mode);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Filters — the shared bodies, compiled for the CPU
// ---------------------------------------------------------------------------

thread_local const Buf* g_filter_src = nullptr;

vec3 filterSample(vec2 uv) {
    const Buf& b = *g_filter_src;
    int x = static_cast<int>(uv.x * b.w);
    int y = static_cast<int>(uv.y * b.h);
    x = std::clamp(x, 0, b.w - 1);
    y = std::clamp(y, 0, b.h - 1);
    const float* p = b.at(x, y);
    // Bodies see straight rgb, like a GPU sampling a resolved texture.
    const float a = p[3];
    if (a <= 0) {
        return {0, 0, 0};
    }
    return {p[0] / a, p[1] / a, p[2] / a};
}

vec2 filterTexel() { return {1.0f / g_filter_src->w, 1.0f / g_filter_src->h}; }

namespace filter_bodies {
using namespace glsl;
#define FS_SAMPLE(uv) filterSample(uv)
#define FS_TEXEL filterTexel()
#include "fluent_stage/shared/filters_shared.h"
#undef FS_SAMPLE
#undef FS_TEXEL
}  // namespace filter_bodies

// Separable gaussian blur on the premultiplied buffer (radius in buffer px).
void gaussianBlur(Buf& buf, float radius_px) {
    if (radius_px <= 0.01f || buf.w == 0 || buf.h == 0) {
        return;
    }
    const float sigma = radius_px * 0.5f;
    const int half = std::max(1, static_cast<int>(std::ceil(sigma * 2.5f)));
    std::vector<float> weights(half + 1);
    float sum = 0;
    for (int i = 0; i <= half; ++i) {
        weights[i] = std::exp(-0.5f * (i / sigma) * (i / sigma));
        sum += i == 0 ? weights[i] : 2 * weights[i];
    }
    for (float& w : weights) {
        w /= sum;
    }
    Buf tmp;
    tmp.reset(buf.w, buf.h);
    for (int y = 0; y < buf.h; ++y) {  // horizontal
        for (int x = 0; x < buf.w; ++x) {
            float acc[4] = {0, 0, 0, 0};
            for (int i = -half; i <= half; ++i) {
                const int xi = std::clamp(x + i, 0, buf.w - 1);
                const float* p = buf.at(xi, y);
                const float w = weights[std::abs(i)];
                for (int c = 0; c < 4; ++c) {
                    acc[c] += p[c] * w;
                }
            }
            std::memcpy(tmp.at(x, y), acc, sizeof acc);
        }
    }
    for (int y = 0; y < buf.h; ++y) {  // vertical
        for (int x = 0; x < buf.w; ++x) {
            float acc[4] = {0, 0, 0, 0};
            for (int i = -half; i <= half; ++i) {
                const int yi = std::clamp(y + i, 0, buf.h - 1);
                const float* p = tmp.at(x, yi);
                const float w = weights[std::abs(i)];
                for (int c = 0; c < 4; ++c) {
                    acc[c] += p[c] * w;
                }
            }
            std::memcpy(buf.at(x, y), acc, sizeof acc);
        }
    }
}

// Applies one filter to the buffer. `scale` converts logical units to
// buffer pixels; `ext` is the local-space rect the buffer covers (§5-3).
void applyFilter(Buf& buf, const Filter& f, float scale, Rect ext) {
    float values[5];
    plan::scaleFilterValues(f, scale, ext, static_cast<float>(buf.w),
                            static_cast<float>(buf.h), values);
    if (f.mode == FS_BLUR) {
        gaussianBlur(buf, values[0]);
        return;
    }
    Buf src = buf;  // filters read the unmodified source
    g_filter_src = &src;
    for (int y = 0; y < buf.h; ++y) {
        for (int x = 0; x < buf.w; ++x) {
            const float src_a = src.at(x, y)[3];
            const vec2 uv{(x + 0.5f) / buf.w, (y + 0.5f) / buf.h};
            const vec4 out = filter_bodies::fs_apply(f.mode, uv, values[0], values[1],
                                                     values[2], values[3], values[4]);
            float* d = buf.at(x, y);
            // Filters keep the source's alpha shape (resampling filters
            // approximate; exact per-filter alpha is refined with L1 golden
            // work). FS_OPACITY scales it.
            const float a = f.mode == FS_OPACITY ? src_a * out.w : src_a;
            d[0] = out.x * a;
            d[1] = out.y * a;
            d[2] = out.z * a;
            d[3] = a;
        }
    }
    g_filter_src = nullptr;
}

// Signed distance to a polygon (negative inside, any winding; per-edge math
// mirrors sd_segment — the loop is renderer structure, per shapes_shared.h).
float polygonDistance(const std::vector<Vec2>& pts, vec2 p) {
    float d2 = 1e30f;
    float sign = 1.0f;
    for (size_t i = 0, j = pts.size() - 1; i < pts.size(); j = i++) {
        const vec2 vi = gv(pts[i]);
        const vec2 vj = gv(pts[j]);
        const vec2 e = vj - vi;
        const vec2 w = p - vi;
        const float t = glsl::clamp(glsl::dot(w, e) / std::max(glsl::dot(e, e), 1e-12f), 0.0f, 1.0f);
        const vec2 b = w - e * t;
        d2 = std::min(d2, glsl::dot(b, b));
        const bool c0 = p.y >= vi.y;
        const bool c1 = p.y < vj.y;
        const bool c2 = e.x * w.y > e.y * w.x;
        if ((c0 && c1 && c2) || (!c0 && !c1 && !c2)) {
            sign = -sign;
        }
    }
    return sign * std::sqrt(d2);
}

}  // namespace

// ===========================================================================
// CpuRenderer::Impl
// ===========================================================================

struct CpuRenderer::Impl {
    Options options;
    detail::TextAtlas atlas;
    bool atlas_tried = false;
    Buf frame;
    std::vector<uint8_t> out_rgba;
    Surface surface;

    // ---- fonts ------------------------------------------------------------

    void ensureAtlas() {
        if (atlas.ready() || atlas_tried) {
            return;
        }
        atlas_tried = true;
        detail::initAtlasWithFallback(atlas, options.font_file, options.font_pixel_size);
    }

    // Text metrics in logical units for a content (Stage measurement hook).
    Rect measureText(const TextContent& c) {
        ensureAtlas();
        if (!atlas.ready()) {
            return {};
        }
        const float s = c.size / static_cast<float>(atlas.pixelSize());
        const float w = atlas.measure(c.utf8) * s;
        const float h = atlas.lineHeight() * s;
        float x = c.position.x;
        if (c.align == Align::Center) {
            x -= w * 0.5f;
        } else if (c.align == Align::Right) {
            x -= w;
        }
        return {x, c.position.y, w, h};
    }

    // ---- extent of a subtree in its own local space -----------------------

    plan::TextMeasure measurer() {
        return [this](const TextContent& c) { return measureText(c); };
    }

    // ---- content rasterization --------------------------------------------

    // Draws the layer's content into `target` through `m` (content-local →
    // target), with `alpha` folded in and the layer's blend mode.
    void drawContent(Buf& target, const Layer& layer, Rect bounds, const Mat23& m, float alpha) {
        const Content& content = layer.content();
        if (std::holds_alternative<std::monostate>(content)) {
            return;
        }
        const Mat23 inv = m.inverse();
        const float scale = scaleOf(m);
        if (scale <= 0) {
            return;
        }
        const float aa = 1.0f / scale;
        const float th = layer.thickness();
        const Color color = layer.colorValue();
        const Blend mode = layer.blendMode();
        const float pad_px = (th * 0.5f + 2.0f) * scale;

        // Shared plumbing: build a mask over the whole content, rasterize
        // parts into it, composite once.
        Mask mask;
        const auto begin = [&](Rect local_bbox, float extra_pad_logical) {
            mask.reset(targetBBox(m, local_bbox, pad_px + extra_pad_logical * scale,
                                  target.w, target.h));
            return !mask.box.empty();
        };
        const auto part = [&](Rect local_bbox, float extra_pad_logical, auto coverage) {
            rasterizePart(mask, inv, m, local_bbox, pad_px + extra_pad_logical * scale,
                          target.w, target.h, coverage);
        };
        const auto finish = [&] { compositeMask(target, mask, color, alpha, mode); };

        const auto strokeSegs = [&](const std::vector<Seg>& segs, Cap capstyle) {
            for (const Seg& s : segs) {
                const vec2 a = gv(s.a);
                const vec2 b = gv(s.b);
                if (capstyle == Cap::Butt) {
                    part(segBounds(s), 0, [&, a, b](vec2 p) {
                        return shapes::sd_fill(shapes::sd_segment_butt(p, a, b, th * 0.5f), aa);
                    });
                } else {
                    part(segBounds(s), 0, [&, a, b](vec2 p) {
                        return shapes::sd_stroke(shapes::sd_segment(p, a, b), th, aa);
                    });
                }
            }
        };

        if (const auto* c = std::get_if<ImageContent>(&content)) {
            drawImage(target, *c, bounds, m, inv, alpha, mode);
        } else if (const auto* c = std::get_if<TextContent>(&content)) {
            drawText(target, *c, m, inv, color, alpha, mode, scale);
        } else if (const auto* c = std::get_if<LineContent>(&content)) {
            if (begin(segBounds({c->from, c->to}), 0)) {
                strokeSegs(dashSegments({{c->from, c->to}}, layer.dashValue()), layer.capValue());
                finish();
            }
        } else if (const auto* c = std::get_if<PolylineContent>(&content)) {
            if (c->points.size() >= 2 && begin(layer.contentBounds(), 0)) {
                strokeSegs(dashSegments(pathSegments(c->points, false), layer.dashValue()),
                           layer.capValue());
                finish();
            }
        } else if (const auto* c = std::get_if<PolygonContent>(&content)) {
            if (c->points.size() >= 3 && begin(layer.contentBounds(), 0)) {
                if (th > 0) {
                    strokeSegs(dashSegments(pathSegments(c->points, true), layer.dashValue()),
                               layer.capValue());
                } else {
                    const auto& pts = c->points;
                    part(layer.contentBounds(), 0, [&](vec2 p) {
                        return shapes::sd_fill(polygonDistance(pts, p), aa);
                    });
                }
                finish();
            }
        } else if (const auto* c = std::get_if<RectContent>(&content)) {
            if (begin(c->rect, 0)) {
                const vec2 center = gv(c->rect.center());
                const vec2 half{c->rect.w * 0.5f, c->rect.h * 0.5f};
                const float cr = std::min(layer.cornerRadius(),
                                          std::min(half.x, half.y));
                part(c->rect, 0, [&, center, half, cr](vec2 p) {
                    const float d = shapes::sd_rounded_rect(p, center, half, cr);
                    return th > 0 ? shapes::sd_stroke(d, th, aa) : shapes::sd_fill(d, aa);
                });
                finish();
            }
        } else if (const auto* c = std::get_if<CircleContent>(&content)) {
            if (begin(layer.contentBounds(), 0)) {
                part(layer.contentBounds(), 0, [&](vec2 p) {
                    const float d = shapes::sd_circle(p, gv(c->center), c->radius);
                    return th > 0 ? shapes::sd_stroke(d, th, aa) : shapes::sd_fill(d, aa);
                });
                finish();
            }
        } else if (const auto* c = std::get_if<CirclesContent>(&content)) {
            if (!c->centers.empty() && begin(layer.contentBounds(), 0)) {
                for (const Vec2& center : c->centers) {
                    const Rect b{center.x - c->radius, center.y - c->radius, 2 * c->radius,
                                 2 * c->radius};
                    part(b, 0, [&, center](vec2 p) {
                        const float d = shapes::sd_circle(p, gv(center), c->radius);
                        return th > 0 ? shapes::sd_stroke(d, th, aa) : shapes::sd_fill(d, aa);
                    });
                }
                finish();
            }
        } else if (const auto* c = std::get_if<ArcContent>(&content)) {
            if (begin(layer.contentBounds(), 0)) {
                float a0 = c->start_deg * kPi / 180.0f;
                float a1 = c->end_deg * kPi / 180.0f;
                if (a1 < a0) {
                    std::swap(a0, a1);
                }
                a1 = std::min(a1, a0 + 2 * kPi);
                part(layer.contentBounds(), 0, [&, a0, a1](vec2 p) {
                    const float d = shapes::sd_arc(p, gv(c->center), c->radius, a0, a1);
                    return shapes::sd_fill(d - th * 0.5f, aa);
                });
                finish();
            }
        } else if (const auto* c = std::get_if<ArrowContent>(&content)) {
            const float len = std::hypot(c->to.x - c->from.x, c->to.y - c->from.y);
            if (len > 1e-4f) {
                const float head =
                    c->head_size > 0 ? c->head_size : std::max(3.0f * th, 9.0f);
                if (begin(rectPad(layer.contentBounds(), head), 0)) {
                    const Vec2 dir{(c->to.x - c->from.x) / len, (c->to.y - c->from.y) / len};
                    const Vec2 neck{c->to.x - dir.x * head, c->to.y - dir.y * head};
                    const Vec2 side{-dir.y * head * 0.5f, dir.x * head * 0.5f};
                    const vec2 tip = gv(c->to);
                    const vec2 w0 = gv({neck.x + side.x, neck.y + side.y});
                    const vec2 w1 = gv({neck.x - side.x, neck.y - side.y});
                    strokeSegs({{c->from, neck}}, layer.capValue());
                    part(rectPad(segBounds({neck, c->to}), head), 0, [&, tip, w0, w1](vec2 p) {
                        return shapes::sd_fill(shapes::sd_triangle(p, tip, w0, w1), aa);
                    });
                    finish();
                }
            }
        } else if (const auto* c = std::get_if<CrosshairContent>(&content)) {
            if (begin(layer.contentBounds(), 0)) {
                const float gap = c->gap > 0 ? c->gap : c->size * 0.25f;
                const Vec2 o = c->center;
                const Seg ticks[4] = {{{o.x + gap, o.y}, {o.x + c->size, o.y}},
                                      {{o.x - gap, o.y}, {o.x - c->size, o.y}},
                                      {{o.x, o.y + gap}, {o.x, o.y + c->size}},
                                      {{o.x, o.y - gap}, {o.x, o.y - c->size}}};
                strokeSegs({ticks[0], ticks[1], ticks[2], ticks[3]}, layer.capValue());
                finish();
            }
        } else if (const auto* c = std::get_if<GridContent>(&content)) {
            if (begin(bounds, 0)) {
                const vec2 center = gv(bounds.center());
                const vec2 half{bounds.w * 0.5f, bounds.h * 0.5f};
                const float spacing = c->spacing;
                part(bounds, 0, [&, center, half, spacing](vec2 p) {
                    const float lines = shapes::sd_stroke(shapes::sd_grid(p, spacing), th, aa);
                    const float inside =
                        shapes::sd_fill(shapes::sd_rounded_rect(p, center, half, 0.0f), aa);
                    return lines * inside;
                });
                finish();
            }
        } else if (const auto* c = std::get_if<BoxesContent>(&content)) {
            if (!c->smoothed.empty() && begin(rectPad(layer.contentBounds(), 20.0f), 0)) {
                for (const Box& box : c->smoothed) {
                    const vec2 center = gv(box.rect.center());
                    const vec2 half{box.rect.w * 0.5f, box.rect.h * 0.5f};
                    const float cr = std::min(layer.cornerRadius(),
                                              std::min(half.x, half.y));
                    part(box.rect, 0, [&, center, half, cr](vec2 p) {
                        const float d = shapes::sd_rounded_rect(p, center, half, cr);
                        return th > 0 ? shapes::sd_stroke(d, th, aa) : shapes::sd_fill(d, aa);
                    });
                }
                finish();
                if (c->show_label) {
                    for (const Box& box : c->smoothed) {
                        std::string label = box.label;
                        if (box.score > 0) {
                            char score[16];
                            std::snprintf(score, sizeof score, "%.2f", box.score);
                            label += (label.empty() ? "" : " ") + std::string(score);
                        }
                        if (label.empty()) {
                            continue;
                        }
                        TextContent t;
                        t.utf8 = label;
                        t.size = 14;
                        t.position = {box.rect.x + 2, std::max(box.rect.y - 18.0f, 0.0f)};
                        drawText(target, t, m, inv, color, alpha, mode, scale);
                    }
                }
            }
        }
    }

    void drawImage(Buf& target, const ImageContent& c, Rect bounds, const Mat23& m,
                   const Mat23& inv, float alpha, Blend mode) {
        if (!c.view.valid() || bounds.w <= 0 || bounds.h <= 0) {
            return;
        }
        // Source window (sourceRect crop, clamped to the image).
        Rect src{0, 0, static_cast<float>(c.view.width), static_cast<float>(c.view.height)};
        if (c.source_rect.w > 0 && c.source_rect.h > 0) {
            src = rectIntersect(c.source_rect, src);
        }
        if (src.w <= 0 || src.h <= 0) {
            return;
        }
        // Fit: dest rect inside bounds, and the matching source window.
        Rect dest = bounds;
        if (c.fit == Fit::Contain) {
            const float s = std::min(bounds.w / src.w, bounds.h / src.h);
            dest = {bounds.x + (bounds.w - src.w * s) * 0.5f,
                    bounds.y + (bounds.h - src.h * s) * 0.5f, src.w * s, src.h * s};
        } else if (c.fit == Fit::Cover) {
            const float s = std::max(bounds.w / src.w, bounds.h / src.h);
            const float vis_w = bounds.w / s;
            const float vis_h = bounds.h / s;
            src = {src.x + (src.w - vis_w) * 0.5f, src.y + (src.h - vis_h) * 0.5f, vis_w, vis_h};
        }
        const IRect box = targetBBox(m, dest, 1, target.w, target.h);
        if (box.empty()) {
            return;
        }
        const uint32_t stride = c.view.stride();
        const auto texel = [&](int sx, int sy, float out[4]) {
            sx = std::clamp(sx, static_cast<int>(src.x),
                            static_cast<int>(src.x + src.w) - 1);
            sy = std::clamp(sy, static_cast<int>(src.y),
                            static_cast<int>(src.y + src.h) - 1);
            const uint8_t* p = c.view.pixels + static_cast<size_t>(sy) * stride + sx * 4;
            const float a = p[3] / 255.0f;
            out[0] = p[0] / 255.0f * a;
            out[1] = p[1] / 255.0f * a;
            out[2] = p[2] / 255.0f * a;
            out[3] = a;
        };
        for (int y = box.y0; y < box.y1; ++y) {
            for (int x = box.x0; x < box.x1; ++x) {
                const Vec2 local = inv.apply({x + 0.5f, y + 0.5f});
                if (local.x < dest.x || local.y < dest.y || local.x >= dest.x + dest.w ||
                    local.y >= dest.y + dest.h) {
                    continue;
                }
                const float sxf = src.x + (local.x - dest.x) * (src.w / dest.w) - 0.5f;
                const float syf = src.y + (local.y - dest.y) * (src.h / dest.h) - 0.5f;
                const int sx0 = static_cast<int>(std::floor(sxf));
                const int sy0 = static_cast<int>(std::floor(syf));
                const float tx = sxf - sx0;
                const float ty = syf - sy0;
                float acc[4] = {0, 0, 0, 0};
                float t00[4], t10[4], t01[4], t11[4];
                texel(sx0, sy0, t00);
                texel(sx0 + 1, sy0, t10);
                texel(sx0, sy0 + 1, t01);
                texel(sx0 + 1, sy0 + 1, t11);
                for (int ch = 0; ch < 4; ++ch) {
                    acc[ch] = (t00[ch] * (1 - tx) + t10[ch] * tx) * (1 - ty) +
                              (t01[ch] * (1 - tx) + t11[ch] * tx) * ty;
                    acc[ch] *= alpha;
                }
                blendPremul(target.at(x, y), acc, mode);
            }
        }
    }

    void drawText(Buf& target, const TextContent& c, const Mat23& m, const Mat23& inv,
                  Color color, float alpha, Blend mode, float scale) {
        ensureAtlas();
        if (!atlas.ready() || c.utf8.empty()) {
            return;
        }
        const float s = c.size / static_cast<float>(atlas.pixelSize());
        std::vector<detail::TextAtlas::GlyphQuad> quads;
        atlas.layout(c.utf8, 0, 0, 4096, quads);
        float align_dx = 0;
        if (c.align != Align::Left) {
            const float w = atlas.measure(c.utf8) * s;
            align_dx = c.align == Align::Center ? -w * 0.5f : -w;
        }
        Mask mask;
        const Rect text_box = measureText(c);
        mask.reset(targetBBox(m, rectPad(text_box, 2), 2 * scale, target.w, target.h));
        if (mask.box.empty()) {
            return;
        }
        for (const auto& q : quads) {
            // Quad in logical units.
            const Rect lq{c.position.x + align_dx + q.x * s, c.position.y + q.y * s, q.w * s,
                          q.h * s};
            IRect box = targetBBox(m, lq, 1, target.w, target.h);
            box.x0 = std::max(box.x0, mask.box.x0);
            box.y0 = std::max(box.y0, mask.box.y0);
            box.x1 = std::min(box.x1, mask.box.x1);
            box.y1 = std::min(box.y1, mask.box.y1);
            if (box.empty()) {
                continue;
            }
            const float u_span = q.u1 - q.u0;
            const float v_span = q.v1 - q.v0;
            for (int y = box.y0; y < box.y1; ++y) {
                for (int x = box.x0; x < box.x1; ++x) {
                    const Vec2 local = inv.apply({x + 0.5f, y + 0.5f});
                    const float fx = (local.x - lq.x) / lq.w;
                    const float fy = (local.y - lq.y) / lq.h;
                    if (fx < 0 || fy < 0 || fx >= 1 || fy >= 1) {
                        continue;
                    }
                    // Bilinear R8 atlas sample.
                    const float ax = (q.u0 + fx * u_span) * atlas.width() - 0.5f;
                    const float ay = (q.v0 + fy * v_span) * atlas.height() - 0.5f;
                    const int ax0 = std::clamp(static_cast<int>(std::floor(ax)), 0,
                                               static_cast<int>(atlas.width()) - 1);
                    const int ay0 = std::clamp(static_cast<int>(std::floor(ay)), 0,
                                               static_cast<int>(atlas.height()) - 1);
                    const int ax1 = std::min(ax0 + 1, static_cast<int>(atlas.width()) - 1);
                    const int ay1 = std::min(ay0 + 1, static_cast<int>(atlas.height()) - 1);
                    const float tx = std::clamp(ax - ax0, 0.0f, 1.0f);
                    const float ty = std::clamp(ay - ay0, 0.0f, 1.0f);
                    const uint8_t* px = atlas.pixels();
                    const auto tex = [&](int xx, int yy) {
                        return px[static_cast<size_t>(yy) * atlas.width() + xx] / 255.0f;
                    };
                    const float cov = (tex(ax0, ay0) * (1 - tx) + tex(ax1, ay0) * tx) * (1 - ty) +
                                      (tex(ax0, ay1) * (1 - tx) + tex(ax1, ay1) * tx) * ty;
                    if (cov > 0) {
                        float* mp = mask.at(x, y);
                        *mp = std::max(*mp, cov);
                    }
                }
            }
        }
        compositeMask(target, mask, color, alpha, mode);
    }

    // ---- background / border ---------------------------------------------

    void drawRoundedRect(Buf& target, Rect r, float corner, float stroke_w, const Mat23& m,
                         Color color, float alpha, Blend mode) {
        if (r.w <= 0 || r.h <= 0 || color.a <= 0 || alpha <= 0) {
            return;
        }
        const Mat23 inv = m.inverse();
        const float scale = scaleOf(m);
        if (scale <= 0) {
            return;
        }
        const float aa = 1.0f / scale;
        const vec2 center = gv(r.center());
        const vec2 half{r.w * 0.5f, r.h * 0.5f};
        const float cr = std::min(corner, std::min(half.x, half.y));
        const IRect box = targetBBox(m, r, (stroke_w + 2) * scale, target.w, target.h);
        for (int y = box.y0; y < box.y1; ++y) {
            for (int x = box.x0; x < box.x1; ++x) {
                const Vec2 lp = inv.apply({x + 0.5f, y + 0.5f});
                const float d = shapes::sd_rounded_rect(gv(lp), center, half, cr);
                const float cov =
                    stroke_w > 0 ? shapes::sd_stroke(d, stroke_w, aa) : shapes::sd_fill(d, aa);
                if (cov > 0) {
                    blendPixel(target.at(x, y), color, alpha * cov, mode);
                }
            }
        }
    }

    // ---- layer recursion ---------------------------------------------------

    void renderLayer(Buf& target, const Layer& layer, const Mat23& parent_m, Vec2 parent_size) {
        if (layer.hidden()) {
            return;
        }
        const float opacity = layer.presentedOpacity();
        if (opacity <= 0) {
            return;
        }
        const Layer::Resolved r = layer.resolve(parent_size);
        const Mat23 m = parent_m * r.to_parent;

        const bool needs_offscreen = plan::needsOffscreen(layer, opacity);

        if (!needs_offscreen) {
            const Blend mode = layer.blendMode();
            drawRoundedRect(target, r.bounds, layer.cornerRadius(), 0, m,
                            layer.backgroundValue(), opacity, mode);
            drawContent(target, layer, r.bounds, m, opacity);
            for (const auto& child : layer.sublayers()) {
                renderLayer(target, *child, m, r.bounds.size());
            }
            if (layer.borderValue()) {
                drawRoundedRect(target, r.bounds, layer.cornerRadius(),
                                layer.borderValue()->width, m, layer.borderValue()->color,
                                opacity, mode);
            }
            return;
        }

        // ---- offscreen path ----
        const float scale = scaleOf(m);
        Rect ext = plan::subtreeExtent(layer, parent_size, measurer());
        if (ext.w <= 0 || ext.h <= 0 || scale <= 0) {
            return;
        }
        ext = rectPad(ext, 2.0f / scale);
        const int bw = std::max(1, static_cast<int>(std::ceil(ext.w * scale)));
        const int bh = std::max(1, static_cast<int>(std::ceil(ext.h * scale)));
        if (static_cast<int64_t>(bw) * bh > 64LL * 1024 * 1024) {
            return;  // degenerate transform; refuse absurd allocations
        }
        Buf buf;
        buf.reset(bw, bh);
        // Local space → buffer: axis-aligned, at target density. The outer
        // rotation/position happens at composite time (bilinear), which is
        // exactly the §4.1-5 contract for filtered/shadowed subtrees.
        const Mat23 mb =
            Mat23::scaling({scale, scale}) * Mat23::translation({-ext.x, -ext.y});

        drawRoundedRect(buf, r.bounds, layer.cornerRadius(), 0, mb, layer.backgroundValue(),
                        1.0f, Blend::Normal);
        drawContent(buf, layer, r.bounds, mb, 1.0f);
        for (const auto& child : layer.sublayers()) {
            renderLayer(buf, *child, mb, r.bounds.size());
        }
        if (layer.borderValue()) {
            drawRoundedRect(buf, r.bounds, layer.cornerRadius(), layer.borderValue()->width,
                            mb, layer.borderValue()->color, 1.0f, Blend::Normal);
        }

        for (const Filter& f : layer.filters()) {
            applyFilter(buf, f, scale, ext);
        }

        const Mat23 inv = m.inverse();
        const vec2 mask_center = gv(r.bounds.center());
        const vec2 mask_half{r.bounds.w * 0.5f, r.bounds.h * 0.5f};
        const float mask_cr =
            std::min(layer.cornerRadius(), std::min(mask_half.x, mask_half.y));
        const float aa = 1.0f / scale;

        if (const auto& sh = layer.shadowValue()) {
            // Silhouette → blur → tint, composited under the body.
            Buf shadow_buf;
            shadow_buf.reset(bw, bh);
            for (size_t i = 3; i < buf.px.size(); i += 4) {
                shadow_buf.px[i] = buf.px[i];
            }
            gaussianBlur(shadow_buf, sh->radius * scale);
            const IRect box = targetBBox(
                m, rectPad({ext.x + sh->offset.x, ext.y + sh->offset.y, ext.w, ext.h},
                           sh->radius * 2),
                2, target.w, target.h);
            // blendPixel multiplies in sh->color.a itself.
            const float shadow_alpha = sh->opacity * opacity;
            for (int y = box.y0; y < box.y1; ++y) {
                for (int x = box.x0; x < box.x1; ++x) {
                    const Vec2 local = inv.apply({x + 0.5f, y + 0.5f});
                    float smp[4];
                    sampleBilinear(shadow_buf, (local.x - sh->offset.x - ext.x) * scale,
                                   (local.y - sh->offset.y - ext.y) * scale, smp);
                    if (smp[3] > 0) {
                        blendPixel(target.at(x, y), sh->color, shadow_alpha * smp[3],
                                   Blend::Normal);
                    }
                }
            }
        }

        const IRect box = targetBBox(m, ext, 2, target.w, target.h);
        for (int y = box.y0; y < box.y1; ++y) {
            for (int x = box.x0; x < box.x1; ++x) {
                const Vec2 local = inv.apply({x + 0.5f, y + 0.5f});
                float smp[4];
                sampleBilinear(buf, (local.x - ext.x) * scale, (local.y - ext.y) * scale, smp);
                if (smp[3] <= 0) {
                    continue;
                }
                float a = opacity;
                if (layer.masksToBounds()) {
                    a *= shapes::sd_fill(
                        shapes::sd_rounded_rect(gv(local), mask_center, mask_half, mask_cr), aa);
                    if (a <= 0) {
                        continue;
                    }
                }
                for (int ch = 0; ch < 4; ++ch) {
                    smp[ch] *= a;
                }
                blendPremul(target.at(x, y), smp, layer.blendMode());
            }
        }
    }

    // ---- frame ------------------------------------------------------------

    const Surface& renderFrame(Stage& stage, uint32_t out_w, uint32_t out_h, float dt) {
        stage.setTextMeasurer([this](const TextContent& c) { return measureText(c); });
        stage.advance(dt);

        frame.reset(static_cast<int>(out_w), static_cast<int>(out_h));

        // Logical canvas → output mapping (§4: Stage fit policy).
        const float W = stage.width();
        const float H = stage.height();
        float sx = out_w / W;
        float sy = out_h / H;
        if (stage.fit() == Fit::Contain) {
            sx = sy = std::min(sx, sy);
        } else if (stage.fit() == Fit::Cover) {
            sx = sy = std::max(sx, sy);
        }
        const Mat23 m0 = Mat23::translation({(out_w - W * sx) * 0.5f, (out_h - H * sy) * 0.5f}) *
                         Mat23::scaling({sx, sy});

        renderLayer(frame, stage.root(), m0, {W, H});

        // Premultiplied float → straight RGBA8.
        out_rgba.resize(static_cast<size_t>(out_w) * out_h * 4);
        for (size_t i = 0; i < out_rgba.size(); i += 4) {
            const float* p = frame.px.data() + i;
            const float a = std::clamp(p[3], 0.0f, 1.0f);
            const float inv_a = a > 0 ? 1.0f / a : 0.0f;
            out_rgba[i + 0] = static_cast<uint8_t>(std::clamp(p[0] * inv_a, 0.0f, 1.0f) * 255.0f + 0.5f);
            out_rgba[i + 1] = static_cast<uint8_t>(std::clamp(p[1] * inv_a, 0.0f, 1.0f) * 255.0f + 0.5f);
            out_rgba[i + 2] = static_cast<uint8_t>(std::clamp(p[2] * inv_a, 0.0f, 1.0f) * 255.0f + 0.5f);
            out_rgba[i + 3] = static_cast<uint8_t>(a * 255.0f + 0.5f);
        }
        surface.width = out_w;
        surface.height = out_h;
        surface.strideBytes = static_cast<size_t>(out_w) * 4;
        surface.pixels = out_rgba.data();
        return surface;
    }
};

// ===========================================================================
// CpuRenderer
// ===========================================================================

CpuRenderer::CpuRenderer() : CpuRenderer(Options{}) {}

CpuRenderer::CpuRenderer(Options options) : impl_(new Impl) { impl_->options = std::move(options); }

CpuRenderer::~CpuRenderer() = default;

const Surface& CpuRenderer::render(Stage& stage, float dt) {
    return impl_->renderFrame(stage,
                              static_cast<uint32_t>(std::lround(stage.width())),
                              static_cast<uint32_t>(std::lround(stage.height())), dt);
}

const Surface& CpuRenderer::render(Stage& stage, uint32_t out_width, uint32_t out_height,
                                   float dt) {
    return impl_->renderFrame(stage, out_width, out_height, dt);
}

}  // namespace fluent_stage
