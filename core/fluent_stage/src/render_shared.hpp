#pragma once

// Internal: the backend-independent compositing PLAN, shared by the CPU
// reference and the Vulkan backend so both walk the tree with byte-equal
// decisions — same offscreen conditions, same extents, same dash geometry,
// same filter parameter scaling. Divergence here is exactly what the golden
// parity tests exist to catch; keeping one copy makes most divergence
// impossible in the first place.
//
// Everything is pure functions over the public Layer API; no pixels, no GPU
// vocabulary.

#include <algorithm>
#include <cmath>
#include <functional>
#include <variant>
#include <vector>

#include "fluent_stage/filters.hpp"
#include "fluent_stage/layer.hpp"

namespace fluent_stage {
namespace plan {

// ---- transforms and rects --------------------------------------------------

inline float scaleOf(const Mat23& m) {
    const float det = std::fabs(m.a * m.d - m.b * m.c);
    return det > 0 ? std::sqrt(det) : 0.0f;
}

struct IRect {
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;  // half-open
    bool empty() const { return x1 <= x0 || y1 <= y0; }
};

// Target-space integer bbox of a local rect under m, padded and clipped.
inline IRect targetBBox(const Mat23& m, Rect local, float pad, int w, int h) {
    const Vec2 corners[4] = {{local.x, local.y},
                             {local.x + local.w, local.y},
                             {local.x, local.y + local.h},
                             {local.x + local.w, local.y + local.h}};
    float min_x = 1e30f, min_y = 1e30f, max_x = -1e30f, max_y = -1e30f;
    for (const Vec2& c : corners) {
        const Vec2 p = m.apply(c);
        min_x = std::min(min_x, p.x);
        min_y = std::min(min_y, p.y);
        max_x = std::max(max_x, p.x);
        max_y = std::max(max_y, p.y);
    }
    IRect r;
    r.x0 = std::max(0, static_cast<int>(std::floor(min_x - pad)));
    r.y0 = std::max(0, static_cast<int>(std::floor(min_y - pad)));
    r.x1 = std::min(w, static_cast<int>(std::ceil(max_x + pad)) + 1);
    r.y1 = std::min(h, static_cast<int>(std::ceil(max_y + pad)) + 1);
    return r;
}

inline Rect rectUnion(Rect a, Rect b) {
    if (a.w <= 0 || a.h <= 0) {
        return b;
    }
    if (b.w <= 0 || b.h <= 0) {
        return a;
    }
    const float x0 = std::min(a.x, b.x);
    const float y0 = std::min(a.y, b.y);
    const float x1 = std::max(a.x + a.w, b.x + b.w);
    const float y1 = std::max(a.y + a.h, b.y + b.h);
    return {x0, y0, x1 - x0, y1 - y0};
}

inline Rect rectIntersect(Rect a, Rect b) {
    const float x0 = std::max(a.x, b.x);
    const float y0 = std::max(a.y, b.y);
    const float x1 = std::min(a.x + a.w, b.x + b.w);
    const float y1 = std::min(a.y + a.h, b.y + b.h);
    return {x0, y0, std::max(0.0f, x1 - x0), std::max(0.0f, y1 - y0)};
}

inline Rect rectPad(Rect r, float pad) {
    return {r.x - pad, r.y - pad, r.w + 2 * pad, r.h + 2 * pad};
}

// ---- dash splitting (geometry preprocessing; keeps the SDFs simple) --------

struct Seg {
    Vec2 a, b;
};

inline std::vector<Seg> pathSegments(const std::vector<Vec2>& pts, bool closed) {
    std::vector<Seg> out;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        out.push_back({pts[i], pts[i + 1]});
    }
    if (closed && pts.size() >= 3) {
        out.push_back({pts.back(), pts.front()});
    }
    return out;
}

// Splits segments into on/off runs of `dash` length (the phase carries
// across joints so the pattern flows along the path).
inline std::vector<Seg> dashSegments(const std::vector<Seg>& segs, float dash) {
    if (dash <= 0) {
        return segs;
    }
    std::vector<Seg> out;
    float phase = 0;  // position within the 2*dash cycle
    for (const Seg& s : segs) {
        const float len = std::hypot(s.b.x - s.a.x, s.b.y - s.a.y);
        if (len <= 1e-6f) {
            continue;
        }
        const Vec2 dir{(s.b.x - s.a.x) / len, (s.b.y - s.a.y) / len};
        float t = 0;
        while (t < len) {
            const bool on = phase < dash;
            const float run = std::min(len - t, (on ? dash : 2 * dash) - phase);
            if (on) {
                out.push_back({{s.a.x + dir.x * t, s.a.y + dir.y * t},
                               {s.a.x + dir.x * (t + run), s.a.y + dir.y * (t + run)}});
            }
            t += run;
            phase += run;
            if (phase >= 2 * dash) {
                phase -= 2 * dash;
            }
        }
    }
    return out;
}

inline Rect segBounds(const Seg& s) {
    const float x0 = std::min(s.a.x, s.b.x);
    const float y0 = std::min(s.a.y, s.b.y);
    return {x0, y0, std::max(s.a.x, s.b.x) - x0, std::max(s.a.y, s.b.y) - y0};
}

// ---- compositing decisions -------------------------------------------------

// Folding opacity into the draw is exact when the layer composites in one
// pass: any single content (multi-part shapes union through the coverage
// mask first). Sublayers or boxes-with-labels blend in several passes that
// may overlap, so those go through a buffer.
inline bool safeOpacityFold(const Layer& layer) {
    if (!layer.sublayers().empty()) {
        return false;
    }
    if (const auto* b = std::get_if<BoxesContent>(&layer.content())) {
        return !b->show_label;
    }
    return true;
}

inline bool needsOffscreen(const Layer& layer, float opacity) {
    return !layer.filters().empty() || layer.shadowValue().has_value() ||
           layer.masksToBounds() ||
           (layer.blendMode() != Blend::Normal && !layer.sublayers().empty()) ||
           (opacity < 1.0f && !safeOpacityFold(layer));
}

// ---- extents ---------------------------------------------------------------

using TextMeasure = std::function<Rect(const TextContent&)>;

// Local-space extent of a layer's own content (stroke and decoration pads
// included), given its resolved bounds.
inline Rect contentExtent(const Layer& layer, Rect resolved_bounds, const TextMeasure& measure) {
    const Content& content = layer.content();
    if (std::holds_alternative<std::monostate>(content)) {
        return {};
    }
    if (std::holds_alternative<ImageContent>(content) ||
        std::holds_alternative<GridContent>(content)) {
        return resolved_bounds;
    }
    if (const auto* t = std::get_if<TextContent>(&content)) {
        return measure(*t);
    }
    Rect box = layer.contentBounds();
    float pad = layer.thickness() * 0.5f + 1.0f;
    if (const auto* a = std::get_if<ArrowContent>(&content)) {
        const float head = a->head_size > 0 ? a->head_size : std::max(3.0f * layer.thickness(), 9.0f);
        pad += head;
    }
    if (const auto* b = std::get_if<BoxesContent>(&content)) {
        if (b->show_label) {
            box = rectUnion(box, rectPad(box, 18.0f));  // label row above
        }
    }
    return rectPad(box, pad);
}

// Local-space extent of a whole subtree (recursive; masksToBounds clips).
inline Rect subtreeExtent(const Layer& layer, Vec2 parent_size, const TextMeasure& measure) {
    const Layer::Resolved r = layer.resolve(parent_size);
    Rect ext = contentExtent(layer, r.bounds, measure);
    if (layer.backgroundValue().a > 0 || layer.borderValue().has_value() ||
        layer.masksToBounds()) {
        ext = rectUnion(ext, r.bounds);
    }
    for (const auto& child : layer.sublayers()) {
        if (child->hidden()) {
            continue;
        }
        const Rect ce = subtreeExtent(*child, r.bounds.size(), measure);
        if (ce.w <= 0 || ce.h <= 0) {
            continue;
        }
        const Layer::Resolved cr = child->resolve(r.bounds.size());
        const Vec2 corners[4] = {{ce.x, ce.y},
                                 {ce.x + ce.w, ce.y},
                                 {ce.x, ce.y + ce.h},
                                 {ce.x + ce.w, ce.y + ce.h}};
        Rect cb{};
        for (const Vec2& c : corners) {
            const Vec2 p = cr.to_parent.apply(c);
            cb = rectUnion(cb, {p.x, p.y, 0.001f, 0.001f});
        }
        ext = rectUnion(ext, cb);
    }
    if (layer.masksToBounds()) {
        ext = rectIntersect(ext, r.bounds);
    }
    return ext;
}

// ---- filters ---------------------------------------------------------------

// Converts a filter's parameters into what the shared bodies expect (§5-3);
// both backends feed the bodies through this one function:
//   Length — logical units × the logical→buffer-pixel factor;
//   CoordX/CoordY — positions in the layer's local space, mapped into the
//   filter pass's normalized uv over the buffer (`ext` is the local-space
//   rect the buffer covers);
//   Scalar — untouched.
inline void scaleFilterValues(const Filter& f, float scale, Rect ext, float buf_w, float buf_h,
                              float out[5]) {
    for (int i = 0; i < 5; ++i) {
        out[i] = f.values[i];
    }
    if (const FilterSpec* spec = findFilterSpec(f.mode)) {
        for (size_t i = 0; i < spec->params.size() && i < 5; ++i) {
            switch (spec->params[i].unit) {
                case FilterUnit::Scalar:
                    break;
                case FilterUnit::Length:
                    out[i] *= scale;
                    break;
                case FilterUnit::CoordX:
                    out[i] = (out[i] - ext.x) * scale / buf_w;
                    break;
                case FilterUnit::CoordY:
                    out[i] = (out[i] - ext.y) * scale / buf_h;
                    break;
            }
        }
    }
}

}  // namespace plan
}  // namespace fluent_stage
