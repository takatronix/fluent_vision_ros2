// linter.cpp — §13-2 design lints. Type checks reject the invalid; this
// rejects the unreadable: low contrast, escaped text, buried layers,
// offscreen placement. Contrast is measured against rendered pixels, not
// declared colors — what the eye meets is what gets judged.

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "fluent_stage/scene/linter.hpp"

namespace fluent_stage::scene {

namespace {

/// A layer fixed in stage space, with everything the lints need.
struct Placed {
    const Layer* layer;
    const LayerDecl* decl;   ///< Document node (nullptr for synthetic layers).
    Mat23 to_stage;          ///< Layer-local → stage coordinates.
    Rect bounds;             ///< Resolved local bounds.
    Rect stage_bbox;         ///< Axis-aligned bbox of bounds in stage space.
    float eff_opacity;       ///< Product of opacities down the chain.
    bool axis_aligned;       ///< No rotation/shear anywhere in the chain.
    int paint_index;         ///< Pre-order position (greater = painted later).
    int parent_index;        ///< Placed index of the parent (-1 for root).
};

Rect bboxOf(const Mat23& m, Rect r) {
    const Vec2 corners[4] = {m.apply({r.x, r.y}), m.apply({r.x + r.w, r.y}),
                             m.apply({r.x, r.y + r.h}), m.apply({r.x + r.w, r.y + r.h})};
    float min_x = corners[0].x, max_x = corners[0].x;
    float min_y = corners[0].y, max_y = corners[0].y;
    for (const Vec2& c : corners) {
        min_x = std::min(min_x, c.x);
        max_x = std::max(max_x, c.x);
        min_y = std::min(min_y, c.y);
        max_y = std::max(max_y, c.y);
    }
    return {min_x, min_y, max_x - min_x, max_y - min_y};
}

bool covers(Rect outer, Rect inner) {
    return outer.x <= inner.x && outer.y <= inner.y &&
           outer.x + outer.w >= inner.x + inner.w && outer.y + outer.h >= inner.y + inner.h;
}

bool disjoint(Rect a, Rect b) {
    return a.x + a.w <= b.x || b.x + b.w <= a.x || a.y + a.h <= b.y || b.y + b.h <= a.y;
}

/// WCAG relative luminance of an sRGB color (components 0-1).
float relativeLuminance(float r, float g, float b) {
    auto lin = [](float c) {
        return c <= 0.03928f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f);
    };
    return 0.2126f * lin(r) + 0.7152f * lin(g) + 0.0722f * lin(b);
}

float contrastRatio(float l1, float l2) {
    const float bright = std::max(l1, l2), dark = std::min(l1, l2);
    return (bright + 0.05f) / (dark + 0.05f);
}

void placeTree(const Layer& layer, const CompiledScene& scene, Mat23 parent_to_stage,
               Vec2 parent_size, float parent_opacity, bool parent_axis_aligned,
               int parent_index, int& counter, std::vector<Placed>& out) {
    if (layer.hidden()) {
        return;
    }
    const Layer::Resolved res = layer.resolve(parent_size);
    Placed p;
    p.layer = &layer;
    p.decl = nullptr;
    for (const auto& [decl, live] : scene.nodes()) {
        if (live == &layer) {
            p.decl = decl;
            break;
        }
    }
    p.to_stage = parent_to_stage * res.to_parent;
    p.bounds = res.bounds;
    p.stage_bbox = bboxOf(p.to_stage, res.bounds);
    p.eff_opacity = parent_opacity * layer.presentedOpacity();
    p.axis_aligned = parent_axis_aligned && p.to_stage.b == 0 && p.to_stage.c == 0 &&
                     p.to_stage.a > 0 && p.to_stage.d > 0;
    p.paint_index = counter++;
    p.parent_index = parent_index;
    const int self_index = static_cast<int>(out.size());
    out.push_back(p);
    for (const auto& sub : layer.sublayers()) {
        placeTree(*sub, scene, p.to_stage, {res.bounds.w, res.bounds.h}, p.eff_opacity,
                  p.axis_aligned, self_index, counter, out);
    }
}

bool isAncestor(const std::vector<Placed>& placed, int maybe_ancestor, int node) {
    for (int i = placed[static_cast<size_t>(node)].parent_index; i >= 0;
         i = placed[static_cast<size_t>(i)].parent_index) {
        if (i == maybe_ancestor) {
            return true;
        }
    }
    return false;
}

/// Whether this layer paints an opaque plate over its whole bounds.
bool paintsOpaquePlate(const Placed& p) {
    if (p.eff_opacity < 0.999f || !p.axis_aligned) {
        return false;
    }
    if (p.layer->backgroundValue().a >= 0.999f) {
        return true;
    }
    if (const auto* rect = std::get_if<RectContent>(&p.layer->content())) {
        return p.layer->thickness() == 0 && p.layer->colorValue().a >= 0.999f &&
               covers(rect->rect, p.bounds);
    }
    return false;
}

std::string describeLayer(const Placed& p) {
    std::string name = p.layer->id();
    if (p.decl != nullptr && p.decl->content) {
        name += std::string(" (") + p.decl->content->spec->name + ")";
    }
    return name;
}

}  // namespace

DiagnosticList lint(CompiledScene& scene, Renderer& renderer) {
    DiagnosticList diags;
    Stage& stage = scene.stage();

    // One render installs the text measurer and settles auto bounds.
    renderer.render(stage, 0.0f);

    std::vector<Placed> placed;
    int counter = 0;
    placeTree(stage.root(), scene, Mat23::identity(), {stage.width(), stage.height()}, 1.0f,
              true, -1, counter, placed);

    const Rect canvas{0, 0, stage.width(), stage.height()};

    // -- offscreen and occlusion -------------------------------------------
    for (size_t i = 0; i < placed.size(); ++i) {
        const Placed& p = placed[i];
        if (p.decl == nullptr || p.stage_bbox.w <= 0 || p.stage_bbox.h <= 0) {
            continue;  // synthetic (placeholder) or empty layers
        }
        const bool has_content = p.decl->content.has_value();
        if (has_content && disjoint(p.stage_bbox, canvas)) {
            diags.add("lint.offscreen", Severity::kWarning, Phase::kPreview, p.decl->span,
                      "layer " + describeLayer(p) + " lies entirely outside the canvas");
            continue;
        }
        if (!has_content && !p.layer->backgroundValue().a) {
            continue;  // bare groups cannot be occluded in a meaningful way
        }
        for (size_t j = 0; j < placed.size(); ++j) {
            const Placed& q = placed[j];
            if (j == i || q.paint_index <= p.paint_index ||
                isAncestor(placed, static_cast<int>(i), static_cast<int>(j))) {
                continue;
            }
            if (paintsOpaquePlate(q) && covers(q.stage_bbox, p.stage_bbox)) {
                const bool is_protected = p.decl->protected_;
                diags.add(is_protected ? "lint.protected_occluded" : "lint.occlusion",
                          is_protected ? Severity::kError : Severity::kWarning,
                          Phase::kPreview, p.decl->span,
                          "layer " + describeLayer(p) + " is completely covered by " +
                              describeLayer(q) +
                              (is_protected ? " — protected layers must stay visible (§13-4)"
                                            : ""));
                break;
            }
        }
    }

    // -- text overflow ------------------------------------------------------
    for (const Placed& p : placed) {
        if (p.decl == nullptr || !std::holds_alternative<TextContent>(p.layer->content())) {
            continue;
        }
        const bool declared_box = p.decl->find(AttrId::Bounds) != nullptr ||
                                  p.decl->find(AttrId::Frame) != nullptr;
        if (!declared_box) {
            continue;
        }
        const Rect text_box = p.layer->contentBounds();
        if (text_box.w > 0 && !covers(p.bounds, text_box)) {
            diags.add("lint.text_overflow", Severity::kWarning, Phase::kPreview, p.decl->span,
                      "text of layer " + describeLayer(p) +
                          " overflows the layer's declared bounds");
        }
    }

    // -- contrast (WCAG 4.5:1) against the rendered backdrop ----------------
    std::vector<Layer*> texts;
    for (const auto& [decl, layer] : scene.nodes()) {
        if (decl != nullptr && std::holds_alternative<TextContent>(layer->content()) &&
            !layer->hidden()) {
            texts.push_back(layer);
        }
    }
    if (!texts.empty()) {
        for (Layer* t : texts) {
            t->hidden(true);
        }
        const Surface& backdrop = renderer.render(stage, 0.0f);
        for (Layer* t : texts) {
            t->hidden(false);
        }
        for (const Placed& p : placed) {
            if (std::find(texts.begin(), texts.end(), p.layer) == texts.end()) {
                continue;
            }
            const Rect box = bboxOf(p.to_stage, p.layer->contentBounds());
            const float sx = static_cast<float>(backdrop.width) / stage.width();
            const float sy = static_cast<float>(backdrop.height) / stage.height();
            const int x0 = std::max(0, static_cast<int>(box.x * sx));
            const int y0 = std::max(0, static_cast<int>(box.y * sy));
            const int x1 = std::min(static_cast<int>(backdrop.width),
                                    static_cast<int>((box.x + box.w) * sx + 1));
            const int y1 = std::min(static_cast<int>(backdrop.height),
                                    static_cast<int>((box.y + box.h) * sy + 1));
            if (x1 <= x0 || y1 <= y0) {
                continue;
            }
            double sum_r = 0, sum_g = 0, sum_b = 0;
            long count = 0;
            for (int y = y0; y < y1; ++y) {
                const uint8_t* row = backdrop.row(static_cast<uint32_t>(y));
                for (int x = x0; x < x1; ++x) {
                    sum_r += row[x * 4 + 0];
                    sum_g += row[x * 4 + 1];
                    sum_b += row[x * 4 + 2];
                    ++count;
                }
            }
            const float br = static_cast<float>(sum_r / count) / 255.0f;
            const float bg = static_cast<float>(sum_g / count) / 255.0f;
            const float bb = static_cast<float>(sum_b / count) / 255.0f;
            // The text ink as the eye meets it: its color composited over
            // the sampled backdrop by its own alpha and effective opacity.
            const Color ink = p.layer->colorValue();
            const float a = ink.a * p.eff_opacity;
            const float tr = ink.r * a + br * (1 - a);
            const float tg = ink.g * a + bg * (1 - a);
            const float tb = ink.b * a + bb * (1 - a);
            const float ratio = contrastRatio(relativeLuminance(tr, tg, tb),
                                              relativeLuminance(br, bg, bb));
            if (ratio < 4.5f) {
                char buf[32];
                std::snprintf(buf, sizeof(buf), "%.2f", static_cast<double>(ratio));
                diags.add("lint.contrast", Severity::kWarning, Phase::kPreview,
                          p.decl != nullptr ? p.decl->span : Span{},
                          "text of layer " + describeLayer(p) + " has contrast " + buf +
                              ":1 against its rendered backdrop (WCAG needs 4.5:1)");
            }
        }
    }

    diags.sortCanonical();
    return diags;
}

}  // namespace fluent_stage::scene
