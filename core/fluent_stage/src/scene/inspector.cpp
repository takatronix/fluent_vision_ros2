// inspector.cpp — §13-3: the placement walk (shared with the linter) and
// its JSON faces. One geometry, used by checks and questions alike.

#include <algorithm>
#include <cstdio>
#include <variant>

#include "fluent_stage/scene/inspector.hpp"

namespace fluent_stage::scene {

namespace {

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

void placeTree(const Layer& layer, const CompiledScene& scene, Mat23 parent_to_stage,
               Vec2 parent_size, float parent_opacity, bool parent_axis_aligned,
               int parent_index, int& counter, std::vector<PlacedLayer>& out) {
    if (layer.hidden()) {
        return;
    }
    const Layer::Resolved res = layer.resolve(parent_size);
    PlacedLayer p;
    p.layer = &layer;
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

const char* contentName(const Layer& layer) {
    struct Namer {
        const char* operator()(std::monostate) { return "group"; }
        const char* operator()(const ImageContent&) { return "image"; }
        const char* operator()(const TextContent&) { return "text"; }
        const char* operator()(const LineContent&) { return "line"; }
        const char* operator()(const PolylineContent&) { return "polyline"; }
        const char* operator()(const PolygonContent&) { return "polygon"; }
        const char* operator()(const RectContent&) { return "rect"; }
        const char* operator()(const CircleContent&) { return "circle"; }
        const char* operator()(const CirclesContent&) { return "circles"; }
        const char* operator()(const ArcContent&) { return "arc"; }
        const char* operator()(const ArrowContent&) { return "arrow"; }
        const char* operator()(const CrosshairContent&) { return "crosshair"; }
        const char* operator()(const GridContent&) { return "grid"; }
        const char* operator()(const BoxesContent&) { return "boxes"; }
    };
    return std::visit(Namer{}, layer.content());
}

std::string jsonEscape(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '"' || c == '\\') {
            out += '\\';
            out += c;
        } else if (c == '\n') {
            out += "\\n";
        } else {
            out += c;
        }
    }
    return out;
}

std::string num(float v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.2f", static_cast<double>(v));
    return buf;
}

std::string placedJson(const PlacedLayer& p) {
    std::string out = "{\"id\": \"" + jsonEscape(p.layer->id()) + "\"";
    out += std::string(", \"content\": \"") + contentName(*p.layer) + "\"";
    if (p.decl != nullptr && !p.decl->role.empty()) {
        out += ", \"role\": \"" + jsonEscape(p.decl->role) + "\"";
    }
    if (p.decl != nullptr && p.decl->protected_) {
        out += ", \"protected\": true";
    }
    if (p.decl == nullptr) {
        out += ", \"synthetic\": true";  // placeholder panels, banners, …
    }
    out += ", \"frame\": [" + num(p.stage_bbox.x) + ", " + num(p.stage_bbox.y) + ", " +
           num(p.stage_bbox.w) + ", " + num(p.stage_bbox.h) + "]";
    out += ", \"opacity\": " + num(p.eff_opacity);
    out += ", \"paint_index\": " + std::to_string(p.paint_index);
    out += ", \"parent\": " + std::to_string(p.parent_index);
    if (!p.axis_aligned) {
        out += ", \"rotated\": true";
    }
    out += "}";
    return out;
}

}  // namespace

std::vector<PlacedLayer> placeLayers(const CompiledScene& scene) {
    std::vector<PlacedLayer> placed;
    int counter = 0;
    const Stage& stage = scene.stage();
    placeTree(stage.root(), scene, Mat23::identity(), {stage.width(), stage.height()}, 1.0f,
              true, -1, counter, placed);
    return placed;
}

std::vector<const PlacedLayer*> visibleAt(const std::vector<PlacedLayer>& placed, Vec2 point) {
    std::vector<const PlacedLayer*> hits;
    for (const PlacedLayer& p : placed) {
        if (p.stage_bbox.w <= 0 || p.stage_bbox.h <= 0 || p.eff_opacity <= 0) {
            continue;
        }
        // Test in local space so rotated layers answer exactly.
        const Mat23 inv = p.to_stage.inverse();
        if (p.bounds.contains(inv.apply(point))) {
            hits.push_back(&p);
        }
    }
    std::sort(hits.begin(), hits.end(), [](const PlacedLayer* a, const PlacedLayer* b) {
        return a->paint_index > b->paint_index;  // topmost first
    });
    return hits;
}

std::string inspectJson(const CompiledScene& scene, const std::vector<PlacedLayer>& placed) {
    std::string out = "{\"digest\": \"" + scene.digest() + "\", \"stage\": [" +
                      num(scene.stage().width()) + ", " + num(scene.stage().height()) +
                      "], \"layers\": [";
    for (size_t i = 0; i < placed.size(); ++i) {
        out += (i ? ", " : "") + placedJson(placed[i]);
    }
    out += "]}";
    return out;
}

std::string placedLayerJson(const PlacedLayer& placed) { return placedJson(placed); }

std::string atJson(const std::vector<PlacedLayer>& placed, Vec2 point) {
    const auto hits = visibleAt(placed, point);
    std::string out = "{\"at\": [" + num(point.x) + ", " + num(point.y) + "], \"layers\": [";
    for (size_t i = 0; i < hits.size(); ++i) {
        out += (i ? ", " : "") + placedJson(*hits[i]);
    }
    out += "]}";
    return out;
}

}  // namespace fluent_stage::scene
