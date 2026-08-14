// stage.cpp — the retained layer tree: creation, attributes, geometry
// resolution, animation stepping, box smoothing, and hit-testing.
// Rendering lives in cpu_renderer.cpp; this file never touches pixels.

#include "fluent_stage/stage.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fluent_stage {

namespace {

// Axis-aligned bounding box of a point list (zero rect when empty).
Rect bboxOf(const std::vector<Vec2>& points) {
    if (points.empty()) {
        return {};
    }
    float min_x = points[0].x, min_y = points[0].y;
    float max_x = min_x, max_y = min_y;
    for (const Vec2& p : points) {
        min_x = std::min(min_x, p.x);
        min_y = std::min(min_y, p.y);
        max_x = std::max(max_x, p.x);
        max_y = std::max(max_y, p.y);
    }
    return {min_x, min_y, max_x - min_x, max_y - min_y};
}

Rect bboxUnion(Rect a, Rect b) {
    if (a.w <= 0 && a.h <= 0) {
        return b;
    }
    if (b.w <= 0 && b.h <= 0) {
        return a;
    }
    const float x0 = std::min(a.x, b.x);
    const float y0 = std::min(a.y, b.y);
    const float x1 = std::max(a.x + a.w, b.x + b.w);
    const float y1 = std::max(a.y + a.h, b.y + b.h);
    return {x0, y0, x1 - x0, y1 - y0};
}

// Temporal smoothing for detection boxes (§7 boxes, ported from the proven
// fluent_scene BoxSmoother): targets with matching nonzero ids pair up
// directly; the rest greedily associate to the nearest remaining track
// within a gate. Matched boxes blend with a rate-independent exponential
// filter; new ones appear exactly at their target (no pop-in), unmatched
// tracks drop immediately. Deterministic given inputs and the dt sequence.
void smoothBoxes(BoxesContent& content, float dt) {
    if (content.smoothing <= 0) {
        content.smoothed = content.targets;
        return;
    }
    const float alpha = 1.0f - std::exp(-std::max(dt, 0.0f) / content.smoothing);
    std::vector<bool> used(content.smoothed.size(), false);
    std::vector<Box> next;
    next.reserve(content.targets.size());
    for (const Box& target : content.targets) {
        int best = -1;
        float best_distance = 0;
        for (size_t i = 0; i < content.smoothed.size(); ++i) {
            if (used[i]) {
                continue;
            }
            const Box& track = content.smoothed[i];
            if (target.id != 0 || track.id != 0) {
                if (target.id == track.id && target.id != 0) {
                    best = static_cast<int>(i);
                    best_distance = 0;
                    break;
                }
                continue;  // ids present but different: never associate
            }
            const Vec2 tc = target.rect.center();
            const Vec2 kc = track.rect.center();
            const float distance = std::hypot(tc.x - kc.x, tc.y - kc.y);
            if (best < 0 || distance < best_distance) {
                best = static_cast<int>(i);
                best_distance = distance;
            }
        }
        Box out = target;
        // Association gate: without ids, only a nearby track may claim the
        // target; distant ones are treated as new objects.
        const float gate = 1.5f * std::max({target.rect.w, target.rect.h, 40.0f});
        if (best >= 0 && best_distance <= gate) {
            used[static_cast<size_t>(best)] = true;
            const Box& track = content.smoothed[static_cast<size_t>(best)];
            out.rect.x = track.rect.x + (target.rect.x - track.rect.x) * alpha;
            out.rect.y = track.rect.y + (target.rect.y - track.rect.y) * alpha;
            out.rect.w = track.rect.w + (target.rect.w - track.rect.w) * alpha;
            out.rect.h = track.rect.h + (target.rect.h - track.rect.h) * alpha;
            out.score = track.score + (target.score - track.score) * alpha;
        }
        next.push_back(std::move(out));
    }
    content.smoothed = std::move(next);
}

}  // namespace

// ===========================================================================
// Stage
// ===========================================================================

Stage::Stage(float width, float height, StageLimits limits)
    : width_(width), height_(height), limits_(limits) {
    root_.reset(new Layer(*this, "root"));
    root_->last_parent_size_ = {width_, height_};
    root_->bounds({0, 0, width_, height_});  // the root spans the canvas
    layer_count_ = 1;
}

Stage::~Stage() = default;

Stage& Stage::fit(Fit policy) {
    fit_ = policy;
    return *this;
}

void Stage::advance(float dt) { root_->step(dt); }

std::vector<std::string> Stage::drainDiagnostics() {
    std::vector<std::string> out;
    out.swap(diagnostics_);
    return out;
}

void Stage::registerLayer(uint32_t depth) {
    if (layer_count_ >= limits_.max_layers) {
        throw std::length_error("fluent_stage: max_layers (" +
                                std::to_string(limits_.max_layers) + ") exceeded");
    }
    if (depth > limits_.max_depth) {
        throw std::length_error("fluent_stage: max_depth (" +
                                std::to_string(limits_.max_depth) + ") exceeded");
    }
    ++layer_count_;
}

void Stage::unregisterSubtree(uint32_t count) {
    layer_count_ = count <= layer_count_ ? layer_count_ - count : 0;
}

std::string Stage::nextAutoId() { return "layer_" + std::to_string(next_id_++); }

// ---- pointer injection (§10-3) --------------------------------------------

Vec2 Stage::toLocal(const Layer& target, Vec2 stage_point, Rect& bounds_out) const {
    std::vector<const Layer*> chain;
    for (const Layer* l = &target; l != nullptr; l = l->parent()) {
        chain.push_back(l);
    }
    Vec2 p = stage_point;
    Vec2 parent_size{width_, height_};
    bounds_out = {};
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
        const Layer::Resolved r = (*it)->resolve(parent_size);
        p = r.to_parent.inverse().apply(p);
        parent_size = r.bounds.size();
        bounds_out = r.bounds;
    }
    return p;
}

void Stage::deliverPointer(PointerPhase phase, Vec2 p) {
    Rect bounds{};
    const Vec2 local = toLocal(*pointer_capture_, p, bounds);
    const PointerEvent event{phase, p, local,
                             bounds.w > 0 && bounds.h > 0 && bounds.contains(local)};
    pointer_capture_->pointer_handler_(event);
}

bool Stage::pointerDown(Vec2 p) {
    if (pointer_capture_ != nullptr) {
        pointerCancel();  // a second Down aborts the gesture in flight
    }
    Layer* hit = root_->hitTestInteractive(p, {width_, height_}, false);
    while (hit != nullptr && !hit->interactive()) {
        hit = hit->parent();
    }
    if (hit == nullptr) {
        return false;
    }
    pointer_capture_ = hit;
    deliverPointer(PointerPhase::Down, p);
    return true;
}

bool Stage::pointerMove(Vec2 p) {
    if (pointer_capture_ == nullptr) {
        return false;
    }
    deliverPointer(PointerPhase::Move, p);
    return true;
}

bool Stage::pointerUp(Vec2 p) {
    if (pointer_capture_ == nullptr) {
        return false;
    }
    deliverPointer(PointerPhase::Up, p);
    pointer_capture_ = nullptr;
    return true;
}

void Stage::pointerCancel() {
    if (pointer_capture_ == nullptr) {
        return;
    }
    deliverPointer(PointerPhase::Cancel, {-1, -1});
    pointer_capture_ = nullptr;
}

void Stage::handleRemoval(const Layer* subtree_root) {
    for (const Layer* l = pointer_capture_; l != nullptr; l = l->parent()) {
        if (l == subtree_root) {
            // The control is going away mid-gesture; drop the capture
            // without delivering into a dying handler.
            pointer_capture_ = nullptr;
            return;
        }
    }
}

void Stage::diagnose(std::string message) {
    // Bounded like everything else; drop the oldest past a sane cap.
    if (diagnostics_.size() >= 256) {
        diagnostics_.erase(diagnostics_.begin());
    }
    diagnostics_.push_back(std::move(message));
}

// ===========================================================================
// Layer — tree and content creation
// ===========================================================================

Layer& Layer::addSublayer(Content content, const std::string& explicit_id) {
    uint32_t depth = 1;
    for (const Layer* up = this; up->parent_ != nullptr; up = up->parent_) {
        ++depth;
    }
    stage_->registerLayer(depth);
    std::unique_ptr<Layer> child(
        new Layer(*stage_, explicit_id.empty() ? stage_->nextAutoId() : explicit_id));
    child->parent_ = this;
    child->content_ = std::move(content);
    sublayers_.push_back(std::move(child));
    return *sublayers_.back();
}

Layer& Layer::image(const ImageView& view, Fit f) {
    ImageContent c;
    c.view = view;
    c.fit = f;
    return addSublayer(std::move(c));
}

Layer& Layer::text(const std::string& utf8, Vec2 pos) {
    TextContent c;
    c.utf8 = utf8;
    c.position = pos;
    if (c.utf8.size() > stage_->limits().max_text_bytes) {
        warn("text clamped to max_text_bytes");
        c.utf8.resize(stage_->limits().max_text_bytes);
    }
    return addSublayer(std::move(c));
}

Layer& Layer::line(Vec2 from, Vec2 to) {
    LineContent c{from, to};
    return addSublayer(std::move(c)).thickness(3);
}

Layer& Layer::polyline(const std::vector<Vec2>& points) {
    PolylineContent c;
    c.points = points;
    if (c.points.size() > stage_->limits().max_points) {
        warn("polyline clamped to max_points");
        c.points.resize(stage_->limits().max_points);
    }
    return addSublayer(std::move(c)).thickness(3);
}

Layer& Layer::polygon(const std::vector<Vec2>& points) {
    PolygonContent c;
    c.points = points;
    if (c.points.size() > stage_->limits().max_points) {
        warn("polygon clamped to max_points");
        c.points.resize(stage_->limits().max_points);
    }
    return addSublayer(std::move(c));
}

Layer& Layer::rect(Rect r) {
    RectContent c{r};
    return addSublayer(std::move(c));
}

Layer& Layer::circle(Vec2 center, float radius) {
    CircleContent c{center, radius};
    return addSublayer(std::move(c));
}

Layer& Layer::circles(const std::vector<Vec2>& centers, float radius) {
    CirclesContent c;
    c.centers = centers;
    c.radius = radius;
    if (c.centers.size() > stage_->limits().max_points) {
        warn("circles clamped to max_points");
        c.centers.resize(stage_->limits().max_points);
    }
    return addSublayer(std::move(c));
}

Layer& Layer::arc(Vec2 center, float radius, float start_deg, float end_deg) {
    ArcContent c{center, radius, start_deg, end_deg};
    return addSublayer(std::move(c)).thickness(3);
}

Layer& Layer::arrow(Vec2 from, Vec2 to) {
    ArrowContent c;
    c.from = from;
    c.to = to;
    return addSublayer(std::move(c)).thickness(3);
}

Layer& Layer::crosshair(Vec2 center, float size) {
    CrosshairContent c;
    c.center = center;
    c.size = size;
    return addSublayer(std::move(c)).thickness(3);
}

Layer& Layer::grid(float spacing) {
    GridContent c{spacing};
    return addSublayer(std::move(c)).thickness(1);
}

Layer& Layer::boxes(const std::vector<Box>& detections) {
    BoxesContent c;
    c.targets = detections;
    if (c.targets.size() > stage_->limits().max_boxes) {
        warn("boxes clamped to max_boxes");
        c.targets.resize(stage_->limits().max_boxes);
    }
    c.smoothed = c.targets;
    return addSublayer(std::move(c)).thickness(3);
}

Layer& Layer::group(const std::string& id) { return addSublayer(std::monostate{}, id); }

// ===========================================================================
// Layer — geometry
// ===========================================================================

Layer& Layer::bounds(Rect r) {
    if (!has_bounds_) {
        // First explicit value: animations must start from what is on
        // screen now — the auto-resolved bounds (§9: never jump).
        bounds_.snap(presentedBounds());
        has_bounds_ = true;
    }
    setAttr(bounds_, r);
    return *this;
}

Rect Layer::bounds() const {
    if (has_bounds_) {
        return bounds_.target();
    }
    return autoBounds(last_parent_size_);
}

Layer& Layer::position(Vec2 p) {
    if (!has_position_) {
        // First explicit value: seed from the auto-derived position so an
        // animated first set glides from the current place (§9).
        position_.snap(presentedPosition());
        has_position_ = true;
    }
    setAttr(position_, p);
    return *this;
}

Vec2 Layer::position() const {
    if (has_position_) {
        return position_.target();
    }
    const Rect b = bounds();
    return {b.x + anchor_.x * b.w, b.y + anchor_.y * b.h};
}

Layer& Layer::anchor(float ax, float ay) {
    anchor_ = {ax, ay};
    return *this;
}

Layer& Layer::frame(Rect r) {
    if (rotation_.target() != 0 || !transform_.isIdentity()) {
        warn("frame() set on a transformed layer is ill-defined; set bounds/position instead");
    }
    bounds({0, 0, r.w, r.h});
    return position({r.x + anchor_.x * r.w, r.y + anchor_.y * r.h});
}

Rect Layer::frame() const {
    const Rect b = bounds();
    const Vec2 p = position();
    return {p.x - anchor_.x * b.w, p.y - anchor_.y * b.h, b.w, b.h};
}

Layer& Layer::rotation(float degrees) {
    setAttr(rotation_, degrees);
    return *this;
}

Layer& Layer::scale(float sx, float sy) {
    setAttr(scale_, Vec2{sx, sy});
    return *this;
}

Layer& Layer::transform(const Mat23& m) {
    transform_ = m;
    return *this;
}

Rect Layer::autoBounds(Vec2 parent_size) const {
    // Content with an intrinsic extent adopts its bounding box (§15-1);
    // images and grids span the parent (§6.2 "frame 既定 親いっぱい").
    // A bare group keeps CALayer's default — a zero rect at the origin —
    // so `group().position(24, 24)` moves the group's origin (its anchor
    // point of a zero rect IS its origin), exactly like the §2 example.
    // Give a group explicit bounds/frame to rotate about its center, clip,
    // or paint a background.
    if (std::holds_alternative<std::monostate>(content_)) {
        return {};
    }
    if (std::holds_alternative<ImageContent>(content_) ||
        std::holds_alternative<GridContent>(content_)) {
        return {0, 0, parent_size.x, parent_size.y};
    }
    return contentBounds();
}

Rect Layer::contentBounds() const {
    struct Visitor {
        const Layer& layer;
        Rect operator()(const std::monostate&) const { return {}; }
        Rect operator()(const ImageContent&) const { return {}; }
        Rect operator()(const GridContent&) const { return {}; }
        Rect operator()(const TextContent& c) const { return layer.stage_->measureText(c); }
        Rect operator()(const LineContent& c) const { return bboxOf({c.from, c.to}); }
        Rect operator()(const PolylineContent& c) const { return bboxOf(c.points); }
        Rect operator()(const PolygonContent& c) const { return bboxOf(c.points); }
        Rect operator()(const RectContent& c) const { return c.rect; }
        Rect operator()(const CircleContent& c) const {
            return {c.center.x - c.radius, c.center.y - c.radius, 2 * c.radius, 2 * c.radius};
        }
        Rect operator()(const CirclesContent& c) const {
            Rect b = bboxOf(c.centers);
            return {b.x - c.radius, b.y - c.radius, b.w + 2 * c.radius, b.h + 2 * c.radius};
        }
        Rect operator()(const ArcContent& c) const {
            return {c.center.x - c.radius, c.center.y - c.radius, 2 * c.radius, 2 * c.radius};
        }
        Rect operator()(const ArrowContent& c) const { return bboxOf({c.from, c.to}); }
        Rect operator()(const CrosshairContent& c) const {
            return {c.center.x - c.size, c.center.y - c.size, 2 * c.size, 2 * c.size};
        }
        Rect operator()(const BoxesContent& c) const {
            Rect all{};
            for (const Box& b : c.targets) {
                all = bboxUnion(all, b.rect);
            }
            return all;
        }
    };
    return std::visit(Visitor{*this}, content_);
}

Layer::Resolved Layer::resolve(Vec2 parent_size) const {
    last_parent_size_ = parent_size;
    const Rect b = has_bounds_ ? bounds_.value() : autoBounds(parent_size);
    const Vec2 anchor_pt{b.x + anchor_.x * b.w, b.y + anchor_.y * b.h};
    const Vec2 pos = has_position_ ? position_.value() : anchor_pt;
    Mat23 m = Mat23::translation(pos);
    const float rot = rotation_.value();
    if (rot != 0) {
        m = m * Mat23::rotationDeg(rot);
    }
    const Vec2 s = scale_.value();
    if (s.x != 1 || s.y != 1) {
        m = m * Mat23::scaling(s);
    }
    if (!transform_.isIdentity()) {
        m = m * transform_;
    }
    m = m * Mat23::translation({-anchor_pt.x, -anchor_pt.y});
    return {b, m};
}

Vec2 Layer::presentedPosition() const {
    if (has_position_) {
        return position_.value();
    }
    const Rect b = presentedBounds();
    return {b.x + anchor_.x * b.w, b.y + anchor_.y * b.h};
}

Rect Layer::presentedBounds() const {
    return has_bounds_ ? bounds_.value() : autoBounds(last_parent_size_);
}

// ===========================================================================
// Layer — compositing attributes and content style
// ===========================================================================

Layer& Layer::opacity(float alpha01) {
    setAttr(opacity_, alpha01);
    return *this;
}

Layer& Layer::hidden(bool on) {
    hidden_ = on;
    return *this;
}

Layer& Layer::masksToBounds(bool on) {
    masks_to_bounds_ = on;
    return *this;
}

Layer& Layer::cornerRadius(float r) {
    corner_radius_ = r;
    return *this;
}

Layer& Layer::shadow(const Shadow& s) {
    shadow_ = s;
    return *this;
}

Layer& Layer::border(const Border& b) {
    border_ = b;
    return *this;
}

Layer& Layer::background(Color c) {
    background_ = c;
    return *this;
}

Layer& Layer::blend(Blend mode) {
    blend_ = mode;
    return *this;
}

Layer& Layer::color(Color c) {
    color_ = c;
    return *this;
}

Layer& Layer::thickness(float logical_units) {
    thickness_ = logical_units;
    return *this;
}

Layer& Layer::dash(float length) {
    dash_ = length;
    return *this;
}

Layer& Layer::cap(Cap c) {
    cap_ = c;
    return *this;
}

Layer& Layer::fit(Fit f) {
    if (auto* c = std::get_if<ImageContent>(&content_)) {
        c->fit = f;
    } else {
        warn("fit() applies to image content only");
    }
    return *this;
}

Layer& Layer::sourceRect(Rect source_pixels) {
    if (auto* c = std::get_if<ImageContent>(&content_)) {
        c->source_rect = source_pixels;
    } else {
        warn("sourceRect() applies to image content only");
    }
    return *this;
}

Layer& Layer::size(float font_size) {
    if (auto* c = std::get_if<TextContent>(&content_)) {
        c->size = font_size;
    } else {
        warn("size() applies to text content only");
    }
    return *this;
}

Layer& Layer::align(Align a) {
    if (auto* c = std::get_if<TextContent>(&content_)) {
        c->align = a;
    } else {
        warn("align() applies to text content only");
    }
    return *this;
}

Layer& Layer::smoothing(float time_constant) {
    if (auto* c = std::get_if<BoxesContent>(&content_)) {
        c->smoothing = time_constant;
    } else {
        warn("smoothing() applies to boxes content only");
    }
    return *this;
}

Layer& Layer::showLabel(bool on) {
    if (auto* c = std::get_if<BoxesContent>(&content_)) {
        c->show_label = on;
    } else {
        warn("showLabel() applies to boxes content only");
    }
    return *this;
}

Layer& Layer::headSize(float length) {
    if (auto* c = std::get_if<ArrowContent>(&content_)) {
        c->head_size = length;
    } else {
        warn("headSize() applies to arrow content only");
    }
    return *this;
}

Layer& Layer::filter(const Filter& f) {
    filters_.push_back(f);
    return *this;
}

Layer& Layer::clearFilters() {
    filters_.clear();
    return *this;
}

// ===========================================================================
// Layer — data updates
// ===========================================================================

Layer& Layer::setImage(const ImageView& view) {
    if (auto* c = std::get_if<ImageContent>(&content_)) {
        c->view = view;
    } else {
        warn("setImage() applies to image content only");
    }
    return *this;
}

Layer& Layer::setText(const std::string& utf8) {
    if (auto* c = std::get_if<TextContent>(&content_)) {
        c->utf8 = utf8;
        if (c->utf8.size() > stage_->limits().max_text_bytes) {
            warn("text clamped to max_text_bytes");
            c->utf8.resize(stage_->limits().max_text_bytes);
        }
    } else {
        warn("setText() applies to text content only");
    }
    return *this;
}

Layer& Layer::setPoints(const std::vector<Vec2>& points) {
    std::vector<Vec2>* dst = nullptr;
    if (auto* c = std::get_if<PolylineContent>(&content_)) {
        dst = &c->points;
    } else if (auto* c = std::get_if<PolygonContent>(&content_)) {
        dst = &c->points;
    } else if (auto* c = std::get_if<CirclesContent>(&content_)) {
        dst = &c->centers;
    }
    if (dst == nullptr) {
        warn("setPoints() applies to polyline/polygon/circles content only");
        return *this;
    }
    *dst = points;
    if (dst->size() > stage_->limits().max_points) {
        warn("points clamped to max_points");
        dst->resize(stage_->limits().max_points);
    }
    return *this;
}

Layer& Layer::setBoxes(const std::vector<Box>& detections) {
    if (auto* c = std::get_if<BoxesContent>(&content_)) {
        c->targets = detections;
        if (c->targets.size() > stage_->limits().max_boxes) {
            warn("boxes clamped to max_boxes");
            c->targets.resize(stage_->limits().max_boxes);
        }
    } else {
        warn("setBoxes() applies to boxes content only");
    }
    return *this;
}

// ===========================================================================
// Layer — identity, tree, stepping, hit-test
// ===========================================================================

Layer& Layer::id(const std::string& name) {
    id_ = name;
    return *this;
}

Layer& Layer::onPointer(std::function<void(const PointerEvent&)> handler) {
    if (!handler && stage_->pointer_capture_ == this) {
        stage_->pointer_capture_ = nullptr;  // handler removed mid-gesture
    }
    pointer_handler_ = std::move(handler);
    return *this;
}

Layer* Layer::find(const std::string& name) {
    if (id_ == name) {
        return this;
    }
    for (const auto& child : sublayers_) {
        if (Layer* hit = child->find(name)) {
            return hit;
        }
    }
    return nullptr;
}

void Layer::remove() {
    if (parent_ == nullptr) {
        warn("remove() on the root layer is a no-op");
        return;
    }
    stage_->handleRemoval(this);
    uint32_t count = 0;
    struct Counter {
        static void count(const Layer& l, uint32_t& n) {
            ++n;
            for (const auto& child : l.sublayers_) {
                count(*child, n);
            }
        }
    };
    Counter::count(*this, count);
    stage_->unregisterSubtree(count);
    auto& siblings = parent_->sublayers_;
    for (auto it = siblings.begin(); it != siblings.end(); ++it) {
        if (it->get() == this) {
            siblings.erase(it);  // destroys *this — no member access past here
            return;
        }
    }
}

void Layer::step(float dt) {
    bounds_.step(dt);
    position_.step(dt);
    rotation_.step(dt);
    scale_.step(dt);
    opacity_.step(dt);
    if (auto* boxes = std::get_if<BoxesContent>(&content_)) {
        smoothBoxes(*boxes, dt);
    }
    for (const auto& child : sublayers_) {
        child->step(dt);
    }
}

Layer* Layer::hitTest(Vec2 point_in_parent, Vec2 parent_size) {
    if (hidden_) {
        return nullptr;
    }
    const Resolved r = resolve(parent_size);
    const Vec2 local = r.to_parent.inverse().apply(point_in_parent);
    // UIKit rule: a layer with real bounds only responds (and only forwards
    // to sublayers) inside them. A zero-bounds group is a pure coordinate
    // space: it forwards but is never hit itself.
    const bool has_area = r.bounds.w > 0 && r.bounds.h > 0;
    if (has_area && !r.bounds.contains(local)) {
        return nullptr;
    }
    for (auto it = sublayers_.rbegin(); it != sublayers_.rend(); ++it) {
        if (Layer* hit = (*it)->hitTest(local, r.bounds.size())) {
            return hit;
        }
    }
    return has_area ? this : nullptr;
}

Layer* Layer::hitTestInteractive(Vec2 point_in_parent, Vec2 parent_size,
                                 bool ancestor_interactive) {
    if (hidden_) {
        return nullptr;
    }
    const Resolved r = resolve(parent_size);
    const Vec2 local = r.to_parent.inverse().apply(point_in_parent);
    const bool has_area = r.bounds.w > 0 && r.bounds.h > 0;
    if (has_area && !r.bounds.contains(local)) {
        return nullptr;
    }
    const bool chain_interactive = ancestor_interactive || interactive();
    for (auto it = sublayers_.rbegin(); it != sublayers_.rend(); ++it) {
        if (Layer* hit = (*it)->hitTestInteractive(local, r.bounds.size(), chain_interactive)) {
            return hit;
        }
    }
    return has_area && chain_interactive ? this : nullptr;
}

void Layer::warn(const std::string& message) const {
    stage_->diagnose(id_ + ": " + message);
}

}  // namespace fluent_stage
