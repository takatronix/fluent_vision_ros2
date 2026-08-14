#pragma once

/// \file stage.hpp
/// \brief Stage — the root of the runtime layer tree (§1 of the design).
///
/// The Stage declares the **logical coordinate system** (`Stage(1920, 1080)`)
/// that every length in the API is written in: positions, thicknesses,
/// corner radii, font sizes, blur radii. Rendering scales logically —
/// everything is SDF-evaluated, so any output resolution stays vector-crisp.
///
/// ```cpp
/// Stage stage(1920, 1080);
/// stage.image(camera);
/// stage.boxes(detections).color(Color::Teal).smoothing(0.2f);
///
/// auto& hud = stage.group("hud").position(24, 24).opacity(0.9f).shadow();
/// hud.rect({0, 0, 340, 96}).cornerRadius(12).color({0, 0, 0, 0.45f});
/// hud.text("走行中", {16, 12}).size(28);
///
/// CpuRenderer renderer;
/// const Surface& frame = renderer.render(stage, dt);
/// ```
///
/// The Stage is the C++ **body** of the system; the Scene document layer
/// (Phase L2) is a declarative skin that builds one of these. A Stage is
/// single-thread owned; renderers exchange data at frame boundaries.

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "fluent_stage/layer.hpp"
#include "fluent_stage/types.hpp"

namespace fluent_stage {

class Stage {
public:
    /// Creates a stage with a logical canvas of `width × height` units and
    /// static resource limits (S-6; exceeding them throws
    /// `std::length_error` at the violating call, §15-4).
    Stage(float width, float height, StageLimits limits = {});
    ~Stage();
    Stage(const Stage&) = delete;
    Stage& operator=(const Stage&) = delete;

    /// Logical canvas width.
    float width() const { return width_; }
    /// Logical canvas height.
    float height() const { return height_; }
    /// The limits declared at construction.
    const StageLimits& limits() const { return limits_; }

    /// How the logical canvas maps to an output surface with a different
    /// aspect ratio (§4): Contain letterboxes (default), Cover crops,
    /// Fill stretches.
    Stage& fit(Fit policy);
    Fit fit() const { return fit_; }

    /// The root layer. Drawing on the stage is drawing on the root.
    Layer& root() { return *root_; }
    const Layer& root() const { return *root_; }

    /// Finds a layer anywhere in the tree by id, or nullptr.
    Layer* find(const std::string& id) { return root_->find(id); }

    /// \name Drawing (forwards to the root layer)
    /// @{
    Layer& image(const ImageView& view, Fit f = Fit::Contain) { return root_->image(view, f); }
    Layer& text(const std::string& utf8, Vec2 pos = {0, 0}) { return root_->text(utf8, pos); }
    Layer& line(Vec2 from, Vec2 to) { return root_->line(from, to); }
    Layer& polyline(const std::vector<Vec2>& pts) { return root_->polyline(pts); }
    Layer& polygon(const std::vector<Vec2>& pts) { return root_->polygon(pts); }
    Layer& rect(Rect r) { return root_->rect(r); }
    Layer& circle(Vec2 center, float radius) { return root_->circle(center, radius); }
    Layer& circles(const std::vector<Vec2>& c, float radius = 6) {
        return root_->circles(c, radius);
    }
    Layer& arc(Vec2 center, float radius, float a0_deg, float a1_deg) {
        return root_->arc(center, radius, a0_deg, a1_deg);
    }
    Layer& arrow(Vec2 from, Vec2 to) { return root_->arrow(from, to); }
    Layer& crosshair(Vec2 center, float size = 24) { return root_->crosshair(center, size); }
    Layer& grid(float spacing = 100) { return root_->grid(spacing); }
    Layer& boxes(const std::vector<Box>& b) { return root_->boxes(b); }
    Layer& group(const std::string& id = {}) { return root_->group(id); }
    /// @}

    /// Advances every implicit animation and box-smoothing filter by \p dt
    /// seconds. Renderers call this from `render(stage, dt)`; tests may
    /// call it directly to reproduce "the screen at t = 0.15 s" without
    /// rendering (§13-6). Time never advances any other way.
    void advance(float dt);

    /// The deepest visible layer under a point in **stage coordinates**
    /// (presentation geometry, transforms considered — CALayer rules,
    /// §10-3), or nullptr.
    Layer* hitTest(Vec2 point) { return root_->hitTest(point, {width_, height_}); }

    /// \name Pointer injection (§10-3)
    /// The Stage owns no input device. Web viewer clicks, touch screens,
    /// and VR controller rays all normalize to these three calls in
    /// logical stage coordinates. Down hit-tests and lets the nearest
    /// interactive layer (see Layer::onPointer) capture the gesture;
    /// Move/Up follow the capture even outside the layer, UIControl-style.
    /// Each call returns true when an interactive layer consumed the event.
    /// @{
    bool pointerDown(Vec2 p);
    bool pointerMove(Vec2 p);
    bool pointerUp(Vec2 p);
    /// Aborts the gesture in flight (device lost, stream dropped); the
    /// captured layer receives PointerPhase::Cancel.
    void pointerCancel();
    /// @}

    /// Total layers currently in the tree (root included).
    uint32_t layerCount() const { return layer_count_; }

    /// Diagnostics accumulated since the last drain (misuse that is not
    /// worth throwing for: frame() on a rotated layer, setText on a shape,
    /// data clamped to a limit, …). Rendering never emits diagnostics; only
    /// API misuse does.
    std::vector<std::string> drainDiagnostics();

    /// Installs the text measurement service. Renderers own the font
    /// engine, so they install this on first render; afterwards text layers
    /// resolve exact automatic bounds (§15-1). Before any renderer touches
    /// the stage, text bounds resolve as a zero rect at the text position.
    void setTextMeasurer(std::function<Rect(const TextContent&)> measurer) {
        text_measurer_ = std::move(measurer);
    }
    /// Measures a text content via the installed service (zero rect
    /// without one).
    Rect measureText(const TextContent& text) const {
        return text_measurer_ ? text_measurer_(text) : Rect{};
    }

private:
    friend class Layer;

    /// Bookkeeping for Layer::addSublayer — enforces max_layers/max_depth.
    void registerLayer(uint32_t depth);
    void unregisterSubtree(uint32_t count);
    std::string nextAutoId();
    void diagnose(std::string message);

    /// Maps a stage-coordinate point into `target`'s bounds space (walking
    /// the parent chain with presentation geometry); also yields the
    /// target's resolved bounds.
    Vec2 toLocal(const Layer& target, Vec2 stage_point, Rect& bounds_out) const;
    /// Delivers a pointer event to the captured layer.
    void deliverPointer(PointerPhase phase, Vec2 p);
    /// Clears the capture if it lies inside a subtree being removed.
    void handleRemoval(const Layer* subtree_root);

    float width_;
    float height_;
    StageLimits limits_;
    Fit fit_ = Fit::Contain;
    std::unique_ptr<Layer> root_;
    uint32_t layer_count_ = 0;
    uint64_t next_id_ = 0;
    std::vector<std::string> diagnostics_;
    std::function<Rect(const TextContent&)> text_measurer_;
    Layer* pointer_capture_ = nullptr;
};

}  // namespace fluent_stage
