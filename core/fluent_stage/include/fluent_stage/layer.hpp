#pragma once

/// \file layer.hpp
/// \brief Layer — one node of the retained tree: content + attributes +
///        sublayers (§6 of the design).
///
/// Every drawing call creates a layer and returns `Layer&` (principle S-2),
/// so decoration is chaining and nothing is fire-and-forget:
///
/// ```cpp
/// auto& hud = stage.group("hud").position(24, 24).opacity(0.9f).shadow();
/// hud.rect({0, 0, 340, 96}).cornerRadius(12).color({0, 0, 0, 0.45f});
/// hud.text("走行中", {16, 12}).size(28);
/// ```
///
/// ### Geometry (CALayer-accurate, §4.1)
/// A layer has `bounds` (its own coordinate space), `anchor` (a normalized
/// point in bounds, default center), and `position` (**where the anchor
/// lands in the parent's space**). Rotation and scale pivot on the anchor.
/// `frame` is sugar over the three. Unset geometry resolves automatically:
/// shape and text bounds default to the content's bounding box, images and
/// grids to the parent's bounds, and a bare group to a zero rect at the
/// origin (CALayer's default) — so `group().position(24, 24)` simply moves
/// the group's origin. Give a group explicit bounds/frame when you want to
/// rotate it about its center, clip it, or paint its background. An unset
/// position leaves the layer's space coincident with its parent's.
///
/// ### Animation (§9)
/// Attribute setters snap by default and animate when called inside a
/// `Transaction` — see transaction.hpp. Getters return the **model** value
/// (what you set); `presented*()` getters return what renders this frame.
///
/// ### Threading
/// A Stage and its layers are single-thread owned (§6.3). Hand data to the
/// renderer only at frame boundaries.

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "fluent_stage/animation.hpp"
#include "fluent_stage/content.hpp"
#include "fluent_stage/filters.hpp"
#include "fluent_stage/geometry.hpp"
#include "fluent_stage/transaction.hpp"
#include "fluent_stage/types.hpp"

namespace fluent_stage {

class Stage;

/// Phase of a pointer gesture (§10-3). Every input device — web viewer
/// clicks, touch, VR controller rays — normalizes to this one vocabulary.
enum class PointerPhase {
    Down,    ///< Contact started on the layer (it captures the gesture).
    Move,    ///< Contact moved while captured (may be outside the layer).
    Up,      ///< Contact ended; `inside` tells whether it counts as a tap.
    Cancel,  ///< Gesture aborted (device lost, layer removed, system takeover).
};

/// One pointer event delivered to a capturing layer's handler.
struct PointerEvent {
    PointerPhase phase;  ///< See PointerPhase.
    Vec2 stage_pos;      ///< Position in stage coordinates.
    Vec2 local_pos;      ///< Position in the capturing layer's bounds space.
    bool inside;         ///< True when local_pos lies within the layer bounds.
};

class Layer {
public:
    Layer(const Layer&) = delete;
    Layer& operator=(const Layer&) = delete;

    // -----------------------------------------------------------------------
    /// \name Drawing — create content sublayers (one line each, S-1)
    /// Each call appends a new sublayer under this layer and returns it.
    /// Later siblings composite on top (tree pre-order; there is no zIndex).
    /// @{

    /// A borrowed image (camera frame, …) fitted into the layer bounds.
    Layer& image(const ImageView& view, Fit fit = Fit::Contain);
    /// UTF-8 text (Japanese included); `pos` is the top-left. Chain
    /// `.size(28)`, `.align(...)`, `.color(...)`.
    Layer& text(const std::string& utf8, Vec2 pos = {0, 0});
    /// A straight line (default thickness 3, round caps).
    Layer& line(Vec2 from, Vec2 to);
    /// An open path with round joints (default thickness 3).
    Layer& polyline(const std::vector<Vec2>& points);
    /// A closed shape: filled by default; `.thickness(w)` outlines instead.
    Layer& polygon(const std::vector<Vec2>& points);
    /// A rectangle: filled by default; `.cornerRadius(r)` rounds it.
    Layer& rect(Rect r);
    /// A circle: filled by default; `.thickness(w)` makes a ring.
    Layer& circle(Vec2 center, float radius);
    /// Point-cloud markers: filled dots by default.
    Layer& circles(const std::vector<Vec2>& centers, float radius = 6);
    /// A circular arc (degrees; 0° = +x, clockwise on screen). Default
    /// thickness 3, round caps.
    Layer& arc(Vec2 center, float radius, float start_deg, float end_deg);
    /// An arrow with a filled head (default thickness 3).
    Layer& arrow(Vec2 from, Vec2 to);
    /// A crosshair around `center` (default thickness 3).
    Layer& crosshair(Vec2 center, float size = 24);
    /// A debug/calibration grid across the layer (default thickness 1).
    Layer& grid(float spacing = 100);
    /// Detection boxes (default thickness 3). Chain `.smoothing(0.2f)`.
    Layer& boxes(const std::vector<Box>& detections);
    /// An empty group layer: give it sublayers, move/fade/filter them as one.
    Layer& group(const std::string& id = {});

    /// @}
    // -----------------------------------------------------------------------
    /// \name Geometry (parent-local logical units; animatable)
    /// @{

    /// Sets this layer's coordinate space rectangle (origin may be nonzero,
    /// like CALayer.bounds). Content and sublayer positions are written in
    /// this space.
    Layer& bounds(Rect r);
    /// The model bounds. When never set explicitly, the automatically
    /// resolved value (content bounding box; parent-filling for images and
    /// grids; zero rect for bare groups) — resolved against the last known
    /// parent size.
    Rect bounds() const;

    /// Places the **anchor** at \p p in the parent's space (CALayer
    /// semantics — not the frame origin).
    Layer& position(Vec2 p);
    Layer& position(float x, float y) { return position(Vec2{x, y}); }
    /// The model position (auto-derived when never set).
    Vec2 position() const;

    /// The pivot for rotation/scale/transform as a normalized point in
    /// bounds (default (0.5, 0.5) = center).
    Layer& anchor(float ax, float ay);
    Vec2 anchor() const { return anchor_; }

    /// Frame sugar: where the layer lies in the parent, axis-aligned.
    /// Setting it derives bounds.size and position. Setting a frame on a
    /// rotated layer is flagged as a diagnostic, exactly like iOS warns
    /// (§4.1-3) — set bounds/position instead.
    Layer& frame(Rect r);
    /// The model frame `{position − anchor⊙size, size}` (meaningless under
    /// rotation; see bounds()).
    Rect frame() const;

    /// Rotation in degrees around the anchor (clockwise on screen, +y down).
    /// The whole subtree rotates — content, sublayers, shadow, clip (§4.1).
    Layer& rotation(float degrees);
    float rotation() const { return rotation_.target(); }

    /// Scale around the anchor. Negative values mirror: `scale(-1, 1)`
    /// flips horizontally.
    Layer& scale(float s) { return scale(s, s); }
    Layer& scale(float sx, float sy);
    Vec2 scale() const { return scale_.target(); }

    /// A full affine transform (shear included) composed with the simple
    /// attributes, anchor-based like everything else. Not animated in L0.
    Layer& transform(const Mat23& m);
    const Mat23& transform() const { return transform_; }

    /// @}
    // -----------------------------------------------------------------------
    /// \name Compositing attributes (all defaulted; animatable where noted)
    /// @{

    /// Opacity 0–1 (animatable). On a group, applies to the composited
    /// result — sublayers fade as one, without internal double-blending.
    Layer& opacity(float alpha01);
    float opacity() const { return opacity_.target(); }

    /// Skips rendering without removing from the tree (use remove() to
    /// delete). Hidden layers are not hit-testable.
    Layer& hidden(bool on);
    bool hidden() const { return hidden_; }

    /// Clips content and sublayers to bounds (cornerRadius included).
    /// Default false, same as iOS: a corner radius alone does not clip
    /// children.
    Layer& masksToBounds(bool on);
    bool masksToBounds() const { return masks_to_bounds_; }

    /// Rounds the layer's clip shape, background, and border — and rect
    /// and boxes content, so one value styles the whole panel.
    Layer& cornerRadius(float r);
    float cornerRadius() const { return corner_radius_; }

    /// A drop shadow from the layer's silhouette — works for text, shapes,
    /// images, and whole groups alike. `.shadow()` alone looks right.
    Layer& shadow(const Shadow& s = Shadow{});
    Layer& shadow(float dx, float dy, float radius) {
        Shadow s;
        s.offset = {dx, dy};
        s.radius = radius;
        return shadow(s);
    }
    const std::optional<Shadow>& shadowValue() const { return shadow_; }

    /// A border stroked along the clip shape (corner radius included).
    Layer& border(const Border& b = Border{});
    Layer& border(float width, Color c) { return border(Border{width, c}); }
    const std::optional<Border>& borderValue() const { return border_; }

    /// A background plate behind the content, filling bounds (with
    /// cornerRadius). Transparent by default.
    Layer& background(Color c);
    Color backgroundValue() const { return background_; }

    /// Blend mode against what is behind this layer (default Normal).
    Layer& blend(Blend mode);
    Blend blendMode() const { return blend_; }

    /// @}
    // -----------------------------------------------------------------------
    /// \name Content style (shared vocabulary across every content type)
    /// @{

    /// Content color (shapes, text, box outlines). Default white.
    Layer& color(Color c);
    Color colorValue() const { return color_; }

    /// Stroke width in logical units; 0 fills closed shapes.
    Layer& thickness(float logical_units);
    float thickness() const { return thickness_; }

    /// Dash length in logical units (equal on/off); 0 = solid. Applies to
    /// line, polyline, and polygon outlines (arc dashing is an L1 item).
    Layer& dash(float length);
    float dashValue() const { return dash_; }

    /// Line end / joint style (Round default). Arc content honors Butt in
    /// L1; L0 arcs draw round caps.
    Layer& cap(Cap c);
    Cap capValue() const { return cap_; }

    /// (image) How the image maps into bounds.
    Layer& fit(Fit f);
    /// (image) Crop rectangle in **source pixels** before mapping; partial
    /// copy-and-paste is `sourceRect()` + `frame()` (§5-2b).
    Layer& sourceRect(Rect source_pixels);
    /// (text) Font size in logical units.
    Layer& size(float font_size);
    /// (text) Which edge the text position refers to.
    Layer& align(Align a);
    /// (boxes) Temporal smoothing time constant in seconds; 0 disables.
    Layer& smoothing(float time_constant);
    /// (boxes) Draw the label and score above each box (default true).
    Layer& showLabel(bool on);
    /// (arrow) Head length in logical units; 0 = auto from thickness.
    Layer& headSize(float length);

    /// @}
    // -----------------------------------------------------------------------
    /// \name Filters (§8) — apply to this layer; on a group, to the
    /// composited subtree. Chained in call order.
    /// @{

    /// Appends any filter — typically via a typed struct:
    /// `layer.filter(Bilateral().radius(4).sigma_color(0.2))`.
    Layer& filter(const Filter& f);
    /// The attached filter chain, in application order.
    const std::vector<Filter>& filters() const { return filters_; }
    /// Removes all filters.
    Layer& clearFilters();

    /// Gaussian blur, radius in logical units.
    Layer& blur(float radius) { return filter(Blur().radius(radius)); }
    Layer& grayscale() { return filter(Grayscale()); }
    Layer& sepia(float intensity = 1.0f) { return filter(Sepia().intensity(intensity)); }
    Layer& invert() { return filter(Invert()); }
    /// Brightness offset (−1…1), via color_transform.
    Layer& brightness(float add) { return filter(ColorTransform().brightness(add)); }
    /// Contrast multiplier (1 = unchanged), via color_transform.
    Layer& contrast(float mul) { return filter(ColorTransform().contrast(mul)); }
    /// Saturation multiplier (0 = grayscale, 1 = unchanged), via
    /// color_transform.
    Layer& saturation(float mul) { return filter(ColorTransform().saturation(mul)); }
    Layer& pixelate(float block_size = 12) { return filter(Pixelate().size(block_size)); }
    Layer& vignette() { return filter(Vignette()); }

    /// @}
    // -----------------------------------------------------------------------
    /// \name Data updates (retained tree: call only when the data changed)
    /// @{

    /// Replaces the image view (image content only).
    Layer& setImage(const ImageView& view);
    /// Replaces the text (text content only).
    Layer& setText(const std::string& utf8);
    /// Replaces the points (polyline / polygon / circles content).
    Layer& setPoints(const std::vector<Vec2>& points);
    /// Replaces the detections (boxes content only). Smoothing carries
    /// tracks across the update.
    Layer& setBoxes(const std::vector<Box>& detections);
    /// Replaces the arc's angular range (arc content only) — the
    /// data-driven update for gauges and angle indicators.
    Layer& setArc(float start_deg, float end_deg);

    /// @}
    // -----------------------------------------------------------------------
    /// \name Identity and tree
    /// @{

    /// Names this layer so `stage.find("hud")` can address it later.
    /// Unnamed layers get an automatic id ("layer_7").
    Layer& id(const std::string& name);
    /// This layer's id.
    const std::string& id() const { return id_; }

    /// Finds a layer by id in this subtree (depth-first), or nullptr.
    Layer* find(const std::string& name);

    /// Detaches this layer (and its whole subtree) from the tree and
    /// destroys it. The reference is dead afterwards. Removing the root is
    /// a diagnosed no-op. Prefer `hidden(true)` for temporary hiding.
    void remove();

    /// The parent layer; nullptr on the root.
    Layer* parent() { return parent_; }
    const Layer* parent() const { return parent_; }

    /// Immediate sublayers in compositing order (later = on top).
    const std::vector<std::unique_ptr<Layer>>& sublayers() const { return sublayers_; }

    /// The content this layer draws (`std::monostate` for groups).
    const Content& content() const { return content_; }

    /// The Stage this layer belongs to.
    Stage& stage() { return *stage_; }
    const Stage& stage() const { return *stage_; }

    /// @}
    // -----------------------------------------------------------------------
    /// \name Pointer events (§10-3)
    /// @{

    /// Installs a pointer handler, making this layer an interactive target:
    /// `Stage::pointerDown` routes the deepest hit layer's nearest
    /// handler-bearing ancestor-or-self here, and that layer then captures
    /// the gesture until Up/Cancel (UIControl-style tracking). Pass an
    /// empty function to remove the handler. UI controls (ui.hpp) install
    /// these for you — install your own only for custom interactions.
    Layer& onPointer(std::function<void(const PointerEvent&)> handler);
    /// True when a pointer handler is installed.
    bool interactive() const { return static_cast<bool>(pointer_handler_); }

    /// @}
    // -----------------------------------------------------------------------
    /// \name Presentation values (what renders this frame; see §9, §13-3)
    /// @{

    float presentedOpacity() const { return opacity_.value(); }
    Vec2 presentedPosition() const;
    Rect presentedBounds() const;
    float presentedRotation() const { return rotation_.value(); }
    Vec2 presentedScale() const { return scale_.value(); }

    /// The layer's resolved geometry against a given parent size:
    /// effective bounds and the local→parent affine transform, using
    /// presentation values. This is what the renderer and hit-testing use;
    /// it is public so tools can ask "where is this layer really?" (§13-3).
    struct Resolved {
        Rect bounds;      ///< Effective bounds (auto-resolved when unset).
        Mat23 to_parent;  ///< Maps this layer's space into the parent's.
    };
    Resolved resolve(Vec2 parent_size) const;

    /// The content's intrinsic bounding box in layer space (strokes not
    /// included), or a zero rect when there is none (groups, images and
    /// grids span their bounds).
    Rect contentBounds() const;

    /// @}

private:
    friend class Stage;

    Layer(Stage& stage, std::string auto_id) : stage_(&stage), id_(std::move(auto_id)) {}

    /// Creates, registers, and appends a sublayer (throws std::length_error
    /// past the Stage limits).
    Layer& addSublayer(Content content, const std::string& explicit_id = {});

    /// Applies a value through the active Transaction (animate) or
    /// immediately (snap).
    template <typename T>
    void setAttr(Animated<T>& attr, const T& v) {
        if (const Transaction* t = Transaction::active()) {
            attr.to(v, t->duration(), t->ease());
        } else {
            attr.snap(v);
        }
    }

    /// Advances animations and box smoothing for this subtree.
    void step(float dt);

    /// Recursive hit-test in parent coordinates (presentation geometry).
    Layer* hitTest(Vec2 point_in_parent, Vec2 parent_size);

    /// Pointer-routing hit-test: like hitTest, but only layers whose
    /// ancestor-or-self chain carries a pointer handler count — a control
    /// with its handler removed (disabled) is transparent to pointers and
    /// whatever lies beneath receives them (UIKit rule).
    Layer* hitTestInteractive(Vec2 point_in_parent, Vec2 parent_size, bool ancestor_interactive);

    /// Automatic bounds when none were set: content bbox for shapes/text,
    /// parent-filling for images and grids, a zero rect for bare groups.
    Rect autoBounds(Vec2 parent_size) const;

    void warn(const std::string& message) const;

    Stage* stage_ = nullptr;
    Layer* parent_ = nullptr;
    std::string id_;
    std::vector<std::unique_ptr<Layer>> sublayers_;
    Content content_;

    // geometry
    Animated<Rect> bounds_;
    bool has_bounds_ = false;
    Animated<Vec2> position_;
    bool has_position_ = false;
    Vec2 anchor_{0.5f, 0.5f};
    Animated<float> rotation_{0.0f};
    Animated<Vec2> scale_{Vec2{1, 1}};
    Mat23 transform_;
    mutable Vec2 last_parent_size_{};  // for model-value auto resolution

    // compositing
    Animated<float> opacity_{1.0f};
    bool hidden_ = false;
    bool masks_to_bounds_ = false;
    float corner_radius_ = 0;
    Color background_ = Color::Transparent;
    Blend blend_ = Blend::Normal;
    std::optional<Shadow> shadow_;
    std::optional<Border> border_;
    std::vector<Filter> filters_;

    // content style
    Color color_ = Color::White;
    float thickness_ = 0;
    float dash_ = 0;
    Cap cap_ = Cap::Round;

    // interaction
    std::function<void(const PointerEvent&)> pointer_handler_;
};

}  // namespace fluent_stage
