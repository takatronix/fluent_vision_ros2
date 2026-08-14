#pragma once

/// \file types.hpp
/// \brief Shared enums and small value types used by layer attributes and
///        content — the vocabulary of the public API.
///
/// Everything here is measured in the Stage's **logical units** (top-left
/// origin, +y down; see geometry.hpp). No normalized or device coordinates
/// appear in this file or anywhere else in the public API.

#include <cstdint>
#include <string>

#include "fluent_stage/geometry.hpp"

namespace fluent_stage {

/// Line end (and joint) style for stroked content.
enum class Cap {
    Round,  ///< Semicircular ends; polyline joints are round too (default).
    Butt,   ///< Squared-off ends exactly at the endpoint.
};

/// How content is mapped into a destination rectangle.
///
/// Used both by image content (image → its layer bounds) and by the Stage
/// itself (logical canvas → output surface when the aspect ratios differ).
enum class Fit {
    Contain,  ///< Fit entirely inside, preserving aspect (letterbox). Default.
    Cover,    ///< Fill the destination, preserving aspect (crop the overflow).
    Fill,     ///< Stretch to the destination, ignoring aspect.
};

/// How a layer's pixels combine with what is already behind them.
enum class Blend {
    Normal,    ///< Source-over alpha compositing (default).
    Add,       ///< Additive: glows, highlights.
    Multiply,  ///< Darkening: shading, tinted glass.
    Screen,    ///< Inverse-multiply: soft lightening.
};

/// Easing curve for implicit animations (see Transaction, §9 of the design).
enum class Ease {
    Linear,  ///< Constant speed.
    In,      ///< Starts slow, ends fast.
    Out,     ///< Starts fast, ends slow.
    InOut,   ///< Slow at both ends (default for UI motion).
};

/// Horizontal text alignment relative to the text position.
enum class Align {
    Left,    ///< Position is the left edge (default).
    Center,  ///< Position is the center.
    Right,   ///< Position is the right edge.
};

/// A drop shadow generated from the layer's silhouette (its alpha), so it
/// works equally for text, borders, images, and whole groups.
///
/// Field names follow CALayer (`shadowOffset` / `shadowRadius` /
/// `shadowColor` / `shadowOpacity`), with defaults chosen so a bare
/// `.shadow()` call already looks right.
struct Shadow {
    Vec2 offset{0, 3};        ///< Displacement in logical units (+y is down).
    float radius = 8;         ///< Gaussian blur radius in logical units.
    Color color{0, 0, 0, 1};  ///< Shadow color (alpha multiplies opacity).
    float opacity = 0.55f;    ///< Overall strength, 0–1.
};

/// A border stroked along the layer's clip shape (corner radius included).
struct Border {
    float width = 2;             ///< Stroke width in logical units.
    Color color = Color::White;  ///< Stroke color.
};

/// A borrowed, non-owning view of RGBA8 pixels for image content.
///
/// The pointed-to memory must stay valid until the next `render()` call —
/// the Stage never copies image pixels (a camera frame flows through
/// zero-copy). Rows are top-to-bottom, like everything else in this library.
struct ImageView {
    uint32_t width = 0;            ///< Width in pixels.
    uint32_t height = 0;           ///< Height in pixels.
    const uint8_t* pixels = nullptr;  ///< Tightly packed RGBA8 unless strideBytes is set.
    uint32_t strideBytes = 0;      ///< Bytes per row; 0 means `width * 4`.

    /// True when the view points at actual pixels.
    bool valid() const { return pixels != nullptr && width > 0 && height > 0; }
    /// Effective bytes per row.
    uint32_t stride() const { return strideBytes != 0 ? strideBytes : width * 4; }
};

/// One detection box for the data-driven `boxes()` content.
struct Box {
    Rect rect;          ///< Box in logical units.
    float score = 0;    ///< Confidence 0–1, shown with the label when present.
    std::string label;  ///< Optional class label ("asparagus", …).
    uint32_t id = 0;    ///< Optional stable track id; 0 lets smoothing match
                        ///< boxes frame-to-frame by nearest neighbor instead.
};

/// Static bounds of a Stage (design principle S-6: everything is bounded).
///
/// Declared at Stage construction. Structural limits (layers, depth) throw
/// `std::length_error` at the violating call — a tree that grows without
/// bound is a programming error (§15-4). Data limits (points, boxes, text)
/// clamp with a diagnostic instead: live sensor data spiking past a bound
/// must degrade the picture, never crash the robot's HUD.
struct StageLimits {
    uint32_t max_layers = 1024;      ///< Total layers in the tree.
    uint32_t max_depth = 16;         ///< Nesting depth (root is depth 0).
    uint32_t max_points = 4096;      ///< Points per polyline/polygon/circles.
    uint32_t max_boxes = 512;        ///< Boxes per boxes content.
    uint32_t max_text_bytes = 4096;  ///< UTF-8 bytes per text content.
};

}  // namespace fluent_stage
