#pragma once

/// \file content.hpp
/// \brief Content — what a layer draws (§7 of the design).
///
/// A layer holds **zero or one** content; a layer with no content and
/// sublayers is a group. Content stores only *what* to draw, in the layer's
/// own bounds space; *where and how it composites* (position, rotation,
/// opacity, shadow, …) are layer attributes, never content fields (principle
/// S-3). Shared styling — color, thickness, dash, cap, corner radius — also
/// lives on the layer so every content type spells it identically.
///
/// All shapes render as signed distance fields (shared/shapes_shared.h):
/// antialiased, and re-evaluated (not upscaled) at any output resolution.

#include <string>
#include <variant>
#include <vector>

#include "fluent_stage/geometry.hpp"
#include "fluent_stage/types.hpp"

namespace fluent_stage {

/// A borrowed image mapped into the layer bounds with a Fit policy.
/// `source_rect` (CALayer `contentsRect` in spirit, but in **source pixels**,
/// never 0–1) cuts a sub-rectangle out of the source first — partial
/// copy-and-paste is this plus `frame()` (§5-2b). A zero-size source_rect
/// means the whole image.
struct ImageContent {
    ImageView view;             ///< Borrowed pixels; valid until render.
    Fit fit = Fit::Contain;     ///< Mapping into the layer bounds.
    Rect source_rect{};         ///< Source crop in pixels; zero size = all.
};

/// A single run of UTF-8 text (Japanese included; shaped by HarfBuzz).
/// `position` is the top-left of the text box in layer space (Align moves
/// the reference edge). Missing glyphs draw a deterministic replacement box.
struct TextContent {
    std::string utf8;           ///< The text.
    Vec2 position{};            ///< Reference point in layer space.
    float size = 26;            ///< Font size in logical units.
    Align align = Align::Left;  ///< Which edge `position` refers to.
};

/// A single straight line from `from` to `to`.
struct LineContent {
    Vec2 from{};
    Vec2 to{};
};

/// An open path through `points` (round joints; capsule SDF strokes).
struct PolylineContent {
    std::vector<Vec2> points;
};

/// A closed path through `points`: filled when the layer's thickness is 0,
/// outlined otherwise.
struct PolygonContent {
    std::vector<Vec2> points;
};

/// A rectangle (the layer's cornerRadius rounds it; thickness 0 fills).
struct RectContent {
    Rect rect{};
};

/// A circle (thickness 0 fills; otherwise a ring).
struct CircleContent {
    Vec2 center{};
    float radius = 0;
};

/// Markers at each point — the point-cloud content. Thickness 0 draws
/// filled dots; a positive thickness draws rings.
struct CirclesContent {
    std::vector<Vec2> centers;
    float radius = 6;           ///< Marker radius in logical units.
};

/// A circular arc from `start_deg` to `end_deg` (degrees; 0° = +x,
/// increasing clockwise on screen). Gauges and angle indicators.
/// L0 draws round caps; Cap::Butt on arcs is a documented L1 item.
struct ArcContent {
    Vec2 center{};
    float radius = 0;
    float start_deg = 0;
    float end_deg = 0;
};

/// An arrow from `from` to `to` with a filled triangular head.
struct ArrowContent {
    Vec2 from{};
    Vec2 to{};
    float head_size = 0;        ///< Head length in logical units; 0 = auto
                                ///< (scales with the layer's thickness).
};

/// A crosshair: four ticks around a center with a central gap.
struct CrosshairContent {
    Vec2 center{};
    float size = 24;            ///< Overall half-extent from the center.
    float gap = 0;              ///< Central gap radius; 0 = auto (size / 4).
};

/// An axis-aligned debug/calibration grid across the whole layer,
/// lines every `spacing` logical units (through the layer origin).
struct GridContent {
    float spacing = 100;
};

/// Data-driven detection boxes with optional temporal smoothing.
/// `targets` is what the caller set; `smoothed` is the presentation state
/// the smoothing filter advances each `Stage::advance(dt)` — deterministic
/// given the same inputs and dt sequence.
struct BoxesContent {
    std::vector<Box> targets;   ///< Latest detections (model value).
    std::vector<Box> smoothed;  ///< Presentation boxes actually drawn.
    float smoothing = 0;        ///< Time constant in seconds (~63% of the
                                ///< gap closes per constant); 0 disables.
    bool show_label = true;     ///< Draw label + score above each box.
};

/// The content sum type. `std::monostate` = no content (a group).
using Content = std::variant<std::monostate, ImageContent, TextContent, LineContent,
                             PolylineContent, PolygonContent, RectContent, CircleContent,
                             CirclesContent, ArcContent, ArrowContent, CrosshairContent,
                             GridContent, BoxesContent>;

}  // namespace fluent_stage
