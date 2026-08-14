#pragma once

/// \file inspector.hpp
/// \brief The §13-3 scene inspector — "why can't I see it", answerable by
///        machine (the DOM-inspector counterpart for a compiled Scene).
///
/// The inspector fixes every layer of a compiled scene in stage space —
/// resolved bounds, the local→stage transform, effective opacity, paint
/// order — and answers the questions an author (human or AI) asks a
/// picture:
///
///   - `placeLayers()`  — the placement table itself (the linter builds its
///     checks on this same walk; §13-2 and §13-3 share one geometry).
///   - `visibleAt()`    — "what is at (x, y)?": the layers whose resolved
///     bounds contain the point, topmost first.
///   - `inspectJson()`  — the whole placed tree as JSON (ids, content
///     types, stage-space frames, opacity, occlusion candidates), for
///     tools, scene_web, and MCP.
///
/// Placement uses presentation values (what renders this frame), so an
/// in-flight animation inspects mid-flight — render once (or advance) and
/// the numbers match the pixels.

#include <string>
#include <vector>

#include "fluent_stage/scene/compiler.hpp"

namespace fluent_stage::scene {

/// One layer fixed in stage space.
struct PlacedLayer {
    const Layer* layer = nullptr;   ///< The live layer.
    const LayerDecl* decl = nullptr;  ///< Its document node (nullptr for
                                      ///< synthetic layers, e.g. placeholders).
    Mat23 to_stage;          ///< Layer-local → stage coordinates.
    Rect bounds;             ///< Resolved local bounds.
    Rect stage_bbox;         ///< Axis-aligned bbox of bounds in stage space.
    float eff_opacity = 1;   ///< Product of opacities down the chain.
    bool axis_aligned = true;  ///< No rotation/shear anywhere in the chain.
    int paint_index = 0;     ///< Pre-order position (greater = painted later).
    int parent_index = -1;   ///< Index of the parent in the placed list.
};

/// Places every visible (non-hidden) layer of the compiled scene in stage
/// space, in paint order. Uses presentation geometry — call after a render
/// or `advance()` for numbers that match the pixels.
std::vector<PlacedLayer> placeLayers(const CompiledScene& scene);

/// The layers whose resolved stage-space bounds contain `point`, topmost
/// (last painted) first. Bounds containment, not pixel alpha — a transparent
/// corner of a layer still counts as that layer, exactly like a DOM
/// inspector.
std::vector<const PlacedLayer*> visibleAt(const std::vector<PlacedLayer>& placed, Vec2 point);

/// The placed tree as a JSON array (paint order): id, content type, role,
/// declared/synthetic, stage-space frame, effective opacity. `point` queries
/// serialize with `atJson`.
std::string inspectJson(const CompiledScene& scene, const std::vector<PlacedLayer>& placed);

/// The `visibleAt` answer as JSON, topmost first.
std::string atJson(const std::vector<PlacedLayer>& placed, Vec2 point);

/// One placed layer as a JSON object (the element form used by inspectJson
/// and atJson). Lets a server snapshot the serialized entries together with
/// their geometry and answer queries later without holding Layer pointers.
std::string placedLayerJson(const PlacedLayer& placed);

}  // namespace fluent_stage::scene
