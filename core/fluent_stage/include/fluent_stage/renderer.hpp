#pragma once

/// \file renderer.hpp
/// \brief Renderer — the contract every backend fulfills (§11 of the design).
///
/// `render()` advances the stage's animations by the injected `dt` and
/// composites the tree into a Surface the renderer owns (the returned view
/// is valid until the next render on the same renderer). Backends are
/// retained-mode: per frame they transfer changed data and replay recorded
/// work; identical trees produce identical images across backends within
/// the golden-test tolerance.
///
/// Time is always the caller's: pass real frame delta for live rendering,
/// a fixed dt sequence for deterministic tests, or 0 to re-render the
/// current state without advancing anything.

#include <cstdint>

#include "fluent_stage/surface.hpp"

namespace fluent_stage {

class Stage;

class Renderer {
public:
    virtual ~Renderer() = default;

    /// Renders at the stage's logical size (one output pixel per logical
    /// unit), advancing animations by \p dt seconds first.
    virtual const Surface& render(Stage& stage, float dt) = 0;

    /// Renders at an explicit output size. The logical canvas maps onto it
    /// per `stage.fit()` (letterbox / crop / stretch); lengths scale
    /// uniformly, and SDF evaluation keeps every edge vector-crisp.
    virtual const Surface& render(Stage& stage, uint32_t out_width, uint32_t out_height,
                                  float dt) = 0;
};

}  // namespace fluent_stage
