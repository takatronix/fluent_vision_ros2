#pragma once

/// \file effects.hpp
/// \brief Pointer-driven visual effects — same philosophy as ui.hpp:
///        prefab layers + implicit animation, no new drawing mechanisms.
///
/// Effects are app-owned helpers with an explicit clock: feed them pointer
/// positions from your input path and `tick(dt)` once per frame with the
/// same dt you pass to `render()`. Everything stays deterministic — a fixed
/// dt sequence reproduces the exact same rings, which is how the golden
/// test draws "a ripple at t = 0.15 s".
///
/// ```cpp
/// fx::Ripple ripple(stage.root());       // construct after your UI so the
///                                        // rings draw on top
/// // input path (web viewer / touch / VR):
/// ripple.pointerMoved(pos);              // hover trail (spacing-throttled)
/// ripple.splash(pos);                    // tap splash (double ring)
/// // frame loop:
/// ripple.tick(dt);
/// renderer.render(stage, dt);
/// ```
///
/// This is the *ring* ripple (Material-style wake). The refractive water
/// ripple that warps the image beneath is a GPU filter planned with the
/// Vulkan backend (L1) — same single-source filter pipeline, one more
/// function in filters_shared.h.

#include <vector>

#include "fluent_stage/layer.hpp"

namespace fluent_stage {
namespace fx {

/// Visual parameters of the ring ripple; every field has a working default.
struct RippleStyle {
    Color color{1, 1, 1, 0.45f};  ///< Ring color (alpha = starting opacity).
    float radius = 44;            ///< Ring radius at full expansion.
    float thickness = 2.5f;       ///< Ring stroke width.
    float duration = 0.7f;        ///< Seconds from spawn to fully faded.
    float start_scale = 0.15f;    ///< Rings start at radius × start_scale.
    float spawn_spacing = 36;     ///< Trail: logical units between rings.
    uint32_t max_rings = 24;      ///< Hard cap (S-6); oldest rings recycle.
};

/// Expanding, fading rings that trail the pointer like a wake on water.
/// Bounded, deterministic, and made of ordinary circle layers animated by
/// Transaction — remove the helper and the effect is gone.
class Ripple {
public:
    /// Builds the effect group under \p parent (append after your UI so
    /// rings composite on top).
    explicit Ripple(Layer& parent, RippleStyle style = {});
    ~Ripple();
    Ripple(const Ripple&) = delete;
    Ripple& operator=(const Ripple&) = delete;

    /// Feed every pointer position (hover or drag). Spawns a trail ring
    /// whenever the pointer moved `spawn_spacing` units since the last one.
    void pointerMoved(Vec2 pos);

    /// A tap splash: an immediate ring plus a delayed second ring.
    void splash(Vec2 pos);

    /// Advances ring lifetimes and prunes finished rings. Call once per
    /// frame with the same dt you pass to render().
    void tick(float dt);

    /// Live ring count (for tests and budgeting).
    size_t ringCount() const { return rings_.size(); }

    /// The effect's root group.
    Layer& layer() { return *group_; }

private:
    void spawn(Vec2 pos, float alpha_mult);

    struct Ring {
        Layer* layer;
        float remaining;
    };
    struct Pending {
        Vec2 pos;
        float delay;
    };

    Layer* group_ = nullptr;
    std::vector<Ring> rings_;
    std::vector<Pending> pending_;
    RippleStyle style_;
    Vec2 last_spawn_{-1e9f, -1e9f};
};

}  // namespace fx
}  // namespace fluent_stage
