#pragma once

/// \file effects.hpp
/// \brief Pointer-driven effects.
///
/// fx::Ripple is the **real water ripple**: each wave is an `fs_ripple`
/// refraction filter (shared/filters_shared.h — the same body runs as
/// SPIR-V on the GPU) whose sampling positions are displaced by a damped
/// sinusoid around an expanding wavefront. The image beneath genuinely
/// bends; nothing is drawn on top.
///
/// The helper owns the *filter chain of a target layer* — point it at the
/// layer you want to behave like a water surface (the camera image, or a
/// background group):
///
/// ```cpp
/// fx::Ripple ripple(video_layer);        // this layer becomes the water
/// // input path (web viewer / touch / VR):
/// ripple.pointerMoved(pos);              // wake along the hover trail
/// ripple.splash(pos);                    // tap → a stronger wave
/// // frame loop, same dt as render():
/// ripple.tick(dt);
/// renderer.render(stage, dt);
/// ```
///
/// Positions are in the target layer's local logical space (for a
/// full-canvas layer that is simply stage coordinates). Deterministic:
/// waves advance only through tick(dt), so a fixed dt sequence reproduces
/// the exact refraction — golden-testable like everything else.

#include <vector>

#include "fluent_stage/layer.hpp"

namespace fluent_stage {
namespace fx {

/// Wave parameters; every field has a working default.
struct RippleStyle {
    float amplitude = 7;      ///< Peak refraction displacement (logical units).
    float wavelength = 26;    ///< Crest-to-crest distance (logical units).
    float max_radius = 220;   ///< Wavefront radius when fully expanded.
    float duration = 1.1f;    ///< Seconds from splash to dissipated.
    float trail_scale = 0.5f; ///< Amplitude factor for hover-trail waves.
    float spawn_spacing = 48; ///< Trail: logical units between waves.
    uint32_t max_waves = 8;   ///< Hard cap (each wave is one filter pass);
                              ///< the oldest wave recycles past it.
};

/// Expanding refraction waves on a target layer (see file comment).
class Ripple {
public:
    /// Takes ownership of \p target's filter chain: filters present now are
    /// kept as the base; waves are appended after them each tick.
    explicit Ripple(Layer& target, RippleStyle style = {});
    ~Ripple();
    Ripple(const Ripple&) = delete;
    Ripple& operator=(const Ripple&) = delete;

    /// Feed every pointer position (hover or drag); spawns a trail wave
    /// each `spawn_spacing` units of travel.
    void pointerMoved(Vec2 pos);

    /// A tap splash: one full-strength wave.
    void splash(Vec2 pos);

    /// Advances the waves and rewrites the target's filter chain. Call
    /// once per frame with the same dt you pass to render().
    void tick(float dt);

    /// Live wave count (for tests and budgeting).
    size_t waveCount() const { return waves_.size(); }

private:
    void spawn(Vec2 pos, float strength);
    void applyFilters();

    struct Wave {
        Vec2 pos;
        float age;
        float strength;
    };

    Layer* target_ = nullptr;
    std::vector<Filter> base_filters_;
    std::vector<Wave> waves_;
    RippleStyle style_;
    Vec2 last_spawn_{-1e9f, -1e9f};
};

}  // namespace fx
}  // namespace fluent_stage
