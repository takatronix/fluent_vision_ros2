// effects.cpp — fx::Ripple drives fs_ripple refraction filters on its
// target layer; the wave math itself lives once in filters_shared.h and
// runs on whichever backend renders the frame.

#include "fluent_stage/effects.hpp"

#include <cmath>

namespace fluent_stage {
namespace fx {

Ripple::Ripple(Layer& target, RippleStyle style)
    : target_(&target), base_filters_(target.filters()), style_(style) {}

Ripple::~Ripple() {
    // Leave the layer as we found it.
    if (target_ != nullptr) {
        target_->clearFilters();
        for (const Filter& f : base_filters_) {
            target_->filter(f);
        }
    }
}

void Ripple::spawn(Vec2 pos, float strength) {
    if (waves_.size() >= style_.max_waves) {
        waves_.erase(waves_.begin());  // oldest wave yields its pass
    }
    waves_.push_back({pos, 0, strength});
    last_spawn_ = pos;
    applyFilters();
}

void Ripple::pointerMoved(Vec2 pos) {
    const float dist = std::hypot(pos.x - last_spawn_.x, pos.y - last_spawn_.y);
    if (dist >= style_.spawn_spacing) {
        spawn(pos, style_.trail_scale);
    }
}

void Ripple::splash(Vec2 pos) { spawn(pos, 1.0f); }

void Ripple::tick(float dt) {
    if (waves_.empty()) {
        return;
    }
    bool changed = false;
    for (size_t i = waves_.size(); i-- > 0;) {
        waves_[i].age += dt;
        if (waves_[i].age >= style_.duration) {
            waves_.erase(waves_.begin() + static_cast<ptrdiff_t>(i));
        }
        changed = true;
    }
    if (changed) {
        applyFilters();
    }
}

void Ripple::applyFilters() {
    target_->clearFilters();
    for (const Filter& f : base_filters_) {
        target_->filter(f);
    }
    for (const Wave& w : waves_) {
        const float t = w.age / style_.duration;  // 0..1
        // Water waves travel at constant speed: the wavefront expands
        // linearly, and the refraction energy decays quadratically as the
        // ring spreads out.
        const float radius = style_.max_radius * t;
        const float amplitude = style_.amplitude * w.strength * (1.0f - t) * (1.0f - t);
        target_->filter(RippleWave()
                            .center_x(w.pos.x)
                            .center_y(w.pos.y)
                            .radius(radius)
                            .amplitude(amplitude)
                            .wavelength(style_.wavelength));
    }
}

}  // namespace fx
}  // namespace fluent_stage
