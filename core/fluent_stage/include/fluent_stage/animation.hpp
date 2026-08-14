#pragma once

/// \file animation.hpp
/// \brief Implicit-animation machinery: easing curves and Animated<T>.
///
/// The model follows Core Animation (§9 of the design): every animatable
/// attribute stores a **model value** (the target you set) and a
/// **presentation value** (what renders this frame). Setting an attribute
/// inside a Transaction retargets the animation *from the current
/// presentation value*, so re-changing mid-flight never jumps. Time only
/// advances through an explicit `dt` (injected by the renderer or a test),
/// which makes every animation deterministic and replayable (§13-6).

#include "fluent_stage/geometry.hpp"
#include "fluent_stage/types.hpp"

namespace fluent_stage {

/// Applies an easing curve to normalized progress \p t (0–1).
inline float easeApply(Ease ease, float t) {
    switch (ease) {
        case Ease::Linear: return t;
        case Ease::In:     return t * t;
        case Ease::Out:    return 1.0f - (1.0f - t) * (1.0f - t);
        case Ease::InOut:  return t * t * (3.0f - 2.0f * t);
    }
    return t;
}

/// \name Linear interpolation for animatable value types
/// @{
inline float animMix(float a, float b, float t) { return a + (b - a) * t; }
inline Vec2 animMix(Vec2 a, Vec2 b, float t) { return {animMix(a.x, b.x, t), animMix(a.y, b.y, t)}; }
inline Rect animMix(const Rect& a, const Rect& b, float t) {
    return {animMix(a.x, b.x, t), animMix(a.y, b.y, t),
            animMix(a.w, b.w, t), animMix(a.h, b.h, t)};
}
/// @}

/// One animatable attribute: model value, presentation value, and the curve
/// between them. Attribute setters call snap() (immediate) or to()
/// (animated, from inside a Transaction); the render loop calls step(dt).
template <typename T>
class Animated {
public:
    Animated() = default;
    explicit Animated(const T& initial) : from_(initial), target_(initial), value_(initial) {}

    /// The presentation value — what renders this frame.
    const T& value() const { return value_; }
    /// The model value — the most recently set target.
    const T& target() const { return target_; }
    /// True while an animation is in flight.
    bool animating() const { return elapsed_ < duration_; }

    /// Sets the value immediately, cancelling any animation in flight.
    void snap(const T& v) {
        from_ = target_ = value_ = v;
        duration_ = elapsed_ = 0;
    }

    /// Animates from the **current presentation value** to \p v over
    /// \p duration seconds. A non-positive duration snaps.
    void to(const T& v, float duration, Ease ease) {
        if (duration <= 0) {
            snap(v);
            return;
        }
        from_ = value_;
        target_ = v;
        duration_ = duration;
        elapsed_ = 0;
        ease_ = ease;
    }

    /// Advances the animation by \p dt seconds (idempotent once finished).
    void step(float dt) {
        if (!animating()) {
            return;
        }
        elapsed_ += dt;
        if (elapsed_ >= duration_) {
            elapsed_ = duration_;
            value_ = target_;
            return;
        }
        value_ = animMix(from_, target_, easeApply(ease_, elapsed_ / duration_));
    }

private:
    T from_{};
    T target_{};
    T value_{};
    float duration_ = 0;
    float elapsed_ = 0;
    Ease ease_ = Ease::InOut;
};

}  // namespace fluent_stage
