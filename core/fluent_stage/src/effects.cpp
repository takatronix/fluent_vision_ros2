// effects.cpp — pointer-driven effects built from ordinary layers.

#include "fluent_stage/effects.hpp"

#include <cmath>

#include "fluent_stage/stage.hpp"
#include "fluent_stage/transaction.hpp"

namespace fluent_stage {
namespace fx {

Ripple::Ripple(Layer& parent, RippleStyle style) : style_(style) {
    group_ = &parent.group();
}

Ripple::~Ripple() {
    if (group_ != nullptr) {
        group_->remove();
    }
}

void Ripple::spawn(Vec2 pos, float alpha_mult) {
    if (rings_.size() >= style_.max_rings) {
        rings_.front().layer->remove();
        rings_.erase(rings_.begin());
    }
    Layer& ring = group_->circle({0, 0}, style_.radius)
                      .thickness(style_.thickness)
                      .color(style_.color.faded(alpha_mult))
                      .position(pos)
                      .scale(style_.start_scale);
    {
        Transaction t(style_.duration, Ease::Out);
        ring.scale(1.0f);
        ring.opacity(0.0f);
    }
    rings_.push_back({&ring, style_.duration});
    last_spawn_ = pos;
}

void Ripple::pointerMoved(Vec2 pos) {
    const float dist = std::hypot(pos.x - last_spawn_.x, pos.y - last_spawn_.y);
    if (dist >= style_.spawn_spacing) {
        spawn(pos, 0.7f);
    }
}

void Ripple::splash(Vec2 pos) {
    spawn(pos, 1.0f);
    pending_.push_back({pos, 0.1f});  // second ring rides 0.1 s behind
}

void Ripple::tick(float dt) {
    for (size_t i = pending_.size(); i-- > 0;) {
        pending_[i].delay -= dt;
        if (pending_[i].delay <= 0) {
            spawn(pending_[i].pos, 0.8f);
            pending_.erase(pending_.begin() + static_cast<ptrdiff_t>(i));
        }
    }
    for (size_t i = rings_.size(); i-- > 0;) {
        rings_[i].remaining -= dt;
        if (rings_[i].remaining <= 0) {
            rings_[i].layer->remove();
            rings_.erase(rings_.begin() + static_cast<ptrdiff_t>(i));
        }
    }
}

}  // namespace fx
}  // namespace fluent_stage
