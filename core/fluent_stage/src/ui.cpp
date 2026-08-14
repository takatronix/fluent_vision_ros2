// ui.cpp — Button and Switch prefabs (§10). No new drawing mechanisms:
// every visual is ordinary layers, every state change is attribute
// overrides inside a Transaction, every gesture arrives through the
// Stage's pointer injection.

#include "fluent_stage/ui.hpp"

#include "fluent_stage/stage.hpp"
#include "fluent_stage/transaction.hpp"

namespace fluent_stage {
namespace ui {

namespace {

// Vertical centering of a single text line inside a plate of height `h`.
// Uses the measured line height when the text engine is up; before the
// first render falls back to the Noto line-height ratio (≈1.45 × size).
float centeredTextY(Stage& stage, const std::string& utf8, float font_size, float h) {
    TextContent probe;
    probe.utf8 = utf8;
    probe.size = font_size;
    float line_h = stage.measureText(probe).h;
    if (line_h <= 0) {
        line_h = font_size * 1.45f;
    }
    return (h - line_h) * 0.5f;
}

}  // namespace

// ===========================================================================
// Button
// ===========================================================================

Button::Button(Layer& parent, Rect frame, std::string label, ButtonStyle style)
    : style_(style) {
    group_ = &parent.group();
    group_->frame(frame);  // explicit bounds: pressed_scale shrinks about the center
    plate_ = &group_->rect({0, 0, frame.w, frame.h})
                  .cornerRadius(style_.corner_radius)
                  .color(style_.background);
    Stage& stage = group_->stage();
    label_ = &group_->text(label,
                           {frame.w * 0.5f,
                            centeredTextY(stage, label, style_.font_size, frame.h)})
                  .size(style_.font_size)
                  .align(Align::Center)
                  .color(style_.label);

    installHandler();
}

void Button::installHandler() {
    group_->onPointer([this](const PointerEvent& e) {
        switch (e.phase) {
            case PointerPhase::Down:
                setPressed(true);
                break;
            case PointerPhase::Move:
                setPressed(e.inside);
                break;
            case PointerPhase::Up: {
                const bool tapped = pressed_ && e.inside;
                setPressed(false);
                if (tapped && on_tap_) {
                    on_tap_();
                }
                break;
            }
            case PointerPhase::Cancel:
                setPressed(false);
                break;
        }
    });
}

Button::~Button() {
    if (group_ != nullptr) {
        group_->onPointer({});
    }
}

void Button::setPressed(bool on) {
    if (pressed_ == on) {
        return;
    }
    pressed_ = on;
    // State = attribute overrides, transitioned by implicit animation (§10-2).
    Transaction t(style_.press_duration, Ease::Out);
    group_->scale(on ? style_.pressed_scale : 1.0f);
    plate_->color(on ? style_.background_pressed : style_.background);  // snaps (color is L1)
}

Button& Button::onTap(std::function<void()> fn) {
    on_tap_ = std::move(fn);
    return *this;
}

Button& Button::enabled(bool on) {
    if (enabled_ == on) {
        return *this;
    }
    enabled_ = on;
    if (!on) {
        setPressed(false);
    }
    {
        Transaction t(style_.press_duration, Ease::Out);
        group_->opacity(on ? 1.0f : style_.disabled_opacity);
    }
    // A disabled control is not an interactive target at all: the pointer
    // falls through to whatever lies beneath.
    if (on) {
        installHandler();
    } else {
        group_->onPointer({});
    }
    return *this;
}

Button& Button::label(const std::string& utf8) {
    label_->setText(utf8);
    return *this;
}

void Button::remove() {
    if (group_ != nullptr) {
        group_->onPointer({});
        group_->remove();
        group_ = plate_ = label_ = nullptr;
    }
}

// ===========================================================================
// Switch
// ===========================================================================

Switch::Switch(Layer& parent, Rect frame, SwitchStyle style)
    : style_(style), frame_(frame) {
    group_ = &parent.group();
    group_->frame(frame);
    const float radius = frame.h * 0.5f;
    group_->rect({0, 0, frame.w, frame.h}).cornerRadius(radius).color(style_.track_off);
    // The on-state track is a second plate whose opacity cross-fades:
    // color itself is not animatable in L0, opacity is (§9).
    track_on_ = &group_->rect({0, 0, frame.w, frame.h})
                     .cornerRadius(radius)
                     .color(style_.track_on)
                     .opacity(0.0f);
    knob_ = &group_->circle({0, 0}, radius - style_.knob_margin)
                 .color(style_.knob)
                 .shadow(0, 1, 3)
                 .position(radius, radius);

    installHandler();
}

void Switch::installHandler() {
    group_->onPointer([this](const PointerEvent& e) {
        if (e.phase == PointerPhase::Up && e.inside) {
            on_ = !on_;
            applyValue(true);
            if (on_change_) {
                on_change_(on_);
            }
        }
    });
}

Switch::~Switch() {
    if (group_ != nullptr) {
        group_->onPointer({});
    }
}

void Switch::applyValue(bool animated) {
    const float radius = frame_.h * 0.5f;
    const float knob_x = on_ ? frame_.w - radius : radius;
    if (animated) {
        Transaction t(style_.toggle_duration, Ease::InOut);
        knob_->position(knob_x, radius);
        track_on_->opacity(on_ ? 1.0f : 0.0f);
    } else {
        knob_->position(knob_x, radius);
        track_on_->opacity(on_ ? 1.0f : 0.0f);
    }
}

Switch& Switch::setOn(bool value, bool animated) {
    if (on_ != value) {
        on_ = value;
        applyValue(animated);
    }
    return *this;
}

Switch& Switch::onChange(std::function<void(bool)> fn) {
    on_change_ = std::move(fn);
    return *this;
}

Switch& Switch::enabled(bool on) {
    if (enabled_ == on) {
        return *this;
    }
    enabled_ = on;
    {
        Transaction t(style_.toggle_duration, Ease::Out);
        group_->opacity(on ? 1.0f : style_.disabled_opacity);
    }
    if (on) {
        installHandler();
    } else {
        group_->onPointer({});
    }
    return *this;
}

void Switch::remove() {
    if (group_ != nullptr) {
        group_->onPointer({});
        group_->remove();
        group_ = track_on_ = knob_ = nullptr;
    }
}

}  // namespace ui
}  // namespace fluent_stage
