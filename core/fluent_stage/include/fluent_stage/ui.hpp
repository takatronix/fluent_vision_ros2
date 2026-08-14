#pragma once

/// \file ui.hpp
/// \brief UI controls — layers all the way down (§10 of the design).
///
/// A control is **not** a new drawing mechanism. It is:
///
///   1. a *prefab subtree* of ordinary layers (a button = background rect
///      + text label; a switch = track + knob),
///   2. a small *state machine* whose states are nothing but **attribute
///      overrides** (pressed → scale 0.96 + accent background; disabled →
///      opacity 0.4), applied inside a Transaction so every state change
///      animates by the ordinary §9 rules,
///   3. an *event contract*: pointer gestures come in through
///      `Stage::pointerDown/Move/Up` (injected from a web viewer, touch
///      screen, or VR ray — the Stage owns no input device) and results
///      leave through C++ callbacks (`onTap`, `onChange`). ROS topic
///      output rides on the same callbacks in Phase L3.
///
/// ```cpp
/// ui::Button start(stage.root(), {24, 900, 220, 64}, "収穫開始");
/// start.onTap([&] { robot.startHarvest(); });
///
/// ui::Switch light(stage.root(), {270, 908, 96, 48});
/// light.onChange([&](bool on) { robot.setLight(on); });
///
/// // per input event from the viewer:
/// stage.pointerDown(pos);  // …Move / Up — that's the whole integration
/// ```
///
/// ### Lifetime
/// A control object owns its behavior, the Stage owns its layers. Keep the
/// control alive while it should react (destroying it detaches the handler
/// and leaves the visuals inert); call `remove()` to delete the visuals
/// too. Do not remove a control's layers out from under a live control.

#include <functional>
#include <string>

#include "fluent_stage/layer.hpp"

namespace fluent_stage {
namespace ui {

/// Visual parameters of a Button; every field has a working default.
struct ButtonStyle {
    Color background{0.15f, 0.17f, 0.21f, 0.92f};        ///< Normal plate.
    Color background_pressed{0.10f, 0.70f, 0.55f, 1.0f}; ///< Pressed plate (§10 example).
    Color label{1, 1, 1, 1};                             ///< Label color.
    float corner_radius = 12;      ///< Plate rounding.
    float font_size = 22;          ///< Label size in logical units.
    float pressed_scale = 0.96f;   ///< Shrink while pressed.
    float press_duration = 0.08f;  ///< Press/release transition seconds.
    float disabled_opacity = 0.4f; ///< Whole-control opacity when disabled.
};

/// A momentary push button (`ui.button` of the §10 catalog).
class Button {
public:
    /// Builds the prefab under \p parent at \p frame (parent-local units).
    Button(Layer& parent, Rect frame, std::string label, ButtonStyle style = {});
    ~Button();
    Button(const Button&) = delete;
    Button& operator=(const Button&) = delete;

    /// Called on a completed tap (Up inside the button).
    Button& onTap(std::function<void()> fn);
    /// Enables/disables: a disabled button ignores pointers and dims.
    Button& enabled(bool on);
    bool isEnabled() const { return enabled_; }
    /// Replaces the label text.
    Button& label(const std::string& utf8);
    /// True while visually pressed (for tests and inspection).
    bool isPressed() const { return pressed_; }

    /// The prefab's root group — style or reposition it like any layer.
    Layer& layer() { return *group_; }
    /// Removes the visuals from the stage; the control becomes inert.
    void remove();

private:
    void setPressed(bool on);
    void installHandler();

    Layer* group_ = nullptr;
    Layer* plate_ = nullptr;
    Layer* label_ = nullptr;
    ButtonStyle style_;
    bool pressed_ = false;
    bool enabled_ = true;
    std::function<void()> on_tap_;
};

/// Visual parameters of a Switch; every field has a working default.
struct SwitchStyle {
    Color track_off{0.32f, 0.35f, 0.40f, 0.95f};  ///< Track when off.
    Color track_on{0.10f, 0.70f, 0.55f, 1.0f};    ///< Track when on.
    Color knob{1, 1, 1, 1};                       ///< Knob color.
    float knob_margin = 4;         ///< Gap between knob and track edge.
    float toggle_duration = 0.15f; ///< Slide/fade transition seconds.
    float disabled_opacity = 0.4f; ///< Whole-control opacity when disabled.
};

/// A toggle switch (`ui.switch` of the §10 catalog). Tap anywhere on it to
/// flip. The on-state track cross-fades and the knob slides — both via the
/// ordinary implicit-animation machinery, so a fixed-dt test can render
/// the switch mid-toggle.
class Switch {
public:
    /// Builds the prefab under \p parent at \p frame (a wide, short rect;
    /// the §10 example uses 96×48).
    Switch(Layer& parent, Rect frame, SwitchStyle style = {});
    ~Switch();
    Switch(const Switch&) = delete;
    Switch& operator=(const Switch&) = delete;

    /// Sets the value; animated by default, immediate when \p animated is
    /// false. Does not fire onChange (that is for user gestures).
    Switch& setOn(bool value, bool animated = true);
    bool isOn() const { return on_; }
    /// Called when a tap flips the value.
    Switch& onChange(std::function<void(bool)> fn);
    /// Enables/disables: a disabled switch ignores pointers and dims.
    Switch& enabled(bool on);
    bool isEnabled() const { return enabled_; }

    /// The prefab's root group.
    Layer& layer() { return *group_; }
    /// Removes the visuals from the stage; the control becomes inert.
    void remove();

private:
    void applyValue(bool animated);
    void installHandler();

    Layer* group_ = nullptr;
    Layer* track_on_ = nullptr;
    Layer* knob_ = nullptr;
    SwitchStyle style_;
    Rect frame_;
    bool on_ = false;
    bool enabled_ = true;
    std::function<void(bool)> on_change_;
};

}  // namespace ui
}  // namespace fluent_stage
