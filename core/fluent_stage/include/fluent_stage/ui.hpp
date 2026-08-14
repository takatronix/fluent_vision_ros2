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
#include <vector>

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

/// Visual parameters of a Slider; every field has a working default.
struct SliderStyle {
    Color track{0.32f, 0.35f, 0.40f, 0.95f};  ///< Full-length track.
    Color fill{0.10f, 0.70f, 0.55f, 1.0f};    ///< Active portion.
    Color knob{1, 1, 1, 1};                   ///< Knob color.
    float track_height = 8;        ///< Track thickness in logical units.
    float knob_margin = 2;         ///< Knob radius = frame.h/2 − margin.
    float set_duration = 0.15f;    ///< Programmatic setValue transition.
    float disabled_opacity = 0.4f; ///< Whole-control opacity when disabled.
};

/// A horizontal value slider, 0..1 (`ui.slider` of the §10 catalog).
/// Dragging tracks the pointer directly (no lag); programmatic setValue
/// animates. onChange fires continuously while dragging — live parameters
/// (speed limits, thresholds) want every step.
class Slider {
public:
    /// Builds the prefab under \p parent at \p frame with an initial value.
    Slider(Layer& parent, Rect frame, float value = 0, SliderStyle style = {});
    ~Slider();
    Slider(const Slider&) = delete;
    Slider& operator=(const Slider&) = delete;

    /// Sets the value (clamped to 0..1); animated by default. Does not
    /// fire onChange (that is for user gestures).
    Slider& setValue(float value01, bool animated = true);
    float value() const { return value_; }
    /// Called with the new value on every user-driven change.
    Slider& onChange(std::function<void(float)> fn);
    /// Enables/disables: a disabled slider ignores pointers and dims.
    Slider& enabled(bool on);
    bool isEnabled() const { return enabled_; }

    /// The prefab's root group.
    Layer& layer() { return *group_; }
    /// Removes the visuals from the stage; the control becomes inert.
    void remove();

private:
    void applyValue(bool animated);
    void installHandler();
    float valueForLocalX(float x) const;

    Layer* group_ = nullptr;
    Layer* fill_clip_ = nullptr;
    Layer* knob_ = nullptr;
    SliderStyle style_;
    Rect frame_;
    float value_ = 0;
    bool enabled_ = true;
    std::function<void(float)> on_change_;
};

/// Visual parameters of a Segmented control.
struct SegmentedStyle {
    Color background{0.15f, 0.17f, 0.21f, 0.92f};  ///< Whole-control plate.
    Color pill{0.10f, 0.70f, 0.55f, 1.0f};         ///< Selected-segment pill.
    Color label{1, 1, 1, 1};                       ///< Selected label.
    Color label_unselected{1, 1, 1, 0.55f};        ///< Other labels.
    float corner_radius = 10;      ///< Plate rounding (pill uses a bit less).
    float pill_margin = 3;         ///< Gap between pill and plate edge.
    float font_size = 18;          ///< Label size.
    float slide_duration = 0.15f;  ///< Pill slide transition.
    float disabled_opacity = 0.4f; ///< Whole-control opacity when disabled.
};

/// An exclusive-choice control (2–5 options): every option stays visible
/// and one tap selects — on a touch screen or through a VR ray this beats
/// a dropdown whenever the choice set is small.
class Segmented {
public:
    /// Builds the prefab under \p parent at \p frame; segments divide the
    /// width equally.
    Segmented(Layer& parent, Rect frame, std::vector<std::string> options, int selected = 0,
              SegmentedStyle style = {});
    ~Segmented();
    Segmented(const Segmented&) = delete;
    Segmented& operator=(const Segmented&) = delete;

    /// Selects an index (clamped); animated by default. Does not fire
    /// onChange (that is for user gestures).
    Segmented& select(int index, bool animated = true);
    int selected() const { return selected_; }
    /// Called with the newly selected index on a user tap.
    Segmented& onChange(std::function<void(int)> fn);
    /// Enables/disables: a disabled control ignores pointers and dims.
    Segmented& enabled(bool on);
    bool isEnabled() const { return enabled_; }

    /// The prefab's root group.
    Layer& layer() { return *group_; }
    /// Removes the visuals from the stage; the control becomes inert.
    void remove();

private:
    void applySelection(bool animated);
    void installHandler();

    Layer* group_ = nullptr;
    Layer* pill_ = nullptr;
    std::vector<Layer*> labels_;
    SegmentedStyle style_;
    Rect frame_;
    int selected_ = 0;
    bool enabled_ = true;
    std::function<void(int)> on_change_;
};

/// Visual parameters of a Gauge.
struct GaugeStyle {
    Color track{1, 1, 1, 0.18f};             ///< Full-sweep background arc.
    Color value{0.10f, 0.70f, 0.55f, 1.0f};  ///< Value arc.
    Color text{1, 1, 1, 1};                  ///< Percent readout color.
    float thickness = 10;    ///< Arc stroke width.
    float start_deg = 135;   ///< Sweep start (0° = +x, clockwise on screen).
    float sweep_deg = 270;   ///< Total sweep for value 1.0.
    float font_size = 0;     ///< Percent size; 0 = auto (radius × 0.5).
    bool show_percent = true;  ///< Draw "64%" in the center.
};

/// A display-only radial gauge (`ui.gauge` of the §10 catalog): battery,
/// speed, progress. setValue is immediate — live telemetry is its own
/// smoothing; wrap updates in your own filter if the source is jumpy.
class Gauge {
public:
    /// Builds the prefab under \p parent around \p center.
    Gauge(Layer& parent, Vec2 center, float radius, GaugeStyle style = {});
    ~Gauge() = default;
    Gauge(const Gauge&) = delete;
    Gauge& operator=(const Gauge&) = delete;

    /// Sets the displayed value (clamped to 0..1).
    Gauge& setValue(float value01);
    float value() const { return value_; }

    /// The prefab's root group.
    Layer& layer() { return *group_; }
    /// Removes the visuals from the stage.
    void remove();

private:
    Layer* group_ = nullptr;
    Layer* value_arc_ = nullptr;
    Layer* readout_ = nullptr;
    GaugeStyle style_;
    float value_ = 0;
};

/// Visual parameters of a Dropdown.
struct DropdownStyle {
    Color plate{0.15f, 0.17f, 0.21f, 0.95f};   ///< Closed control plate.
    Color popup{0.13f, 0.15f, 0.19f, 0.98f};   ///< Open list background.
    Color label{1, 1, 1, 1};                   ///< Text color.
    Color accent{0.10f, 0.70f, 0.55f, 1.0f};   ///< Chevron + selected row.
    float corner_radius = 10;      ///< Plate and popup rounding.
    float font_size = 20;          ///< Label and row text size.
    float row_height = 44;         ///< Popup row height.
    uint32_t max_visible = 8;      ///< Rows shown at once; longer lists
                                   ///< scroll by dragging inside the popup.
    float open_duration = 0.12f;   ///< Open/close fade+slide transition.
    float disabled_opacity = 0.4f; ///< Whole-control opacity when disabled.
};

/// A pulldown selector for larger choice sets (`ui.dropdown`). The popup
/// is nothing special: a group appended at the top of the root (tree order
/// = stacking order), a transparent full-stage scrim beneath it catches
/// the outside tap that closes it, and the chevron flips via ordinary
/// rotation. Lists longer than `max_visible` scroll by dragging inside the
/// popup (a move beyond ~6 units becomes a drag, not a tap); opening
/// scrolls the selected row into view.
class Dropdown {
public:
    /// Builds the prefab under \p parent at \p frame (the closed plate).
    Dropdown(Layer& parent, Rect frame, std::vector<std::string> options, int selected = 0,
             DropdownStyle style = {});
    ~Dropdown();
    Dropdown(const Dropdown&) = delete;
    Dropdown& operator=(const Dropdown&) = delete;

    /// Selects an index (clamped) and updates the plate label. Does not
    /// fire onChange (that is for user taps).
    Dropdown& select(int index);
    int selected() const { return selected_; }
    /// Called with the newly selected index when the user picks a row.
    Dropdown& onChange(std::function<void(int)> fn);
    /// True while the popup is showing.
    bool isOpen() const { return popup_ != nullptr; }
    /// Closes the popup if open (an outside tap does the same).
    Dropdown& close();
    /// Enables/disables: a disabled dropdown closes and ignores pointers.
    Dropdown& enabled(bool on);
    bool isEnabled() const { return enabled_; }

    /// The prefab's root group (the closed plate).
    Layer& layer() { return *group_; }
    /// Removes the visuals (popup included); the control becomes inert.
    void remove();

private:
    void installHandler();
    void openPopup();

    Layer* group_ = nullptr;
    Layer* plate_label_ = nullptr;
    Layer* chevron_ = nullptr;
    Layer* scrim_ = nullptr;
    Layer* popup_ = nullptr;
    Layer* popup_rows_ = nullptr;
    DropdownStyle style_;
    Rect frame_;
    std::vector<std::string> options_;
    int selected_ = 0;
    bool enabled_ = true;
    // popup drag-scroll state
    float scroll_ = 0;
    float min_scroll_ = 0;
    float drag_start_y_ = 0;
    float drag_start_scroll_ = 0;
    bool dragging_ = false;
    std::function<void(int)> on_change_;
};

}  // namespace ui
}  // namespace fluent_stage
