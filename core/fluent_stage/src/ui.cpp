// ui.cpp — Button and Switch prefabs (§10). No new drawing mechanisms:
// every visual is ordinary layers, every state change is attribute
// overrides inside a Transaction, every gesture arrives through the
// Stage's pointer injection.

#include "fluent_stage/ui.hpp"

#include <algorithm>
#include <cmath>

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

float clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

// Maps a point in `layer`'s bounds space to stage coordinates (presentation
// geometry) — the dropdown uses it to place its popup on the root.
Vec2 localToStage(Layer& layer, Vec2 local) {
    std::vector<Layer*> chain;
    for (Layer* l = &layer; l != nullptr; l = l->parent()) {
        chain.push_back(l);
    }
    Stage& stage = layer.stage();
    Vec2 parent_size{stage.width(), stage.height()};
    Mat23 m = Mat23::identity();
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
        const Layer::Resolved r = (*it)->resolve(parent_size);
        m = m * r.to_parent;
        parent_size = r.bounds.size();
    }
    return m.apply(local);
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

// ===========================================================================
// Slider
// ===========================================================================

Slider::Slider(Layer& parent, Rect frame, float value, SliderStyle style)
    : style_(style), frame_(frame), value_(clamp01(value)) {
    group_ = &parent.group();
    group_->frame(frame);
    const float track_y = (frame.h - style_.track_height) * 0.5f;
    const float track_r = style_.track_height * 0.5f;
    group_->rect({0, track_y, frame.w, style_.track_height})
        .cornerRadius(track_r)
        .color(style_.track);
    // The fill is a full-length pill revealed by an animated clip: bounds
    // width is animatable, content size is not — so the clip moves, the
    // pill never deforms.
    fill_clip_ = &group_->group();
    fill_clip_->anchor(0, 0).position(0, 0).masksToBounds(true);
    fill_clip_->rect({0, track_y, frame.w, style_.track_height})
        .cornerRadius(track_r)
        .color(style_.fill);
    knob_ = &group_->circle({0, 0}, frame.h * 0.5f - style_.knob_margin)
                 .color(style_.knob)
                 .shadow(0, 1, 3);
    applyValue(false);
    installHandler();
}

Slider::~Slider() {
    if (group_ != nullptr) {
        group_->onPointer({});
    }
}

float Slider::valueForLocalX(float x) const {
    const float r = frame_.h * 0.5f;
    return clamp01((x - r) / std::max(frame_.w - 2 * r, 1e-3f));
}

void Slider::applyValue(bool animated) {
    const float r = frame_.h * 0.5f;
    const float knob_x = r + value_ * (frame_.w - 2 * r);
    const Rect clip{0, 0, knob_x, frame_.h};
    if (animated) {
        Transaction t(style_.set_duration, Ease::InOut);
        knob_->position(knob_x, frame_.h * 0.5f);
        fill_clip_->bounds(clip);
    } else {
        knob_->position(knob_x, frame_.h * 0.5f);
        fill_clip_->bounds(clip);
    }
}

void Slider::installHandler() {
    group_->onPointer([this](const PointerEvent& e) {
        if (e.phase != PointerPhase::Down && e.phase != PointerPhase::Move) {
            return;
        }
        const float v = valueForLocalX(e.local_pos.x);
        if (v != value_) {
            value_ = v;
            applyValue(false);  // dragging tracks the pointer directly
            if (on_change_) {
                on_change_(value_);
            }
        }
    });
}

Slider& Slider::setValue(float value01, bool animated) {
    const float v = clamp01(value01);
    if (v != value_) {
        value_ = v;
        applyValue(animated);
    }
    return *this;
}

Slider& Slider::onChange(std::function<void(float)> fn) {
    on_change_ = std::move(fn);
    return *this;
}

Slider& Slider::enabled(bool on) {
    if (enabled_ == on) {
        return *this;
    }
    enabled_ = on;
    {
        Transaction t(style_.set_duration, Ease::Out);
        group_->opacity(on ? 1.0f : style_.disabled_opacity);
    }
    if (on) {
        installHandler();
    } else {
        group_->onPointer({});
    }
    return *this;
}

void Slider::remove() {
    if (group_ != nullptr) {
        group_->onPointer({});
        group_->remove();
        group_ = fill_clip_ = knob_ = nullptr;
    }
}

// ===========================================================================
// Segmented
// ===========================================================================

Segmented::Segmented(Layer& parent, Rect frame, std::vector<std::string> options, int selected,
                     SegmentedStyle style)
    : style_(style), frame_(frame) {
    const int count = std::max<int>(1, static_cast<int>(options.size()));
    selected_ = std::clamp(selected, 0, count - 1);
    group_ = &parent.group();
    group_->frame(frame);
    group_->rect({0, 0, frame.w, frame.h})
        .cornerRadius(style_.corner_radius)
        .color(style_.background);
    const float seg_w = frame.w / count;
    const float m = style_.pill_margin;
    pill_ = &group_->rect({0, 0, seg_w - 2 * m, frame.h - 2 * m})
                 .cornerRadius(std::max(style_.corner_radius - m, 0.0f))
                 .color(style_.pill);
    Stage& stage = group_->stage();
    for (int i = 0; i < count; ++i) {
        const std::string& text = options[static_cast<size_t>(i)];
        labels_.push_back(&group_->text(text,
                                        {seg_w * (i + 0.5f),
                                         centeredTextY(stage, text, style_.font_size, frame.h)})
                               .size(style_.font_size)
                               .align(Align::Center));
    }
    applySelection(false);
    installHandler();
}

Segmented::~Segmented() {
    if (group_ != nullptr) {
        group_->onPointer({});
    }
}

void Segmented::applySelection(bool animated) {
    const float seg_w = frame_.w / static_cast<float>(labels_.size());
    const Vec2 pill_center{seg_w * (selected_ + 0.5f), frame_.h * 0.5f};
    if (animated) {
        Transaction t(style_.slide_duration, Ease::InOut);
        pill_->position(pill_center);
    } else {
        pill_->position(pill_center);
    }
    for (size_t i = 0; i < labels_.size(); ++i) {
        labels_[i]->color(static_cast<int>(i) == selected_ ? style_.label
                                                           : style_.label_unselected);
    }
}

void Segmented::installHandler() {
    group_->onPointer([this](const PointerEvent& e) {
        if (e.phase != PointerPhase::Up || !e.inside) {
            return;
        }
        const float seg_w = frame_.w / static_cast<float>(labels_.size());
        const int index = std::clamp(static_cast<int>(e.local_pos.x / seg_w), 0,
                                     static_cast<int>(labels_.size()) - 1);
        if (index != selected_) {
            selected_ = index;
            applySelection(true);
            if (on_change_) {
                on_change_(selected_);
            }
        }
    });
}

Segmented& Segmented::select(int index, bool animated) {
    const int clamped = std::clamp(index, 0, static_cast<int>(labels_.size()) - 1);
    if (clamped != selected_) {
        selected_ = clamped;
        applySelection(animated);
    }
    return *this;
}

Segmented& Segmented::onChange(std::function<void(int)> fn) {
    on_change_ = std::move(fn);
    return *this;
}

Segmented& Segmented::enabled(bool on) {
    if (enabled_ == on) {
        return *this;
    }
    enabled_ = on;
    {
        Transaction t(style_.slide_duration, Ease::Out);
        group_->opacity(on ? 1.0f : style_.disabled_opacity);
    }
    if (on) {
        installHandler();
    } else {
        group_->onPointer({});
    }
    return *this;
}

void Segmented::remove() {
    if (group_ != nullptr) {
        group_->onPointer({});
        group_->remove();
        group_ = pill_ = nullptr;
        labels_.clear();
    }
}

// ===========================================================================
// Gauge
// ===========================================================================

Gauge::Gauge(Layer& parent, Vec2 center, float radius, GaugeStyle style) : style_(style) {
    group_ = &parent.group();
    group_->arc(center, radius, style_.start_deg, style_.start_deg + style_.sweep_deg)
        .thickness(style_.thickness)
        .color(style_.track);
    value_arc_ = &group_->arc(center, radius, style_.start_deg, style_.start_deg)
                      .thickness(style_.thickness)
                      .color(style_.value);
    if (style_.show_percent) {
        const float size = style_.font_size > 0 ? style_.font_size : radius * 0.5f;
        readout_ = &group_->text("0%", {center.x, center.y - size * 0.72f})
                        .size(size)
                        .align(Align::Center)
                        .color(style_.text);
    }
    setValue(0);
}

Gauge& Gauge::setValue(float value01) {
    value_ = clamp01(value01);
    value_arc_->setArc(style_.start_deg, style_.start_deg + style_.sweep_deg * value_);
    if (readout_ != nullptr) {
        readout_->setText(std::to_string(static_cast<int>(value_ * 100 + 0.5f)) + "%");
    }
    return *this;
}

void Gauge::remove() {
    if (group_ != nullptr) {
        group_->remove();
        group_ = value_arc_ = readout_ = nullptr;
    }
}

// ===========================================================================
// Dropdown
// ===========================================================================

Dropdown::Dropdown(Layer& parent, Rect frame, std::vector<std::string> options, int selected,
                   DropdownStyle style)
    : style_(style), frame_(frame), options_(std::move(options)) {
    if (options_.empty()) {
        options_.push_back("");
    }
    selected_ = std::clamp(selected, 0, static_cast<int>(options_.size()) - 1);
    group_ = &parent.group();
    group_->frame(frame);
    group_->rect({0, 0, frame.w, frame.h})
        .cornerRadius(style_.corner_radius)
        .color(style_.plate);
    Stage& stage = group_->stage();
    plate_label_ = &group_->text(options_[static_cast<size_t>(selected_)],
                                 {16, centeredTextY(stage, options_[0], style_.font_size,
                                                    frame.h)})
                        .size(style_.font_size)
                        .color(style_.label);
    // Chevron: a small "v" that flips (rotates 180°) while open.
    const float ch = 7;
    chevron_ = &group_->polyline({{0, 0}, {ch, ch}, {2 * ch, 0}})
                    .thickness(2.5f)
                    .color(style_.accent)
                    .position(frame.w - 16 - ch, frame.h * 0.5f);
    installHandler();
}

Dropdown::~Dropdown() {
    close();
    if (group_ != nullptr) {
        group_->onPointer({});
    }
}

void Dropdown::installHandler() {
    group_->onPointer([this](const PointerEvent& e) {
        if (e.phase == PointerPhase::Up && e.inside) {
            if (isOpen()) {
                close();
            } else {
                openPopup();
            }
        }
    });
}

void Dropdown::openPopup() {
    Stage& stage = group_->stage();
    const uint32_t visible =
        std::min<uint32_t>(static_cast<uint32_t>(options_.size()), style_.max_visible);
    const float pad = 6;
    const float pw = frame_.w;
    const float ph = visible * style_.row_height + 2 * pad;

    // Popup below the plate in stage coordinates; open upward when it
    // would leave the canvas.
    const Vec2 plate_origin = localToStage(*group_, {0, 0});
    float px = std::clamp(plate_origin.x, 0.0f, std::max(stage.width() - pw, 0.0f));
    float py = plate_origin.y + frame_.h + 4;
    if (py + ph > stage.height()) {
        py = std::max(plate_origin.y - ph - 4, 0.0f);
    }

    // Scrim first (under the popup): a transparent full-stage plate whose
    // only job is to catch the outside tap that closes the popup.
    scrim_ = &stage.root().rect({0, 0, stage.width(), stage.height()}).color(Color::Transparent);
    scrim_->onPointer([this](const PointerEvent& e) {
        if (e.phase == PointerPhase::Down) {
            close();
        }
    });

    popup_ = &stage.root().group();
    popup_->frame({px, py, pw, ph});
    popup_->masksToBounds(true);
    popup_->rect({0, 0, pw, ph}).cornerRadius(style_.corner_radius).color(style_.popup);

    // All rows live in an inner group whose origin drag-scrolls; the popup
    // clips it. Opening scrolls the selected row into view.
    const float content_h = static_cast<float>(options_.size()) * style_.row_height;
    const float visible_h = visible * style_.row_height;
    min_scroll_ = std::min(visible_h - content_h, 0.0f);
    scroll_ = std::clamp(-(selected_ * style_.row_height) + (visible_h - style_.row_height) * 0.5f,
                         min_scroll_, 0.0f);
    dragging_ = false;
    popup_rows_ = &popup_->group();
    popup_rows_->position(0, pad + scroll_);
    for (size_t i = 0; i < options_.size(); ++i) {
        const float row_y = i * style_.row_height;
        if (static_cast<int>(i) == selected_) {
            popup_rows_->rect({pad, row_y, pw - 2 * pad, style_.row_height})
                .cornerRadius(style_.corner_radius - 4)
                .color(style_.accent.faded(0.22f));
        }
        popup_rows_->text(options_[i],
                          {16, row_y + centeredTextY(stage, options_[i], style_.font_size,
                                                     style_.row_height)})
            .size(style_.font_size)
            .color(static_cast<int>(i) == selected_ ? style_.accent : style_.label);
    }

    popup_->onPointer([this, pad](const PointerEvent& e) {
        switch (e.phase) {
            case PointerPhase::Down:
                drag_start_y_ = e.local_pos.y;
                drag_start_scroll_ = scroll_;
                dragging_ = false;
                break;
            case PointerPhase::Move: {
                const float dy = e.local_pos.y - drag_start_y_;
                if (!dragging_ && std::fabs(dy) > 6.0f) {
                    dragging_ = true;
                }
                if (dragging_ && min_scroll_ < 0) {
                    scroll_ = std::clamp(drag_start_scroll_ + dy, min_scroll_, 0.0f);
                    popup_rows_->position(0, pad + scroll_);
                }
                break;
            }
            case PointerPhase::Up: {
                if (dragging_ || !e.inside) {
                    return;
                }
                const int row =
                    static_cast<int>((e.local_pos.y - pad - scroll_) / style_.row_height);
                if (row >= 0 && row < static_cast<int>(options_.size())) {
                    const bool changed = row != selected_;
                    select(row);
                    close();
                    if (changed && on_change_) {
                        on_change_(selected_);
                    }
                }
                break;
            }
            case PointerPhase::Cancel:
                dragging_ = false;
                break;
        }
    });

    // Open transition: fade + a short drop; chevron flips.
    popup_->opacity(0.0f);
    const Vec2 target = popup_->position();
    popup_->position(target.x, target.y - 8);
    {
        Transaction t(style_.open_duration, Ease::Out);
        popup_->opacity(1.0f);
        popup_->position(target);
        chevron_->rotation(180);
    }
}

Dropdown& Dropdown::close() {
    if (popup_ != nullptr) {
        popup_->onPointer({});
        popup_->remove();
        popup_ = nullptr;
        popup_rows_ = nullptr;
    }
    if (scrim_ != nullptr) {
        scrim_->onPointer({});
        scrim_->remove();
        scrim_ = nullptr;
    }
    if (chevron_ != nullptr) {
        Transaction t(style_.open_duration, Ease::Out);
        chevron_->rotation(0);
    }
    return *this;
}

Dropdown& Dropdown::select(int index) {
    selected_ = std::clamp(index, 0, static_cast<int>(options_.size()) - 1);
    plate_label_->setText(options_[static_cast<size_t>(selected_)]);
    return *this;
}

Dropdown& Dropdown::onChange(std::function<void(int)> fn) {
    on_change_ = std::move(fn);
    return *this;
}

Dropdown& Dropdown::enabled(bool on) {
    if (enabled_ == on) {
        return *this;
    }
    enabled_ = on;
    if (!on) {
        close();
    }
    {
        Transaction t(style_.open_duration, Ease::Out);
        group_->opacity(on ? 1.0f : style_.disabled_opacity);
    }
    if (on) {
        installHandler();
    } else {
        group_->onPointer({});
    }
    return *this;
}

void Dropdown::remove() {
    close();
    if (group_ != nullptr) {
        group_->onPointer({});
        group_->remove();
        group_ = plate_label_ = chevron_ = nullptr;
    }
}

}  // namespace ui
}  // namespace fluent_stage
