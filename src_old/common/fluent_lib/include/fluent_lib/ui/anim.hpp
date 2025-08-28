#pragma once

// Minimal samples:
//   auto h = fluent_ui::move_to(node->shared_from_this(), 0,0, 100,40, 300ms, [&](float x,float y){ widget.setPos(x,y); });
//   fluent_ui::Sequence::begin(node->shared_from_this()).delay(5s).fade_out(400ms, [&](float a){ widget.setAlpha(a); }).start();

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <cmath>
#include <algorithm>
#include <vector>
#include <thread>
#include <utility>

#include "fluent_lib/ros/timer.hpp"

namespace fluent_ui
{

enum class Ease { Linear, InOutQuad, InOutCubic };

inline double ease_apply(Ease e, double t)
{
    t = std::clamp(t, 0.0, 1.0);
    switch (e) {
        case Ease::Linear: return t;
        case Ease::InOutQuad: {
            return t < 0.5 ? 2*t*t : 1 - std::pow(-2*t + 2, 2) / 2;
        }
        case Ease::InOutCubic: {
            return t < 0.5 ? 4*t*t*t : 1 - std::pow(-2*t + 2, 3) / 2;
        }
        default: return t;
    }
}

// Tween a single value v in [v0 -> v1] over duration; calls set(v) each tick
inline fluent_lib::ros::TimedHandle tween_to(
    const rclcpp::Node::SharedPtr &node,
    double v0, double v1,
    std::chrono::nanoseconds duration,
    std::function<void(float)> set,
    Ease easing = Ease::InOutCubic,
    std::chrono::nanoseconds interval = std::chrono::milliseconds(16))
{
    // Fall back to thread-based timer to avoid function_traits issues on Humble
    fluent_lib::ros::TimedHandle h{};
    std::thread([node, v0, v1, duration, set=std::move(set), easing, interval]{
        auto start = std::chrono::steady_clock::now();
        auto endt = start + duration;
        while (std::chrono::steady_clock::now() < endt) {
            auto now = std::chrono::steady_clock::now();
            double u = std::chrono::duration<double>(now - start).count() /
                       std::chrono::duration<double>(duration).count();
            u = ease_apply(easing, u);
            float v = static_cast<float>(v0 + (v1 - v0) * u);
            set(v);
            std::this_thread::sleep_for(interval);
        }
        set(static_cast<float>(v1));
    }).detach();
    return h;
}

// Fade alpha helpers (0..1)
inline fluent_lib::ros::TimedHandle fade_to(
    const rclcpp::Node::SharedPtr &node,
    float a0, float a1,
    std::chrono::nanoseconds duration,
    std::function<void(float)> setAlpha,
    Ease easing = Ease::InOutQuad,
    std::chrono::nanoseconds interval = std::chrono::milliseconds(16))
{
    return tween_to(node, a0, a1, duration, std::move(setAlpha), easing, interval);
}

inline fluent_lib::ros::TimedHandle fade_in(
    const rclcpp::Node::SharedPtr &node,
    std::chrono::nanoseconds duration,
    std::function<void(float)> setAlpha,
    Ease easing = Ease::InOutQuad)
{
    return fade_to(node, 0.0f, 1.0f, duration, std::move(setAlpha), easing);
}

inline fluent_lib::ros::TimedHandle fade_out(
    const rclcpp::Node::SharedPtr &node,
    std::chrono::nanoseconds duration,
    std::function<void(float)> setAlpha,
    Ease easing = Ease::InOutQuad)
{
    return fade_to(node, 1.0f, 0.0f, duration, std::move(setAlpha), easing);
}

// Move (x,y) from (x0,y0) to (x1,y1) over duration. Calls setPos(x,y) each tick.
// Returns TimedHandle so caller can keep it alive; stops automatically at end.
inline fluent_lib::ros::TimedHandle move_to(
    const rclcpp::Node::SharedPtr &node,
    float x0, float y0,
    float x1, float y1,
    std::chrono::nanoseconds duration,
    std::function<void(float, float)> setPos,
    Ease easing = Ease::InOutCubic,
    std::chrono::nanoseconds interval = std::chrono::milliseconds(16))
{
    fluent_lib::ros::TimedHandle h{};
    std::thread([x0, y0, x1, y1, duration, setPos=std::move(setPos), easing, interval]{
        auto start = std::chrono::steady_clock::now();
        auto endt = start + duration;
        while (std::chrono::steady_clock::now() < endt) {
            auto now = std::chrono::steady_clock::now();
            double u = std::chrono::duration<double>(now - start).count() /
                       std::chrono::duration<double>(duration).count();
            u = ease_apply(easing, u);
            float x = static_cast<float>(x0 + (x1 - x0) * u);
            float y = static_cast<float>(y0 + (y1 - y0) * u);
            setPos(x, y);
            std::this_thread::sleep_for(interval);
        }
        setPos(x1, y1);
    }).detach();
    return h;
}

} // namespace fluent_ui

// ===== Sequencing (fluent chaining) =====
namespace fluent_ui {

inline rclcpp::TimerBase::SharedPtr after(
    const rclcpp::Node::SharedPtr &node,
    std::chrono::nanoseconds delay,
    std::function<void()> fn)
{
    return fluent_lib::ros::make_timer(node, delay, std::move(fn), true);
}

class Sequence {
public:
    explicit Sequence(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

    static Sequence begin(const rclcpp::Node::SharedPtr &node) { return Sequence(node); }

    Sequence &delay(std::chrono::nanoseconds d) { t_ += d; return *this; }

    Sequence &call(std::function<void()> fn) {
        steps_.push_back({t_, [n=node_, fn=std::move(fn)](){ after(n, std::chrono::nanoseconds(0), fn); }});
        return *this;
    }

    Sequence &fade_out(std::chrono::nanoseconds duration, std::function<void(float)> setAlpha,
                       Ease easing = Ease::InOutQuad) {
        steps_.push_back({t_, [=]{ ::fluent_ui::fade_out(node_, duration, setAlpha, easing); }});
        t_ += duration; return *this;
    }

    Sequence &fade_in(std::chrono::nanoseconds duration, std::function<void(float)> setAlpha,
                      Ease easing = Ease::InOutQuad) {
        steps_.push_back({t_, [=]{ ::fluent_ui::fade_in(node_, duration, setAlpha, easing); }});
        t_ += duration; return *this;
    }

    Sequence &move_to(float x0, float y0, float x1, float y1,
                      std::chrono::nanoseconds duration,
                      std::function<void(float,float)> setPos,
                      Ease easing = Ease::InOutCubic) {
        steps_.push_back({t_, [=]{ ::fluent_ui::move_to(node_, x0,y0,x1,y1, duration, setPos, easing); }});
        t_ += duration; return *this;
    }

    void start() {
        for (auto &s : steps_) {
            fluent_lib::ros::make_timer(node_, s.offset, s.fn, true);
        }
    }

private:
    struct Step { std::chrono::nanoseconds offset; std::function<void()> fn; };
    rclcpp::Node::SharedPtr node_;
    std::chrono::nanoseconds t_{0};
    std::vector<Step> steps_;
};

} // namespace fluent_ui


