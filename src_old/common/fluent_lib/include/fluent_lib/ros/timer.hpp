#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>

namespace fluent_lib::ros
{

// Create a one-shot or periodic timer with a lambda callback.
// Example:
//   auto t = fluent_lib::ros::make_timer(node, 5s, []{ /* run once after 5s */ }, true);
//   auto p = fluent_lib::ros::make_timer(node, 33ms, []{ /* 30FPS */ });
inline rclcpp::TimerBase::SharedPtr make_timer(
    const rclcpp::Node::SharedPtr &node,
    std::chrono::nanoseconds period,
    std::function<void()> callback,
    bool oneshot = false)
{
    // Normal periodic timer
    if (!oneshot) {
        std::function<void()> fn = [cb = std::move(callback)]() mutable { cb(); };
        auto t = node->create_wall_timer(period, fn);
        return t;
    }
    // One-shot: cancel after first run (safe capture)
    auto weak_holder = std::make_shared<std::weak_ptr<rclcpp::TimerBase>>();
    std::function<void()> fn = [cb = std::move(callback), weak_holder]() mutable {
        cb();
        if (auto tt = weak_holder->lock()) tt->cancel();
    };
    auto t = node->create_wall_timer(period, fn);
    *weak_holder = t;
    return t;
}

// Overload: accept raw Node* for use inside constructors (avoids shared_from_this)
inline rclcpp::TimerBase::SharedPtr make_timer(
    rclcpp::Node* node,
    std::chrono::nanoseconds period,
    std::function<void()> callback,
    bool oneshot = false)
{
    // Normal periodic timer
    if (!oneshot) {
        std::function<void()> fn = [cb = std::move(callback)]() mutable { cb(); };
        auto t = node->create_wall_timer(period, fn);
        return t;
    }
    // One-shot: cancel after first run (safe capture)
    auto weak_holder = std::make_shared<std::weak_ptr<rclcpp::TimerBase>>();
    std::function<void()> fn = [cb = std::move(callback), weak_holder]() mutable {
        cb();
        if (auto tt = weak_holder->lock()) tt->cancel();
    };
    auto t = node->create_wall_timer(period, fn);
    *weak_holder = t;
    return t;
}

// Run a lambda repeatedly for a fixed duration; cancels automatically after duration.
// Example: run for 5 seconds at 30ms interval
//   auto h = fluent_lib::ros::run_for(node, 30ms, 5s, []{ ... });
struct TimedHandle {
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr stopper;
};

inline TimedHandle run_for(
    const rclcpp::Node::SharedPtr &node,
    std::chrono::nanoseconds interval,
    std::chrono::nanoseconds duration,
    std::function<void()> callback)
{
    TimedHandle h;
    {
        std::function<void()> tick = [cb = std::move(callback)]() mutable { cb(); };
        h.timer = node->create_wall_timer(interval, tick);
    }
    auto weak_holder = std::make_shared<std::weak_ptr<rclcpp::TimerBase>>();
    *weak_holder = h.timer;
    {
        std::function<void()> stopfn = [weak_holder]() mutable {
            if (auto t = weak_holder->lock()) t->cancel();
        };
        h.stopper = node->create_wall_timer(duration, stopfn);
    }
    return h;
}

// Overload: run_for with raw Node* (constructor-safe)
inline TimedHandle run_for(
    rclcpp::Node* node,
    std::chrono::nanoseconds interval,
    std::chrono::nanoseconds duration,
    std::function<void()> callback)
{
    TimedHandle h;
    {
        std::function<void()> tick = [cb = std::move(callback)]() mutable { cb(); };
        h.timer = node->create_wall_timer(interval, tick);
    }
    auto weak_holder = std::make_shared<std::weak_ptr<rclcpp::TimerBase>>();
    *weak_holder = h.timer;
    {
        std::function<void()> stopfn = [weak_holder]() mutable {
            if (auto t = weak_holder->lock()) t->cancel();
        };
        h.stopper = node->create_wall_timer(duration, stopfn);
    }
    return h;
}

} // namespace fluent_lib::ros


