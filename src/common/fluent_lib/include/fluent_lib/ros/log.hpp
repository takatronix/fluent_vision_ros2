#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>

// Basic, readable logging helpers that keep callsites short without sacrificing clarity.

#define FL_INFO(node, ...)  RCLCPP_INFO((node)->get_logger(), __VA_ARGS__)
#define FL_WARN(node, ...)  RCLCPP_WARN((node)->get_logger(), __VA_ARGS__)
#define FL_ERROR(node, ...) RCLCPP_ERROR((node)->get_logger(), __VA_ARGS__)
#define FL_DEBUG(node, ...) RCLCPP_DEBUG((node)->get_logger(), __VA_ARGS__)

// Throttled and once-only logging
#define FL_INFO_THROTTLE(node, period_ms, ...)  RCLCPP_INFO_THROTTLE((node)->get_logger(), *(node)->get_clock(), period_ms, __VA_ARGS__)
#define FL_WARN_THROTTLE(node, period_ms, ...)  RCLCPP_WARN_THROTTLE((node)->get_logger(), *(node)->get_clock(), period_ms, __VA_ARGS__)
#define FL_ERROR_ONCE(node, ...) RCLCPP_ERROR_ONCE((node)->get_logger(), __VA_ARGS__)
#define FL_WARN_ONCE(node, ...)  RCLCPP_WARN_ONCE((node)->get_logger(), __VA_ARGS__)
#define FL_INFO_ONCE(node, ...)  RCLCPP_INFO_ONCE((node)->get_logger(), __VA_ARGS__)

// Require helpers: log and return early if condition fails
#define FL_REQUIRE(node, expr, ...) do { \
  if (!(expr)) { RCLCPP_WARN((node)->get_logger(), __VA_ARGS__); return; } \
} while(0)

#define FL_REQUIRE_RET(node, expr, ret, ...) do { \
  if (!(expr)) { RCLCPP_WARN((node)->get_logger(), __VA_ARGS__); return (ret); } \
} while(0)

namespace fluent_lib::ros::log {

// RAII scoped timer: logs elapsed time on destruction
class ScopedTimer {
public:
  ScopedTimer(const rclcpp::Logger &logger, std::string label, bool debug=false)
  : logger_(logger), label_(std::move(label)), debug_(debug), t0_(std::chrono::steady_clock::now()) {}

  ~ScopedTimer() {
    using namespace std::chrono;
    const auto t1 = steady_clock::now();
    const auto ms = duration_cast<milliseconds>(t1 - t0_).count();
    if (debug_) RCLCPP_DEBUG(logger_, "[%s] %ld ms", label_.c_str(), static_cast<long>(ms));
    else        RCLCPP_INFO(logger_,  "[%s] %ld ms", label_.c_str(), static_cast<long>(ms));
  }

private:
  rclcpp::Logger logger_;
  std::string label_;
  bool debug_;
  std::chrono::steady_clock::time_point t0_;
};

} // namespace fluent_lib::ros::log


