#pragma once

#include <rclcpp/rclcpp.hpp>

namespace fluent_lib::ros::qos {

inline rclcpp::QoS sensor() { return rclcpp::SensorDataQoS(); }

inline rclcpp::QoS best_effort(size_t depth = 10) {
    rclcpp::QoS q(depth);
    q.best_effort();
    q.durability_volatile();
    return q;
}

inline rclcpp::QoS reliable(size_t depth = 10) {
    rclcpp::QoS q(depth);
    q.reliable();
    return q;
}

inline rclcpp::QoS latched(size_t depth = 1) {
    rclcpp::QoS q(depth);
    q.reliable();
    q.transient_local();
    return q;
}

} // namespace fluent_lib::ros::qos


