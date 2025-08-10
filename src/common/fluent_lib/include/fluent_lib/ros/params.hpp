#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace fluent_lib::ros
{

// Declare a parameter with default value and return it (one-liner)
template <typename T>
inline T param(const rclcpp::Node::SharedPtr &node, const std::string &name, const T &default_value)
{
    try {
        return node->declare_parameter<T>(name, default_value);
    } catch (...) {
        // Fallback if already declared elsewhere
        try { return node->get_parameter(name).get_value<T>(); } catch (...) { return default_value; }
    }
}

// Convenience alias for topic parameters (string)
inline std::string topic(const rclcpp::Node::SharedPtr &node, const std::string &name, const std::string &default_topic)
{
    return param<std::string>(node, name, default_topic);
}

} // namespace fluent_lib::ros


