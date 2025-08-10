#pragma once

// Minimal sample:
//   class MyNode : public fluent_lib::ros::FluentNode {
//   public:
//     MyNode(): FluentNode("my_node") {
//       auto pub = this->pub_image("out", "image/out");
//       auto sub = this->sub_image("in", "image/in", [&](auto msg){ pub->publish(*msg); });
//     }
//   };

#include <rclcpp/rclcpp.hpp>
#include "fluent_lib/ros/params.hpp"
#include "fluent_lib/ros/pubsub.hpp"
#include "fluent_lib/ros/timer.hpp"
#include "fluent_lib/ros/log.hpp"

namespace fluent_lib::ros {

class FluentNode : public rclcpp::Node {
public:
  explicit FluentNode(const std::string &name) : rclcpp::Node(name) {}

  template <typename T>
  T param(const std::string &name, const T &def) { return fluent_lib::ros::param<T>(shared_from_this(), name, def); }

  template <typename MsgT>
  auto pub(const std::string &param_name, const std::string &def_topic, const rclcpp::QoS &q=rclcpp::QoS(10)) {
    return fluent_lib::ros::pub<MsgT>(shared_from_this(), param_name, def_topic, q);
  }

  template <typename MsgT, typename Fn>
  auto sub(const std::string &param_name, const std::string &def_topic, Fn &&fn, const rclcpp::QoS &q=rclcpp::QoS(10)) {
    return fluent_lib::ros::sub<MsgT>(shared_from_this(), param_name, def_topic, std::forward<Fn>(fn), q);
  }
};

} // namespace fluent_lib::ros


