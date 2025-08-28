#pragma once

// Minimal sample:
//   using namespace fluent_lib::ros::dsl;
//   auto sub = FluentPubImage(node, "out", "image/out")
//             - [&](const sensor_msgs::msg::Image &in, sensor_msgs::msg::Image &out){
//                 fluent_image::Image img(in); img.text({20,40}, "OK", {0,255,0}); out = sensor_msgs::msg::Image(img);
//               }
//             .from<sensor_msgs::msg::Image>("in", "image/in");

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "fluent_lib/ros/pubsub.hpp"

namespace fluent_lib::ros::dsl {

template <typename OutMsg>
class PubBuilder {
public:
  PubBuilder(rclcpp::Node::SharedPtr node,
             std::string out_param,
             std::string out_default,
             rclcpp::QoS qos)
    : node_(std::move(node)), out_param_(std::move(out_param)), out_default_(std::move(out_default)), qos_(std::move(qos)) {}

  template <typename Fn>
  class Pipe {
  public:
    Pipe(PubBuilder b, Fn f) : b_(std::move(b)), fn_(std::move(f)) {}

    template <typename InMsg>
    auto from(const std::string &in_param, const std::string &in_default, const rclcpp::QoS &in_qos = rclcpp::SensorDataQoS()) {
      auto pub = fluent_lib::ros::pub<OutMsg>(b_.node_, b_.out_param_, b_.out_default_, b_.qos_);
      return fluent_lib::ros::sub<InMsg>(b_.node_, in_param, in_default,
        [pub, fn = fn_](typename InMsg::SharedPtr in) mutable {
          OutMsg out;
          fn(*in, out);
          pub->publish(out);
        }, in_qos);
    }

  private:
    PubBuilder b_;
    Fn fn_;
  };

  template <typename Fn>
  Pipe<Fn> operator-(Fn fn) { return Pipe<Fn>(*this, std::move(fn)); }

private:
  rclcpp::Node::SharedPtr node_;
  std::string out_param_;
  std::string out_default_;
  rclcpp::QoS qos_;
};

inline PubBuilder<sensor_msgs::msg::Image> FluentPubImage(
    const rclcpp::Node::SharedPtr &node,
    const std::string &out_param,
    const std::string &out_default,
    const rclcpp::QoS &qos = rclcpp::SensorDataQoS())
{
  return PubBuilder<sensor_msgs::msg::Image>(node, out_param, out_default, qos);
}

inline PubBuilder<sensor_msgs::msg::PointCloud2> FluentPubPoints(
    const rclcpp::Node::SharedPtr &node,
    const std::string &out_param,
    const std::string &out_default,
    const rclcpp::QoS &qos = rclcpp::SensorDataQoS())
{
  return PubBuilder<sensor_msgs::msg::PointCloud2>(node, out_param, out_default, qos);
}

} // namespace fluent_lib::ros::dsl


