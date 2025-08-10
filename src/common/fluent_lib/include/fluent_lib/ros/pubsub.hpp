#pragma once

// Minimal samples:
//  - Image in→annotate→out
//    auto pub = fluent_lib::ros::pub_image(node, "out", "image/out");
//    auto sub = fluent_lib::ros::sub_fimage_bgr(node, "in", "image/in", [&](fluent_image::Image img){
//      img.text({20,40}, "OK", {0,255,0}); pub->publish(sensor_msgs::msg::Image(img));
//    });
//  - PointCloud2 in→filter→metrics
//    auto subp = fluent_lib::ros::sub_cloud(node, "points", "/fv/points", [&](auto cloud, auto hdr){
//      auto den = fluent_cloud::filters::VoxelGrid<pcl::PointXYZRGB>().setLeafSize(0.005).filter(cloud);
//      auto m = fluent_cloud::compute_pca_metrics(den);
//    });

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <functional>
#include "fluent_lib/ros/params.hpp"
// Common message shortcuts
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "fluent_lib/fluent_image/image.hpp"
#include "fluent_lib/fluent_cloud/pc2.hpp"

namespace fluent_lib::ros {

// Publisher one-liner: topic name is a parameter with default
template <typename MsgT>
inline typename rclcpp::Publisher<MsgT>::SharedPtr pub(
    const rclcpp::Node::SharedPtr &node,
    const std::string &param_name,
    const std::string &default_topic,
    const rclcpp::QoS &qos = rclcpp::QoS(rclcpp::KeepLast(10)))
{
    auto topic_name = topic(node, param_name, default_topic);
    return node->create_publisher<MsgT>(topic_name, qos);
}

// Subscriber one-liner: topic name is a parameter with default
template <typename MsgT, typename Fn>
inline typename rclcpp::Subscription<MsgT>::SharedPtr sub(
    const rclcpp::Node::SharedPtr &node,
    const std::string &param_name,
    const std::string &default_topic,
    Fn &&callback,
    const rclcpp::QoS &qos = rclcpp::QoS(rclcpp::KeepLast(10)))
{
    auto topic_name = topic(node, param_name, default_topic);
    std::function<void(typename MsgT::SharedPtr)> cb = std::forward<Fn>(callback);
    return node->create_subscription<MsgT>(topic_name, qos, cb);
}

// Relay helper: subscribe, transform, publish
template <typename InMsg, typename OutMsg, typename Fn>
struct Relay {
    typename rclcpp::Publisher<OutMsg>::SharedPtr pub;
    typename rclcpp::Subscription<InMsg>::SharedPtr sub;
    Relay(const rclcpp::Node::SharedPtr &node,
          const std::string &in_param, const std::string &in_default,
          const std::string &out_param, const std::string &out_default,
          Fn &&fn,
          const rclcpp::QoS &qos_in = rclcpp::QoS(rclcpp::KeepLast(10)),
          const rclcpp::QoS &qos_out = rclcpp::QoS(rclcpp::KeepLast(10)))
    {
        pub = fluent_lib::ros::pub<OutMsg>(node, out_param, out_default, qos_out);
        sub = fluent_lib::ros::sub<InMsg>(node, in_param, in_default,
            [this, fn = std::forward<Fn>(fn)](typename InMsg::SharedPtr msg_in) mutable {
                OutMsg out;
                fn(*msg_in, out);
                pub->publish(out);
            }, qos_in);
    }
};

} // namespace fluent_lib::ros

// ===== Convenience overloads for common messages (readable, no templates at callsite) =====
namespace fluent_lib::ros {

inline rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image(
    const rclcpp::Node::SharedPtr &node,
    const std::string &param_name,
    const std::string &default_topic,
    const rclcpp::QoS &qos = rclcpp::SensorDataQoS())
{
    return pub<sensor_msgs::msg::Image>(node, param_name, default_topic, qos);
}

inline rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_image_compressed(
    const rclcpp::Node::SharedPtr &node,
    const std::string &param_name,
    const std::string &default_topic,
    const rclcpp::QoS &qos = rclcpp::SensorDataQoS())
{
    return pub<sensor_msgs::msg::CompressedImage>(node, param_name, default_topic, qos);
}

template <typename Fn>
inline rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image(
    const rclcpp::Node::SharedPtr &node,
    const std::string &param_name,
    const std::string &default_topic,
    Fn &&callback,
    const rclcpp::QoS &qos = rclcpp::SensorDataQoS())
{
    return sub<sensor_msgs::msg::Image>(node, param_name, default_topic, std::forward<Fn>(callback), qos);
}
// Quick publish helper from fluent_image::Image with header
template <typename PubT>
inline void publish(PubT &pub, const fluent_image::Image &img, const std_msgs::msg::Header &hdr) {
    auto msg = fluent_image::to_msg(img, hdr);
    pub->publish(msg);
}

// Compressed publish helper (jpeg/png)
inline void publish_compressed(rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr &pub,
                               const fluent_image::Image &img,
                               const std_msgs::msg::Header &hdr,
                               int quality = 85,
                               const std::string &format = "jpeg")
{
    auto c = img.to_compressed(hdr, quality, format);
    pub->publish(c);
}


inline rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points(
    const rclcpp::Node::SharedPtr &node,
    const std::string &param_name,
    const std::string &default_topic,
    const rclcpp::QoS &qos = rclcpp::SensorDataQoS())
{
    return pub<sensor_msgs::msg::PointCloud2>(node, param_name, default_topic, qos);
}

template <typename Fn>
inline rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points(
    const rclcpp::Node::SharedPtr &node,
    const std::string &param_name,
    const std::string &default_topic,
    Fn &&callback,
    const rclcpp::QoS &qos = rclcpp::SensorDataQoS())
{
    return sub<sensor_msgs::msg::PointCloud2>(node, param_name, default_topic, std::forward<Fn>(callback), qos);
}

// Subscribe and receive fluent_image::Image directly
template <typename Fn>
inline rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_fimage(
    const rclcpp::Node::SharedPtr &node,
    const std::string &param_name,
    const std::string &default_topic,
    Fn &&callback,
    const rclcpp::QoS &qos = rclcpp::SensorDataQoS())
{
    std::function<void(sensor_msgs::msg::Image::SharedPtr)> cbwrap = [cb = std::forward<Fn>(callback)](sensor_msgs::msg::Image::SharedPtr msg) mutable {
        fluent_image::Image img(*msg);
        cb(img);
    };
    return sub<sensor_msgs::msg::Image>(node, param_name, default_topic, cbwrap, qos);
}

// Subscribe and receive fluent_image::Image normalized to BGR8
template <typename Fn>
inline rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_fimage_bgr(
    const rclcpp::Node::SharedPtr &node,
    const std::string &param_name,
    const std::string &default_topic,
    Fn &&callback,
    const rclcpp::QoS &qos = rclcpp::SensorDataQoS())
{
    std::function<void(sensor_msgs::msg::Image::SharedPtr)> cbwrap = [cb = std::forward<Fn>(callback)](sensor_msgs::msg::Image::SharedPtr msg) mutable {
        fluent_image::Image img(*msg);
        img = img.to_bgr8();
        cb(img);
    };
    return sub<sensor_msgs::msg::Image>(node, param_name, default_topic, cbwrap, qos);
}

// Subscribe and receive PCL cloud directly
template <typename Fn>
inline rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud(
    const rclcpp::Node::SharedPtr &node,
    const std::string &param_name,
    const std::string &default_topic,
    Fn &&callback,
    const rclcpp::QoS &qos = rclcpp::SensorDataQoS())
{
    std::function<void(sensor_msgs::msg::PointCloud2::SharedPtr)> cbwrap = [cb = std::forward<Fn>(callback)](sensor_msgs::msg::PointCloud2::SharedPtr msg) mutable {
        auto cloud = fluent_cloud::pc2::from_msg(*msg);
        cb(cloud, msg->header);
    };
    return sub<sensor_msgs::msg::PointCloud2>(node, param_name, default_topic, cbwrap, qos);
}

} // namespace fluent_lib::ros


