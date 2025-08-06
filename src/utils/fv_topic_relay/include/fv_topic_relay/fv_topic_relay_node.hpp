#ifndef FV_TOPIC_RELAY_NODE_HPP
#define FV_TOPIC_RELAY_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <memory>
#include <string>
#include <vector>

namespace fv_topic_relay
{

class FVTopicRelayNode : public rclcpp::Node
{
public:
    explicit FVTopicRelayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~FVTopicRelayNode() = default;

private:
    struct RelayConfig {
        std::string from_topic;
        std::string to_topic;
    };

    void loadParameters();
    void setupRelays();
    
    std::vector<RelayConfig> relay_configs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscriptions_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;
};

} // namespace fv_topic_relay

#endif // FV_TOPIC_RELAY_NODE_HPP