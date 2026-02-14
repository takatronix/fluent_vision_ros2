#include "fv_topic_relay/fv_topic_relay_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace fv_topic_relay
{

FVTopicRelayNode::FVTopicRelayNode(const rclcpp::NodeOptions & options)
    : Node("fv_topic_relay", options)
{
    RCLCPP_INFO(this->get_logger(), "🚀 FV Topic Relay Node starting...");
    
    loadParameters();
    setupRelays();
    
    RCLCPP_INFO(this->get_logger(), "✅ FV Topic Relay Node started with %zu relays", relay_configs_.size());
}

void FVTopicRelayNode::loadParameters()
{
    // トピックリレー設定を読み込む
    relay_configs_.clear();
    
    // パラメータから設定を読み込む
    this->declare_parameter("relay_mappings", std::vector<std::string>{});
    auto relay_mappings = this->get_parameter("relay_mappings").as_string_array();
    
    // 文字列配列からRelayConfig構造体に変換
    // フォーマット: "from_topic:to_topic"
    for (const auto& mapping : relay_mappings) {
        size_t delimiter_pos = mapping.find(':');
        if (delimiter_pos != std::string::npos) {
            std::string from_topic = mapping.substr(0, delimiter_pos);
            std::string to_topic = mapping.substr(delimiter_pos + 1);
            relay_configs_.push_back({from_topic, to_topic});
            RCLCPP_INFO(this->get_logger(), "📌 Loaded mapping: %s -> %s", 
                from_topic.c_str(), to_topic.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ Invalid mapping format: %s (expected 'from:to')", 
                mapping.c_str());
        }
    }
    
    // デフォルト設定（パラメータが空の場合）
    if (relay_configs_.empty()) {
        RCLCPP_WARN(this->get_logger(), "⚠️ No relay mappings found in parameters, using defaults");
        // No hardcoded defaults — configure via relay_mappings parameter
        RCLCPP_INFO(this->get_logger(), "📝 Configure relay_mappings parameter to set up relays");
    }
    
    RCLCPP_INFO(this->get_logger(), "📋 Loaded %zu relay configurations", relay_configs_.size());
}

void FVTopicRelayNode::setupRelays()
{
    subscriptions_.clear();
    publishers_.clear();
    
    // 画像用の軽量QoS（ベストエフォート・低遅延）
    auto img_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    img_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    img_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    for (const auto& config : relay_configs_) {
        // パブリッシャーを作成（best_effort）
        auto pub = this->create_publisher<sensor_msgs::msg::Image>(config.to_topic, img_qos);
        publishers_.push_back(pub);
        
        // サブスクライバーを作成（best_effort、ラムダでパブリッシャーをキャプチャ）
        auto sub = this->create_subscription<sensor_msgs::msg::Image>(
            config.from_topic, img_qos,
            [pub, this, config](const sensor_msgs::msg::Image::SharedPtr msg) {
                // そのままリレー
                pub->publish(*msg);
                
                // デバッグ出力（毎秒1回程度）
                static std::chrono::steady_clock::time_point last_log = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log).count() >= 1) {
                    RCLCPP_DEBUG(this->get_logger(), "📡 Relayed: %s -> %s", 
                        config.from_topic.c_str(), config.to_topic.c_str());
                    last_log = now;
                }
            });
        
        subscriptions_.push_back(sub);
        
        RCLCPP_INFO(this->get_logger(), "🔗 Relay setup: %s -> %s", 
            config.from_topic.c_str(), config.to_topic.c_str());
    }
}

} // namespace fv_topic_relay

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(fv_topic_relay::FVTopicRelayNode)

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<fv_topic_relay::FVTopicRelayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
