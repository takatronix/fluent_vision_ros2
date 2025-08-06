#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <set>
#include <mutex>

// WebSocket includes
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

namespace fv_websocket_server
{

class FVWebSocketServer : public rclcpp::Node
{
public:
    FVWebSocketServer();
    ~FVWebSocketServer();

private:
    // WebSocket types
    typedef websocketpp::server<websocketpp::config::asio> WebSocketServer;
    typedef WebSocketServer::message_ptr message_ptr;
    typedef websocketpp::connection_hdl connection_hdl;

    // Parameters
    std::string topic_name_;
    int websocket_port_;
    bool compressed_;
    double frame_rate_;
    
    // Publishers/Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_subscription_;
    
    // WebSocket server
    WebSocketServer server_;
    std::set<connection_hdl, std::owner_less<connection_hdl>> connections_;
    std::mutex connections_mutex_;
    
    // Thread management
    std::thread websocket_thread_;
    std::atomic<bool> running_;
    
    // Current image
    cv::Mat current_image_;
    std::mutex image_mutex_;
    std::atomic<bool> has_image_;
    
    // Methods
    void loadParameters();
    void initializeSubscriptions();
    void startWebSocketServer();
    void stopWebSocketServer();
    void websocketWorker();
    
    // WebSocket callbacks
    void onOpen(connection_hdl hdl);
    void onClose(connection_hdl hdl);
    void onMessage(connection_hdl hdl, message_ptr msg);
    
    // Image callbacks
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    
    // Broadcasting
    void broadcastImage();
    void broadcastToAll(const std::string& message);
    
    // Performance tracking
    std::chrono::steady_clock::time_point last_frame_time_;
    uint64_t frame_count_;
    double average_fps_;
};

} // namespace fv_websocket_server
