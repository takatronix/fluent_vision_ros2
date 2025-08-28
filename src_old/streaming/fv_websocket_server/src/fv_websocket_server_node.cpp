#include "fv_websocket_server/fv_websocket_server.hpp"

#include <chrono>
#include <thread>
#include <iostream>
#include <sstream>
#include <vector>

namespace fv_websocket_server
{

FVWebSocketServer::FVWebSocketServer()
    : Node("fv_websocket_server"),
      running_(false),
      has_image_(false),
      frame_count_(0),
      average_fps_(0.0)
{
    RCLCPP_INFO(this->get_logger(), "Initializing FV WebSocket Server C++");
    
    loadParameters();
    initializeSubscriptions();
    startWebSocketServer();
    
    RCLCPP_INFO(this->get_logger(), "FV WebSocket Server C++ initialized successfully");
}

FVWebSocketServer::~FVWebSocketServer()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down FV WebSocket Server C++");
    
    running_ = false;
    stopWebSocketServer();
    
    if (websocket_thread_.joinable()) {
        websocket_thread_.join();
    }
}

void FVWebSocketServer::loadParameters()
{
    // Declare parameters with default values
    this->declare_parameter("topic_name", "/fv_mjpeg_image");
    this->declare_parameter("websocket_port", 8765);
    this->declare_parameter("compressed", false);
    this->declare_parameter("frame_rate", 30.0);
    
    // Get parameters
    topic_name_ = this->get_parameter("topic_name").as_string();
    websocket_port_ = this->get_parameter("websocket_port").as_int();
    compressed_ = this->get_parameter("compressed").as_bool();
    frame_rate_ = this->get_parameter("frame_rate").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
    RCLCPP_INFO(this->get_logger(), "  Topic: %s", topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  WebSocket Port: %d", websocket_port_);
    RCLCPP_INFO(this->get_logger(), "  Compressed: %s", compressed_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Frame Rate: %.1f", frame_rate_);
}

void FVWebSocketServer::initializeSubscriptions()
{
    if (compressed_) {
        compressed_subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            topic_name_, 10, std::bind(&FVWebSocketServer::compressedImageCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Compressed image subscription initialized");
    } else {
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_name_, 10, std::bind(&FVWebSocketServer::imageCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Raw image subscription initialized");
    }
}

void FVWebSocketServer::startWebSocketServer()
{
    // Initialize WebSocket server
    server_.init_asio();
    
    // Set up callbacks
    server_.set_open_handler(std::bind(&FVWebSocketServer::onOpen, this, std::placeholders::_1));
    server_.set_close_handler(std::bind(&FVWebSocketServer::onClose, this, std::placeholders::_1));
    server_.set_message_handler(std::bind(&FVWebSocketServer::onMessage, this, std::placeholders::_1, std::placeholders::_2));
    
    // Start WebSocket thread
    running_ = true;
    websocket_thread_ = std::thread(&FVWebSocketServer::websocketWorker, this);
    
    RCLCPP_INFO(this->get_logger(), "WebSocket server started on port %d", websocket_port_);
}

void FVWebSocketServer::stopWebSocketServer()
{
    if (server_.is_listening()) {
        server_.stop_listening();
        server_.stop();
    }
}

void FVWebSocketServer::websocketWorker()
{
    try {
        server_.listen(websocket_port_);
        server_.start_accept();
        
        while (running_) {
            server_.run_one();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "WebSocket server error: %s", e.what());
    }
}

void FVWebSocketServer::onOpen(connection_hdl hdl)
{
    std::lock_guard<std::mutex> lock(connections_mutex_);
    connections_.insert(hdl);
    RCLCPP_INFO(this->get_logger(), "WebSocket client connected. Total clients: %zu", connections_.size());
}

void FVWebSocketServer::onClose(connection_hdl hdl)
{
    std::lock_guard<std::mutex> lock(connections_mutex_);
    connections_.erase(hdl);
    RCLCPP_INFO(this->get_logger(), "WebSocket client disconnected. Total clients: %zu", connections_.size());
}

void FVWebSocketServer::onMessage(connection_hdl hdl, message_ptr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->get_payload().c_str());
    
    // Echo back for now
    server_.send(hdl, msg->get_payload(), websocketpp::frame::opcode::text);
}

void FVWebSocketServer::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        auto cv_image = cv_bridge::toCvShare(msg, "bgr8");
        if (cv_image) {
            std::lock_guard<std::mutex> lock(image_mutex_);
            current_image_ = cv_image->image.clone();
            has_image_ = true;
            
            // Broadcast to WebSocket clients
            broadcastImage();
            
            // Calculate FPS
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_time_).count();
            if (duration > 0) {
                average_fps_ = 1000.0 / duration;
            }
            last_frame_time_ = now;
            
            frame_count_++;
            
            // Log every 30 frames
            if (frame_count_ % 30 == 0) {
                RCLCPP_INFO(this->get_logger(), "Processed frame %lu, FPS: %.1f, Clients: %zu", 
                           frame_count_, average_fps_, connections_.size());
            }
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void FVWebSocketServer::compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    try {
        // Decode compressed image
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (!image.empty()) {
            std::lock_guard<std::mutex> lock(image_mutex_);
            current_image_ = image.clone();
            has_image_ = true;
            
            // Broadcast to WebSocket clients
            broadcastImage();
            
            // Calculate FPS
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_time_).count();
            if (duration > 0) {
                average_fps_ = 1000.0 / duration;
            }
            last_frame_time_ = now;
            
            frame_count_++;
            
            // Log every 30 frames
            if (frame_count_ % 30 == 0) {
                RCLCPP_INFO(this->get_logger(), "Processed compressed frame %lu, FPS: %.1f, Clients: %zu", 
                           frame_count_, average_fps_, connections_.size());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Compressed image processing error: %s", e.what());
    }
}

void FVWebSocketServer::broadcastImage()
{
    if (!has_image_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(connections_mutex_);
    if (connections_.empty()) {
        return;
    }
    
    try {
        // Encode image to base64
        std::vector<uchar> buffer;
        cv::imencode(".jpg", current_image_, buffer);
        
        std::string base64_image;
        // Simple base64 encoding (in production, use a proper library)
        const std::string chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        for (size_t i = 0; i < buffer.size(); i += 3) {
            uint32_t n = buffer[i] << 16;
            if (i + 1 < buffer.size()) n |= buffer[i + 1] << 8;
            if (i + 2 < buffer.size()) n |= buffer[i + 2];
            
            base64_image += chars[(n >> 18) & 63];
            base64_image += chars[(n >> 12) & 63];
            base64_image += chars[(n >> 6) & 63];
            base64_image += chars[n & 63];
        }
        
        // Create JSON message
        std::stringstream json;
        json << "{\"type\":\"image\",\"data\":\"data:image/jpeg;base64," << base64_image << "\"}";
        
        // Broadcast to all clients
        for (auto& hdl : connections_) {
            try {
                server_.send(hdl, json.str(), websocketpp::frame::opcode::text);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to send to client: %s", e.what());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Image broadcasting error: %s", e.what());
    }
}

void FVWebSocketServer::broadcastToAll(const std::string& message)
{
    std::lock_guard<std::mutex> lock(connections_mutex_);
    for (auto& hdl : connections_) {
        try {
            server_.send(hdl, message, websocketpp::frame::opcode::text);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to send message to client: %s", e.what());
        }
    }
}

} // namespace fv_websocket_server

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<fv_websocket_server::FVWebSocketServer>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
