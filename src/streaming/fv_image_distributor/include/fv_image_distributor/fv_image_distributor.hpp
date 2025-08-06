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
#include <mutex>
#include <vector>

// HTTP server includes
#include <boost/asio.hpp>
#include <boost/beast.hpp>

namespace fv_image_distributor
{

class FVImageDistributor : public rclcpp::Node
{
public:
    FVImageDistributor();
    ~FVImageDistributor();

private:
    // Parameters
    std::string input_topic_;
    int http_port_;
    bool compressed_;
    double frame_rate_;
    
    // Publishers/Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_subscription_;
    
    // HTTP server
    boost::asio::io_context io_context_;
    std::unique_ptr<boost::asio::ip::tcp::acceptor> acceptor_;
    std::thread http_thread_;
    std::atomic<bool> running_;
    
    // Current image
    cv::Mat current_image_;
    std::mutex image_mutex_;
    std::atomic<bool> has_image_;
    std::chrono::steady_clock::time_point last_image_time_;
    
    // Methods
    void loadParameters();
    void initializeSubscriptions();
    void startHttpServer();
    void stopHttpServer();
    void httpWorker();
    void handleHttpRequest(boost::asio::ip::tcp::socket socket);
    void sendHttpResponse(boost::asio::ip::tcp::socket& socket, 
                         const std::string& content_type, 
                         const std::string& body);
    
    // Image callbacks
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    
    // Image processing
    std::string encodeImageToBase64(const cv::Mat& image);
    std::string createHtmlPage();
    
    // Performance tracking
    uint64_t frame_count_;
    double average_fps_;
};

} // namespace fv_image_distributor
