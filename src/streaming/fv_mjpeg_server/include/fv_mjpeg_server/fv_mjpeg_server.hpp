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

namespace fv_mjpeg_server
{

class FVMJPEGServer : public rclcpp::Node
{
public:
    FVMJPEGServer();
    ~FVMJPEGServer();

private:
    // Parameters
    std::string stream_url_;
    std::string topic_name_;
    bool compressed_;
    double frame_rate_;
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher_;
    
    // Thread management
    std::thread stream_thread_;
    std::atomic<bool> running_;
    
    // OpenCV capture
    cv::VideoCapture cap_;
    
    // Methods
    void loadParameters();
    void initializePublishers();
    void streamWorker();
    void processFrame(const cv::Mat& frame);
    void publishImage(const cv::Mat& frame);
    void publishCompressedImage(const cv::Mat& frame);
    bool connectToStream();
    void disconnectFromStream();
    
    // Performance tracking
    std::chrono::steady_clock::time_point last_frame_time_;
    uint64_t frame_count_;
    double average_fps_;
};

} // namespace fv_mjpeg_server
