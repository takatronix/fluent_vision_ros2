#include "fv_mjpeg_server/fv_mjpeg_server.hpp"

#include <chrono>
#include <thread>
#include <iostream>

namespace fv_mjpeg_server
{

FVMJPEGServer::FVMJPEGServer()
    : Node("fv_mjpeg_server"),
      running_(false),
      frame_count_(0),
      average_fps_(0.0)
{
    RCLCPP_INFO(this->get_logger(), "Initializing FV MJPEG Server C++");
    
    loadParameters();
    initializePublishers();
    
    // Start streaming thread
    running_ = true;
    stream_thread_ = std::thread(&FVMJPEGServer::streamWorker, this);
    
    RCLCPP_INFO(this->get_logger(), "FV MJPEG Server C++ initialized successfully");
}

FVMJPEGServer::~FVMJPEGServer()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down FV MJPEG Server C++");
    
    running_ = false;
    if (stream_thread_.joinable()) {
        stream_thread_.join();
    }
    
    disconnectFromStream();
}

void FVMJPEGServer::loadParameters()
{
    // Declare parameters with default values
    this->declare_parameter("stream_url", "http://unitcams3.local/stream");
    this->declare_parameter("topic_name", "/fv_mjpeg_image");
    this->declare_parameter("compressed", false);
    this->declare_parameter("frame_rate", 30.0);
    
    // Get parameters
    stream_url_ = this->get_parameter("stream_url").as_string();
    topic_name_ = this->get_parameter("topic_name").as_string();
    compressed_ = this->get_parameter("compressed").as_bool();
    frame_rate_ = this->get_parameter("frame_rate").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
    RCLCPP_INFO(this->get_logger(), "  Stream URL: %s", stream_url_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Topic: %s", topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Compressed: %s", compressed_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Frame Rate: %.1f", frame_rate_);
}

void FVMJPEGServer::initializePublishers()
{
    if (compressed_) {
        compressed_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            topic_name_, 10);
        RCLCPP_INFO(this->get_logger(), "Compressed image publisher initialized");
    } else {
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            topic_name_, 10);
        RCLCPP_INFO(this->get_logger(), "Raw image publisher initialized");
    }
}

void FVMJPEGServer::streamWorker()
{
    RCLCPP_INFO(this->get_logger(), "Starting stream worker thread");
    
    while (running_) {
        if (!connectToStream()) {
            RCLCPP_WARN(this->get_logger(), "Failed to connect to stream, retrying in 1 second");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        
        RCLCPP_INFO(this->get_logger(), "Connected to MJPEG stream: %s", stream_url_.c_str());
        
        // Frame processing loop
        cv::Mat frame;
        while (running_ && cap_.isOpened()) {
            if (cap_.read(frame)) {
                processFrame(frame);
                
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
                    RCLCPP_INFO(this->get_logger(), "Processed frame %lu, FPS: %.1f", frame_count_, average_fps_);
                }
                
                // Frame rate control
                if (frame_rate_ > 0) {
                    int delay_ms = static_cast<int>(1000.0 / frame_rate_);
                    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to read frame from stream");
                break;
            }
        }
        
        disconnectFromStream();
        RCLCPP_INFO(this->get_logger(), "Stream disconnected, attempting to reconnect");
    }
    
    RCLCPP_INFO(this->get_logger(), "Stream worker thread finished");
}

void FVMJPEGServer::processFrame(const cv::Mat& frame)
{
    if (compressed_) {
        publishCompressedImage(frame);
    } else {
        publishImage(frame);
    }
}

void FVMJPEGServer::publishImage(const cv::Mat& frame)
{
    try {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_link";
        
        image_publisher_->publish(*msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void FVMJPEGServer::publishCompressedImage(const cv::Mat& frame)
{
    try {
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_link";
        msg->format = "jpeg";
        
        // Encode to JPEG
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(85);
        
        cv::imencode(".jpg", frame, msg->data, compression_params);
        
        compressed_publisher_->publish(*msg);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Compression error: %s", e.what());
    }
}

bool FVMJPEGServer::connectToStream()
{
    try {
        cap_.open(stream_url_);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open stream: %s", stream_url_.c_str());
            return false;
        }
        
        // Set buffer size for low latency
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
        
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Connection error: %s", e.what());
        return false;
    }
}

void FVMJPEGServer::disconnectFromStream()
{
    if (cap_.isOpened()) {
        cap_.release();
    }
}

} // namespace fv_mjpeg_server

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<fv_mjpeg_server::FVMJPEGServer>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
