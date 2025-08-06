#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <image_transport/image_transport.hpp>

#include <memory>
#include <thread>
#include <atomic>
#include <string>

// Forward declarations
namespace fv_rtmp_server {
class RTMPServer;
class VideoProcessor;
}

namespace fv_rtmp_server
{

class RTMPBridgeNode : public rclcpp::Node
{
public:
    RTMPBridgeNode();
    ~RTMPBridgeNode();

private:
    // Initialization methods
    void initializeParameters();
    void initializePublishers();
    void initializeServer();

    // Callback methods
    void onFrameReceived(const uint8_t* data, int width, int height, int64_t timestamp);
    void onStatusChanged(const std::string& status, const std::string& detail = "");
    
    // IP and info display methods
    bool isPortInUse(int port);
    std::string getLocalIPAddress();
    void publishInfoImage(const std::string& status, const std::string& ip_address);
    void startInfoImageTimer();
    
    // Camera info methods
    bool loadCameraInfo();
    bool loadSingleCameraInfo(const std::string& file_path, sensor_msgs::msg::CameraInfo& camera_info);
    void publishCameraInfo();

    // Server management
    void startServer();
    void stopServer();

    // Parameters
    int server_port_;
    std::string server_endpoint_;
    std::string output_topic_name_;
    std::string frame_id_;
    int target_width_;
    int target_height_;
    double publish_rate_;
    bool enable_low_latency_;
    int buffer_size_;
    std::string quality_mode_;
    std::string calibration_file_;
    std::string camera_info_topic_;
    bool compressed_;
    
    // Camera info messages
    sensor_msgs::msg::CameraInfo camera_info_o4_;
    sensor_msgs::msg::CameraInfo camera_info_o4_pro_;

    // ROS2 publishers
    image_transport::Publisher image_transport_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_o4_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_o4_pro_publisher_;
    
    // Image transport node
    std::shared_ptr<image_transport::ImageTransport> image_transport_;

    // Components
    std::unique_ptr<fv_rtmp_server::RTMPServer> rtmp_server_;
    std::unique_ptr<fv_rtmp_server::VideoProcessor> video_processor_;

    // Threading
    std::thread server_thread_;
    std::atomic<bool> should_stop_;

    // Statistics
    std::atomic<uint64_t> frames_received_;
    std::atomic<uint64_t> frames_published_;
    rclcpp::Time last_status_time_;
    
    // Info display
    rclcpp::TimerBase::SharedPtr info_timer_;
    std::string local_ip_address_;
    bool client_connected_;
};

} // namespace fv_rtmp_server 