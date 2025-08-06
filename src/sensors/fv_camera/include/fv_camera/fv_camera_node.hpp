#ifndef FV_USB_CAMERA_NODE_HPP
#define FV_USB_CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>

#include <memory>
#include <string>
#include <mutex>
#include <unordered_map>
#include <opencv2/opencv.hpp>

// Include service headers
#include "fv_camera/srv/get_camera_info.hpp"
#include "fv_camera/srv/set_camera_settings.hpp"

class FVUSBCameraNode : public rclcpp::Node
{
public:
    explicit FVUSBCameraNode();
    ~FVUSBCameraNode();

private:
    // Configuration structures
    struct CameraSelectionConfig
    {
        std::string selection_method = "auto";
        std::string device_name = "";
        int device_index = 0;
    };

    struct PowerManagementConfig
    {
        double startup_delay = 1.0;
    };

    struct CameraConfig
    {
        int width = 640;
        int height = 480;
        int fps = 30;
        int brightness = -1;  // -1 means auto
        int contrast = -1;
        int saturation = -1;
        int hue = -1;
        int gain = -1;
        int exposure = -1;
    };

    struct StreamConfig
    {
        bool color_enabled = true;
        bool compressed_enabled = true;
    };

    struct CameraInfoConfig
    {
        bool enable_camera_info = true;
        bool enable_compressed_topics = true;
        int compressed_quality = 85;
    };

    struct ServicesConfig
    {
        bool get_camera_info_enabled = true;
        bool set_camera_settings_enabled = true;
    };

    struct TFConfig
    {
        bool enabled = true;
        std::string base_frame = "base_link";
        std::string camera_frame = "camera_link";
        std::string optical_frame = "camera_optical_frame";
    };

    struct TopicConfig
    {
        std::string color = "image_raw";
        std::string color_compressed = "image_raw/compressed";
    };

    // Configuration members
    CameraSelectionConfig camera_selection_config_;
    PowerManagementConfig power_management_config_;
    CameraConfig camera_config_;
    StreamConfig stream_config_;
    CameraInfoConfig camera_info_config_;
    ServicesConfig services_config_;
    TFConfig tf_config_;
    TopicConfig topic_config_;

    // OpenCV camera
    cv::VideoCapture camera_;
    cv::Mat current_frame_;
    std::mutex frame_mutex_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

    // Image transport for compressed topics
    std::unique_ptr<image_transport::ImageTransport> image_transport_;
    image_transport::Publisher color_compressed_pub_;

    // Services
    rclcpp::Service<fv_camera::srv::GetCameraInfo>::SharedPtr get_camera_info_service_;
    rclcpp::Service<fv_camera::srv::SetCameraSettings>::SharedPtr set_camera_settings_service_;

    // TF
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Processing thread
    std::thread processing_thread_;
    std::atomic<bool> running_;

    // Camera info
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    // Methods
    void loadParameters();
    bool initializeCamera();
    bool selectCamera();
    void initializePublishers();
    void initializeServices();
    void initializeTF();
    void processingLoop();
    void publishFrame(const cv::Mat& frame);
    void publishTF();
    void updateCameraInfo();

    // Service handlers
    void handleGetCameraInfo(
        const std::shared_ptr<fv_camera::srv::GetCameraInfo::Request> request,
        std::shared_ptr<fv_camera::srv::GetCameraInfo::Response> response);

    void handleSetCameraSettings(
        const std::shared_ptr<fv_camera::srv::SetCameraSettings::Request> request,
        std::shared_ptr<fv_camera::srv::SetCameraSettings::Response> response);

    // Utility methods
    std::vector<cv::VideoCapture> getAvailableCameras();
    bool setCameraProperty(int property, double value);
    double getCameraProperty(int property);
    std::string getCameraPropertyName(int property);
};

#endif // FV_USB_CAMERA_NODE_HPP 