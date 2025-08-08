#ifndef FV_DEPTH_CAMERA_NODE_HPP
#define FV_DEPTH_CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <librealsense2/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>
#include <mutex>
#include <unordered_map>

// Include service headers
#include "fv_realsense/srv/get_distance.hpp"
#include "fv_realsense/srv/get_camera_info.hpp"
#include "fv_realsense/srv/set_mode.hpp"
// GeneratePointCloud service removed; use topic-based pipeline only

class FVDepthCameraNode : public rclcpp::Node
{
public:
    explicit FVDepthCameraNode(const std::string& node_name = "fv_realsense");
    ~FVDepthCameraNode();

private:
    // Configuration structures
    struct CameraSelectionConfig
    {
        std::string selection_method = "auto";
        std::string serial_number = "";
        std::string device_name = "";
        int device_index = 0;
    };

    struct PowerManagementConfig
    {
        double startup_delay = 2.0;
    };

    struct CameraConfig
    {
        int color_width = 640;
        int color_height = 480;
        int color_fps = 30;
        int depth_width = 640;
        int depth_height = 480;
        int depth_fps = 30;
    };

    struct StreamConfig
    {
        bool color_enabled = true;
        bool depth_enabled = true;
        bool infrared_enabled = false;
        bool pointcloud_enabled = true;
        bool depth_colormap_enabled = true;
        bool sync_enabled = true;        // 深度・カラー同期設定
    };

    struct CameraInfoConfig
    {
        bool enable_camera_info = true;
        bool enable_compressed_topics = true;
        int compressed_quality = 85;
        bool enable_depth_compressed = false;
    };

    struct ServicesConfig
    {
        bool get_distance_enabled = true;
        bool get_camera_info_enabled = true;
        bool set_mode_enabled = true;
    };

    struct TFConfig
    {
        bool enabled = true;
        std::string base_frame = "base_link";
        std::string camera_frame = "camera_link";
        std::string color_optical_frame = "color_optical_frame";
        std::string depth_optical_frame = "depth_optical_frame";
    };

    struct TopicConfig
    {
        std::string color = "color/image_raw";
        std::string depth = "depth/image_rect_raw";
        std::string color_compressed = "color/image_raw/compressed";
        std::string depth_colormap = "depth/colormap";
        std::string pointcloud = "depth/color/points";
        std::string color_camera_info = "color/camera_info";
        std::string depth_camera_info = "depth/camera_info";
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

    // RealSense members
    rs2::context ctx_;
    rs2::pipeline pipe_;
    rs2::config cfg_;
    rs2::pipeline_profile profile_;
    rs2::device device_;
    
    // Synchronization flag
    bool sync_enabled_ = false;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_colormap_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;

    // Compressed image publisher
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr color_compressed_pub_;

    // Services
    rclcpp::Service<fv_realsense::srv::GetDistance>::SharedPtr get_distance_service_;
    rclcpp::Service<fv_realsense::srv::GetCameraInfo>::SharedPtr get_camera_info_service_;
    rclcpp::Service<fv_realsense::srv::SetMode>::SharedPtr set_mode_service_;
    // GeneratePointCloud service removed

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr click_event_sub_;

    // TF
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Threading
    std::thread processing_thread_;
    std::atomic<bool> running_;
    std::mutex frame_mutex_;
    // Cache of latest frames for service-safe access
    std::mutex latest_frame_mutex_;
    cv::Mat latest_color_image_mat_;
    cv::Mat latest_depth_image_mat_;
    rclcpp::Time latest_frame_stamp_;
    
    // Mode control
    std::atomic<int> current_mode_{1};  // デフォルトは基本動作モード

    // Point marker for display modes
    struct PointMarker {
        cv::Point point;
        rclcpp::Time start_time;
        bool active;
        int mode;  // 0: 表示なし, 1: カーソルのみ, 2: カーソル+座標+距離
        float x, y, z;  // 3D座標
    };
    mutable PointMarker point_marker_;

    // Camera intrinsics
    rs2_intrinsics color_intrinsics_;
    rs2_intrinsics depth_intrinsics_;
    float depth_scale_ = 0.001f;
    float config_depth_scale_ = -1.0;  // 設定ファイルからのオーバーライド値（-1.0はセンサーから取得）

    // Methods
    void loadParameters();
    bool initializeRealSense();
    bool selectCamera();
    void initializePublishers();
    void initializeServices();
    void initializeTF();
    void processingLoop();
    void publishFrames(const rs2::frame& color_frame, const rs2::frame& depth_frame);
    void publishPointCloud(const rs2::frame& color_frame, const rs2::frame& depth_frame);
    cv::Mat createDepthColormap(const rs2::frame& depth_frame);
    void publishTF();
    
    // Service callbacks
    void handleGetDistance(
        const std::shared_ptr<fv_realsense::srv::GetDistance::Request> request,
        std::shared_ptr<fv_realsense::srv::GetDistance::Response> response);
    
    void handleGetCameraInfo(
        const std::shared_ptr<fv_realsense::srv::GetCameraInfo::Request> request,
        std::shared_ptr<fv_realsense::srv::GetCameraInfo::Response> response);

    void handleSetMode(
        const std::shared_ptr<fv_realsense::srv::SetMode::Request> request,
        std::shared_ptr<fv_realsense::srv::SetMode::Response> response);
    
    // GeneratePointCloud handler removed

    // Utility methods
    bool get3DCoordinate(int x, int y, float& world_x, float& world_y, float& world_z);
    std::vector<rs2::device> getAvailableDevices();
    
    // Display methods
    void clickEventCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void drawMarker(cv::Mat& frame) const;
    void initializeSubscribers();
};

#endif // FV_DEPTH_CAMERA_NODE_HPP 