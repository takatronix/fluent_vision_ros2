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

#ifdef ENABLE_REALSENSE
#include <librealsense2/rs.hpp>
#endif
#include <fluent_lib/cv_bridge_compat.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>
#include <mutex>
#include <unordered_map>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <chrono>

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
        std::string registered_points = ""; // organized cloud topic（空なら無効）
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
    std::unique_ptr<rs2::context> ctx_;
    rs2::device device_;

    // Sensor-based streaming (preferred: decouple color/depth without wait_for_frames)
    rs2::sensor color_sensor_;
    rs2::sensor depth_sensor_;
    rs2::stream_profile color_profile_;
    rs2::stream_profile depth_profile_;
    bool sensors_started_ = false;
    
    // Synchronization flag
    bool sync_enabled_ = false;
    bool align_to_color_ = false;  // Align depth frames to color stream
    bool organized_pointcloud_enabled_ = false; // organized cloud publish flag
    int organized_pointcloud_decimation_ = 1;    // 1=full, >1 decimated with NaN fill
    bool organized_pointcloud_rgb_ = true;      // include RGB in organized cloud
    // Frame sync warning threshold (ms)
    double sync_warn_ms_ = 1.0;
    // Soft sync parameters (pairing by device timestamp)
    double sync_max_skew_ms_ = 20.0;
    int sync_max_wait_ms_ = 15;
    std::size_t sync_queue_size_ = 5;
    // Timestamping: map device timestamp to ROS time
    bool use_device_timestamp_ = true;
    double device_ts_reset_threshold_ms_ = 1000.0;
    std::mutex device_time_mutex_;
    bool device_time_initialized_ = false;
    rs2_timestamp_domain device_time_domain_{RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK};
    double base_device_ts_ms_ = 0.0;
    rclcpp::Time base_ros_stamp_{0, 0, RCL_SYSTEM_TIME};
    double last_device_ts_ms_ = 0.0;
    rclcpp::Time last_ros_stamp_{0, 0, RCL_SYSTEM_TIME};

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_colormap_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_points_pub_;
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
    // Guard any calls into rs2::pipeline (not thread-safe across callbacks/services)
    std::mutex pipeline_mutex_;
    // Buffers for decoupled stream pairing
    struct FrameItem {
        rs2::frame frame;
        double ts_ms = 0.0;  // device timestamp (ms)
        std::chrono::steady_clock::time_point recv_tp{};
    };
    std::mutex sync_mutex_;
    std::condition_variable sync_cv_;
    std::deque<FrameItem> color_queue_;
    std::deque<FrameItem> depth_queue_;
    std::atomic<int64_t> last_color_recv_ns_{0};
    std::atomic<int64_t> last_depth_recv_ns_{0};
    std::atomic<int64_t> last_paired_publish_ns_{0};
    std::atomic<uint64_t> dropped_color_frames_{0};
    std::atomic<uint64_t> dropped_depth_frames_{0};

    // Cache of latest frames for service-safe access (avoid per-frame cv::Mat clones)
    std::mutex latest_frame_mutex_;
    rs2::frame latest_color_frame_;
    rs2::frame latest_depth_frame_;
    double latest_color_ts_ms_ = 0.0;
    double latest_depth_ts_ms_ = 0.0;
    rclcpp::Time latest_color_stamp_;
    rclcpp::Time latest_depth_stamp_;

    // Cache settings
    bool cache_latest_frames_enabled_ = false;

    // Frame wait/recovery tuning
    int frame_wait_timeout_ms_ = 1000;
    int stall_warn_ms_ = 0;
    int stall_restart_ms_ = 0;

    // Periodic stats report (helps distinguish camera stall vs publish bottleneck)
    bool stats_report_enabled_ = false;
    int stats_report_period_ms_ = 1000;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    uint64_t last_stats_color_cb_ = 0;
    uint64_t last_stats_depth_cb_ = 0;
    uint64_t last_stats_color_pub_ = 0;
    uint64_t last_stats_depth_pub_ = 0;
    std::atomic<uint64_t> color_cb_count_{0};
    std::atomic<uint64_t> depth_cb_count_{0};
    std::atomic<uint64_t> color_pub_count_{0};
    std::atomic<uint64_t> depth_pub_count_{0};

    // Mode control
    std::atomic<int> current_mode_{1};  // デフォルトは基本動作モード（パラメータinitial_modeで上書き）

    // Point marker for display modes
    struct PointMarker {
        cv::Point point;
        rclcpp::Time start_time;
        bool active;
        int mode;  // 0: 表示なし, 1: カーソルのみ, 2: カーソル+座標+距離
        float x, y, z;  // 3D座標
    };
    mutable std::mutex point_marker_mutex_;
    mutable PointMarker point_marker_{cv::Point(), rclcpp::Time(0, 0, RCL_SYSTEM_TIME), false, 0, 0.0f, 0.0f, 0.0f};

    // Camera intrinsics
    rs2_intrinsics color_intrinsics_;
    rs2_intrinsics depth_intrinsics_;
    float depth_scale_ = 0.001f;
    float config_depth_scale_ = -1.0;  // 設定ファイルからのオーバーライド値（-1.0はセンサーから取得）
    // Point cloud distance clipping (applies to registered_points and raw point cloud)
    float min_distance_m_ = 0.1f;
    float max_distance_m_ = 3.0f;

    // Methods
    void loadParameters();
    bool initializeRealSense();
    bool selectCamera();
    bool startSensors();
    void stopSensors();
    void onColorFrame(const rs2::frame& frame);
    void onDepthFrame(const rs2::frame& frame);
    void initializePublishers();
    void initializeServices();
    void initializeTF();
    void processingLoop();
    rclcpp::Time stampFromDeviceTime(const rs2::frame& frame, double device_ts_ms);
    void publishFrames(const rs2::frame& color_frame, const rs2::frame& depth_frame, const rclcpp::Time& stamp);
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
    void drawHUD(cv::Mat& frame) const;
    void initializeSubscribers();
};

#endif // FV_DEPTH_CAMERA_NODE_HPP 
