/**
 * @file aspara_analyzer_base.hpp
 * @brief アスパラガス解析ノードの基底クラス
 * @details ROS2インターフェースと基本的なデータ管理
 * @author FluentVision Team
 * @date 2025
 * @version 1.0
 */

#ifndef FV_ASPARA_ANALYZER__ASPARA_ANALYZER_BASE_HPP_
#define FV_ASPARA_ANALYZER__ASPARA_ANALYZER_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <fluent.hpp>
#include "aspara_info.hpp"
#include "aspara_selection.hpp"

namespace fv_aspara_analyzer
{

/**
 * @class AsparaAnalyzerBase
 * @brief アスパラガス解析ノードの基底クラス
 * @details ROS2のインターフェース管理とデータストレージを担当
 */
class AsparaAnalyzerBase : public rclcpp::Node
{
public:
    explicit AsparaAnalyzerBase(const std::string& node_name = "fv_aspara_analyzer");
    virtual ~AsparaAnalyzerBase() = default;

protected:
    // ===== ROS2 コールバック関数（純粋仮想関数） =====
    virtual void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) = 0;
    virtual void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) = 0;
    virtual void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) = 0;
    virtual void maskCallback(const sensor_msgs::msg::Image::SharedPtr msg) = 0;
    virtual void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) = 0;
    virtual void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) = 0;

    // ===== ROS2 サービスコールバック（純粋仮想関数） =====
    virtual void nextAsparaguService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) = 0;
    virtual void prevAsparaguService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) = 0;

protected:
    // ===== ROS2 サブスクライバー =====
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    // ===== ROS2 パブリッシャー =====
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr selected_pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_pub_;

    // ===== ROS2 サービス =====
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr next_asparagus_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prev_asparagus_service_;

    // ===== TF2（座標変換）関連 =====
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ===== データストレージ =====
    std::vector<AsparaInfo> aspara_list_;
    AsparaSelection aspara_selection_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
    sensor_msgs::msg::Image::SharedPtr latest_depth_image_;
    sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;
    sensor_msgs::msg::Image::SharedPtr latest_color_image_;
    cv::Mat latest_mask_;
    int selected_aspara_id_ = -1;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr selected_pointcloud_;

    // ===== パラメータ群 =====
    double min_confidence_;
    double pointcloud_distance_min_;
    double pointcloud_distance_max_;
    double aspara_filter_distance_;
    int noise_reduction_neighbors_;
    double noise_reduction_std_dev_;
    double voxel_leaf_size_;
    double harvest_min_length_;
    double harvest_max_length_;
    double straightness_threshold_;
    bool enable_pointcloud_processing_;

    // ===== FPS測定用 =====
    std::unique_ptr<fluent::utils::FPSMeter> color_fps_meter_;
    std::unique_ptr<fluent::utils::FPSMeter> depth_fps_meter_;
    std::unique_ptr<fluent::utils::FPSMeter> detection_fps_meter_;
    fluent::utils::Stopwatch detection_stopwatch_;

private:
    void initializeROS2Interface();
    void declareParameters();
    void loadParameters();
};

}  // namespace fv_aspara_analyzer

#endif  // FV_ASPARA_ANALYZER__ASPARA_ANALYZER_BASE_HPP_