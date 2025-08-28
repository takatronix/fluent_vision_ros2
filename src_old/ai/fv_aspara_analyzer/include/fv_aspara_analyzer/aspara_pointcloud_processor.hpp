#ifndef ASPARA_POINTCLOUD_PROCESSOR_HPP
#define ASPARA_POINTCLOUD_PROCESSOR_HPP

// ROS2関連のインクルード
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
// Markers
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// OpenCV関連のインクルード
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// PCL（Point Cloud Library）関連のインクルード
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>
#include <pcl/exceptions.h>

// TF2（座標変換）関連のインクルード
#include <tf2_ros/transform_broadcaster.h>

#include "aspara_info.hpp"

namespace fv_aspara_analyzer
{

// 前方宣言
class FvAsparaAnalyzerNode;

/**
 * @brief アスパラガス点群処理専用クラス
 * @details 重い点群処理を担当する独立したクラス
 */
class AsparaPointcloudProcessor {
public:
    /**
     * @brief コンストラクタ
     * @param node_ptr 親ノードのポインタ（パラメータとパブリッシャー用）
     */
    explicit AsparaPointcloudProcessor(FvAsparaAnalyzerNode* node_ptr);
    
    /**
     * @brief 深度画像から効率的に点群を生成
     * @param bbox 2Dバウンディングボックス
     * @param depth_image 深度画像
     * @param color_image カラー画像
     * @param camera_info カメラ情報
     * @return フィルタリング済み点群
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractPointCloudFromDepth(
        const cv::Rect& bbox,
        const sensor_msgs::msg::Image::SharedPtr& depth_image,
        const sensor_msgs::msg::Image::SharedPtr& color_image,
        const sensor_msgs::msg::CameraInfo::SharedPtr& camera_info);
    
    /**
     * @brief 点群のノイズ除去処理
     * @param input_cloud 入力点群
     * @return ノイズ除去済み点群
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyNoiseReduction(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud);
    
    /**
     * @brief アスパラガスの根元位置を推定
     * @param aspara_cloud アスパラガス点群
     * @return 根元の3D座標
     */
    geometry_msgs::msg::Point estimateRootPosition(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aspara_cloud);
    
    /**
     * @brief アスパラガスの真っ直ぐ度を計算
     * @param aspara_cloud アスパラガス点群
     * @return 真っ直ぐ度（0.0-1.0）
     */
    float calculateStraightness(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aspara_cloud);
    
    /**
     * @brief アスパラガスの長さを計算
     * @param aspara_cloud アスパラガス点群
     * @return 長さ（メートル）
     */
    float calculateLength(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aspara_cloud);
    
    /**
     * @brief PCA直線を生成
     * @param aspara_cloud アスパラガス点群
     * @return PCA直線の点群
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr generatePCALine(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aspara_cloud);
    
    /**
     * @brief 3D点を2D画像座標に投影
     * @param point 3D点
     * @param camera_info カメラ情報
     * @return 2D画像座標
     */
    cv::Point2f project3DTo2D(
        const pcl::PointXYZRGB& point,
        const sensor_msgs::msg::CameraInfo& camera_info);
    
    /**
     * @brief フィルタリング済み点群をパブリッシュ
     * @param cloud 点群データ
     * @param frame_id 座標系ID
     * @param aspara_id アスパラガスID
     */
    void publishFilteredPointCloud(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
        const std::string& frame_id,
        int aspara_id);

    /**
     * @brief フィルタリング済み点群をパブリッシュ（タイムスタンプ指定）
     * @param cloud 点群データ
     * @param frame_id 座標系ID
     * @param aspara_id アスパラガスID
     * @param stamp 元フレームのタイムスタンプ
     */
    void publishFilteredPointCloud(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
        const std::string& frame_id,
        int aspara_id,
        const rclcpp::Time& stamp);
    
    /**
     * @brief 根元位置のTF座標をパブリッシュ
     * @param root_position 根元位置
     * @param frame_id 座標系ID
     * @param aspara_id アスパラガスID
     */
    void publishRootTF(
        const geometry_msgs::msg::Point& root_position,
        const std::string& frame_id,
        int aspara_id);
    
    // Marker publisher (preferred over TF for per-detection visualization)
    void publishAsparaMarker(
        const geometry_msgs::msg::Point& root_position,
        const std::string& frame_id,
        int aspara_id,
        float length_m,
        bool is_harvestable,
        const rclcpp::Time& stamp);

    // Overload: specify principal axis direction in frame_id
    void publishAsparaMarker(
        const geometry_msgs::msg::Point& root_position,
        const std::string& frame_id,
        int aspara_id,
        float length_m,
        bool is_harvestable,
        const rclcpp::Time& stamp,
        const geometry_msgs::msg::Vector3& axis_dir);
    
    /**
     * @brief 注釈付き画像をパブリッシュ
     * @param image 元画像
     * @param aspara_info アスパラガス情報
     * @param filtered_cloud フィルタリング済み点群
     * @param pca_line_cloud PCA直線点群
     * @param length 長さ
     * @param straightness 真っ直ぐ度
     * @param is_harvestable 収穫可能フラグ
     */
    void publishAnnotatedImage(
        const cv::Mat& image,
        const AsparaInfo& aspara_info,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& pca_line_cloud,
        float length,
        float straightness,
        bool is_harvestable);

private:
    // ROS2ノードへの参照
    FvAsparaAnalyzerNode* node_;
    
    // パブリッシャー
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    
    // TFブロードキャスター
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // パラメータ
    double pointcloud_distance_min_;
    double pointcloud_distance_max_;
    int noise_reduction_neighbors_;
    double noise_reduction_std_dev_;
    double voxel_leaf_size_;
    double marker_lifetime_sec_ {30.0};
};

} // namespace fv_aspara_analyzer

#endif // ASPARA_POINTCLOUD_PROCESSOR_HPP