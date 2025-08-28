/**
 * @file aspara_visualizer.hpp
 * @brief アスパラガス可視化専用クラス
 * @details 画像描画、アニメーション、パブリッシュを担当
 * @author FluentVision Team
 * @date 2025
 * @version 1.0
 */

#ifndef FV_ASPARA_ANALYZER__ASPARA_VISUALIZER_HPP_
#define FV_ASPARA_ANALYZER__ASPARA_VISUALIZER_HPP_

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fluent.hpp>

#include "aspara_info.hpp"

namespace fv_aspara_analyzer
{

/**
 * @class AsparaVisualizer
 * @brief アスパラガス可視化専用クラス
 * @details 画像描画、アニメーション効果、ROS2パブリッシュを担当
 */
class AsparaVisualizer
{
public:
    explicit AsparaVisualizer(rclcpp::Node* node);
    ~AsparaVisualizer() = default;

    // ===== 画像描画・可視化 =====

    /**
     * @brief 現在の画像を出力（検出結果がある場合はオーバーレイ付き）
     * @param color_image 元画像
     * @param aspara_list アスパラガス情報リスト
     * @param selected_aspara_id 選択中ID
     * @param fps_info FPS情報
     * @param enable_pointcloud_processing ポイントクラウド処理有効フラグ
     * @return 描画済み画像
     */
    cv::Mat renderCurrentImage(
        const cv::Mat& color_image,
        const std::vector<AsparaInfo>& aspara_list,
        int selected_aspara_id,
        const std::map<std::string, float>& fps_info,
        bool enable_pointcloud_processing);

    /**
     * @brief 注釈付き画像を描画
     * @param image 元画像
     * @param aspara_info アスパラガス情報
     * @param filtered_cloud フィルタリング済み点群
     * @param pca_line_cloud PCA直線点群
     * @param camera_info カメラ情報
     * @param length 長さ
     * @param straightness 真っ直ぐ度
     * @param is_harvestable 収穫可能フラグ
     * @return 注釈付き画像
     */
    cv::Mat renderAnnotatedImage(
        const cv::Mat& image,
        const AsparaInfo& aspara_info,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& pca_line_cloud,
        const sensor_msgs::msg::CameraInfo& camera_info,
        float length,
        float straightness,
        bool is_harvestable);

    // ===== アニメーション =====

    /**
     * @brief スムーズアニメーション更新
     * @param aspara_list アスパラガス情報リスト（更新対象）
     * @param delta_time フレーム間時間
     */
    void updateSmoothAnimation(
        std::vector<AsparaInfo>& aspara_list,
        float delta_time);

    // ===== ROS2パブリッシュ =====

    /**
     * @brief フィルタリング済み点群をパブリッシュ
     * @param cloud 点群データ
     * @param frame_id 座標系ID
     * @param aspara_id アスパラガスID
     */
    // ===== 3D可視化・マーカー =====

    /**
     * @brief アスパラガス3D円柱ポリゴン可視化マーカー生成
     * @param aspara_list アスパラガス情報リスト
     * @param selected_aspara_id 選択中アスパラID
     * @param frame_id 座標系ID
     * @return MarkerArray（円柱＋穂＋ID表示）
     */
    visualization_msgs::msg::MarkerArray createAsparaCylinderMarkers(
        const std::vector<AsparaInfo>& aspara_list,
        int selected_aspara_id,
        const std::string& frame_id);

    /**
     * @brief 3Dマーカーアレイをパブリッシュ
     * @param markers マーカーアレイ
     */
    void publishMarkerArray(const visualization_msgs::msg::MarkerArray& markers);

    void publishFilteredPointCloud(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
        const std::string& frame_id,
        int aspara_id);

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

    /**
     * @brief 画像をパブリッシュ
     * @param image 画像
     * @param header ヘッダー情報
     * @param publisher パブリッシャー
     */
    void publishImage(
        const cv::Mat& image,
        const std_msgs::msg::Header& header,
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher);

    // ===== 描画ユーティリティ =====

    /**
     * @brief 矩形内に詳細情報を描画
     * @param image 描画対象画像
     * @param aspara アスパラガス情報
     * @param is_selected 選択状態
     */
    void drawAsparaDetailInfo(
        cv::Mat& image,
        const AsparaInfo& aspara,
        bool is_selected);

    /**
     * @brief 3D点を2D画像座標に投影
     * @param point 3D点
     * @param camera_info カメラ情報
     * @return 2D画像座標
     */
    cv::Point2f project3DTo2D(
        const pcl::PointXYZRGB& point,
        const sensor_msgs::msg::CameraInfo& camera_info);

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr selected_pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // アニメーション用定数
    static constexpr float ANIMATION_SPEED = 0.15f;
    static constexpr float FADE_IN_SPEED = 3.0f;
};

}  // namespace fv_aspara_analyzer

#endif  // FV_ASPARA_ANALYZER__ASPARA_VISUALIZER_HPP_