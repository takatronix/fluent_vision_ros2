#ifndef FLUENT_CLOUD_HPP
#define FLUENT_CLOUD_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <memory>
#include <functional>

namespace fluent_cloud {

/**
 * @brief FluentCloud - 美しいポイントクラウド処理
 * 
 * 使用例:
 * auto result = FluentCloud::from(depth_image)
 *     .removeNoise()
 *     .findPlanes()
 *     .colorize(rgb_image)
 *     .publish("/cloud");
 */
template<typename PointT = pcl::PointXYZRGB>
class FluentCloud {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using FilterFunc = std::function<bool(const PointT&)>;
    
    // ========== ファクトリメソッド ==========
    // PCLから
    static FluentCloud from(const PointCloudPtr& cloud);
    static FluentCloud from(const PointCloud& cloud);
    
    // ROS2メッセージから
    static FluentCloud from(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    static FluentCloud from(const sensor_msgs::msg::PointCloud2& msg);
    
    // 深度画像から
    using CameraInfo = sensor_msgs::msg::CameraInfo;
    static FluentCloud fromDepth(const cv::Mat& depth, const CameraInfo& info);
    static FluentCloud fromRGBD(const cv::Mat& rgb, const cv::Mat& depth, const CameraInfo& info);
    
    // ========== フィルタリング ==========
    // 基本フィルタ
    FluentCloud& filter(FilterFunc condition);
    FluentCloud& filterByDistance(float min_dist, float max_dist);
    FluentCloud& filterByHeight(float min_z, float max_z);
    FluentCloud& cropBox(const Eigen::Vector3f& min, const Eigen::Vector3f& max);
    
    // ノイズ除去
    FluentCloud& removeOutliers(int neighbors = 50, float std_dev = 1.0);
    FluentCloud& removeGround(float distance_threshold = 0.01);
    FluentCloud& removeNoise(float radius = 0.05);
    FluentCloud& smoothMLS(float radius = 0.03);  // Moving Least Squares
    
    // ダウンサンプリング
    FluentCloud& downsample(float leaf_size = 0.01);
    FluentCloud& uniformSample(int target_points = 10000);
    
    // ========== セグメンテーション ==========
    FluentCloud& findPlanes(float distance_threshold = 0.01);
    FluentCloud& findCylinders(float radius_min = 0.01, float radius_max = 0.5);
    FluentCloud& findSpheres(float radius_min = 0.05, float radius_max = 1.0);
    
    // クラスタリング
    FluentCloud& cluster(float tolerance = 0.02, int min_size = 100, int max_size = 25000);
    FluentCloud& euclideanCluster(float tolerance = 0.02);
    FluentCloud& regionGrow(float smoothness = 3.0, float curvature = 1.0);
    
    // ========== 特徴抽出 ==========
    FluentCloud& computeNormals(float radius = 0.03);
    FluentCloud& computeCurvature();
    FluentCloud& computeFPFH(float radius = 0.05);  // Fast Point Feature Histograms
    
    // ========== 変換 ==========
    FluentCloud& transform(const Eigen::Matrix4f& matrix);
    FluentCloud& translate(float x, float y, float z);
    FluentCloud& rotate(float roll, float pitch, float yaw);
    FluentCloud& rotateAroundAxis(const Eigen::Vector3f& axis, float angle);
    FluentCloud& scale(float factor);
    FluentCloud& scale(float sx, float sy, float sz);
    
    // TF2連携
    FluentCloud& transformWithTF(const std::string& target_frame, 
                                const std::string& source_frame = "");
    FluentCloud& waitAndTransform(const std::string& target_frame, 
                                 double timeout = 1.0);
    
    // 追加の変換メソッド
    FluentCloud& alignToGround();
    FluentCloud& centerAtOrigin();
    FluentCloud& lookAt(const Eigen::Vector3f& target, const Eigen::Vector3f& up = Eigen::Vector3f(0, 0, 1));
    
    // ========== 色付け ==========
    FluentCloud& colorize(const cv::Mat& rgb_image, const Eigen::Matrix4f& transform = Eigen::Matrix4f::Identity());
    FluentCloud& colorByHeight(float min_z = 0.0, float max_z = 3.0);
    FluentCloud& colorByCurvature();
    FluentCloud& colorByCluster();
    FluentCloud& setColor(uint8_t r, uint8_t g, uint8_t b);
    
    // ========== 解析 ==========
    FluentCloud& detectObjects(const std::string& model = "default");
    FluentCloud& classifyShapes();
    FluentCloud& findKeypoints();
    FluentCloud& matchTemplate(const PointCloudPtr& template_cloud);
    
    // ========== 統合・分割 ==========
    FluentCloud& merge(const FluentCloud& other);
    FluentCloud& subtract(const FluentCloud& other, float tolerance = 0.01);
    std::vector<FluentCloud> split() const;  // クラスタごとに分割
    
    // ========== 便利機能 ==========
    FluentCloud& forEach(std::function<void(PointT&)> func);
    FluentCloud& forEachCluster(std::function<void(FluentCloud&)> func);
    
    // 条件処理
    FluentCloud& when(std::function<bool()> condition);
    FluentCloud& otherwise();
    
    // デバッグ
    FluentCloud& visualize(const std::string& window_name = "FluentCloud");
    FluentCloud& save(const std::string& filename);
    FluentCloud& print(const std::string& label = "");
    
    // ========== 出力 ==========
    PointCloudPtr toPointCloud() const;
    sensor_msgs::msg::PointCloud2 toPointCloud2() const;
    void publish(const std::string& topic) const;
    
    // ========== 情報取得 ==========
    size_t size() const;
    bool empty() const;
    Eigen::Vector3f centroid() const;
    Eigen::Vector3f min() const;
    Eigen::Vector3f max() const;
    float volume() const;  // Bounding boxの体積
    
private:
    PointCloudPtr cloud_;
    std::vector<pcl::PointIndices> clusters_;
    std::vector<pcl::ModelCoefficients> planes_;
    
    FluentCloud(const PointCloudPtr& cloud);
};

// ========== カメラ情報 ==========
struct CameraInfo {
    float fx, fy;  // 焦点距離
    float cx, cy;  // 主点
    int width, height;
    
    CameraInfo(float fx, float fy, float cx, float cy, int w, int h)
        : fx(fx), fy(fy), cx(cx), cy(cy), width(w), height(h) {}
};

// ========== 便利な型エイリアス ==========
using FluentCloudXYZ = FluentCloud<pcl::PointXYZ>;
using FluentCloudXYZRGB = FluentCloud<pcl::PointXYZRGB>;
using FluentCloudXYZI = FluentCloud<pcl::PointXYZI>;

} // namespace fluent_cloud

#endif // FLUENT_CLOUD_HPP