#include "fluent_cloud/fluent_cloud.hpp"
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

namespace fluent_cloud {

// ========== コンストラクタ ==========
template<typename PointT>
FluentCloud<PointT>::FluentCloud(const PointCloudPtr& cloud) 
    : cloud_(cloud) {
    if (!cloud_) {
        cloud_ = std::make_shared<PointCloud>();
    }
}

// ========== ファクトリメソッド ==========
template<typename PointT>
FluentCloud<PointT> FluentCloud<PointT>::from(const PointCloudPtr& cloud) {
    return FluentCloud<PointT>(cloud);
}

template<typename PointT>
FluentCloud<PointT> FluentCloud<PointT>::from(const sensor_msgs::msg::PointCloud2& msg) {
    PointCloudPtr cloud(new PointCloud);
    pcl::fromROSMsg(msg, *cloud);
    return FluentCloud<PointT>(cloud);
}

// ========== 地面除去（RANSAC） ==========
template<typename PointT>
FluentCloud<PointT>& FluentCloud<PointT>::removeGround(float distance_threshold) {
    if (cloud_->empty()) return *this;
    
    // RANSACで平面検出
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold);
    seg.setInputCloud(cloud_);
    seg.segment(*inliers, *coefficients);
    
    if (inliers->indices.size() == 0) {
        return *this;  // 平面が見つからない
    }
    
    // 平面の法線をチェック（地面は上向き）
    Eigen::Vector3f normal(coefficients->values[0], 
                          coefficients->values[1], 
                          coefficients->values[2]);
    normal.normalize();
    
    // Z軸との角度が30度以内なら地面と判定
    if (std::abs(normal.dot(Eigen::Vector3f::UnitZ())) > 0.866) {  // cos(30°)
        // 地面以外の点を抽出
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_);
        extract.setIndices(inliers);
        extract.setNegative(true);  // インライア以外を残す
        
        PointCloudPtr filtered(new PointCloud);
        extract.filter(*filtered);
        cloud_ = filtered;
    }
    
    return *this;
}

// ========== 高さフィルタ ==========
template<typename PointT>
FluentCloud<PointT>& FluentCloud<PointT>::filterByHeight(float min_z, float max_z) {
    PointCloudPtr filtered(new PointCloud);
    
    for (const auto& point : cloud_->points) {
        if (point.z >= min_z && point.z <= max_z) {
            filtered->points.push_back(point);
        }
    }
    
    filtered->width = filtered->points.size();
    filtered->height = 1;
    filtered->is_dense = false;
    
    cloud_ = filtered;
    return *this;
}

// ========== ダウンサンプリング ==========
template<typename PointT>
FluentCloud<PointT>& FluentCloud<PointT>::downsample(float leaf_size) {
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(cloud_);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    
    PointCloudPtr filtered(new PointCloud);
    voxel_filter.filter(*filtered);
    cloud_ = filtered;
    
    return *this;
}

// ========== 外れ値除去 ==========
template<typename PointT>
FluentCloud<PointT>& FluentCloud<PointT>::removeOutliers(int neighbors, float std_dev) {
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_);
    sor.setMeanK(neighbors);
    sor.setStddevMulThresh(std_dev);
    
    PointCloudPtr filtered(new PointCloud);
    sor.filter(*filtered);
    cloud_ = filtered;
    
    return *this;
}

// ========== クラスタリング ==========
template<typename PointT>
FluentCloud<PointT>& FluentCloud<PointT>::cluster(float tolerance, int min_size, int max_size) {
    // KdTreeの作成
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_);
    
    // Euclideanクラスタリング
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_);
    
    clusters_.clear();
    ec.extract(clusters_);
    
    return *this;
}

// ========== 各クラスタに対する処理 ==========
template<typename PointT>
FluentCloud<PointT>& FluentCloud<PointT>::forEachCluster(std::function<void(FluentCloud&)> func) {
    for (const auto& cluster : clusters_) {
        PointCloudPtr cluster_cloud(new PointCloud);
        pcl::copyPointCloud(*cloud_, cluster.indices, *cluster_cloud);
        
        FluentCloud cluster_fc(cluster_cloud);
        func(cluster_fc);
    }
    
    return *this;
}

// ========== ROS2メッセージへの変換 ==========
template<typename PointT>
sensor_msgs::msg::PointCloud2 FluentCloud<PointT>::toPointCloud2() const {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud_, msg);
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "base_link";  // TODO: パラメータ化
    return msg;
}

// ========== サイズ取得 ==========
template<typename PointT>
size_t FluentCloud<PointT>::size() const {
    return cloud_->points.size();
}

// ========== 座標変換 ==========
template<typename PointT>
FluentCloud<PointT>& FluentCloud<PointT>::transform(const Eigen::Matrix4f& matrix) {
    pcl::transformPointCloud(*cloud_, *cloud_, matrix);
    return *this;
}

template<typename PointT>
FluentCloud<PointT>& FluentCloud<PointT>::translate(float x, float y, float z) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 3) = x;
    transform(1, 3) = y;
    transform(2, 3) = z;
    return this->transform(transform);
}

template<typename PointT>
FluentCloud<PointT>& FluentCloud<PointT>::rotate(float roll, float pitch, float yaw) {
    // オイラー角から回転行列を作成
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
    
    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3f rotationMatrix = q.matrix();
    
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3,3>(0,0) = rotationMatrix;
    
    return this->transform(transform);
}

template<typename PointT>
FluentCloud<PointT>& FluentCloud<PointT>::rotateAroundAxis(const Eigen::Vector3f& axis, float angle) {
    Eigen::AngleAxisf rotation(angle, axis.normalized());
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3,3>(0,0) = rotation.matrix();
    
    return this->transform(transform);
}

template<typename PointT>
FluentCloud<PointT>& FluentCloud<PointT>::scale(float factor) {
    return scale(factor, factor, factor);
}

template<typename PointT>
FluentCloud<PointT>& FluentCloud<PointT>::scale(float sx, float sy, float sz) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 0) = sx;
    transform(1, 1) = sy;
    transform(2, 2) = sz;
    
    return this->transform(transform);
}

// ========== 便利な変換 ==========
template<typename PointT>
FluentCloud<PointT>& FluentCloud<PointT>::alignToGround() {
    // 地面の法線を検出
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setInputCloud(cloud_);
    seg.segment(*inliers, *coefficients);
    
    if (!inliers->indices.empty()) {
        // 地面の法線ベクトル
        Eigen::Vector3f ground_normal(coefficients->values[0], 
                                     coefficients->values[1], 
                                     coefficients->values[2]);
        ground_normal.normalize();
        
        // Z軸に合わせる回転を計算
        Eigen::Vector3f z_axis(0, 0, 1);
        Eigen::Vector3f rotation_axis = ground_normal.cross(z_axis);
        float rotation_angle = std::acos(ground_normal.dot(z_axis));
        
        if (rotation_axis.norm() > 0.001) {
            rotateAroundAxis(rotation_axis, rotation_angle);
        }
    }
    
    return *this;
}

template<typename PointT>
FluentCloud<PointT>& FluentCloud<PointT>::centerAtOrigin() {
    // 重心を計算
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_, centroid);
    
    // 原点に移動
    return translate(-centroid[0], -centroid[1], -centroid[2]);
}

template<typename PointT>
FluentCloud<PointT>& FluentCloud<PointT>::lookAt(const Eigen::Vector3f& target, 
                                                  const Eigen::Vector3f& up) {
    // カメラ座標系のような向きに変換
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_, centroid);
    Eigen::Vector3f position(centroid[0], centroid[1], centroid[2]);
    
    // 前方向ベクトル
    Eigen::Vector3f forward = (target - position).normalized();
    // 右方向ベクトル
    Eigen::Vector3f right = forward.cross(up).normalized();
    // 上方向ベクトル
    Eigen::Vector3f new_up = right.cross(forward);
    
    // 回転行列を構築
    Eigen::Matrix3f rotation;
    rotation.col(0) = right;
    rotation.col(1) = new_up;
    rotation.col(2) = -forward;  // OpenGLスタイル（-Z前方）
    
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3,3>(0,0) = rotation;
    
    return this->transform(transform);
}

// ========== 明示的インスタンス化 ==========
template class FluentCloud<pcl::PointXYZ>;
template class FluentCloud<pcl::PointXYZRGB>;
template class FluentCloud<pcl::PointXYZI>;

} // namespace fluent_cloud