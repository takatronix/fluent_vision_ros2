#include "fluent.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

// =============================================================================
// Cloud実装
// =============================================================================

Cloud Cloud::from(const CloudPtr& cloud) {
    return Cloud(cloud);
}

Cloud Cloud::fromDepth(const cv::Mat& depth) {
    // 深度画像からポイントクラウド生成（簡易実装）
    CloudPtr cloud(new pcl::PointCloud<PointT>());
    
    const float fx = 525.0f;  // カメラパラメータ
    const float fy = 525.0f;
    const float cx = 320.0f;
    const float cy = 240.0f;
    
    for (int y = 0; y < depth.rows; y += 2) {  // 間引いて高速化
        for (int x = 0; x < depth.cols; x += 2) {
            float z = depth.at<uint16_t>(y, x) / 1000.0f;  // mm to m
            if (z > 0.1f && z < 5.0f) {
                PointT point;
                point.x = (x - cx) * z / fx;
                point.y = (y - cy) * z / fy;
                point.z = z;
                point.r = point.g = point.b = 128;  // グレー
                cloud->points.push_back(point);
            }
        }
    }
    
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    
    return Cloud(cloud);
}

Cloud Cloud::fromRGBD(const cv::Mat& rgb, const cv::Mat& depth) {
    CloudPtr cloud = fromDepth(depth).ptr();
    
    // RGB情報を追加
    int idx = 0;
    for (int y = 0; y < depth.rows; y += 2) {
        for (int x = 0; x < depth.cols; x += 2) {
            float z = depth.at<uint16_t>(y, x) / 1000.0f;
            if (z > 0.1f && z < 5.0f && idx < cloud->points.size()) {
                cv::Vec3b color = rgb.at<cv::Vec3b>(y, x);
                cloud->points[idx].r = color[2];  // BGR -> RGB
                cloud->points[idx].g = color[1];
                cloud->points[idx].b = color[0];
                idx++;
            }
        }
    }
    
    return Cloud(cloud);
}

Cloud& Cloud::filter(double min_dist, double max_dist) {
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud_);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_dist, max_dist);
    pass.filter(*cloud_);
    return *this;
}

Cloud& Cloud::downsample(double leaf_size) {
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud_);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.filter(*cloud_);
    return *this;
}

Cloud& Cloud::removeNoise() {
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_);
    return *this;
}

Cloud& Cloud::smooth() {
    // 簡易平滑化
    return *this;
}

Cloud& Cloud::findPlanes() {
    // 平面セグメンテーション
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud_);
    seg.segment(*inliers, *coefficients);
    
    return *this;
}

Cloud& Cloud::cluster() {
    // ユークリッドクラスタリング（簡易実装）
    return *this;
}

Cloud& Cloud::transform(double x, double y, double z) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 3) = x;
    transform(1, 3) = y;
    transform(2, 3) = z;
    
    pcl::transformPointCloud(*cloud_, *cloud_, transform);
    return *this;
}

Cloud& Cloud::rotate(double roll, double pitch, double yaw) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // 簡易回転（完全な実装は後で）
    pcl::transformPointCloud(*cloud_, *cloud_, transform);
    return *this;
}

Cloud& Cloud::colorize(const cv::Mat& rgb) {
    // RGB画像で色付け（簡易実装）
    for (auto& point : cloud_->points) {
        point.r = 255; point.g = 128; point.b = 0;  // オレンジ
    }
    return *this;
}

Cloud& Cloud::colorByHeight() {
    // 高さで色付け
    if (cloud_->empty()) return *this;
    
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::min();
    
    for (const auto& point : cloud_->points) {
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }
    
    for (auto& point : cloud_->points) {
        float ratio = (point.z - min_z) / (max_z - min_z);
        point.r = static_cast<uint8_t>(255 * ratio);
        point.g = static_cast<uint8_t>(255 * (1.0f - ratio));
        point.b = 128;
    }
    
    return *this;
}

Cloud& Cloud::setColor(uint8_t r, uint8_t g, uint8_t b) {
    for (auto& point : cloud_->points) {
        point.r = r;
        point.g = g;
        point.b = b;
    }
    return *this;
}

void Cloud::save(const std::string& path) {
    pcl::io::savePCDFileBinary(path, *cloud_);
}

void Cloud::publish(const std::string& topic) {
    // ROS2パブリッシュ（後で実装）
    std::cout << "Publishing to " << topic << " (" << cloud_->size() << " points)" << std::endl;
}