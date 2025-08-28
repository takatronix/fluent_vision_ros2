#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

/**
 * @brief Beautiful PointCloud processing API
 */
class Cloud {
public:
    using PointT = pcl::PointXYZRGB;
    using CloudPtr = pcl::PointCloud<PointT>::Ptr;
    
    // ファクトリ
    static Cloud from(const CloudPtr& cloud);
    static Cloud fromDepth(const cv::Mat& depth);
    static Cloud fromRGBD(const cv::Mat& rgb, const cv::Mat& depth);
    
    // フィルタ
    Cloud& filter(double min_dist = 0.1, double max_dist = 5.0);
    Cloud& downsample(double leaf_size = 0.01);
    Cloud& removeNoise();
    Cloud& smooth();
    
    // セグメンテーション
    Cloud& findPlanes();
    Cloud& cluster();
    
    // 変換
    Cloud& transform(double x, double y, double z);
    Cloud& rotate(double roll, double pitch, double yaw);
    
    // 色付け
    Cloud& colorize(const cv::Mat& rgb);
    Cloud& colorByHeight();
    Cloud& setColor(uint8_t r, uint8_t g, uint8_t b);
    
    // 出力
    void save(const std::string& path);
    void publish(const std::string& topic);
    CloudPtr ptr() { return cloud_; }
    
private:
    CloudPtr cloud_;
    explicit Cloud(const CloudPtr& cloud) : cloud_(cloud) {}
};