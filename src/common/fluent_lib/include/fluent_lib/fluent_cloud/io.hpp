#pragma once

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include "fluent_lib/fluent_image/image.hpp"

namespace fluent_cloud::io {

// Placeholder for a stable DepthToCloud facade.
// Keep signature compatible with existing usage sites that include depth_to_cloud.hpp
struct DepthToCloud {
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertAsparagusROI(
        const cv::Mat &depth_mat,
        const cv::Mat &color_mat,
        const cv::Rect &roi,
        const sensor_msgs::msg::CameraInfo &camera_info,
        float depth_scale_m)
    {
        using Cloud = pcl::PointCloud<pcl::PointXYZRGB>;
        Cloud::Ptr cloud(new Cloud);
        if (depth_mat.empty() || roi.width<=0 || roi.height<=0) return cloud;

        // Camera intrinsics
        const double fx = camera_info.k[0];
        const double fy = camera_info.k[4];
        const double cx = camera_info.k[2];
        const double cy = camera_info.k[5];
        if (fx <= 0.0 || fy <= 0.0) return cloud;

        const int x0 = std::max(0, roi.x);
        const int y0 = std::max(0, roi.y);
        const int x1 = std::min(depth_mat.cols, roi.x + roi.width);
        const int y1 = std::min(depth_mat.rows, roi.y + roi.height);
        if (x0>=x1 || y0>=y1) return cloud;

        // カラーサイズが異なる場合は深度サイズに合わせてリサイズ（最近傍）
        cv::Mat color_aligned;
        if (!color_mat.empty() && (color_mat.size() != depth_mat.size())) {
            cv::resize(color_mat, color_aligned, depth_mat.size(), 0, 0, cv::INTER_NEAREST);
        } else {
            color_aligned = color_mat;
        }
        const bool has_color = !color_aligned.empty();
        cloud->points.reserve((x1-x0)*(y1-y0)/2);

        if (depth_mat.type()==CV_16UC1) {
            for (int v=y0; v<y1; ++v) {
                const uint16_t* depth_row = depth_mat.ptr<uint16_t>(v);
                for (int u=x0; u<x1; ++u) {
                    uint16_t dmm = depth_row[u]; if (!dmm) continue;
                    float z = static_cast<float>(dmm) * depth_scale_m; if (z<=0) continue;
                    pcl::PointXYZRGB p; p.z = z;
                    p.x = static_cast<float>((u - cx) * z / fx);
                    p.y = static_cast<float>((v - cy) * z / fy);
                    if (has_color) {
                        const cv::Vec3b& c = color_aligned.at<cv::Vec3b>(v,u);
                        p.b = c[0]; p.g = c[1]; p.r = c[2];
                    }
                    cloud->points.push_back(p);
                }
            }
        } else if (depth_mat.type()==CV_32FC1) {
            for (int v=y0; v<y1; ++v) {
                const float* depth_row = depth_mat.ptr<float>(v);
                for (int u=x0; u<x1; ++u) {
                    float z = depth_row[u]; if (!std::isfinite(z) || z<=0) continue;
                    pcl::PointXYZRGB p; p.z = z;
                    p.x = static_cast<float>((u - cx) * z / fx);
                    p.y = static_cast<float>((v - cy) * z / fy);
                    if (has_color) {
                        const cv::Vec3b& c = color_aligned.at<cv::Vec3b>(v,u);
                        p.b = c[0]; p.g = c[1]; p.r = c[2];
                    }
                    cloud->points.push_back(p);
                }
            }
        }

        cloud->width = static_cast<uint32_t>(cloud->points.size());
        cloud->height = 1; cloud->is_dense = false;
        return cloud;
    }

    // Overload: ROS Image から自動変換（深度は32F化、カラーはBGR8化）
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertAsparagusROI(
        const sensor_msgs::msg::Image &depth_msg,
        const sensor_msgs::msg::Image &color_msg,
        const cv::Rect &roi,
        const sensor_msgs::msg::CameraInfo &camera_info,
        float depth_unit_m)
    {
        fluent_image::Image depth_fi(depth_msg);
        fluent_image::Image color_fi(color_msg);
        auto depth32 = depth_fi.to_depth32f(depth_unit_m);
        auto color_bgr = color_fi.to_bgr8();
        return convertAsparagusROI(static_cast<cv::Mat&>(depth32), static_cast<cv::Mat&>(color_bgr), roi, camera_info, 1.0f);
    }

    // Overload: FluentImage から自動変換
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertAsparagusROI(
        const fluent_image::Image &depth_img,
        const fluent_image::Image &color_img,
        const cv::Rect &roi,
        const sensor_msgs::msg::CameraInfo &camera_info,
        float depth_unit_m)
    {
        auto depth32 = depth_img.to_depth32f(depth_unit_m);
        auto color_bgr = color_img.to_bgr8();
        return convertAsparagusROI(static_cast<cv::Mat&>(depth32), static_cast<cv::Mat&>(color_bgr), roi, camera_info, 1.0f);
    }
};

} // namespace fluent_cloud::io



