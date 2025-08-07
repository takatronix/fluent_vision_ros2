#ifndef FLUENT_CLOUD_FILTERS_ADAPTIVE_DISTANCE_FILTER_HPP
#define FLUENT_CLOUD_FILTERS_ADAPTIVE_DISTANCE_FILTER_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

namespace fluent_cloud {
namespace filters {

/**
 * @brief 距離に応じた適応的フィルタリング
 * アスパラガスの見え方は距離によって変わるため、それに対応
 */
template<typename PointT = pcl::PointXYZRGB>
class AdaptiveDistanceFilter {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    
    /**
     * @brief 距離に応じた解像度設定
     */
    struct DistanceProfile {
        float min_distance;      // 最小距離
        float max_distance;      // 最大距離
        int pixel_skip;          // ピクセルスキップ（1=全部、2=半分）
        float voxel_size;        // ボクセルサイズ
        float min_aspara_width;  // この距離でのアスパラ最小幅（ピクセル）
        float confidence_weight; // 信頼度の重み
    };
    
    AdaptiveDistanceFilter() {
        // デフォルトの距離プロファイル設定
        setupDefaultProfiles();
    }
    
    /**
     * @brief バウンディングボックスサイズから距離を推定
     */
    float estimateDistance(const cv::Rect& bbox, float focal_length) {
        // アスパラガスの典型的な太さ: 15mm (0.015m)
        const float ASPARAGUS_WIDTH = 0.015f;
        
        // ピンホールカメラモデル: width_pixels = (focal_length * width_meters) / distance
        // 逆算: distance = (focal_length * width_meters) / width_pixels
        float estimated_distance = (focal_length * ASPARAGUS_WIDTH) / bbox.width;
        
        return estimated_distance;
    }
    
    /**
     * @brief 距離に応じた適応的な3D変換
     */
    PointCloudPtr convertAdaptive(
        const cv::Mat& depth_image,
        const cv::Mat& color_image,
        const cv::Rect& bbox,
        const sensor_msgs::msg::CameraInfo& camera_info) {
        
        // 距離推定
        float estimated_distance = estimateDistance(bbox, camera_info.k[0]);
        
        // 適切なプロファイル選択
        DistanceProfile profile = selectProfile(estimated_distance);
        
        RCLCPP_DEBUG(rclcpp::get_logger("adaptive_filter"),
            "推定距離: %.2fm, スキップ: %d, ボクセル: %.3fm",
            estimated_distance, profile.pixel_skip, profile.voxel_size);
        
        // プロファイルに基づいて3D変換
        return convertWithProfile(depth_image, color_image, bbox, 
                                 camera_info, profile, estimated_distance);
    }
    
private:
    std::vector<DistanceProfile> profiles_;
    
    void setupDefaultProfiles() {
        // 近距離（20-50cm）: 高精度
        profiles_.push_back({
            0.2f, 0.5f,      // 距離範囲
            1,               // 全ピクセル処理
            0.002f,          // 2mmボクセル
            30.0f,           // 最小30ピクセル幅
            1.0f             // 高信頼度
        });
        
        // 中距離（50cm-1m）: 標準精度
        profiles_.push_back({
            0.5f, 1.0f,      // 距離範囲
            2,               // 2ピクセルおき
            0.005f,          // 5mmボクセル
            15.0f,           // 最小15ピクセル幅
            0.8f             // 中信頼度
        });
        
        // 遠距離（1m-2m）: 低精度
        profiles_.push_back({
            1.0f, 2.0f,      // 距離範囲
            3,               // 3ピクセルおき
            0.01f,           // 10mmボクセル
            8.0f,            // 最小8ピクセル幅
            0.5f             // 低信頼度
        });
        
        // 超遠距離（2m以上）: 最低精度
        profiles_.push_back({
            2.0f, 5.0f,      // 距離範囲
            4,               // 4ピクセルおき
            0.02f,           // 20mmボクセル
            5.0f,            // 最小5ピクセル幅
            0.3f             // 最低信頼度
        });
    }
    
    DistanceProfile selectProfile(float distance) {
        for (const auto& profile : profiles_) {
            if (distance >= profile.min_distance && distance < profile.max_distance) {
                return profile;
            }
        }
        // デフォルトは最後のプロファイル
        return profiles_.back();
    }
    
    PointCloudPtr convertWithProfile(
        const cv::Mat& depth_image,
        const cv::Mat& color_image,
        const cv::Rect& bbox,
        const sensor_msgs::msg::CameraInfo& camera_info,
        const DistanceProfile& profile,
        float estimated_distance) {
        
        PointCloudPtr cloud(new PointCloud);
        
        // カメラパラメータ
        float fx = camera_info.k[0];
        float fy = camera_info.k[4];
        float cx = camera_info.k[2];
        float cy = camera_info.k[5];
        
        // バウンディングボックスの拡張（距離に応じて）
        int margin = static_cast<int>(10.0f * profile.confidence_weight);
        cv::Rect expanded = bbox;
        expanded.x = std::max(0, bbox.x - margin);
        expanded.y = std::max(0, bbox.y - margin);
        expanded.width = std::min(depth_image.cols - expanded.x, bbox.width + 2 * margin);
        expanded.height = std::min(depth_image.rows - expanded.y, bbox.height + 2 * margin);
        
        // 距離に応じた処理
        int valid_points = 0;
        int filtered_by_distance = 0;
        
        for (int v = expanded.y; v < expanded.y + expanded.height; v += profile.pixel_skip) {
            for (int u = expanded.x; u < expanded.x + expanded.width; u += profile.pixel_skip) {
                float depth = getDepthValue(depth_image, u, v) * 0.001f;  // mm to m
                
                // 距離フィルタ（推定距離の±30%）
                float min_depth = estimated_distance * 0.7f;
                float max_depth = estimated_distance * 1.3f;
                
                if (depth < min_depth || depth > max_depth) {
                    filtered_by_distance++;
                    continue;
                }
                
                // 有効な点
                if (depth > 0.1f && depth < 5.0f) {
                    pcl::PointXYZRGB point;
                    
                    point.z = depth;
                    point.x = (u - cx) * depth / fx;
                    point.y = (v - cy) * depth / fy;
                    
                    cv::Vec3b color = color_image.at<cv::Vec3b>(v, u);
                    point.b = color[0];
                    point.g = color[1];
                    point.r = color[2];
                    
                    cloud->push_back(point);
                    valid_points++;
                }
            }
        }
        
        // ボクセルグリッドでダウンサンプリング
        if (cloud->size() > 1000 && profile.voxel_size > 0) {
            pcl::VoxelGrid<pcl::PointXYZRGB> vg;
            vg.setInputCloud(cloud);
            vg.setLeafSize(profile.voxel_size, profile.voxel_size, profile.voxel_size);
            
            PointCloudPtr downsampled(new PointCloud);
            vg.filter(*downsampled);
            cloud = downsampled;
        }
        
        cloud->width = cloud->size();
        cloud->height = 1;
        cloud->is_dense = false;
        
        RCLCPP_DEBUG(rclcpp::get_logger("adaptive_filter"),
            "距離フィルタ結果: %d点有効, %d点除外（推定距離%.2fm±30%%）",
            valid_points, filtered_by_distance, estimated_distance);
        
        return cloud;
    }
    
    float getDepthValue(const cv::Mat& depth, int u, int v) {
        if (depth.type() == CV_16UC1) {
            return depth.at<uint16_t>(v, u);
        } else if (depth.type() == CV_32FC1) {
            return depth.at<float>(v, u) * 1000.0f;
        }
        return 0;
    }
};

// ========== 使用例 ==========
/*
AdaptiveDistanceFilter<pcl::PointXYZRGB> filter;

// YOLOの検出結果から
for (const auto& detection : detections) {
    cv::Rect bbox = getBoundingBox(detection);
    
    // 距離に応じて自動的に最適な解像度で3D化
    auto cloud = filter.convertAdaptive(depth, color, bbox, camera_info);
    
    // 結果：
    // - 近いアスパラ（50cm）: 高解像度、2mmボクセル
    // - 中距離（1m）: 中解像度、5mmボクセル  
    // - 遠い（2m）: 低解像度、10mmボクセル
}
*/

} // namespace filters
} // namespace fluent_cloud

#endif // FLUENT_CLOUD_FILTERS_ADAPTIVE_DISTANCE_FILTER_HPP