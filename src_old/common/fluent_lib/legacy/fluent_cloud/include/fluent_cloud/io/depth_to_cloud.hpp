#ifndef FLUENT_CLOUD_IO_DEPTH_TO_CLOUD_HPP
#define FLUENT_CLOUD_IO_DEPTH_TO_CLOUD_HPP

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/camera_info.hpp>

namespace fluent_cloud {
namespace io {

/**
 * @brief 深度画像から部分的にポイントクラウドを生成（超軽量API）
 */
class DepthToCloud {
public:
    using PointCloudRGB = pcl::PointCloud<pcl::PointXYZRGB>;
    using PointCloudRGBPtr = PointCloudRGB::Ptr;
    
    /**
     * @brief ROI指定で部分的に3D化（最速）
     * @param depth_image 深度画像（CV_16UC1 or CV_32FC1）
     * @param color_image カラー画像（CV_8UC3）
     * @param roi 関心領域（この部分だけ3D化）
     * @param camera_info カメラパラメータ
     * @return 部分点群
     */
    static PointCloudRGBPtr convertROI(
        const cv::Mat& depth_image,
        const cv::Mat& color_image,
        const cv::Rect& roi,
        const sensor_msgs::msg::CameraInfo& camera_info,
        int skip = 1,  // ピクセルスキップ（1=全部、2=半分、4=1/4）
        float depth_scale_m_per_unit = 0.001f)  // 16UC1時の単位変換係数（m/units）
    {
        PointCloudRGBPtr cloud(new PointCloudRGB);
        
        // カメラパラメータ
        float fx = camera_info.k[0];
        float fy = camera_info.k[4];
        float cx = camera_info.k[2];
        float cy = camera_info.k[5];
        
        // 深度スケール（RealSense既定: 0.001。D405は機種により0.0001）
        const float depth_scale = depth_scale_m_per_unit;
        
        // ROI範囲チェック
        cv::Rect safe_roi = roi & cv::Rect(0, 0, depth_image.cols, depth_image.rows);
        
        // 効率的な事前メモリ確保
        cloud->reserve((safe_roi.width / skip) * (safe_roi.height / skip));
        
        // ROI内のみ処理
        for (int v = safe_roi.y; v < safe_roi.y + safe_roi.height; v += skip) {
            for (int u = safe_roi.x; u < safe_roi.x + safe_roi.width; u += skip) {
                float depth_raw = getDepthValue(depth_image, u, v);
                
                // 深度画像の型に応じて単位を統一（m単位に）
                float depth;
                if (depth_image.type() == CV_16UC1) {
                    depth = depth_raw * depth_scale;  // units -> m
                } else {
                    depth = depth_raw;  // 既にm単位
                }
                
                if (depth > 0.1f && depth < 3.0f) {  // 有効範囲
                    pcl::PointXYZRGB point;
                    
                    // 3D座標（カメラ座標系）
                    point.z = depth;
                    point.x = (u - cx) * depth / fx;
                    point.y = (v - cy) * depth / fy;
                    
                    // 色
                    cv::Vec3b bgr = color_image.at<cv::Vec3b>(v, u);
                    // RGB値をパックする（Foxglove仕様：BGRパック、アルファチャンネル付き）
                    uint8_t r = bgr[2];
                    uint8_t g = bgr[1];
                    uint8_t b = bgr[0];
                    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                    point.rgb = *reinterpret_cast<float*>(&rgb);
                    
                    cloud->push_back(point);
                }
            }
        }
        
        cloud->width = cloud->size();
        cloud->height = 1;
        cloud->is_dense = false;
        
        return cloud;
    }
    
    /**
     * @brief マスク画像で指定された部分のみ3D化（効率的）
     */
    static PointCloudRGBPtr convertMasked(
        const cv::Mat& depth_image,
        const cv::Mat& color_image,
        const cv::Mat& mask,  // 0/255のマスク画像
        const sensor_msgs::msg::CameraInfo& camera_info,
        int skip = 1,
        float depth_scale_m_per_unit = 0.001f)
    {
        PointCloudRGBPtr cloud(new PointCloudRGB);
        
        float fx = camera_info.k[0];
        float fy = camera_info.k[4];
        float cx = camera_info.k[2];
        float cy = camera_info.k[5];
        const float depth_scale = depth_scale_m_per_unit;
        
        // マスクの有効ピクセル数を事前カウント（メモリ最適化）
        int valid_pixels = cv::countNonZero(mask) / (skip * skip);
        cloud->reserve(valid_pixels);
        
        for (int v = 0; v < depth_image.rows; v += skip) {
            for (int u = 0; u < depth_image.cols; u += skip) {
                // マスクチェック（高速）
                if (mask.at<uchar>(v, u) == 0) continue;
                
                float depth_raw = getDepthValue(depth_image, u, v);
                
                // 深度画像の型に応じて単位を統一（m単位に）
                float depth;
                if (depth_image.type() == CV_16UC1) {
                    depth = depth_raw * depth_scale;  // units -> m
                } else {
                    depth = depth_raw;  // 既にm単位
                }
                
                if (depth > 0.1f && depth < 3.0f) {
                    pcl::PointXYZRGB point;
                    
                    point.z = depth;
                    point.x = (u - cx) * depth / fx;
                    point.y = (v - cy) * depth / fy;
                    
                    cv::Vec3b bgr = color_image.at<cv::Vec3b>(v, u);
                    // RGB値をパックする（Foxglove仕様：BGRパック、アルファチャンネル付き）
                    uint8_t r = bgr[2];
                    uint8_t g = bgr[1];
                    uint8_t b = bgr[0];
                    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                    point.rgb = *reinterpret_cast<float*>(&rgb);
                    
                    cloud->push_back(point);
                }
            }
        }
        
        cloud->width = cloud->size();
        cloud->height = 1;
        cloud->is_dense = false;
        
        return cloud;
    }
    
    /**
     * @brief アスパラガス専用：縦長ROIの効率的な3D化
     */
    static PointCloudRGBPtr convertAsparagusROI(
        const cv::Mat& depth_image,
        const cv::Mat& color_image,
        const cv::Rect& bbox,  // YOLOのバウンディングボックス
        const sensor_msgs::msg::CameraInfo& camera_info,
        float depth_scale_m_per_unit = 0.001f,
        float min_depth_m = 0.2f,
        float max_depth_m = 1.5f)
    {
        // アスパラガスは縦長なので、横方向は粗く、縦方向は細かく
        PointCloudRGBPtr cloud(new PointCloudRGB);
        
        float fx = camera_info.k[0];
        float fy = camera_info.k[4];
        float cx = camera_info.k[2];
        float cy = camera_info.k[5];
        
        // バウンディングボックスを少し拡張（10%）
        cv::Rect roi = bbox;
        roi.x -= bbox.width * 0.1;
        roi.y -= bbox.height * 0.05;
        roi.width *= 1.2;
        roi.height *= 1.1;
        
        // 範囲チェック
        roi &= cv::Rect(0, 0, depth_image.cols, depth_image.rows);
        
        // アスパラガス用最適化：横は粗く（3ピクセル）、縦は細かく（1ピクセル）
        int u_skip = 3;
        int v_skip = 1;
        
        cloud->reserve((roi.width / u_skip) * (roi.height / v_skip));
        
        // デバッグ用：最初のピクセルの深度値を確認
        static bool debug_printed = false;
        
        for (int v = roi.y; v < roi.y + roi.height; v += v_skip) {
            for (int u = roi.x; u < roi.x + roi.width; u += u_skip) {
                float depth_raw = getDepthValue(depth_image, u, v);
                
                // 深度画像の型に応じて単位を統一（m単位に）
                float depth;
                if (depth_image.type() == CV_16UC1) {
                    depth = depth_raw * depth_scale_m_per_unit;  // units -> m
                } else if (depth_image.type() == CV_32FC1) {
                    depth = depth_raw;  // 既にm単位
                } else {
                    continue;
                }
                
                // デバッグ：最初の有効な深度値を表示
                if (!debug_printed && depth > 0) {
                    printf("[DEBUG] Depth value: raw=%f, converted to m=%f\n", depth_raw, depth);
                    debug_printed = true;
                }
                
                // 指定距離範囲
                if (depth > min_depth_m && depth < max_depth_m) {
                    pcl::PointXYZRGB point;
                    
                    point.z = depth;
                    point.x = (u - cx) * depth / fx;
                    point.y = (v - cy) * depth / fy;
                    
                    cv::Vec3b bgr = color_image.at<cv::Vec3b>(v, u);
                    // RGB値をパックする（Foxglove仕様：BGRパック、アルファチャンネル付き）
                    uint8_t r = bgr[2];
                    uint8_t g = bgr[1];
                    uint8_t b = bgr[0];
                    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                    point.rgb = *reinterpret_cast<float*>(&rgb);
                    
                    cloud->push_back(point);
                }
            }
        }
        
        cloud->width = cloud->size();
        cloud->height = 1;
        cloud->is_dense = false;
        
        return cloud;
    }
    
private:
    /**
     * @brief 深度値取得（型に対応）
     * @note 16UC1はmm単位、32FC1はm単位で返す
     */
    static float getDepthValue(const cv::Mat& depth, int u, int v) {
        if (depth.type() == CV_16UC1) {
            return depth.at<uint16_t>(v, u);  // mm単位（RealSense標準）
        } else if (depth.type() == CV_32FC1) {
            return depth.at<float>(v, u);  // m単位のまま返す
        }
        return 0;
    }
};

} // namespace io
} // namespace fluent_cloud

#endif // FLUENT_CLOUD_IO_DEPTH_TO_CLOUD_HPP