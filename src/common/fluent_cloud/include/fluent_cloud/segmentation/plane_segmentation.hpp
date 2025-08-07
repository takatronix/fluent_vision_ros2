#ifndef FLUENT_CLOUD_SEGMENTATION_PLANE_SEGMENTATION_HPP
#define FLUENT_CLOUD_SEGMENTATION_PLANE_SEGMENTATION_HPP

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fluent_cloud {
namespace segmentation {

template<typename PointT>
class PlaneSegmentation {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    
    struct PlaneResult {
        PointCloudPtr inliers;      // 平面上の点
        PointCloudPtr outliers;     // 平面外の点
        pcl::ModelCoefficients::Ptr coefficients;  // 平面パラメータ ax+by+cz+d=0
        float confidence;           // 信頼度（インライア率）
        Eigen::Vector3f normal;     // 法線ベクトル
        float distance;             // 原点からの距離
    };
    
    PlaneSegmentation() 
        : method_(pcl::SAC_RANSAC),
          distance_threshold_(0.01),
          max_iterations_(100),
          probability_(0.99),
          optimize_coefficients_(true) {}
    
    PlaneSegmentation& setDistanceThreshold(float threshold) {
        distance_threshold_ = threshold;
        return *this;
    }
    
    PlaneSegmentation& setMaxIterations(int iterations) {
        max_iterations_ = iterations;
        return *this;
    }
    
    PlaneSegmentation& setMethod(int method) {
        method_ = method;
        return *this;
    }
    
    PlaneSegmentation& setProbability(float prob) {
        probability_ = prob;
        return *this;
    }
    
    // 単一平面検出
    PlaneResult segment(const PointCloudPtr& input) {
        PlaneResult result;
        
        // セグメンテーション設定
        pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        result.coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
        
        seg.setOptimizeCoefficients(optimize_coefficients_);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(method_);
        seg.setDistanceThreshold(distance_threshold_);
        seg.setMaxIterations(max_iterations_);
        seg.setProbability(probability_);
        seg.setInputCloud(input);
        
        // セグメンテーション実行
        seg.segment(*inliers, *result.coefficients);
        
        // 結果の抽出
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(input);
        extract.setIndices(inliers);
        
        // インライア（平面上の点）
        result.inliers = PointCloudPtr(new PointCloud);
        extract.setNegative(false);
        extract.filter(*result.inliers);
        
        // アウトライア（平面外の点）
        result.outliers = PointCloudPtr(new PointCloud);
        extract.setNegative(true);
        extract.filter(*result.outliers);
        
        // 信頼度計算
        result.confidence = static_cast<float>(inliers->indices.size()) / input->size();
        
        // 法線と距離
        if (result.coefficients->values.size() >= 4) {
            float a = result.coefficients->values[0];
            float b = result.coefficients->values[1];
            float c = result.coefficients->values[2];
            float d = result.coefficients->values[3];
            
            result.normal = Eigen::Vector3f(a, b, c);
            result.normal.normalize();
            result.distance = std::abs(d);
        }
        
        return result;
    }
    
    // 複数平面検出
    std::vector<PlaneResult> segmentMultiple(const PointCloudPtr& input, 
                                            int max_planes = 5,
                                            float min_percentage = 0.05) {
        std::vector<PlaneResult> planes;
        PointCloudPtr remaining = input;
        
        for (int i = 0; i < max_planes && remaining->size() > input->size() * min_percentage; ++i) {
            PlaneResult plane = segment(remaining);
            
            if (plane.inliers->size() < input->size() * min_percentage) {
                break;  // 小さすぎる平面は無視
            }
            
            planes.push_back(plane);
            remaining = plane.outliers;
        }
        
        return planes;
    }
    
    // 地面検出専用（Z軸に対して水平な平面）
    PlaneResult segmentGround(const PointCloudPtr& input, float angle_threshold = 30.0) {
        PlaneResult result = segment(input);
        
        // Z軸との角度チェック
        if (result.coefficients->values.size() >= 3) {
            Eigen::Vector3f z_axis(0, 0, 1);
            float angle = std::acos(std::abs(result.normal.dot(z_axis))) * 180.0 / M_PI;
            
            if (angle > angle_threshold) {
                // 地面ではない
                result.inliers->clear();
                result.outliers = input;
                result.confidence = 0.0;
            }
        }
        
        return result;
    }
    
private:
    int method_;
    float distance_threshold_;
    int max_iterations_;
    float probability_;
    bool optimize_coefficients_;
};

} // namespace segmentation
} // namespace fluent_cloud

#endif // FLUENT_CLOUD_SEGMENTATION_PLANE_SEGMENTATION_HPP