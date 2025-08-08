#ifndef FLUENT_CLOUD_ASPARAGUS_HPP
#define FLUENT_CLOUD_ASPARAGUS_HPP

#include <fluent_cloud/fluent_cloud.hpp>
#include <fluent_cloud/segmentation/asparagus_segmentation.hpp>
#include <fluent_cloud/features/asparagus_analyzer.hpp>

namespace fluent_cloud {

/**
 * @brief FluentCloudのアスパラガス専用拡張
 * 美しいAPIでアスパラガス処理を簡単に
 */
template<typename PointT>
class FluentCloud {
public:
    // ... 既存のメソッド ...
    
    /**
     * @brief アスパラガスセグメンテーション
     * @param mask セグメンテーションマスク（optional）
     * @param camera_info カメラ情報（optional）
     */
    FluentCloud& extractAsparagus(const cv::Mat& mask = cv::Mat(),
                                  const sensor_msgs::msg::CameraInfo* camera_info = nullptr) {
        segmentation::AsparagusSegmentation<PointT> seg;
        
        if (!mask.empty() && camera_info) {
            // マスクベースセグメンテーション
            auto result = seg.segmentWithMask(cloud_, mask, *camera_info);
            cloud_ = result.asparagus_cloud;
        } else {
            // 色ベースセグメンテーション
            auto result = seg.segmentByColor(cloud_);
            cloud_ = result.asparagus_cloud;
        }
        
        return *this;
    }
    
    /**
     * @brief アスパラガス解析
     * @return アスパラガスの特徴量
     */
    features::AsparagusAnalyzer<PointT>::AsparaFeatures analyzeAsparagus() {
        features::AsparagusAnalyzer<PointT> analyzer;
        return analyzer.analyze(cloud_);
    }
    
    /**
     * @brief 収穫可能なアスパラガスのみをフィルタ
     */
    FluentCloud& filterHarvestable(float min_length = 0.23, float max_length = 0.50) {
        features::AsparagusAnalyzer<PointT> analyzer;
        analyzer.setHarvestLengthRange(min_length, max_length);
        
        auto features = analyzer.analyze(cloud_);
        
        if (!features.harvestable) {
            cloud_->clear();  // 収穫不可なら空にする
        }
        
        return *this;
    }
    
    /**
     * @brief アスパラガスの中心線を抽出
     */
    std::vector<Eigen::Vector3f> extractAsparagusCenterline() {
        features::AsparagusAnalyzer<PointT> analyzer;
        auto features = analyzer.analyze(cloud_);
        return features.centerline;
    }
    
    /**
     * @brief アスパラガスを真っ直ぐに補正
     */
    FluentCloud& straightenAsparagus() {
        auto centerline = extractAsparagusCenterline();
        if (centerline.size() < 2) return *this;
        
        // 主軸を垂直に回転
        Eigen::Vector3f start = centerline.front();
        Eigen::Vector3f end = centerline.back();
        Eigen::Vector3f axis = (end - start).normalized();
        
        // Z軸に揃える回転を計算
        Eigen::Vector3f z_axis(0, 0, 1);
        Eigen::Vector3f rotation_axis = axis.cross(z_axis);
        float angle = std::acos(axis.dot(z_axis));
        
        if (rotation_axis.norm() > 0.001) {
            rotateAroundAxis(rotation_axis, -angle);
        }
        
        return *this;
    }
};

// ========== 使用例 ==========
/*
// 1. YOLOマスクを使った高精度抽出
auto asparagus = FluentCloud::from(pointcloud_msg)
    .filterByDistance(0.1, 2.0)         // 10cm-2mの範囲
    .extractAsparagus(yolo_mask, camera_info)  // アスパラガス抽出
    .removeOutliers(50, 1.0)            // ノイズ除去
    .downsample(0.005);                 // 5mmボクセル

// 2. アスパラガス解析
auto features = asparagus.analyzeAsparagus();
if (features.harvestable) {
    std::cout << "収穫可能: 長さ " << features.length * 100 << "cm" << std::endl;
    std::cout << "真っ直ぐ度: " << features.straightness * 100 << "%" << std::endl;
}

// 3. 色だけで簡易抽出
auto green_objects = FluentCloud::from(cloud)
    .extractAsparagus()  // 緑色抽出
    .cluster(0.02)       // クラスタリング
    .forEachCluster([](FluentCloud& cluster) {
        auto features = cluster.analyzeAsparagus();
        if (features.harvestable) {
            cluster.colorize(0, 255, 0);  // 収穫可能は明るい緑
        } else {
            cluster.colorize(255, 0, 0);  // 収穫不可は赤
        }
    });

// 4. リアルタイム処理パイプライン
FluentCloud::stream("/fv/d415/depth/color/points")
    .extractAsparagus()
    .filterHarvestable()
    .straightenAsparagus()  // 表示用に真っ直ぐに
    .publish("/asparagus/harvestable");
*/

} // namespace fluent_cloud

#endif // FLUENT_CLOUD_ASPARAGUS_HPP