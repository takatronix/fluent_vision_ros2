#ifndef FLUENT_CLOUD_FEATURES_ASPARAGUS_ANALYZER_HPP
#define FLUENT_CLOUD_FEATURES_ASPARAGUS_ANALYZER_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Core>

namespace fluent_cloud {
namespace features {

/**
 * @brief アスパラガス解析専用クラス
 * FluentCloudの美しいAPIでアスパラガスを正確に解析
 */
template<typename PointT = pcl::PointXYZRGB>
class AsparagusAnalyzer {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    
    struct AsparaFeatures {
        // 基本形状
        float length;              // 実際の曲線長
        float straight_length;     // 直線距離
        float diameter;            // 平均直径
        float volume;              // 体積推定値
        
        // 品質指標
        float straightness;        // 真っ直ぐ度（0-1）
        float uniformity;          // 太さの均一性（0-1）
        float curvature_score;     // 曲率スコア（低いほど真っ直ぐ）
        
        // 収穫適性
        bool harvestable;          // 収穫可能か
        std::string harvest_reason; // 判定理由
        
        // 位置情報
        Eigen::Vector3f root_position;    // 根元位置
        Eigen::Vector3f tip_position;     // 先端位置
        Eigen::Vector3f growth_direction; // 成長方向ベクトル
        
        // 詳細形状
        std::vector<Eigen::Vector3f> centerline;  // 中心線
        std::vector<float> radius_profile;         // 各位置での半径
        std::vector<float> curvature_profile;      // 各位置での曲率
    };
    
    AsparagusAnalyzer() 
        : harvest_min_length_(0.23),
          harvest_max_length_(0.50),
          straightness_threshold_(0.7),
          diameter_min_(0.008),  // 8mm
          diameter_max_(0.025)   // 25mm
    {}
    
    // パラメータ設定
    AsparagusAnalyzer& setHarvestLengthRange(float min_len, float max_len) {
        harvest_min_length_ = min_len;
        harvest_max_length_ = max_len;
        return *this;
    }
    
    AsparagusAnalyzer& setStraightnessThreshold(float threshold) {
        straightness_threshold_ = threshold;
        return *this;
    }
    
    AsparagusAnalyzer& setDiameterRange(float min_dia, float max_dia) {
        diameter_min_ = min_dia;
        diameter_max_ = max_dia;
        return *this;
    }
    
    /**
     * @brief アスパラガスの完全解析
     */
    AsparaFeatures analyze(const PointCloudPtr& cloud) {
        AsparaFeatures features;
        
        if (cloud->points.size() < 10) {
            features.harvestable = false;
            features.harvest_reason = "点群が少なすぎます";
            return features;
        }
        
        // 1. 中心線抽出と骨格化
        extractCenterline(cloud, features);
        
        // 2. 長さ計算（曲線長）
        features.length = calculateCurveLength(features.centerline);
        features.straight_length = (features.tip_position - features.root_position).norm();
        
        // 3. 真っ直ぐ度の高精度計算
        features.straightness = calculateAdvancedStraightness(features);
        
        // 4. 直径プロファイル解析
        analyzeDiameterProfile(cloud, features);
        
        // 5. 曲率解析
        analyzeCurvature(features);
        
        // 6. 体積推定
        features.volume = estimateVolume(features);
        
        // 7. 収穫適性判定
        evaluateHarvestability(features);
        
        return features;
    }
    
private:
    float harvest_min_length_;
    float harvest_max_length_;
    float straightness_threshold_;
    float diameter_min_;
    float diameter_max_;
    
    /**
     * @brief 中心線抽出（骨格化）
     */
    void extractCenterline(const PointCloudPtr& cloud, AsparaFeatures& features) {
        // シンプルな実装：主軸に沿ってスライスして中心点を取得
        pcl::PCA<PointT> pca;
        pca.setInputCloud(cloud);
        
        Eigen::Vector3f eigenvalues = pca.getEigenValues();
        Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
        
        // 主軸（最大固有値の方向）
        Eigen::Vector3f main_axis = eigenvectors.col(0);
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);
        
        // 主軸に沿って投影
        std::map<float, std::vector<PointT>> slices;
        for (const auto& point : cloud->points) {
            Eigen::Vector3f p(point.x, point.y, point.z);
            Eigen::Vector3f centered = p - centroid.head<3>();
            float projection = centered.dot(main_axis);
            slices[projection].push_back(point);
        }
        
        // 各スライスの中心を計算
        features.centerline.clear();
        for (const auto& slice : slices) {
            Eigen::Vector3f center(0, 0, 0);
            for (const auto& point : slice.second) {
                center += Eigen::Vector3f(point.x, point.y, point.z);
            }
            center /= slice.second.size();
            features.centerline.push_back(center);
        }
        
        // 根元と先端を特定
        if (!features.centerline.empty()) {
            // Z座標が最小の点を根元とする
            auto root_it = std::min_element(features.centerline.begin(), features.centerline.end(),
                [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
                    return a.z() < b.z();
                });
            features.root_position = *root_it;
            
            // 根元から最も遠い点を先端とする
            float max_dist = 0;
            for (const auto& point : features.centerline) {
                float dist = (point - features.root_position).norm();
                if (dist > max_dist) {
                    max_dist = dist;
                    features.tip_position = point;
                }
            }
            
            features.growth_direction = (features.tip_position - features.root_position).normalized();
        }
    }
    
    /**
     * @brief 曲線長を計算
     */
    float calculateCurveLength(const std::vector<Eigen::Vector3f>& centerline) {
        if (centerline.size() < 2) return 0.0f;
        
        float total_length = 0.0f;
        for (size_t i = 1; i < centerline.size(); ++i) {
            total_length += (centerline[i] - centerline[i-1]).norm();
        }
        return total_length;
    }
    
    /**
     * @brief 高精度な真っ直ぐ度計算
     */
    float calculateAdvancedStraightness(const AsparaFeatures& features) {
        if (features.centerline.size() < 3) return 1.0f;
        
        // 方法1: 曲線長と直線長の比
        float length_ratio = features.straight_length / features.length;
        
        // 方法2: 中心線の各点から直線への距離の平均
        float avg_deviation = 0.0f;
        Eigen::Vector3f line_dir = features.growth_direction;
        Eigen::Vector3f line_point = features.root_position;
        
        for (const auto& point : features.centerline) {
            // 点から直線への距離
            Eigen::Vector3f to_point = point - line_point;
            float proj_length = to_point.dot(line_dir);
            Eigen::Vector3f proj_point = line_point + proj_length * line_dir;
            float deviation = (point - proj_point).norm();
            avg_deviation += deviation;
        }
        avg_deviation /= features.centerline.size();
        
        // 偏差を長さで正規化
        float deviation_score = 1.0f - std::min(1.0f, avg_deviation / features.length * 10);
        
        // 総合スコア
        return 0.7f * length_ratio + 0.3f * deviation_score;
    }
    
    /**
     * @brief 直径プロファイル解析
     */
    void analyzeDiameterProfile(const PointCloudPtr& cloud, AsparaFeatures& features) {
        features.radius_profile.clear();
        
        // 中心線の各点での半径を計算
        for (const auto& center : features.centerline) {
            float max_radius = 0.0f;
            
            // 近傍点を探索
            for (const auto& point : cloud->points) {
                Eigen::Vector3f p(point.x, point.y, point.z);
                
                // 中心線上の点への投影
                Eigen::Vector3f to_point = p - features.root_position;
                float proj_length = to_point.dot(features.growth_direction);
                Eigen::Vector3f proj_point = features.root_position + proj_length * features.growth_direction;
                
                // 投影点が現在の中心点に近い場合
                if ((proj_point - center).norm() < 0.01) {  // 1cm以内
                    float radius = (p - center).norm();
                    max_radius = std::max(max_radius, radius);
                }
            }
            
            features.radius_profile.push_back(max_radius);
        }
        
        // 平均直径と均一性を計算
        if (!features.radius_profile.empty()) {
            float sum = 0.0f;
            float min_r = FLT_MAX;
            float max_r = 0.0f;
            
            for (float r : features.radius_profile) {
                sum += r;
                min_r = std::min(min_r, r);
                max_r = std::max(max_r, r);
            }
            
            features.diameter = 2.0f * sum / features.radius_profile.size();
            features.uniformity = (max_r > 0) ? (1.0f - (max_r - min_r) / max_r) : 1.0f;
        }
    }
    
    /**
     * @brief 曲率解析
     */
    void analyzeCurvature(AsparaFeatures& features) {
        features.curvature_profile.clear();
        features.curvature_score = 0.0f;
        
        if (features.centerline.size() < 3) return;
        
        // 各点での曲率を計算
        for (size_t i = 1; i < features.centerline.size() - 1; ++i) {
            Eigen::Vector3f v1 = features.centerline[i] - features.centerline[i-1];
            Eigen::Vector3f v2 = features.centerline[i+1] - features.centerline[i];
            
            float angle = std::acos(std::min(1.0f, v1.normalized().dot(v2.normalized())));
            float curvature = angle / v1.norm();  // 簡易曲率
            
            features.curvature_profile.push_back(curvature);
            features.curvature_score += curvature;
        }
        
        features.curvature_score /= features.curvature_profile.size();
    }
    
    /**
     * @brief 体積推定
     */
    float estimateVolume(const AsparaFeatures& features) {
        float volume = 0.0f;
        
        if (features.centerline.size() < 2) return 0.0f;
        
        // 円柱の集合として体積を計算
        for (size_t i = 1; i < features.centerline.size(); ++i) {
            float segment_length = (features.centerline[i] - features.centerline[i-1]).norm();
            float avg_radius = (features.radius_profile[i] + features.radius_profile[i-1]) / 2.0f;
            volume += M_PI * avg_radius * avg_radius * segment_length;
        }
        
        return volume;
    }
    
    /**
     * @brief 収穫適性判定
     */
    void evaluateHarvestability(AsparaFeatures& features) {
        features.harvestable = true;
        features.harvest_reason = "収穫適期";
        
        // 長さチェック
        if (features.length < harvest_min_length_) {
            features.harvestable = false;
            features.harvest_reason = "短すぎます（" + std::to_string(int(features.length * 100)) + "cm）";
            return;
        }
        if (features.length > harvest_max_length_) {
            features.harvestable = false;
            features.harvest_reason = "長すぎます（" + std::to_string(int(features.length * 100)) + "cm）";
            return;
        }
        
        // 真っ直ぐ度チェック
        if (features.straightness < straightness_threshold_) {
            features.harvestable = false;
            features.harvest_reason = "曲がりすぎています（真っ直ぐ度: " + 
                                    std::to_string(int(features.straightness * 100)) + "%）";
            return;
        }
        
        // 直径チェック
        if (features.diameter < diameter_min_) {
            features.harvestable = false;
            features.harvest_reason = "細すぎます（" + std::to_string(int(features.diameter * 1000)) + "mm）";
            return;
        }
        if (features.diameter > diameter_max_) {
            features.harvestable = false;
            features.harvest_reason = "太すぎます（" + std::to_string(int(features.diameter * 1000)) + "mm）";
            return;
        }
        
        // 均一性チェック
        if (features.uniformity < 0.6) {
            features.harvestable = false;
            features.harvest_reason = "太さが不均一です";
            return;
        }
    }
};

} // namespace features
} // namespace fluent_cloud

#endif // FLUENT_CLOUD_FEATURES_ASPARAGUS_ANALYZER_HPP