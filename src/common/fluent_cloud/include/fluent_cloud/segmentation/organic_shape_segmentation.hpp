#ifndef FLUENT_CLOUD_SEGMENTATION_ORGANIC_SHAPE_SEGMENTATION_HPP
#define FLUENT_CLOUD_SEGMENTATION_ORGANIC_SHAPE_SEGMENTATION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/common.h>

namespace fluent_cloud {
namespace segmentation {

/**
 * @brief 有機的形状のセグメンテーション
 * アスパラガスのような自然物の形状特性を考慮
 */
template<typename PointT = pcl::PointXYZRGB>
class OrganicShapeSegmentation {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    
    struct ShapeFeatures {
        float elongation;           // 細長さ（長さ/幅の比）
        float linearity;           // 直線性（0-1）
        float planarity;           // 平面性（0-1）
        float sphericity;          // 球形度（0-1）
        float surface_variation;   // 表面の変動
        float compactness;         // コンパクトさ
        Eigen::Vector3f main_direction;  // 主方向
        float thickness_variation; // 太さの変動係数
    };
    
    OrganicShapeSegmentation() {
        // アスパラガスの典型的な形状パラメータ
        min_elongation_ = 8.0;      // 長さ/幅が8倍以上
        max_elongation_ = 50.0;     // 50倍以下
        max_thickness_variation_ = 0.3;  // 太さの変動30%以下
        min_linearity_ = 0.7;       // 70%以上直線的
    }
    
    /**
     * @brief アスパラガスらしい形状の点群を抽出
     */
    std::vector<PointCloudPtr> segmentAsparagusCandidates(const PointCloudPtr& cloud) {
        std::vector<PointCloudPtr> candidates;
        
        // 1. 地面除去（アスパラガスは垂直に生える）
        PointCloudPtr non_ground = removeGround(cloud);
        
        // 2. 垂直方向の連続性でクラスタリング
        std::vector<PointCloudPtr> vertical_clusters = clusterVertically(non_ground);
        
        // 3. 各クラスタの形状特徴を評価
        for (const auto& cluster : vertical_clusters) {
            ShapeFeatures features = analyzeShape(cluster);
            
            if (isAsparagusShaped(features)) {
                // 4. 表面の滑らかさチェック
                if (hasSmoothSurface(cluster)) {
                    candidates.push_back(cluster);
                }
            }
        }
        
        return candidates;
    }
    
    /**
     * @brief 成長パターンに基づくセグメンテーション
     * アスパラガスは下から上に向かって少しずつ細くなる
     */
    PointCloudPtr segmentByGrowthPattern(const PointCloudPtr& cloud,
                                        const Eigen::Vector3f& base_point) {
        PointCloudPtr result(new PointCloud);
        
        // 基準点から上方向に成長をトレース
        std::vector<float> height_levels;
        std::map<float, std::vector<PointT>> height_slices;
        
        // 高さごとにスライス
        for (const auto& point : cloud->points) {
            float height = point.z - base_point.z();
            float slice_height = std::round(height * 100) / 100;  // 1cm単位
            height_slices[slice_height].push_back(point);
        }
        
        // 各スライスの特性を確認
        float prev_radius = -1;
        bool is_continuous = true;
        
        for (const auto& slice : height_slices) {
            if (slice.second.size() < 5) continue;
            
            // スライスの中心と半径を計算
            Eigen::Vector3f center(0, 0, 0);
            for (const auto& p : slice.second) {
                center += Eigen::Vector3f(p.x, p.y, p.z);
            }
            center /= slice.second.size();
            
            float max_radius = 0;
            for (const auto& p : slice.second) {
                float r = std::sqrt(std::pow(p.x - center.x(), 2) + 
                                   std::pow(p.y - center.y(), 2));
                max_radius = std::max(max_radius, r);
            }
            
            // アスパラガスの成長パターン：上に行くほど細くなる
            if (prev_radius > 0) {
                if (max_radius > prev_radius * 1.2) {  // 20%以上太くなったら異常
                    is_continuous = false;
                    break;
                }
            }
            prev_radius = max_radius;
            
            // 連続性が保たれていれば追加
            if (is_continuous) {
                for (const auto& p : slice.second) {
                    result->points.push_back(p);
                }
            }
        }
        
        result->width = result->points.size();
        result->height = 1;
        result->is_dense = false;
        
        return result;
    }
    
    /**
     * @brief テーパー形状（先細り）検出
     */
    bool hasTaperedShape(const PointCloudPtr& cloud) {
        if (cloud->points.size() < 50) return false;
        
        // 主軸を計算
        pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
        feature_extractor.setInputCloud(cloud);
        feature_extractor.compute();
        
        Eigen::Vector3f major_vector, middle_vector, minor_vector;
        feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
        
        // 主軸に沿って10分割
        const int num_sections = 10;
        std::vector<float> section_radii(num_sections, 0);
        std::vector<int> section_counts(num_sections, 0);
        
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);
        
        float total_length = (max_pt - min_pt).head<3>().dot(major_vector);
        
        for (const auto& point : cloud->points) {
            Eigen::Vector3f p(point.x, point.y, point.z);
            Eigen::Vector3f to_point = p - min_pt.head<3>();
            float projection = to_point.dot(major_vector);
            
            int section = std::min(num_sections - 1, 
                                  static_cast<int>(projection / total_length * num_sections));
            
            // 主軸からの距離
            float distance = (to_point - projection * major_vector).norm();
            section_radii[section] += distance;
            section_counts[section]++;
        }
        
        // 平均半径を計算
        for (int i = 0; i < num_sections; ++i) {
            if (section_counts[i] > 0) {
                section_radii[i] /= section_counts[i];
            }
        }
        
        // テーパーチェック：下から上に向かって細くなるか
        int decreasing_count = 0;
        for (int i = 1; i < num_sections; ++i) {
            if (section_radii[i] <= section_radii[i-1] * 1.1) {  // 10%の余裕
                decreasing_count++;
            }
        }
        
        return decreasing_count >= num_sections * 0.7;  // 70%以上で減少
    }
    
private:
    float min_elongation_;
    float max_elongation_;
    float max_thickness_variation_;
    float min_linearity_;
    
    /**
     * @brief 地面除去
     */
    PointCloudPtr removeGround(const PointCloudPtr& cloud) {
        // Progressive Morphological Filterを使用
        pcl::ProgressiveMorphologicalFilter<PointT> pmf;
        pmf.setInputCloud(cloud);
        pmf.setMaxWindowSize(20);
        pmf.setSlope(1.0f);
        pmf.setInitialDistance(0.5f);
        pmf.setMaxDistance(3.0f);
        
        pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
        pmf.extract(ground_indices->indices);
        
        // 非地面点を抽出
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(ground_indices);
        extract.setNegative(true);
        
        PointCloudPtr non_ground(new PointCloud);
        extract.filter(*non_ground);
        
        return non_ground;
    }
    
    /**
     * @brief 垂直方向の連続性でクラスタリング
     */
    std::vector<PointCloudPtr> clusterVertically(const PointCloudPtr& cloud) {
        // 簡易実装：高さベースのクラスタリング
        std::vector<PointCloudPtr> clusters;
        
        // TODO: より洗練された垂直クラスタリング
        // - 連結成分解析
        // - 垂直方向の密度ベースクラスタリング
        
        return clusters;
    }
    
    /**
     * @brief 形状特徴の解析
     */
    ShapeFeatures analyzeShape(const PointCloudPtr& cloud) {
        ShapeFeatures features;
        
        pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
        feature_extractor.setInputCloud(cloud);
        feature_extractor.compute();
        
        // 固有値と固有ベクトル
        Eigen::Vector3f eigenvalues;
        Eigen::Matrix3f eigenvectors;
        feature_extractor.getEigenValues(eigenvalues[0], eigenvalues[1], eigenvalues[2]);
        feature_extractor.getEigenVectors(eigenvectors);
        
        // 正規化
        float sum = eigenvalues.sum();
        if (sum > 0) {
            eigenvalues /= sum;
        }
        
        // 形状特徴の計算
        features.linearity = (eigenvalues[0] - eigenvalues[1]) / eigenvalues[0];
        features.planarity = (eigenvalues[1] - eigenvalues[2]) / eigenvalues[0];
        features.sphericity = eigenvalues[2] / eigenvalues[0];
        features.surface_variation = eigenvalues[2] / sum;
        
        // OBBから細長さを計算
        PointT min_point_OBB, max_point_OBB;
        Eigen::Vector3f position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        
        Eigen::Vector3f size;
        size[0] = max_point_OBB.x - min_point_OBB.x;
        size[1] = max_point_OBB.y - min_point_OBB.y;
        size[2] = max_point_OBB.z - min_point_OBB.z;
        
        // 最大寸法を長さ、その他を幅として扱う
        float length = size.maxCoeff();
        float width = (size.sum() - length) / 2.0f;
        
        features.elongation = (width > 0) ? length / width : 0;
        features.main_direction = eigenvectors.col(0);
        
        // コンパクトさ
        float volume = size[0] * size[1] * size[2];
        float surface_area = 2 * (size[0]*size[1] + size[1]*size[2] + size[2]*size[0]);
        features.compactness = (surface_area > 0) ? 
            (36 * M_PI * volume * volume) / (surface_area * surface_area * surface_area) : 0;
        
        return features;
    }
    
    /**
     * @brief アスパラガス形状かどうかの判定
     */
    bool isAsparagusShaped(const ShapeFeatures& features) {
        return features.elongation >= min_elongation_ &&
               features.elongation <= max_elongation_ &&
               features.linearity >= min_linearity_ &&
               features.sphericity < 0.3 &&  // 球形ではない
               features.planarity < 0.3;      // 平面的でもない
    }
    
    /**
     * @brief 表面の滑らかさチェック
     */
    bool hasSmoothSurface(const PointCloudPtr& cloud) {
        // 法線の変動をチェック
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setKSearch(20);
        ne.compute(*normals);
        
        // 隣接法線間の角度変化を計算
        float avg_angle_change = 0;
        int valid_count = 0;
        
        for (size_t i = 0; i < normals->size(); ++i) {
            if (!pcl::isFinite(normals->points[i])) continue;
            
            std::vector<int> indices;
            std::vector<float> distances;
            tree->nearestKSearch(cloud->points[i], 10, indices, distances);
            
            for (const auto& idx : indices) {
                if (idx != i && pcl::isFinite(normals->points[idx])) {
                    float angle = std::acos(std::abs(
                        normals->points[i].getNormalVector3fMap().dot(
                            normals->points[idx].getNormalVector3fMap())));
                    avg_angle_change += angle;
                    valid_count++;
                }
            }
        }
        
        if (valid_count > 0) {
            avg_angle_change /= valid_count;
        }
        
        // 平均角度変化が小さければ滑らか
        return avg_angle_change < 0.2;  // 約11度以下
    }
};

} // namespace segmentation
} // namespace fluent_cloud

#endif // FLUENT_CLOUD_SEGMENTATION_ORGANIC_SHAPE_SEGMENTATION_HPP