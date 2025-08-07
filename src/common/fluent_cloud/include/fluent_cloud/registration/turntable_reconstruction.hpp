#ifndef FLUENT_CLOUD_REGISTRATION_TURNTABLE_RECONSTRUCTION_HPP
#define FLUENT_CLOUD_REGISTRATION_TURNTABLE_RECONSTRUCTION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/ply_io.h>

namespace fluent_cloud {
namespace registration {

/**
 * @brief ターンテーブル式360度3D再構成
 * アスパラガスを回転させながら完全な3Dモデルを作成
 */
template<typename PointT = pcl::PointXYZRGB>
class TurntableReconstruction {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudNormal = pcl::PointCloud<pcl::PointXYZRGBNormal>;
    using PointCloudNormalPtr = typename PointCloudNormal::Ptr;
    
    struct ReconstructionResult {
        PointCloudPtr merged_cloud;           // 統合された点群
        PointCloudNormalPtr cloud_with_normals; // 法線付き点群
        pcl::PolygonMesh mesh;               // 生成されたメッシュ
        std::vector<Eigen::Matrix4f> poses;  // 各ビューの姿勢
        float reconstruction_quality;         // 再構成品質（0-1）
        float coverage_percentage;            // カバー率（%）
    };
    
    TurntableReconstruction() 
        : rotation_axis_(Eigen::Vector3f::UnitZ()),
          rotation_center_(Eigen::Vector3f::Zero()),
          angle_step_(10.0),  // 10度刻み
          use_color_icp_(true),
          voxel_size_(0.002),  // 2mm
          max_correspondence_distance_(0.01),  // 1cm
          overlap_threshold_(0.3) {}
    
    // 回転軸の設定（デフォルトはZ軸）
    TurntableReconstruction& setRotationAxis(const Eigen::Vector3f& axis) {
        rotation_axis_ = axis.normalized();
        return *this;
    }
    
    // 回転中心の設定
    TurntableReconstruction& setRotationCenter(const Eigen::Vector3f& center) {
        rotation_center_ = center;
        return *this;
    }
    
    // 角度ステップの設定
    TurntableReconstruction& setAngleStep(float degrees) {
        angle_step_ = degrees;
        return *this;
    }
    
    /**
     * @brief 複数視点からの点群を統合して3Dモデルを作成
     * @param views 各角度からの点群（順番に並んでいることを想定）
     */
    ReconstructionResult reconstruct(const std::vector<PointCloudPtr>& views) {
        ReconstructionResult result;
        
        if (views.empty()) return result;
        
        // 1. 初期点群の準備
        PointCloudPtr accumulated(new PointCloud);
        *accumulated = *views[0];
        
        result.poses.push_back(Eigen::Matrix4f::Identity());
        
        // 2. 順次位置合わせして統合
        for (size_t i = 1; i < views.size(); ++i) {
            RCLCPP_INFO(rclcpp::get_logger("turntable"), 
                       "Processing view %zu/%zu", i+1, views.size());
            
            // 予想される回転量
            float expected_angle = angle_step_ * i * M_PI / 180.0;
            Eigen::Matrix4f initial_guess = createRotationMatrix(expected_angle);
            
            // ICPで精密位置合わせ
            Eigen::Matrix4f transformation;
            float fitness_score;
            
            if (use_color_icp_) {
                transformation = alignWithColorICP(views[i], accumulated, 
                                                 initial_guess, fitness_score);
            } else {
                transformation = alignWithICP(views[i], accumulated, 
                                            initial_guess, fitness_score);
            }
            
            // 変換して統合
            PointCloudPtr transformed(new PointCloud);
            pcl::transformPointCloud(*views[i], *transformed, transformation);
            *accumulated += *transformed;
            
            result.poses.push_back(transformation);
            
            // 定期的にダウンサンプリング
            if (i % 3 == 0) {
                accumulated = downsample(accumulated);
            }
        }
        
        // 3. 最終的な統合とクリーンアップ
        result.merged_cloud = cleanupMergedCloud(accumulated);
        
        // 4. 法線推定
        result.cloud_with_normals = computeNormals(result.merged_cloud);
        
        // 5. メッシュ生成（Poisson再構成）
        result.mesh = generateMesh(result.cloud_with_normals);
        
        // 6. 品質評価
        evaluateReconstruction(result);
        
        return result;
    }
    
    /**
     * @brief リアルタイム再構成（1フレームずつ追加）
     */
    class RealtimeReconstructor {
    public:
        RealtimeReconstructor(TurntableReconstruction& parent) 
            : parent_(parent), 
              current_angle_(0),
              frame_count_(0) {
            accumulated_cloud_ = PointCloudPtr(new PointCloud);
        }
        
        void addFrame(const PointCloudPtr& frame) {
            if (frame_count_ == 0) {
                // 最初のフレーム
                *accumulated_cloud_ = *frame;
                poses_.push_back(Eigen::Matrix4f::Identity());
            } else {
                // 位置合わせして追加
                float expected_angle = current_angle_ * M_PI / 180.0;
                Eigen::Matrix4f initial_guess = parent_.createRotationMatrix(expected_angle);
                
                float fitness_score;
                Eigen::Matrix4f transformation = parent_.alignWithICP(
                    frame, accumulated_cloud_, initial_guess, fitness_score);
                
                PointCloudPtr transformed(new PointCloud);
                pcl::transformPointCloud(*frame, *transformed, transformation);
                *accumulated_cloud_ += *transformed;
                
                poses_.push_back(transformation);
                
                // 定期的なクリーンアップ
                if (frame_count_ % 5 == 0) {
                    accumulated_cloud_ = parent_.downsample(accumulated_cloud_);
                }
            }
            
            frame_count_++;
            current_angle_ += parent_.angle_step_;
            
            // 360度完了チェック
            if (current_angle_ >= 360.0) {
                is_complete_ = true;
            }
        }
        
        bool isComplete() const { return is_complete_; }
        
        ReconstructionResult finalize() {
            ReconstructionResult result;
            result.merged_cloud = parent_.cleanupMergedCloud(accumulated_cloud_);
            result.cloud_with_normals = parent_.computeNormals(result.merged_cloud);
            result.mesh = parent_.generateMesh(result.cloud_with_normals);
            result.poses = poses_;
            parent_.evaluateReconstruction(result);
            return result;
        }
        
    private:
        TurntableReconstruction& parent_;
        PointCloudPtr accumulated_cloud_;
        std::vector<Eigen::Matrix4f> poses_;
        float current_angle_;
        int frame_count_;
        bool is_complete_ = false;
    };
    
    RealtimeReconstructor createRealtimeReconstructor() {
        return RealtimeReconstructor(*this);
    }
    
private:
    Eigen::Vector3f rotation_axis_;
    Eigen::Vector3f rotation_center_;
    float angle_step_;
    bool use_color_icp_;
    float voxel_size_;
    float max_correspondence_distance_;
    float overlap_threshold_;
    
    /**
     * @brief 回転行列の作成
     */
    Eigen::Matrix4f createRotationMatrix(float angle) {
        Eigen::AngleAxisf rotation(angle, rotation_axis_);
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        
        // 回転中心への移動
        transform.block<3,1>(0,3) = -rotation_center_;
        
        // 回転
        Eigen::Matrix4f rot_matrix = Eigen::Matrix4f::Identity();
        rot_matrix.block<3,3>(0,0) = rotation.matrix();
        
        // 回転中心から戻す
        Eigen::Matrix4f back_transform = Eigen::Matrix4f::Identity();
        back_transform.block<3,1>(0,3) = rotation_center_;
        
        return back_transform * rot_matrix * transform;
    }
    
    /**
     * @brief ICPによる位置合わせ
     */
    Eigen::Matrix4f alignWithICP(const PointCloudPtr& source,
                                 const PointCloudPtr& target,
                                 const Eigen::Matrix4f& initial_guess,
                                 float& fitness_score) {
        // ダウンサンプリング
        PointCloudPtr source_down = downsample(source);
        PointCloudPtr target_down = downsample(target);
        
        // ICP設定
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(source_down);
        icp.setInputTarget(target_down);
        icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
        icp.setMaximumIterations(50);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-6);
        
        // 初期推定値を使用
        PointCloudPtr aligned(new PointCloud);
        icp.align(*aligned, initial_guess);
        
        fitness_score = icp.getFitnessScore();
        return icp.getFinalTransformation();
    }
    
    /**
     * @brief 色情報を考慮したICP
     */
    Eigen::Matrix4f alignWithColorICP(const PointCloudPtr& source,
                                     const PointCloudPtr& target,
                                     const Eigen::Matrix4f& initial_guess,
                                     float& fitness_score) {
        // TODO: 色情報を重みとして使用するICP実装
        // 現在は通常のICPにフォールバック
        return alignWithICP(source, target, initial_guess, fitness_score);
    }
    
    /**
     * @brief ダウンサンプリング
     */
    PointCloudPtr downsample(const PointCloudPtr& cloud) {
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        
        PointCloudPtr filtered(new PointCloud);
        voxel_filter.filter(*filtered);
        return filtered;
    }
    
    /**
     * @brief 統合点群のクリーンアップ
     */
    PointCloudPtr cleanupMergedCloud(const PointCloudPtr& cloud) {
        // 1. 統計的外れ値除去
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        
        PointCloudPtr cleaned(new PointCloud);
        sor.filter(*cleaned);
        
        // 2. 最終的なダウンサンプリング
        return downsample(cleaned);
    }
    
    /**
     * @brief 法線推定
     */
    PointCloudNormalPtr computeNormals(const PointCloudPtr& cloud) {
        // 法線推定
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(0.01);  // 1cm
        ne.compute(*normals);
        
        // 点群と法線を結合
        PointCloudNormalPtr cloud_with_normals(new PointCloudNormal);
        pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
        
        return cloud_with_normals;
    }
    
    /**
     * @brief Poissonメッシュ生成
     */
    pcl::PolygonMesh generateMesh(const PointCloudNormalPtr& cloud_with_normals) {
        pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
        poisson.setDepth(9);  // 解像度
        poisson.setInputCloud(cloud_with_normals);
        
        pcl::PolygonMesh mesh;
        poisson.reconstruct(mesh);
        
        return mesh;
    }
    
    /**
     * @brief 再構成品質の評価
     */
    void evaluateReconstruction(ReconstructionResult& result) {
        // 点群の密度から品質を推定
        float point_density = result.merged_cloud->size() / 
                            (result.poses.size() * 1000.0f);  // 正規化
        
        // オーバーラップ率の計算
        float overlap_quality = calculateOverlapQuality(result.poses);
        
        // 総合品質
        result.reconstruction_quality = 0.5f * point_density + 0.5f * overlap_quality;
        result.reconstruction_quality = std::min(1.0f, result.reconstruction_quality);
        
        // カバー率（360度のうち何%カバーしたか）
        result.coverage_percentage = std::min(100.0f, 
            result.poses.size() * angle_step_ / 360.0f * 100.0f);
    }
    
    /**
     * @brief オーバーラップ品質の計算
     */
    float calculateOverlapQuality(const std::vector<Eigen::Matrix4f>& poses) {
        if (poses.size() < 2) return 1.0f;
        
        float total_quality = 0;
        for (size_t i = 1; i < poses.size(); ++i) {
            // 隣接ビュー間の回転角度
            Eigen::Matrix3f rot_diff = poses[i].block<3,3>(0,0) * 
                                       poses[i-1].block<3,3>(0,0).transpose();
            float angle = std::acos((rot_diff.trace() - 1) / 2);
            
            // 理想的な角度との差
            float expected = angle_step_ * M_PI / 180.0;
            float quality = 1.0f - std::abs(angle - expected) / expected;
            total_quality += std::max(0.0f, quality);
        }
        
        return total_quality / (poses.size() - 1);
    }
};

} // namespace registration
} // namespace fluent_cloud

#endif // FLUENT_CLOUD_REGISTRATION_TURNTABLE_RECONSTRUCTION_HPP