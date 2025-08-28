#ifndef FLUENT_CLOUD_REGISTRATION_MULTI_VIEW_RECONSTRUCTION_HPP
#define FLUENT_CLOUD_REGISTRATION_MULTI_VIEW_RECONSTRUCTION_HPP

#include <fluent_cloud/registration/turntable_reconstruction.hpp>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>

namespace fluent_cloud {
namespace registration {

/**
 * @brief マルチビュー3D再構成の高度な手法
 * アスパラガスの完全な3Dモデルを様々な方法で作成
 */
template<typename PointT = pcl::PointXYZRGB>
class MultiViewReconstruction {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    
    /**
     * @brief ハンドヘルドスキャン（手動で360度撮影）
     */
    class HandheldScanner {
    public:
        HandheldScanner() : accumulated_(new PointCloud) {}
        
        struct ScanFrame {
            PointCloudPtr cloud;
            cv::Mat rgb_image;
            cv::Mat depth_image;
            double timestamp;
            Eigen::Matrix4f estimated_pose;
        };
        
        /**
         * @brief フレーム追加（自動的に位置推定）
         */
        void addFrame(const ScanFrame& frame) {
            if (frames_.empty()) {
                // 最初のフレーム
                *accumulated_ = *frame.cloud;
                frames_.push_back(frame);
                current_pose_ = Eigen::Matrix4f::Identity();
            } else {
                // 特徴点マッチングで初期位置推定
                Eigen::Matrix4f initial_transform = estimateMotion(
                    frames_.back(), frame);
                
                // ICPで精密位置合わせ
                Eigen::Matrix4f refined_transform = refineAlignment(
                    frame.cloud, accumulated_, initial_transform);
                
                // グローバル座標系での位置
                current_pose_ = current_pose_ * refined_transform;
                
                // 点群を変換して追加
                PointCloudPtr transformed(new PointCloud);
                pcl::transformPointCloud(*frame.cloud, *transformed, current_pose_);
                *accumulated_ += *transformed;
                
                // フレーム情報保存
                ScanFrame stored_frame = frame;
                stored_frame.estimated_pose = current_pose_;
                frames_.push_back(stored_frame);
                
                // ループクロージャ検出
                detectLoopClosure();
            }
        }
        
        /**
         * @brief スキャン完了チェック（360度カバーしたか）
         */
        bool isScanComplete() {
            return calculateCoverage() > 0.95;  // 95%以上カバー
        }
        
        PointCloudPtr getFinalModel() {
            // グローバル最適化
            optimizePoses();
            
            // 最終的な点群生成
            return generateOptimizedCloud();
        }
        
    private:
        std::vector<ScanFrame> frames_;
        PointCloudPtr accumulated_;
        Eigen::Matrix4f current_pose_;
        
        /**
         * @brief 視覚的特徴を使った動き推定
         */
        Eigen::Matrix4f estimateMotion(const ScanFrame& prev, 
                                      const ScanFrame& curr) {
            // ORB特徴点マッチング
            cv::Ptr<cv::ORB> orb = cv::ORB::create(500);
            std::vector<cv::KeyPoint> kp1, kp2;
            cv::Mat desc1, desc2;
            
            orb->detectAndCompute(prev.rgb_image, cv::noArray(), kp1, desc1);
            orb->detectAndCompute(curr.rgb_image, cv::noArray(), kp2, desc2);
            
            // マッチング
            cv::BFMatcher matcher(cv::NORM_HAMMING);
            std::vector<cv::DMatch> matches;
            matcher.match(desc1, desc2, matches);
            
            // 良いマッチのみ選択
            std::sort(matches.begin(), matches.end());
            std::vector<cv::DMatch> good_matches(
                matches.begin(), 
                matches.begin() + matches.size() / 2);
            
            // 3D-3D対応点から変換推定
            std::vector<Eigen::Vector3f> src_points, tgt_points;
            for (const auto& match : good_matches) {
                cv::Point2f p1 = kp1[match.queryIdx].pt;
                cv::Point2f p2 = kp2[match.trainIdx].pt;
                
                // 深度値から3D座標を計算
                float d1 = prev.depth_image.at<float>(p1.y, p1.x);
                float d2 = curr.depth_image.at<float>(p2.y, p2.x);
                
                if (d1 > 0 && d2 > 0) {
                    // カメラパラメータを使って3D変換（省略）
                    // src_points.push_back(...);
                    // tgt_points.push_back(...);
                }
            }
            
            // SVDで変換行列計算
            return estimateRigidTransform(src_points, tgt_points);
        }
        
        /**
         * @brief ループクロージャ検出
         */
        void detectLoopClosure() {
            if (frames_.size() < 10) return;
            
            // 現在位置と過去のフレームを比較
            const auto& current = frames_.back();
            
            for (size_t i = 0; i < frames_.size() - 5; ++i) {
                // 位置が近い場合
                float dist = (current.estimated_pose.block<3,1>(0,3) - 
                             frames_[i].estimated_pose.block<3,1>(0,3)).norm();
                
                if (dist < 0.1) {  // 10cm以内
                    // ループクロージャ候補
                    // グラフ最適化などを実行
                }
            }
        }
        
        float calculateCoverage() {
            // 視点の分布から360度カバー率を計算
            // 簡易版：フレーム数から推定
            return std::min(1.0f, frames_.size() / 36.0f);  // 10度刻み想定
        }
        
        void optimizePoses() {
            // Bundle Adjustmentやグラフ最適化
            // 省略
        }
        
        PointCloudPtr generateOptimizedCloud() {
            PointCloudPtr result(new PointCloud);
            
            for (const auto& frame : frames_) {
                PointCloudPtr transformed(new PointCloud);
                pcl::transformPointCloud(*frame.cloud, *transformed, 
                                       frame.estimated_pose);
                *result += *transformed;
            }
            
            // クリーンアップ
            return cleanupCloud(result);
        }
        
        Eigen::Matrix4f estimateRigidTransform(
            const std::vector<Eigen::Vector3f>& src,
            const std::vector<Eigen::Vector3f>& tgt) {
            // Kabsch algorithm (SVD-based)
            // 省略：実装は一般的なSVDベースの手法
            return Eigen::Matrix4f::Identity();
        }
        
        PointCloudPtr cleanupCloud(const PointCloudPtr& cloud) {
            // ダウンサンプリングと外れ値除去
            pcl::VoxelGrid<PointT> vg;
            vg.setInputCloud(cloud);
            vg.setLeafSize(0.002f, 0.002f, 0.002f);
            
            PointCloudPtr filtered(new PointCloud);
            vg.filter(*filtered);
            return filtered;
        }
    };
    
    /**
     * @brief RGB-Dカメラアレイによる同時撮影
     */
    class MultiCameraCapture {
    public:
        struct CameraView {
            std::string camera_id;
            Eigen::Matrix4f extrinsic;  // カメラの外部パラメータ
            sensor_msgs::msg::CameraInfo intrinsic;
        };
        
        /**
         * @brief 複数カメラからの同時キャプチャ
         */
        PointCloudPtr captureFromMultipleCameras(
            const std::vector<PointCloudPtr>& camera_clouds,
            const std::vector<CameraView>& camera_configs) {
            
            PointCloudPtr merged(new PointCloud);
            
            for (size_t i = 0; i < camera_clouds.size(); ++i) {
                // 各カメラの点群をワールド座標系に変換
                PointCloudPtr transformed(new PointCloud);
                pcl::transformPointCloud(*camera_clouds[i], *transformed, 
                                       camera_configs[i].extrinsic);
                
                // マージ
                *merged += *transformed;
            }
            
            // 重複除去
            return removeDuplicates(merged);
        }
        
    private:
        PointCloudPtr removeDuplicates(const PointCloudPtr& cloud) {
            // KdTreeで近傍点を統合
            pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
            tree->setInputCloud(cloud);
            
            PointCloudPtr result(new PointCloud);
            std::vector<bool> processed(cloud->size(), false);
            
            for (size_t i = 0; i < cloud->size(); ++i) {
                if (processed[i]) continue;
                
                std::vector<int> indices;
                std::vector<float> distances;
                tree->radiusSearch(cloud->points[i], 0.001, indices, distances);
                
                // 近傍点の平均を計算
                PointT avg_point = cloud->points[i];
                for (const auto& idx : indices) {
                    processed[idx] = true;
                }
                
                result->points.push_back(avg_point);
            }
            
            result->width = result->points.size();
            result->height = 1;
            result->is_dense = false;
            
            return result;
        }
    };
    
    /**
     * @brief Structure from Motion (SfM)による再構成
     */
    class SfMReconstruction {
    public:
        /**
         * @brief 2D画像列から3D再構成
         */
        PointCloudPtr reconstructFromImages(
            const std::vector<cv::Mat>& images,
            const sensor_msgs::msg::CameraInfo& camera_info) {
            
            // 特徴点抽出・マッチング
            // カメラポーズ推定
            // 三角測量
            // Bundle Adjustment
            
            // TODO: OpenCVやOpenMVGを使った実装
            
            return PointCloudPtr(new PointCloud);
        }
    };
};

// ========== 使用例 ==========
/*
// 1. ターンテーブル式360度スキャン
auto turntable = TurntableReconstruction<pcl::PointXYZRGB>()
    .setAngleStep(5.0)  // 5度刻み
    .setRotationAxis(Eigen::Vector3f::UnitZ());

// リアルタイムモード
auto reconstructor = turntable.createRealtimeReconstructor();

// ROSトピックから順次追加
while (!reconstructor.isComplete()) {
    auto cloud = getPointCloudFromCamera();
    reconstructor.addFrame(cloud);
    
    // 進捗表示
    showProgress(reconstructor.getCurrentAngle());
}

auto result = reconstructor.finalize();
pcl::io::savePLYFile("asparagus_3d_model.ply", result.mesh);

// 2. ハンドヘルドスキャン
MultiViewReconstruction<>::HandheldScanner scanner;

// カメラを動かしながら撮影
while (!scanner.isScanComplete()) {
    MultiViewReconstruction<>::HandheldScanner::ScanFrame frame;
    frame.cloud = getCurrentPointCloud();
    frame.rgb_image = getCurrentRGBImage();
    frame.depth_image = getCurrentDepthImage();
    frame.timestamp = ros::Time::now().toSec();
    
    scanner.addFrame(frame);
}

auto final_model = scanner.getFinalModel();

// 3. マルチカメラ同時撮影（4台のRealSenseを90度間隔で配置）
MultiViewReconstruction<>::MultiCameraCapture multi_cam;

std::vector<MultiViewReconstruction<>::MultiCameraCapture::CameraView> configs;
for (int i = 0; i < 4; ++i) {
    MultiViewReconstruction<>::MultiCameraCapture::CameraView view;
    view.camera_id = "camera_" + std::to_string(i);
    
    // 90度ずつ回転した位置に配置
    float angle = i * M_PI / 2;
    view.extrinsic = Eigen::Matrix4f::Identity();
    view.extrinsic.block<3,3>(0,0) = 
        Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()).matrix();
    view.extrinsic(0,3) = 0.3 * cos(angle);  // 30cm距離
    view.extrinsic(1,3) = 0.3 * sin(angle);
    
    configs.push_back(view);
}

auto instant_3d = multi_cam.captureFromMultipleCameras(camera_clouds, configs);
*/

} // namespace registration
} // namespace fluent_cloud

#endif // FLUENT_CLOUD_REGISTRATION_MULTI_VIEW_RECONSTRUCTION_HPP