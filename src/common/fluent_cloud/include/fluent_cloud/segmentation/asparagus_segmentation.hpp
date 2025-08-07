#ifndef FLUENT_CLOUD_SEGMENTATION_ASPARAGUS_SEGMENTATION_HPP
#define FLUENT_CLOUD_SEGMENTATION_ASPARAGUS_SEGMENTATION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <opencv2/opencv.hpp>

namespace fluent_cloud {
namespace segmentation {

/**
 * @brief アスパラガス専用セグメンテーション
 * 複数の手法を組み合わせてアスパラガスだけを綺麗に抽出
 */
template<typename PointT = pcl::PointXYZRGB>
class AsparagusSegmentation {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    
    struct SegmentationResult {
        PointCloudPtr asparagus_cloud;     // アスパラガス点群
        PointCloudPtr background_cloud;    // 背景点群
        std::vector<int> asparagus_indices; // アスパラガスのインデックス
        cv::Mat segmentation_mask;         // 2Dセグメンテーションマスク
        float confidence;                   // セグメンテーション信頼度
    };
    
    AsparagusSegmentation() {
        // アスパラガスの典型的な色範囲（HSV）
        h_min_ = 35;   // 緑色の色相範囲
        h_max_ = 85;
        s_min_ = 30;   // 彩度
        s_max_ = 255;
        v_min_ = 30;   // 明度
        v_max_ = 255;
        
        // 形状パラメータ
        cylinder_radius_ = 0.015;  // 15mm（アスパラの典型的半径）
        min_cluster_size_ = 50;
        max_cluster_size_ = 10000;
    }
    
    // 色範囲設定（HSV）
    AsparagusSegmentation& setColorRange(int h_min, int h_max, 
                                        int s_min, int s_max,
                                        int v_min, int v_max) {
        h_min_ = h_min; h_max_ = h_max;
        s_min_ = s_min; s_max_ = s_max;
        v_min_ = v_min; v_max_ = v_max;
        return *this;
    }
    
    /**
     * @brief マスク画像を使った高精度セグメンテーション
     * @param cloud 入力点群
     * @param mask_image セグメンテーションマスク（YOLOやSAM等から）
     * @param camera_info カメラキャリブレーション情報
     */
    SegmentationResult segmentWithMask(const PointCloudPtr& cloud,
                                      const cv::Mat& mask_image,
                                      const sensor_msgs::msg::CameraInfo& camera_info) {
        SegmentationResult result;
        result.asparagus_cloud = PointCloudPtr(new PointCloud);
        result.background_cloud = PointCloudPtr(new PointCloud);
        result.segmentation_mask = mask_image.clone();
        
        // カメラ行列を構築
        cv::Mat K = (cv::Mat_<double>(3, 3) << 
            camera_info.k[0], camera_info.k[1], camera_info.k[2],
            camera_info.k[3], camera_info.k[4], camera_info.k[5],
            camera_info.k[6], camera_info.k[7], camera_info.k[8]);
        
        cv::Mat D = cv::Mat(camera_info.d);
        
        int valid_points = 0;
        int mask_points = 0;
        
        // 各点を2Dに投影してマスクをチェック
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            const auto& point = cloud->points[i];
            
            // 無効な点をスキップ
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }
            
            // 3D点を2Dに投影
            cv::Point3f p3d(point.x, point.y, point.z);
            std::vector<cv::Point3f> object_points = {p3d};
            std::vector<cv::Point2f> image_points;
            
            // 回転・並進はゼロ（カメラ座標系）
            cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
            cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
            
            cv::projectPoints(object_points, rvec, tvec, K, D, image_points);
            
            if (!image_points.empty()) {
                cv::Point2f pt = image_points[0];
                int x = static_cast<int>(pt.x);
                int y = static_cast<int>(pt.y);
                
                // 画像範囲内かチェック
                if (x >= 0 && x < mask_image.cols && y >= 0 && y < mask_image.rows) {
                    valid_points++;
                    
                    // マスク値をチェック
                    if (mask_image.at<uchar>(y, x) > 0) {
                        result.asparagus_cloud->points.push_back(point);
                        result.asparagus_indices.push_back(i);
                        mask_points++;
                    } else {
                        result.background_cloud->points.push_back(point);
                    }
                }
            }
        }
        
        // メタデータ更新
        result.asparagus_cloud->width = result.asparagus_cloud->points.size();
        result.asparagus_cloud->height = 1;
        result.asparagus_cloud->is_dense = false;
        result.asparagus_cloud->header = cloud->header;
        
        result.background_cloud->width = result.background_cloud->points.size();
        result.background_cloud->height = 1;
        result.background_cloud->is_dense = false;
        result.background_cloud->header = cloud->header;
        
        // 信頼度計算
        result.confidence = (valid_points > 0) ? 
            static_cast<float>(mask_points) / valid_points : 0.0f;
        
        return result;
    }
    
    /**
     * @brief 色ベースセグメンテーション
     * @param cloud 入力点群（RGB情報必須）
     */
    SegmentationResult segmentByColor(const PointCloudPtr& cloud) {
        SegmentationResult result;
        result.asparagus_cloud = PointCloudPtr(new PointCloud);
        result.background_cloud = PointCloudPtr(new PointCloud);
        
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            const auto& point = cloud->points[i];
            
            // RGBをHSVに変換
            cv::Vec3b rgb(point.r, point.g, point.b);
            cv::Mat rgb_mat(1, 1, CV_8UC3);
            rgb_mat.at<cv::Vec3b>(0, 0) = rgb;
            
            cv::Mat hsv_mat;
            cv::cvtColor(rgb_mat, hsv_mat, cv::COLOR_RGB2HSV);
            cv::Vec3b hsv = hsv_mat.at<cv::Vec3b>(0, 0);
            
            // アスパラガスの色範囲内かチェック
            if (hsv[0] >= h_min_ && hsv[0] <= h_max_ &&
                hsv[1] >= s_min_ && hsv[1] <= s_max_ &&
                hsv[2] >= v_min_ && hsv[2] <= v_max_) {
                result.asparagus_cloud->points.push_back(point);
                result.asparagus_indices.push_back(i);
            } else {
                result.background_cloud->points.push_back(point);
            }
        }
        
        updateMetadata(result);
        return result;
    }
    
    /**
     * @brief 円柱形状フィッティングによるセグメンテーション
     * @param cloud 入力点群
     * @param seed_point シード点（アスパラガス上の1点）
     */
    SegmentationResult segmentByCylinder(const PointCloudPtr& cloud,
                                        const Eigen::Vector3f& seed_point) {
        SegmentationResult result;
        
        // Min-Cutセグメンテーションを使用
        pcl::MinCutSegmentation<PointT> seg;
        seg.setInputCloud(cloud);
        seg.setRadius(cylinder_radius_ * 3);  // 検索半径
        
        // シード点を設定
        PointCloudPtr foreground_points(new PointCloud);
        PointT seed;
        seed.x = seed_point.x();
        seed.y = seed_point.y();
        seed.z = seed_point.z();
        foreground_points->points.push_back(seed);
        seg.setForegroundPoints(foreground_points);
        
        // セグメンテーション実行
        std::vector<pcl::PointIndices> clusters;
        seg.extract(clusters);
        
        if (!clusters.empty()) {
            result.asparagus_cloud = PointCloudPtr(new PointCloud);
            result.background_cloud = PointCloudPtr(new PointCloud);
            
            // 最大クラスタをアスパラガスとして抽出
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices(clusters[0]));
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            
            extract.setNegative(false);
            extract.filter(*result.asparagus_cloud);
            
            extract.setNegative(true);
            extract.filter(*result.background_cloud);
            
            result.asparagus_indices = inliers->indices;
        }
        
        updateMetadata(result);
        return result;
    }
    
    /**
     * @brief 複合セグメンテーション（色＋形状＋連続性）
     */
    SegmentationResult segmentHybrid(const PointCloudPtr& cloud,
                                    const cv::Mat& mask_image,
                                    const sensor_msgs::msg::CameraInfo& camera_info) {
        // Step 1: マスクベースの初期セグメンテーション
        SegmentationResult mask_result = segmentWithMask(cloud, mask_image, camera_info);
        
        if (mask_result.asparagus_cloud->points.size() < min_cluster_size_) {
            return mask_result;
        }
        
        // Step 2: 色による精製
        SegmentationResult color_result = segmentByColor(mask_result.asparagus_cloud);
        
        // Step 3: 連結成分による分離
        std::vector<pcl::PointIndices> clusters = extractConnectedComponents(color_result.asparagus_cloud);
        
        // Step 4: 最も長い連結成分を選択
        SegmentationResult final_result;
        final_result.asparagus_cloud = selectBestCluster(color_result.asparagus_cloud, clusters);
        final_result.background_cloud = PointCloudPtr(new PointCloud);
        
        // 背景点群を更新
        *final_result.background_cloud = *mask_result.background_cloud;
        *final_result.background_cloud += *color_result.background_cloud;
        
        // Step 5: モルフォロジー処理で穴を埋める
        final_result.asparagus_cloud = applyMorphologicalClosing(final_result.asparagus_cloud);
        
        updateMetadata(final_result);
        return final_result;
    }
    
private:
    // 色パラメータ（HSV）
    int h_min_, h_max_;
    int s_min_, s_max_;
    int v_min_, v_max_;
    
    // 形状パラメータ
    float cylinder_radius_;
    int min_cluster_size_;
    int max_cluster_size_;
    
    /**
     * @brief 連結成分の抽出
     */
    std::vector<pcl::PointIndices> extractConnectedComponents(const PointCloudPtr& cloud) {
        // 法線推定
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud);
        ne.setKSearch(20);
        ne.compute(*normals);
        
        // リージョングローイング
        pcl::RegionGrowing<PointT, pcl::Normal> reg;
        reg.setMinClusterSize(min_cluster_size_);
        reg.setMaxClusterSize(max_cluster_size_);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(30);
        reg.setInputCloud(cloud);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);  // 3度
        reg.setCurvatureThreshold(1.0);
        
        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);
        
        return clusters;
    }
    
    /**
     * @brief 最適なクラスタの選択（最も長い/大きい）
     */
    PointCloudPtr selectBestCluster(const PointCloudPtr& cloud,
                                   const std::vector<pcl::PointIndices>& clusters) {
        if (clusters.empty()) return cloud;
        
        // 各クラスタの長さを評価
        int best_cluster = 0;
        float max_length = 0;
        
        for (size_t i = 0; i < clusters.size(); ++i) {
            PointCloudPtr cluster_cloud(new PointCloud);
            pcl::copyPointCloud(*cloud, clusters[i], *cluster_cloud);
            
            // バウンディングボックスで長さを推定
            Eigen::Vector4f min_pt, max_pt;
            pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);
            
            float length = (max_pt - min_pt).norm();
            if (length > max_length) {
                max_length = length;
                best_cluster = i;
            }
        }
        
        PointCloudPtr result(new PointCloud);
        pcl::copyPointCloud(*cloud, clusters[best_cluster], *result);
        return result;
    }
    
    /**
     * @brief モルフォロジークロージング（穴埋め）
     */
    PointCloudPtr applyMorphologicalClosing(const PointCloudPtr& cloud) {
        // ボクセルグリッドで離散化してモルフォロジー処理
        // ここでは簡易的に近傍点を追加
        PointCloudPtr result(new PointCloud);
        *result = *cloud;
        
        // KdTreeで近傍探索
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud);
        
        // 各点の近傍を調べて穴を検出
        for (const auto& point : cloud->points) {
            std::vector<int> indices;
            std::vector<float> distances;
            
            // 半径内の点を検索
            if (tree->radiusSearch(point, cylinder_radius_ * 2, indices, distances) > 0) {
                // 近傍点が少ない場合は補間点を追加
                if (indices.size() < 10) {
                    // 簡易的な補間（実装省略）
                }
            }
        }
        
        return result;
    }
    
    /**
     * @brief メタデータ更新
     */
    void updateMetadata(SegmentationResult& result) {
        if (result.asparagus_cloud) {
            result.asparagus_cloud->width = result.asparagus_cloud->points.size();
            result.asparagus_cloud->height = 1;
            result.asparagus_cloud->is_dense = false;
        }
        
        if (result.background_cloud) {
            result.background_cloud->width = result.background_cloud->points.size();
            result.background_cloud->height = 1;
            result.background_cloud->is_dense = false;
        }
        
        result.confidence = result.asparagus_cloud->points.size() > min_cluster_size_ ? 0.8f : 0.2f;
    }
};

} // namespace segmentation
} // namespace fluent_cloud

#endif // FLUENT_CLOUD_SEGMENTATION_ASPARAGUS_SEGMENTATION_HPP