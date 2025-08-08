#include "fv_aspara_analyzer/aspara_analyzer_thread.hpp"
#include "fv_aspara_analyzer/fv_aspara_analyzer_node.hpp"
#include "fv_aspara_analyzer/aspara_pointcloud_processor.hpp"
#include "../../../common/fluent_lib/legacy/fluent_cloud/include/fluent_cloud/io/depth_to_cloud.hpp"
#include "../../../common/fluent_lib/legacy/fluent_cloud/include/fluent_cloud/filters/statistical_outlier_removal.hpp"
#include "../../../common/fluent_lib/legacy/fluent_cloud/include/fluent_cloud/filters/voxel_grid.hpp"
#include <fluent.hpp>
#include <iostream>
#include <chrono>
#include <algorithm>

namespace fv_aspara_analyzer {

AnalyzerThread::AnalyzerThread(FvAsparaAnalyzerNode* node_ptr)
    : node_(node_ptr), shutdown_flag_(false), processing_in_progress_(false), has_new_data_(false)
{
    // 点群プロセッサを初期化
    pointcloud_processor_ = std::make_unique<AsparaPointcloudProcessor>(node_ptr);
    
    // ワーカースレッド開始
    worker_thread_ = std::thread(&AnalyzerThread::workerLoop, this);
    std::cout << "Analyzer thread started" << std::endl;
}

AnalyzerThread::~AnalyzerThread()
{
    // スレッド終了処理
    shutdown_flag_ = true;
    data_cv_.notify_all();
    
    if (worker_thread_.joinable()) {
        // 確実に終了を待つ（detachは行わない）
        worker_thread_.join();
        std::cout << "Analyzer thread terminated" << std::endl;
    }
}

void AnalyzerThread::enqueueAnalysis(const AsparaInfo& aspara_info)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 最新の状態で常に上書き
    latest_aspara_info_ = aspara_info;
    has_new_data_ = true;
    data_cv_.notify_one();
}

void AnalyzerThread::workerLoop()
{
    RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"), "Analyzer thread worker loop started");
    
    while (!shutdown_flag_) {
        auto t_wait_start = std::chrono::steady_clock::now();
        std::unique_lock<std::mutex> lock(data_mutex_);
        
        // 新しいデータが来るまで待機
        data_cv_.wait(lock, [this]() {
            return has_new_data_.load() || shutdown_flag_;
        });
        auto t_acq = std::chrono::steady_clock::now();
        double wait_ms = std::chrono::duration<double, std::milli>(t_acq - t_wait_start).count();
        if (has_new_data_.load() && wait_ms > 1.0) {
            RCLCPP_DEBUG(rclcpp::get_logger("analyzer_thread"), "[LOCK ACQ] analyzer/data_queue waited=%.2fms", wait_ms);
        }
        
        if (shutdown_flag_) {
            break;
        }
        
        if (has_new_data_.load()) {
            // 最新データを取得（コピーは1回のみ）
            AsparaInfo aspara_copy = latest_aspara_info_;
            has_new_data_ = false;
            lock.unlock();  // ロック解除してから重い処理を実行
            
            // 処理中フラグを設定
            processing_in_progress_ = true;
            
            try {
                auto analysis_start = std::chrono::steady_clock::now();
            RCLCPP_DEBUG(rclcpp::get_logger("analyzer_thread"), "Starting analysis for asparagus ID %d", aspara_copy.id);
                
                // アスパラの点群分析をスレッド内で直接実行（重い処理）
                auto start_time = std::chrono::high_resolution_clock::now();
                
                // processAsparagu処理をここで直接実行
                processAsparagus(aspara_copy);
                
                auto end_time = std::chrono::high_resolution_clock::now();
                long processing_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
                
                // 結果を安全に反映（ID存在チェック付き）
                updateAnalysisResult(aspara_copy, processing_time_ms);
                
                auto analysis_end = std::chrono::steady_clock::now();
                double analysis_ms = std::chrono::duration<double, std::milli>(analysis_end - analysis_start).count();
                RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"), 
                    "[ANALYZER] ID:%d completed in %.2fms", aspara_copy.id, analysis_ms);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("analyzer_thread"), 
                    "Exception in asparagus %d analysis: %s", aspara_copy.id, e.what());
            } catch (...) {
                RCLCPP_ERROR(rclcpp::get_logger("analyzer_thread"), 
                    "Unknown exception in asparagus %d analysis", aspara_copy.id);
            }
            
            processing_in_progress_ = false;
        }
    }
    
            std::cout << "Analyzer thread worker loop terminated" << std::endl;
}

void AnalyzerThread::processAsparagus(AsparaInfo& aspara_info)
{
    // processAsparagu実装をここに移動
    try {
        // 全体処理時間の計測開始
        fluent::utils::Stopwatch total_stopwatch;

        // 最新データのスナップショット取得（ロック下）
        sensor_msgs::msg::Image::SharedPtr depth_image;
        sensor_msgs::msg::Image::SharedPtr color_image;
        sensor_msgs::msg::CameraInfo::SharedPtr camera_info;
        std::string frame_id;
        {
            std::lock_guard<std::mutex> lk(node_->aspara_list_mutex_);
            depth_image = node_->latest_depth_image_;
            color_image = node_->latest_color_image_;
            camera_info = node_->latest_camera_info_;
            if (depth_image) {
                frame_id = depth_image->header.frame_id;
            }
        }
        if (!color_image || !depth_image || !camera_info) {
            std::cout << "Required data not available yet (color/depth/camera_info)" << std::endl;
            return;
        }

        // 深度画像から効率的に点群を抽出（Fluent libのDepthToCloud使用）
        fluent::utils::Stopwatch filter_stopwatch;
        
        // 深度画像とカラー画像をOpenCV形式に変換
        cv::Mat depth_mat, color_mat;
        try {
            // 深度は 16UC1 または 32FC1 を許容
            if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
                depth_mat = depth_ptr->image; // mm 単位
            } else if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
                depth_ptr->image.convertTo(depth_mat, CV_16UC1, 1000.0); // m -> mm
            } else {
                std::cout << "Unsupported depth encoding: " << depth_image->encoding << std::endl;
                return;
            }

            cv_bridge::CvImagePtr color_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
            color_mat = color_ptr->image;
        } catch (cv_bridge::Exception& e) {
            std::cout << "CV bridge exception in image conversion: " << e.what() << std::endl;
            return;
        }
        
        // Fluent libのDepthToCloudを使用してアスパラガスROI専用の点群生成
        auto filtered_cloud = fluent_cloud::io::DepthToCloud::convertAsparagusROI(
            depth_mat,
            color_mat,
            aspara_info.bounding_box_2d,
            *camera_info);
        aspara_info.processing_times.filter_bbox_ms = filter_stopwatch.elapsed_ms();
        
        if (filtered_cloud->points.empty()) {
            std::cout << "No points found in bounding box" << std::endl;
            return;
        }

        // ノイズ除去（Fluent libのフィルタを使用）
        fluent::utils::Stopwatch noise_stopwatch;
        
        // ボクセルグリッドフィルタでダウンサンプリング（fluent_cloud）
        auto voxel_filtered = fluent_cloud::filters::VoxelGrid<pcl::PointXYZRGB>()
            .setLeafSize(node_->voxel_leaf_size_)
            .filter(filtered_cloud);
        
        // 統計的外れ値除去（fluent_cloud）
        auto denoised_cloud = fluent_cloud::filters::StatisticalOutlierRemoval<pcl::PointXYZRGB>()
            .setMeanK(node_->noise_reduction_neighbors_)
            .setStddevMulThresh(node_->noise_reduction_std_dev_)
            .filter(voxel_filtered);
        aspara_info.processing_times.noise_reduction_ms = noise_stopwatch.elapsed_ms();

        if (denoised_cloud->points.empty()) {
            std::cout << "No points left after noise reduction" << std::endl;
            return;
        }

        // アスパラガス特性を計算
        fluent::utils::Stopwatch measurement_stopwatch;
        auto root_position = pointcloud_processor_->estimateRootPosition(denoised_cloud);
        float straightness = pointcloud_processor_->calculateStraightness(denoised_cloud);
        float length = pointcloud_processor_->calculateLength(denoised_cloud);
        aspara_info.processing_times.measurement_ms = measurement_stopwatch.elapsed_ms();
        
        // 可視化用PCAラインを生成
        fluent::utils::Stopwatch pca_stopwatch;
        auto pca_line = pointcloud_processor_->generatePCALine(denoised_cloud);
        aspara_info.processing_times.pca_calculation_ms = pca_stopwatch.elapsed_ms();

        // 収穫適性を判定
        bool is_harvestable = (length >= node_->harvest_min_length_ && 
                              length <= node_->harvest_max_length_ && 
                              straightness >= node_->straightness_threshold_);

        // 可視化処理時間の計測
        fluent::utils::Stopwatch vis_stopwatch;
        
        // 結果をパブリッシュ
        const std::string frame_id_to_use = frame_id.empty() ? std::string("camera_frame") : frame_id;
        pointcloud_processor_->publishFilteredPointCloud(denoised_cloud, frame_id_to_use, aspara_info.id);
        pointcloud_processor_->publishRootTF(root_position, frame_id_to_use, aspara_info.id);
        
        // アスパラ情報を更新
        aspara_info.length = length;
        aspara_info.straightness = straightness;
        aspara_info.is_harvestable = is_harvestable;
        aspara_info.root_position_3d = root_position;
        
        // 全体処理時間を記録
        aspara_info.processing_times.total_ms = total_stopwatch.elapsed_ms();
        aspara_info.processing_times.visualization_ms = vis_stopwatch.elapsed_ms();
        
        // 画像合成はメインスレッドで行うため、ここでは出力しない

        // 詳細な処理時間ログ出力
        std::cout << "[ANALYZER THREAD] ID:" << aspara_info.id 
                  << " Total:" << aspara_info.processing_times.total_ms << "ms"
                  << " (ROI:" << aspara_info.processing_times.filter_bbox_ms << "ms"
                  << ", Noise:" << aspara_info.processing_times.noise_reduction_ms << "ms"
                  << ", Measure:" << aspara_info.processing_times.measurement_ms << "ms"
                  << ", PCA:" << aspara_info.processing_times.pca_calculation_ms << "ms"
                  << ", Vis:" << aspara_info.processing_times.visualization_ms << "ms)"
                  << " Length:" << length << "m"
                  << " Straight:" << straightness
                  << " Harvest:" << (is_harvestable ? "YES" : "NO")
                  << " Points:" << denoised_cloud->points.size() << std::endl;
                
    } catch (const cv_bridge::Exception& e) {
        std::cout << "CV bridge exception in processAsparagus (ID:" << aspara_info.id << "): " << e.what() << std::endl;
        return;
    } catch (const pcl::PCLException& e) {
        std::cout << "PCL exception in processAsparagus (ID:" << aspara_info.id << "): " << e.what() << std::endl;
        return;
    } catch (const std::exception& e) {
        std::cout << "Standard exception in processAsparagus (ID:" << aspara_info.id << "): " << e.what() << std::endl;
        return;
    } catch (...) {
        std::cout << "Unknown exception in processAsparagus (ID:" << aspara_info.id << ")" << std::endl;
        return;
    }
}

void AnalyzerThread::updateAnalysisResult(const AsparaInfo& aspara_info, long processing_time_ms)
{
    // 点群処理時間とFPSを更新
    node_->last_pointcloud_time_ms_ = processing_time_ms;
    if (node_->pointcloud_fps_meter_) {
        node_->pointcloud_fps_meter_->tick(node_->now());
    }
    
    std::lock_guard<std::mutex> lock(node_->aspara_list_mutex_);
    
    auto it = std::find_if(node_->aspara_list_.begin(), node_->aspara_list_.end(),
                          [&aspara_info](const AsparaInfo& info) { 
                              return info.id == aspara_info.id; 
                          });
    
    if (it != node_->aspara_list_.end()) {
        // まだ画面に表示中 → リアルタイム更新
        it->processing_times = aspara_info.processing_times;
        it->length = aspara_info.length;
        it->straightness = aspara_info.straightness;
        it->is_harvestable = aspara_info.is_harvestable;
        it->root_position_3d = aspara_info.root_position_3d;
        it->filtered_pointcloud = aspara_info.filtered_pointcloud;
        
        std::cout << "Aspara ID " << aspara_info.id << " analysis updated: "
                  << "Length=" << aspara_info.length << "m, "
                  << "Harvestable=" << (aspara_info.is_harvestable ? "YES" : "NO") 
                  << " (live update)" << std::endl;
    } else {
        // 画面から消えた → オフライン登録のみ
        std::cout << "Aspara ID " << aspara_info.id << " analysis completed offline: "
                  << "Length=" << aspara_info.length << "m, "
                  << "Harvestable=" << (aspara_info.is_harvestable ? "YES" : "NO") << std::endl;
    }
    
    // 画面表示の有無に関わらず、RTAB-Map登録とデータ保存は実行
    // registerAsparaToRTABMap(aspara_info.id, aspara_info);
    // saveAsparaResultToDatabase(aspara_info.id, aspara_info);
}

} // namespace fv_aspara_analyzer