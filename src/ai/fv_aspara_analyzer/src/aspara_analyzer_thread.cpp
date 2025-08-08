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
                RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"), 
                    "[DEBUG] Processing asparagus ID:%d in analyzer thread", aspara_copy.id);
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

        // タイムスタンプベースで同期されたデータを取得
        sensor_msgs::msg::Image::SharedPtr depth_image;
        sensor_msgs::msg::Image::SharedPtr color_image;
        sensor_msgs::msg::CameraInfo::SharedPtr camera_info;
        std::string frame_id;
        
        {
            std::lock_guard<std::mutex> lk(node_->aspara_list_mutex_);
            
            // 深度画像を基準にする
            depth_image = node_->latest_depth_image_;
            if (!depth_image) {
                RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"), 
                    "[SYNC] No depth image available");
                return;
            }
            
            frame_id = depth_image->header.frame_id;
            rclcpp::Time depth_time = depth_image->header.stamp;
            
            // カラー画像をタイムスタンプで同期（±100ms以内）
            color_image = node_->latest_color_image_;
            if (color_image) {
                rclcpp::Time color_time = color_image->header.stamp;
                double time_diff = std::abs((depth_time - color_time).seconds());
                if (time_diff > 0.1) { // 100ms以上ずれている場合
                    RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"), 
                        "[SYNC] Color image timestamp mismatch: %.3fs - using grayscale", time_diff);
                    // カラー画像は使用するが、警告を出す（色情報は失われる可能性）
                } else if (time_diff > 0.01) { // 10ms以上ずれている場合
                    RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"), 
                        "[SYNC] Color image timestamp mismatch: %.3fs - minor sync issue", time_diff);
                } else {
                    RCLCPP_DEBUG(rclcpp::get_logger("analyzer_thread"), 
                        "[SYNC] Color image synchronized: %.3fs diff", time_diff);
                }
            }
            
            // カメラ情報は起動時に取得済みなので常に使用可能
            camera_info = node_->latest_camera_info_;
        }
        if (!color_image || !depth_image || !camera_info) {
            RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"), 
                "[POINTCLOUD] Required data not available: color=%d, depth=%d, camera_info=%d", 
                (color_image != nullptr), (depth_image != nullptr), (camera_info != nullptr));
            return;
        }
        
        RCLCPP_DEBUG(rclcpp::get_logger("analyzer_thread"), 
            "[POINTCLOUD] Data available: depth_encoding=%s, color_encoding=%s", 
            depth_image->encoding.c_str(), color_image->encoding.c_str());

        // 深度画像から効率的に点群を抽出（Fluent libのDepthToCloud使用）
        fluent::utils::Stopwatch filter_stopwatch;
        
        // 深度画像とカラー画像をOpenCV形式に変換
        cv::Mat depth_mat, color_mat;
        try {
            RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"), 
                "[POINTCLOUD] Processing depth image: encoding=%s, size=%dx%d", 
                depth_image->encoding.c_str(), depth_image->width, depth_image->height);
            
            // 深度は 16UC1 または 32FC1 を許容
            if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
                depth_mat = depth_ptr->image; // mm 単位
                double min_val, max_val;
                cv::minMaxLoc(depth_mat, &min_val, &max_val);
                
                // 8bitカメラ問題の検出
                if (max_val < 1000) {
                    RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"), 
                        "[POINTCLOUD] ⚠️ 8bit depth detected: range=[%.0f, %.0f] - possible camera issue", 
                        min_val, max_val);
                } else {
                    RCLCPP_DEBUG(rclcpp::get_logger("analyzer_thread"), 
                        "[POINTCLOUD] Depth image converted: 16UC1, range=[%.0f, %.0f]", 
                        min_val, max_val);
                }
            } else if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
                // 32FC1（メートル単位）をそのまま使用（Fluent libが適切に変換する）
                depth_mat = depth_ptr->image;
                double min_val, max_val;
                cv::minMaxLoc(depth_mat, &min_val, &max_val);
                RCLCPP_DEBUG(rclcpp::get_logger("analyzer_thread"), 
                    "[POINTCLOUD] Depth image converted: 32FC1, range=[%.3f, %.3f]m", 
                    min_val, max_val);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("analyzer_thread"), 
                    "[POINTCLOUD] Unsupported depth encoding: %s", depth_image->encoding.c_str());
                return;
            }

            cv_bridge::CvImagePtr color_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
            color_mat = color_ptr->image;
        } catch (cv_bridge::Exception& e) {
            std::cout << "CV bridge exception in image conversion: " << e.what() << std::endl;
            return;
        }
        
        // バウンディングボックスの範囲チェック
        cv::Rect bbox = aspara_info.bounding_box_2d;
        cv::Rect image_rect(0, 0, depth_image->width, depth_image->height);
        cv::Rect clipped = bbox & image_rect;
        if (!clipped.area()) {
            RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"), 
                "[POINTCLOUD] Bounding box outside image bounds: bbox=(%d,%d,%d,%d), image=(%d,%d)", 
                bbox.x, bbox.y, bbox.width, bbox.height, depth_image->width, depth_image->height);
            return;
        }
        if (clipped.area() != bbox.area()) {
            RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"),
                "[POINTCLOUD] ROI clipped to image bounds: in=(%d,%d,%d,%d) -> clip=(%d,%d,%d,%d)",
                bbox.x, bbox.y, bbox.width, bbox.height, clipped.x, clipped.y, clipped.width, clipped.height);
        }
        
        // Fluent libのDepthToCloudを使用してアスパラガスROI専用の点群生成
        RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"), 
            "[POINTCLOUD] Converting depth to cloud for bbox: x=%d, y=%d, w=%d, h=%d", 
            clipped.x, clipped.y, clipped.width, clipped.height);
        
        RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"), 
            "[POINTCLOUD] Depth mat size: %dx%d, type: %d, Color mat size: %dx%d", 
            depth_mat.cols, depth_mat.rows, depth_mat.type(), color_mat.cols, color_mat.rows);
        
        RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"), 
            "[POINTCLOUD] Camera info: fx=%f, fy=%f, cx=%f, cy=%f", 
            camera_info->k[0], camera_info->k[4], camera_info->k[2], camera_info->k[5]);
        
        RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"), 
            "[POINTCLOUD] Calling GeneratePointCloud service...");
        
        // サービスを使用して点群を生成
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud;
        
        // GPT5様よろしくお願いします - ここでサービス呼び出しがタイムアウトする問題の修正が必要
        // 問題: RealSenseパイプラインへの同時アクセスによる競合とミューテックス不足
        // 修正案: パイプラインアクセスをミューテックスで保護し、フレームキャッシュを実装
        // サービスクライアントが利用可能かチェック
        if (node_->generate_pointcloud_client_ && node_->generate_pointcloud_client_->wait_for_service(std::chrono::milliseconds(100))) {
            // サービスリクエストを作成
            auto request = std::make_shared<fv_realsense::srv::GeneratePointCloud::Request>();
            request->roi_x = clipped.x;
            request->roi_y = clipped.y;
            request->roi_width = clipped.width;
            request->roi_height = clipped.height;
            request->use_roi = true;
            
            // サービスを呼び出し
            auto future = node_->generate_pointcloud_client_->async_send_request(request);
            
            // 結果を待つ（最大1秒）
            if (future.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready) {
                auto result = future.get();
                if (result->success) {
                    // PointCloud2メッセージをPCLに変換
                    filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                    pcl::fromROSMsg(result->pointcloud, *filtered_cloud);
                    RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"), 
                        "[POINTCLOUD] Service returned %zu points", filtered_cloud->points.size());
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("analyzer_thread"), 
                        "[POINTCLOUD] Service failed: %s", result->message.c_str());
                    return;
                }
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("analyzer_thread"), 
                    "[POINTCLOUD] Service call timed out");
                return;
            }
        } else {
            // サービスが利用できない場合は従来の方法を使用（フォールバック）
            RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"), 
                "[POINTCLOUD] Service not available, using local conversion");
            filtered_cloud = fluent_cloud::io::DepthToCloud::convertAsparagusROI(
                depth_mat,
                color_mat,
                clipped,
                *camera_info);
        }
        
        RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"), 
            "[POINTCLOUD] convertAsparagusROI returned successfully");
        aspara_info.processing_times.filter_bbox_ms = filter_stopwatch.elapsed_ms();
        
        RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"), 
            "[POINTCLOUD] Generated %zu points from depth image", filtered_cloud->points.size());
        
        if (filtered_cloud->points.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"), 
                "[POINTCLOUD] No points found in bounding box - check depth data");
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
        
        // ROI深度統計ログ
        {
            cv::Mat roi_depth = depth_mat(clipped);
            double roi_min, roi_max; cv::minMaxLoc(roi_depth, &roi_min, &roi_max);
            cv::Scalar roi_mean = cv::mean(roi_depth);
            int zero_count = cv::countNonZero(roi_depth == 0);
            int total_count = roi_depth.rows * roi_depth.cols;
            double zero_ratio = total_count > 0 ? (static_cast<double>(zero_count) / total_count) : 0.0;
            RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"),
                "[ROI] depth stats: min=%.0f max=%.0f mean=%.1f zero=%.1f%% size=%dx%d", 
                roi_min, roi_max, roi_mean[0], zero_ratio * 100.0, roi_depth.cols, roi_depth.rows);
        }

        // 結果をパブリッシュ（frame_id, stampを厳密継承）
        const std::string frame_id_to_use = frame_id; // フォールバックしない
        pointcloud_processor_->publishFilteredPointCloud(
            denoised_cloud, frame_id_to_use, aspara_info.id, depth_image->header.stamp);
        
        // 選択中のアスパラガスの点群をasparagus_pointcloudとして公開
        // フィルター前の生データ（ROI抽出後）を公開
        RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"), 
            "[DEBUG] Checking pointcloud publish: selected_id=%d, current_id=%d, pub_valid=%d", 
            node_->selected_aspara_id_, aspara_info.id, (node_->selected_pointcloud_pub_ != nullptr));
        
        if (node_->selected_aspara_id_ == aspara_info.id && node_->selected_pointcloud_pub_) {
            sensor_msgs::msg::PointCloud2 selected_cloud_msg;
            pcl::toROSMsg(*filtered_cloud, selected_cloud_msg);  // フィルター前のデータを使用
            selected_cloud_msg.header.stamp = depth_image->header.stamp;
            selected_cloud_msg.header.frame_id = frame_id_to_use;
            node_->selected_pointcloud_pub_->publish(selected_cloud_msg);
            
            // AsparaInfo構造体にも点群を保存
            aspara_info.asparagus_pointcloud = selected_cloud_msg;
            
            RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"), 
                "[POINTCLOUD] Published asparagus_pointcloud for ID:%d, Points:%zu (raw ROI data)", 
                aspara_info.id, filtered_cloud->points.size());
        }
        
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