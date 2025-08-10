#include "fv_aspara_analyzer/aspara_analyzer_thread.hpp"
#include "fv_aspara_analyzer/fv_aspara_analyzer_node.hpp"
#include "fv_aspara_analyzer/aspara_pointcloud_processor.hpp"
#include "fluent_lib/fluent.hpp"
#include <iostream>
#include <chrono>
#include <algorithm>

namespace fv_aspara_analyzer {

namespace {
// 廃止: 個別の曲がり度計算は FluentCloud メトリクスに統合

// ユーティリティ: スケルトンの曲線長を計算
static double computeCurveLengthFromSkeleton(
    const std::vector<SkeletonPoint>& skel)
{
    if (skel.size() < 2) return 0.0;
    double total = 0.0;
    for (size_t i = 1; i < skel.size(); ++i) {
        const auto& a = skel[i - 1].world_point;
        const auto& b = skel[i].world_point;
        Eigen::Vector3d va(a.x, a.y, a.z);
        Eigen::Vector3d vb(b.x, b.y, b.z);
        total += (vb - va).norm();
    }
    return total;
}

// PCA系のローバーレベル計算は FluentCloud に統合済みのため、このファイルでは保持しない

}

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
                // [AT-START] 解析開始（スレッド内）
                // 目的: 以降の処理をアスパラ1本分に対して完結に行う
                auto analysis_start = std::chrono::steady_clock::now();
                RCLCPP_DEBUG(rclcpp::get_logger("analyzer_thread"), "Starting analysis for asparagus ID %d", aspara_copy.id);
                // 実測用の開始時刻
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
                // 安全: Clock種が異なる場合は同期チェックをスキップ
                double time_diff = 0.0;
                if (depth_time.get_clock_type() == color_time.get_clock_type()) {
                    time_diff = std::abs((depth_time - color_time).seconds());
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"),
                        "[SYNC] Skipping timestamp diff due to different clock types: depth=%d color=%d",
                        static_cast<int>(depth_time.get_clock_type()), static_cast<int>(color_time.get_clock_type()));
                    time_diff = 0.0; // チェックせず続行
                }
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

        // Feature switches (default off for simplicity/perf)
        bool enable_metrics = node_ && node_->has_parameter("enable_metrics") ?
            node_->get_parameter("enable_metrics").get_value<bool>() : true;
        bool enable_pca_skeleton = node_ && node_->has_parameter("enable_pca_skeleton") ?
            node_->get_parameter("enable_pca_skeleton").get_value<bool>() : false;
        bool verbose_logging = node_ && node_->has_parameter("verbose_logging") ?
            node_->get_parameter("verbose_logging").get_value<bool>() : false;

                // [AT-ROI] ROI点群の取得
                // 方針: organizedなregistered_pointsがあればそこからスライス（高速）
                //       無ければ深度+カラーからDepthToCloudで生成（フォールバック）
        fluent::utils::Stopwatch filter_stopwatch;
        // まずはregistered_pointsの有無を確認
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        bool sliced_from_registered = false;
        auto pc2 = node_->latest_pointcloud_;
        if (pc2 && pc2->width > 0 && pc2->height > 1) {
            // organized cloud 前提
            cv::Rect bboxC = aspara_info.bounding_box_2d; // color座標
            int rp_w = static_cast<int>(pc2->width);
            int rp_h = static_cast<int>(pc2->height);
            int col_w = color_image ? static_cast<int>(color_image->width) : rp_w;
            int col_h = color_image ? static_cast<int>(color_image->height) : rp_h;
            double kx = (col_w > 0) ? static_cast<double>(rp_w) / static_cast<double>(col_w) : 1.0;
            double ky = (col_h > 0) ? static_cast<double>(rp_h) / static_cast<double>(col_h) : 1.0;
            cv::Rect bboxRP(
                static_cast<int>(std::round(bboxC.x * kx)),
                static_cast<int>(std::round(bboxC.y * ky)),
                static_cast<int>(std::round(bboxC.width * kx)),
                static_cast<int>(std::round(bboxC.height * ky))
            );
            cv::Rect img_rect(0, 0, rp_w, rp_h);
            cv::Rect roi = bboxRP & img_rect;
            if (roi.area() > 0) {
                pcl::PointCloud<pcl::PointXYZRGB> full;
                pcl::fromROSMsg(*pc2, full);
                for (int v = roi.y; v < roi.y + roi.height; ++v) {
                    for (int u = roi.x; u < roi.x + roi.width; ++u) {
                        const pcl::PointXYZRGB& p = full.at(u, v);
                        if (!std::isfinite(p.z) || p.z <= 0.0f) continue;
                        filtered_cloud->points.push_back(p);
                    }
                }
                filtered_cloud->width = filtered_cloud->points.size();
                filtered_cloud->height = 1;
                filtered_cloud->is_dense = false;
                sliced_from_registered = true;
                RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"),
                    "[POINTCLOUD] Sliced %zu points from registered_points ROI", filtered_cloud->points.size());
            }
        }

        if (!sliced_from_registered) {
            // [AT-ROI-FALLBACK] 深度→点群のフォールバック経路
            RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"),
                "[FALLBACK] organized cloud not available. Building ROI cloud from depth image (slow path)");
            // 深度画像とカラー画像をOpenCV形式に変換し、従来ルートでROI点群を生成
            cv::Mat depth_mat, color_mat;
            try {
                RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"),
                    "[POINTCLOUD] Processing depth image: encoding=%s, size=%dx%d",
                    depth_image->encoding.c_str(), depth_image->width, depth_image->height);
                if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
                    depth_mat = depth_ptr->image;
                } else if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
                    depth_mat = depth_ptr->image;
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("analyzer_thread"), "[POINTCLOUD] Unsupported depth encoding: %s", depth_image->encoding.c_str());
                    return;
                }
                cv_bridge::CvImagePtr color_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
                color_mat = color_ptr->image;
            } catch (cv_bridge::Exception& e) {
                std::cout << "CV bridge exception in image conversion: " << e.what() << std::endl;
                return;
            }
            cv::Rect bbox_color = aspara_info.bounding_box_2d;
            double sx = static_cast<double>(depth_image->width) / static_cast<double>(color_image->width);
            double sy = static_cast<double>(depth_image->height) / static_cast<double>(color_image->height);
            if (!std::isfinite(sx) || sx <= 0.0) sx = 1.0;
            if (!std::isfinite(sy) || sy <= 0.0) sy = 1.0;
            cv::Rect bbox_depth(
                static_cast<int>(std::round(bbox_color.x * sx)),
                static_cast<int>(std::round(bbox_color.y * sy)),
                static_cast<int>(std::round(bbox_color.width * sx)),
                static_cast<int>(std::round(bbox_color.height * sy)));
            cv::Rect image_rect(0, 0, depth_image->width, depth_image->height);
            cv::Rect clipped = bbox_depth & image_rect;
            if (!clipped.area()) {
                RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"), "[POINTCLOUD] ROI outside image bounds");
                return;
            }
            float depth_scale_m = static_cast<float>(node_->depth_unit_m_16u_);
            if (depth_mat.type() == CV_16UC1) {
                double dmin, dmax; cv::minMaxLoc(depth_mat, &dmin, &dmax);
                if (dmax < 200 && depth_scale_m > 0.0001f) {
                    depth_scale_m = 0.0001f;
                    RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"), "[POINTCLOUD] override depth_scale=0.0001");
                }
            }
            filtered_cloud = fluent_cloud::io::DepthToCloud::convertAsparagusROI(
                depth_mat, color_mat, clipped, *camera_info, depth_scale_m);

            // ROI深度統計ログ（fallback経路のみ）
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
        }
        
        RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"), 
            "[POINTCLOUD] convertAsparagusROI returned successfully");
        aspara_info.processing_times.filter_bbox_ms = filter_stopwatch.elapsed_ms();
        
        if (verbose_logging) {
            RCLCPP_DEBUG(rclcpp::get_logger("analyzer_thread"),
                "[POINTCLOUD] Generated %zu points from depth image", filtered_cloud->points.size());
        }
        
        if (filtered_cloud->points.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"), 
                "[POINTCLOUD] No points found in bounding box - check depth data");
            return;
        }

        // [AT-FILTER] ノイズ除去（Voxel → SOR）
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

        // フィルタ済み点群をAsparaInfoにも保存（UI右パネル用）
        try {
            sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
            filtered_cloud_msg.header.stamp = depth_image->header.stamp;
            filtered_cloud_msg.header.frame_id = frame_id;
            filtered_cloud_msg.height = 1;
            filtered_cloud_msg.width = static_cast<uint32_t>(denoised_cloud->points.size());
            filtered_cloud_msg.is_bigendian = false;
            filtered_cloud_msg.is_dense = false;

            // フィールド定義: x,y,z(float32) + rgb(float32)
            filtered_cloud_msg.fields.clear();
            auto makeFieldF = [](const std::string& name, uint32_t offset) {
                sensor_msgs::msg::PointField f; f.name = name; f.offset = offset; f.datatype = 7; f.count = 1; return f; };
            filtered_cloud_msg.fields.push_back(makeFieldF("x", 0));
            filtered_cloud_msg.fields.push_back(makeFieldF("y", 4));
            filtered_cloud_msg.fields.push_back(makeFieldF("z", 8));
            filtered_cloud_msg.fields.push_back(makeFieldF("rgb", 12));

            filtered_cloud_msg.point_step = 16; // 4*float
            filtered_cloud_msg.row_step = filtered_cloud_msg.point_step * filtered_cloud_msg.width;
            filtered_cloud_msg.data.resize(filtered_cloud_msg.row_step);

            uint8_t* dst_f = filtered_cloud_msg.data.data();
            for (const auto& p : denoised_cloud->points) {
                float x = p.x; float y = p.y; float z = p.z;
                std::memcpy(dst_f + 0, &x, 4);
                std::memcpy(dst_f + 4, &y, 4);
                std::memcpy(dst_f + 8, &z, 4);
                uint32_t rgb = (uint32_t(p.r) << 16) | (uint32_t(p.g) << 8) | uint32_t(p.b);
                float rgbf;
                std::memcpy(&rgbf, &rgb, 4);
                std::memcpy(dst_f + 12, &rgbf, 4);
                dst_f += filtered_cloud_msg.point_step;
            }

            aspara_info.filtered_pointcloud = filtered_cloud_msg;
        } catch (...) {
            RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"), "Failed to build filtered_pointcloud msg for aspara %d", aspara_info.id);
        }

        // [AT-METRICS] アスパラガス特性（長さ・真っ直ぐ度など）
        float straightness = 0.0f;
        float length = 0.0f;
        if (enable_metrics) {
            fluent::utils::Stopwatch measurement_stopwatch;
            // メトリクスは denoised 基準
            auto metric_cloud = denoised_cloud;
            auto m = fluent_cloud::compute_pca_metrics(metric_cloud);
            // 直線度に正規化係数（デフォルト0.03）
            double kappa_ref = 0.03;
            try { if (node_->has_parameter("straightness_kappa_ref")) kappa_ref = node_->get_parameter("straightness_kappa_ref").get_value<double>(); } catch (...) {}
            straightness = static_cast<float>(std::clamp(1.0 - m.curvature_ratio / std::max(1e-6, kappa_ref), 0.0, 1.0));
            // スケルトンが生成されていれば曲線長を優先、無ければPCA長
            double L_skeleton = 0.0;
            if (!aspara_info.skeleton_points.empty()) {
                L_skeleton = computeCurveLengthFromSkeleton(aspara_info.skeleton_points);
            }
            length = static_cast<float>((L_skeleton > 0.0) ? L_skeleton : m.length_m);
            aspara_info.diameter = static_cast<float>(m.diameter_m);
            aspara_info.curvature = static_cast<float>(m.curvature_ratio);
            aspara_info.processing_times.measurement_ms = measurement_stopwatch.elapsed_ms();
        }
        
        // [AT-SKELETON] 可視化用 PCA ライン / 骨格生成（必要時のみ）
        if (enable_pca_skeleton) {
        fluent::utils::Stopwatch pca_stopwatch;
        auto pca_line = pointcloud_processor_->generatePCALine(denoised_cloud);
        aspara_info.processing_times.pca_calculation_ms = pca_stopwatch.elapsed_ms();
        }

        // PCA主軸に沿った直線の端点（tip/root）を推定し、骨格点列を作成
        if (enable_pca_skeleton) try {
            // 骨格生成には、実際にasparagus_pointcloudとして保存される点群（filtered_cloud）を使用
            // これはROI抽出後の生データで、2D投影が正しく機能している
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr skeleton_cloud = filtered_cloud;
            
            // デバッグ：filtered_cloudの内容確認
                RCLCPP_DEBUG(rclcpp::get_logger("analyzer_thread"),
                    "[SKELETON] Using filtered_cloud with %zu points", skeleton_cloud->points.size());
            
            // PCA直線の端点を計算（フォールバック用）
            Eigen::Vector3f end_tip, end_root;
            try {
                pcl::PCA<pcl::PointXYZRGB> pca;
                pca.setInputCloud(skeleton_cloud);
                Eigen::Matrix3f eigvec = pca.getEigenVectors();
                Eigen::Vector3f axis = eigvec.col(0).normalized();
                Eigen::Vector4f mean4 = pca.getMean();
                Eigen::Vector3f mean(mean4[0], mean4[1], mean4[2]);
                
                float tmin = std::numeric_limits<float>::max();
                float tmax = std::numeric_limits<float>::lowest();
                for (const auto& pt : skeleton_cloud->points) {
                    if (!std::isfinite(pt.z) || pt.z <= 0.0f) continue;
                    Eigen::Vector3f p(pt.x, pt.y, pt.z);
                    float t = axis.dot(p - mean);
                    tmin = std::min(tmin, t);
                    tmax = std::max(tmax, t);
                }
                
                Eigen::Vector3f end_1 = mean + tmin * axis;
                Eigen::Vector3f end_2 = mean + tmax * axis;
                
                if (end_1.y() < end_2.y()) {
                    end_tip = end_1;
                    end_root = end_2;
                } else {
                    end_tip = end_2;
                    end_root = end_1;
                }
            } catch (...) {
                // PCA失敗時のデフォルト
                end_tip = Eigen::Vector3f(0, 0, 1);
                end_root = Eigen::Vector3f(0, 0.1, 1);
            }
            
            // 骨格サンプル（5点） - 点群の実際の広がりから直接サンプリング
            std::vector<SkeletonPoint> skel;
            const int NUM = 5;
            
            // 点群のY座標（高さ）の最小値と最大値を取得
            float y_min = std::numeric_limits<float>::max();
            float y_max = std::numeric_limits<float>::lowest();
            int valid_count = 0;
            for (const auto& pt : skeleton_cloud->points) {
                if (!std::isfinite(pt.y) || !std::isfinite(pt.z) || pt.z <= 0.0f) continue;
                y_min = std::min(y_min, pt.y);
                y_max = std::max(y_max, pt.y);
                valid_count++;
            }
            
            RCLCPP_DEBUG(rclcpp::get_logger("analyzer_thread"),
                "[SKELETON] Valid points: %d, Y range: [%.3f, %.3f]", 
                valid_count, y_min, y_max);
            
            // Y軸は下向き（RealSenseカメラ座標系）なので、y_minが上端、y_maxが下端
            float y_range = y_max - y_min;
            if (y_range <= 0.0f) {
                RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"),
                    "[SKELETON] Invalid Y range: %.3f", y_range);
                // フォールバック：PCA直線を使用
                for (int i = 0; i < NUM; ++i) {
                    float alpha = static_cast<float>(i) / (NUM - 1);
                    Eigen::Vector3f p = end_root * (1.0f - alpha) + end_tip * alpha;
                    pcl::PointXYZRGB pxyz; 
                    pxyz.x = p.x(); 
                    pxyz.y = p.y(); 
                    pxyz.z = p.z();
                    
                    cv::Point2f uv = pointcloud_processor_->project3DTo2D(pxyz, *camera_info);
                    if (uv.x < 0 || uv.y < 0 || !std::isfinite(uv.x) || !std::isfinite(uv.y)) {
                        uv.x = std::max(0.0f, std::min(uv.x, static_cast<float>(camera_info->width - 1)));
                        uv.y = std::max(0.0f, std::min(uv.y, static_cast<float>(camera_info->height - 1)));
                    }
                    
                    SkeletonPoint sp;
                    sp.image_point = uv;
                    sp.world_point.x = pxyz.x; 
                    sp.world_point.y = pxyz.y; 
                    sp.world_point.z = pxyz.z;
                    sp.distance_from_base = alpha;
                    skel.push_back(sp);
                }
            } else {
                // 高さ方向にスライスして、各スライスの重心を計算
                for (int i = 0; i < NUM; ++i) {
                    // rootからtipに向かって（下から上へ）
                    float slice_ratio = static_cast<float>(i) / (NUM - 1);
                    float y_center = y_max - slice_ratio * y_range;  // y_maxが根元、y_minが先端
                    float slice_height = y_range / NUM;  // 各スライスの高さ
                    float y_lower = y_center - slice_height * 0.5f;
                    float y_upper = y_center + slice_height * 0.5f;
                    
                    // このスライス内の点を収集
                    std::vector<Eigen::Vector3f> slice_points;
                    for (const auto& pt : skeleton_cloud->points) {
                        if (!std::isfinite(pt.y) || !std::isfinite(pt.z) || pt.z <= 0.0f) continue;
                        if (pt.y >= y_lower && pt.y <= y_upper) {
                            slice_points.push_back(Eigen::Vector3f(pt.x, pt.y, pt.z));
                        }
                    }
                    
                    // スライスの重心を計算
                    pcl::PointXYZRGB pxyz;
                    if (!slice_points.empty()) {
                        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
                        for (const auto& p : slice_points) {
                            centroid += p;
                        }
                        centroid /= static_cast<float>(slice_points.size());
                        pxyz.x = centroid.x();
                        pxyz.y = centroid.y();
                        pxyz.z = centroid.z();
                    } else {
                        // スライスに点がない場合は、PCA直線上の点を使用
                        float alpha = static_cast<float>(i) / (NUM - 1);
                        Eigen::Vector3f p = end_root * (1.0f - alpha) + end_tip * alpha;
                        pxyz.x = p.x();
                        pxyz.y = p.y();
                        pxyz.z = p.z();
                    }
                    
                    // 2D投影
                    cv::Point2f uv = pointcloud_processor_->project3DTo2D(pxyz, *camera_info);
                    
                    // デバッグ: 投影結果を確認
                    RCLCPP_DEBUG(rclcpp::get_logger("analyzer_thread"),
                        "[SKELETON] Slice %d: Y[%.3f-%.3f] points=%zu -> 3D(%.3f,%.3f,%.3f) -> 2D(%.1f,%.1f)",
                        i, y_lower, y_upper, slice_points.size(),
                        pxyz.x, pxyz.y, pxyz.z, uv.x, uv.y);
                    
                    // 骨格ポイントとして追加
                    SkeletonPoint skeleton_point;
                    skeleton_point.image_point = uv;
                    skeleton_point.world_point.x = pxyz.x;
                    skeleton_point.world_point.y = pxyz.y;
                    skeleton_point.world_point.z = pxyz.z;
                    skeleton_point.distance_from_base = slice_ratio;
                    skel.push_back(skeleton_point);
                }
                
                // 骨格点数を出力
                RCLCPP_DEBUG(rclcpp::get_logger("analyzer_thread"),
                    "[SKELETON] Generated %zu skeleton points", skel.size());
            }
            
            aspara_info.skeleton_points = skel;
            RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"),
                "[SKELETON] Generated %zu skeleton points for aspara ID %d",
                skel.size(), aspara_info.id);
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"), "PCA skeleton failed: %s", e.what());
        }

        // 収穫適性を判定
        bool is_harvestable = false;
        if (enable_metrics) {
            is_harvestable = (length >= node_->harvest_min_length_ && 
                              length <= node_->harvest_max_length_ && 
                              straightness >= node_->straightness_threshold_);
        }

        // 可視化処理時間の計測
        fluent::utils::Stopwatch vis_stopwatch;
        
        // ROI深度統計ログはfallback経路で実施済み

        // 右パネル用: ROI下部帯(高さ10%)の深度ヒストグラムから最短ピークz0を推定
        const std::string frame_id_to_use = frame_id;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr foreground(new pcl::PointCloud<pcl::PointXYZRGB>);
        // フィルタ基底はROI生点群を優先（点数確保）。無い場合はdenoisedにフォールバック
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud = (!filtered_cloud->points.empty()) ? filtered_cloud : denoised_cloud;
        if (!base_cloud->points.empty()) {
            // パラメータ
            // デフォルトは帯無効（距離のみ）
            double band_alpha = node_ && node_->has_parameter("band_alpha") ?
                node_->get_parameter("band_alpha").get_value<double>() : 1.00; // 1.0で帯カット無効
            double bottom_ratio = node_ && node_->has_parameter("hist_band_bottom_ratio") ?
                node_->get_parameter("hist_band_bottom_ratio").get_value<double>() : 0.05;
            double top_ratio = node_ && node_->has_parameter("hist_band_top_ratio") ?
                node_->get_parameter("hist_band_top_ratio").get_value<double>() : 0.10;
            double percentile = node_ && node_->has_parameter("root_depth_percentile") ?
                node_->get_parameter("root_depth_percentile").get_value<double>() : 0.10;
            // デフォルトは色/セグ判定オフ（距離のみ）
            bool use_seg = node_ && node_->has_parameter("root_use_segmentation") ?
                node_->get_parameter("root_use_segmentation").get_value<bool>() : false;
            // グローバルな領域認識のON/OFF
            bool enable_region = node_ && node_->has_parameter("enable_region_recognition") ?
                node_->get_parameter("enable_region_recognition").get_value<bool>() : false;
            // HSV閾値
            double hmin = 35.0, hmax = 85.0, smin = 0.25, vmin = 0.20;
            if (node_) {
                if (node_->has_parameter("root_hsv_h_min")) hmin = node_->get_parameter("root_hsv_h_min").get_value<double>();
                if (node_->has_parameter("root_hsv_h_max")) hmax = node_->get_parameter("root_hsv_h_max").get_value<double>();
                if (node_->has_parameter("root_hsv_s_min")) smin = node_->get_parameter("root_hsv_s_min").get_value<double>();
                if (node_->has_parameter("root_hsv_v_min")) vmin = node_->get_parameter("root_hsv_v_min").get_value<double>();
            }
            double dist_window = node_ ? node_->aspara_filter_distance_ : 0.05; // ±距離窓

            // 画像データ（カラー/マスク）: 簡易モードでは取得しない
            cv::Mat color_mat, mask_mat, hsv;
            if (use_seg && enable_region) {
                std::lock_guard<std::mutex> lk(node_->image_data_mutex_);
                if (node_->latest_color_image_) {
                    color_mat = cv_bridge::toCvCopy(node_->latest_color_image_, sensor_msgs::image_encodings::BGR8)->image;
                    cv::cvtColor(color_mat, hsv, cv::COLOR_BGR2HSV);
                }
                if (!node_->latest_mask_.empty()) mask_mat = node_->latest_mask_.clone();
            }

            const cv::Rect& roi = aspara_info.bounding_box_2d;
            int y0 = roi.y + static_cast<int>(std::floor(roi.height * (1.0 - top_ratio)));
            int y1 = roi.y + static_cast<int>(std::floor(roi.height * (1.0 - bottom_ratio)));
            y0 = std::clamp(y0, roi.y, roi.y + roi.height - 1);
            y1 = std::clamp(y1, roi.y, roi.y + roi.height - 1);
            if (y1 < y0) std::swap(y0, y1);

            // 色/セグメント判定ヘルパ（この後の全処理で利用）
            auto is_green = [&](int u, int v) -> bool {
                if (!use_seg) return true;                     // 完全オフ
                if (!mask_mat.empty()) {                        // マスク優先
                    if (!color_mat.empty() && mask_mat.size()!=color_mat.size()) {
                        int um = std::clamp(static_cast<int>(std::round(u * (mask_mat.cols / static_cast<double>(color_mat.cols)))), 0, mask_mat.cols-1);
                        int vm = std::clamp(static_cast<int>(std::round(v * (mask_mat.rows / static_cast<double>(color_mat.rows)))), 0, mask_mat.rows-1);
                        return mask_mat.at<uint8_t>(vm, um) > 127;
                    }
                    return mask_mat.at<uint8_t>(v, u) > 127;
                }
                if (hsv.empty()) return true;                 // HSV無しなら許可
                cv::Vec3b px = hsv.at<cv::Vec3b>(v, u);
                double H = (px[0] * 2.0);
                double S = px[1] / 255.0;
                double V = px[2] / 255.0;
                return (H>=hmin && H<=hmax && S>=smin && V>=vmin);
            };

            // ==== 下部帯のDepthを2D（列ごと）に評価して、見た目と一致する帯ストリップを作成 ====
            // ROI（カラー座標）→ Depth座標へスケーリング
            cv::Mat depth_mat;
            if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                depth_mat = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1)->image;
            } else if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                depth_mat = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1)->image;
            }
            const int cw = (!color_mat.empty()) ? color_mat.cols : depth_mat.cols; // 保険
            const int ch = (!color_mat.empty()) ? color_mat.rows : depth_mat.rows;
            const int dw = depth_mat.cols, dh = depth_mat.rows;
            const double sx = cw > 0 ? static_cast<double>(dw) / cw : 1.0;
            const double sy = ch > 0 ? static_cast<double>(dh) / ch : 1.0;
            cv::Rect droi(
                std::clamp(static_cast<int>(std::round(roi.x * sx)), 0, dw-1),
                std::clamp(static_cast<int>(std::round(roi.y * sy)), 0, dh-1),
                std::clamp(static_cast<int>(std::round(roi.width * sx)), 1, dw),
                std::clamp(static_cast<int>(std::round(roi.height * sy)), 1, dh)
            );
            droi.width = std::min(droi.width, dw - droi.x);
            droi.height = std::min(droi.height, dh - droi.y);
            int y0d = droi.y + static_cast<int>(std::floor(droi.height * (1.0 - top_ratio)));
            int y1d = droi.y + static_cast<int>(std::floor(droi.height * (1.0 - bottom_ratio)));
            y0d = std::clamp(y0d, droi.y, droi.y + droi.height - 1);
            y1d = std::clamp(y1d, droi.y, droi.y + droi.height - 1);
            if (y1d < y0d) std::swap(y0d, y1d);
            // 列ごとの最小（近距離）深度を計算
            std::vector<float> col_min_z(droi.width, std::numeric_limits<float>::infinity());
            const double depth_scale = (depth_mat.type()==CV_16UC1) ? node_->depth_unit_m_16u_ : 1.0;
            for (int ux = droi.x; ux < droi.x + droi.width; ++ux) {
                float zmin_col = std::numeric_limits<float>::infinity();
                for (int vy = y0d; vy <= y1d; ++vy) {
                    float z = 0.0f;
                    if (depth_mat.type()==CV_16UC1) {
                        uint16_t mm = depth_mat.at<uint16_t>(vy, ux);
                        if (mm==0) continue;
                        z = static_cast<float>(mm) * static_cast<float>(depth_scale);
                    } else {
                        float m = depth_mat.at<float>(vy, ux);
                        if (!std::isfinite(m) || m<=0.0f) continue;
                        z = m;
                    }
                    // オプションの色/セグ制限
                    if (enable_region && !color_mat.empty()) {
                        int uc = std::clamp(static_cast<int>(std::round(ux / sx)), 0, cw-1);
                        int vc = std::clamp(static_cast<int>(std::round(vy / sy)), 0, ch-1);
                        if (!is_green(uc, vc)) continue;
                    }
                    if (z < zmin_col) zmin_col = z;
                }
                col_min_z[ux - droi.x] = zmin_col;
            }
            // 近距離の列x*を採用
            int best_xi = -1; float best_z = std::numeric_limits<float>::infinity();
            for (int i = 0; i < static_cast<int>(col_min_z.size()); ++i) {
                float z = col_min_z[i];
                if (std::isfinite(z) && z > 0.0f && z < best_z) { best_z = z; best_xi = i; }
            }
            float z0 = best_z;
            int bins = 64; float zmin=0.2f, zmax=2.0f; if (node_) { try{
                if (node_->has_parameter("hist_z_min_m")) zmin = node_->get_parameter("hist_z_min_m").get_value<double>();
                if (node_->has_parameter("hist_z_max_m")) zmax = node_->get_parameter("hist_z_max_m").get_value<double>();
                if (node_->has_parameter("hist_bins")) bins = node_->get_parameter("hist_bins").get_value<int>();
            }catch(...){} }
            bins = std::max(8, std::min(256, bins));
            // zレンジ（表示用）
            int best_v = 1; // 正規化用ダミー
            if (!std::isfinite(zmin) || !std::isfinite(zmax) || !(zmax>zmin)) { zmin = 0.2f; zmax = 2.0f; }

            // ヒストグラムストリップ画像を8px高で作成（UI用にAsparaInfoへ格納）
            try {
                const int strip_h = 8; int strip_w = std::max(16, roi.width);
                cv::Mat strip(strip_h, strip_w, CV_8UC1, cv::Scalar(0));
                // 明度 = 近距離ほど明（列ごとの z を使って BGR化しやすいストリップに）
                for (int x=0; x<strip_w; ++x) {
                    // 表示列x→Depth列ux
                    int ux = droi.x + std::clamp(static_cast<int>(std::round(x * sx)), 0, droi.width-1);
                    float z = col_min_z[std::clamp(ux - droi.x, 0, droi.width-1)];
                    float bright = 0.0f;
                    if (std::isfinite(z) && z>0.0f) bright = std::clamp((zmax - z) / (zmax - zmin), 0.0f, 1.0f);
                    uint8_t yv = static_cast<uint8_t>(std::round(bright * 255.0f));
                    strip.col(x).setTo(yv);
                }
                // 推定z0位置に白ライン（列ベース）
                int zx = (best_xi >= 0) ? static_cast<int>(std::round((static_cast<float>(best_xi) / std::max(1, droi.width-1)) * strip_w)) : -1;
                if (zx>=0 && zx<strip_w) strip.col(zx).setTo(255);
                aspara_info.depth_histogram_strip = strip;
                // z0の正規化位置（0..1）を保存（UI側の赤丸表示用）
                if (best_xi >= 0) {
                    aspara_info.z0_m = z0;
                    aspara_info.z0_x_norm = static_cast<float>(best_xi) / static_cast<float>(std::max(1, droi.width-1));
                    aspara_info.z0_norm = aspara_info.z0_x_norm; // UIは横位置を使う
                } else { aspara_info.z0_norm = -1.0f; aspara_info.z0_x_norm = -1.0f; }
            } catch (...) {}

            // 中央帯境界
            int band_half = static_cast<int>(std::round(roi.width * band_alpha * 0.5));
            int cx_px = roi.x + roi.width/2;
            int band_left = cx_px - band_half;
            int band_right = cx_px + band_half;

            // z0±dist_window かつ 中央帯 かつ セグメント/緑画素 に限定
            // さらに根本X周りの横方向半径で制限（背景面の混入抑制）
            // デフォルトは横半径制限オフ
            double lateral_radius_m = node_ && node_->has_parameter("root_lateral_radius_m") ?
                node_->get_parameter("root_lateral_radius_m").get_value<double>() : 0.0;
            // シンプル背面カット（z0+cut以降を除去）
            bool simple_cut = node_ && node_->has_parameter("simple_z_back_cut") ?
                node_->get_parameter("simple_z_back_cut").get_value<bool>() : true;
            double z_back = node_ && node_->has_parameter("z_back_cut_m") ?
                node_->get_parameter("z_back_cut_m").get_value<double>() : 0.15;
            for (const auto& p : base_cloud->points) {
                if (!std::isfinite(p.z) || p.z <= 0.0f) continue;
                if (simple_cut) {
                    if ((p.z - z0) > z_back) continue; // 背面カットのみ
                } else {
                    if (std::fabs(p.z - z0) > dist_window) continue;
                }
                // 横方向の半径制限（xのみ）。根本X中心から±半径以内のみ採用
                if (std::isfinite(aspara_info.root_position_3d.x) && lateral_radius_m > 0.0) {
                    if (std::fabs(static_cast<double>(p.x) - aspara_info.root_position_3d.x) > lateral_radius_m) continue;
                }
                cv::Point2f uv = pointcloud_processor_->project3DTo2D(p, *camera_info);
                if (!std::isfinite(uv.x) || !std::isfinite(uv.y)) continue;
                if (uv.x < roi.x || uv.x >= roi.x + roi.width || uv.y < roi.y || uv.y >= roi.y + roi.height) continue;
                // 中央帯カットは band_alpha < 0.99 のときだけ有効
                if (band_alpha < 0.99) {
                    if (uv.x < band_left || uv.x > band_right) continue;
                }
                if (!color_mat.empty()) {
                    int uu = static_cast<int>(std::round(uv.x));
                    int vv = static_cast<int>(std::round(uv.y));
                    uu = std::clamp(uu, 0, color_mat.cols - 1);
                    vv = std::clamp(vv, 0, color_mat.rows - 1);
                    if (!is_green(uu, vv)) continue;
                }
                foreground->points.push_back(p);
            }
            foreground->width = foreground->points.size();
            foreground->height = 1;
            foreground->is_dense = false;

            // 本当の根本/先端＋中間（PCA主軸で等間隔にサンプリングして局所重心を取る）
            if (!foreground->points.empty()) {
                // PCA
                Eigen::Vector4f mean4; pcl::compute3DCentroid(*foreground, mean4);
                Eigen::Matrix3f cov; pcl::computeCovarianceMatrixNormalized(*foreground, Eigen::Vector4f(mean4[0],mean4[1],mean4[2],1.0f), cov);
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
                Eigen::Vector3f u = es.eigenvectors().col(2).normalized(); // 最大固有値の固有ベクトル
                Eigen::Vector3f c(mean4[0], mean4[1], mean4[2]);

                // t分布（外れ値5%トリム）
                std::vector<float> ts; ts.reserve(foreground->points.size());
                for (const auto& p : foreground->points) {
                    if (!std::isfinite(p.z) || p.z <= 0.0f) continue;
                    Eigen::Vector3f v(p.x, p.y, p.z);
                    ts.push_back(u.dot(v - c));
                }
                if (!ts.empty()) {
                    size_t q05i = static_cast<size_t>(ts.size() * 0.05f);
                    size_t q95i = static_cast<size_t>(ts.size() * 0.95f);
                    std::nth_element(ts.begin(), ts.begin()+q05i, ts.end());
                    float tmin = ts[q05i];
                    std::nth_element(ts.begin(), ts.begin()+q95i, ts.end());
                    float tmax = ts[q95i];

                    // サンプリング点数と半径
                    int skel_n = 5;
                    if (node_ && node_->has_parameter("skeleton_points_count")) {
                        skel_n = std::max<int>(2, static_cast<int>(node_->get_parameter("skeleton_points_count").get_value<int>()));
                    }
                    double R = node_ && node_->has_parameter("skeleton_radius_m") ?
                        node_->get_parameter("skeleton_radius_m").get_value<double>() : 0.01; // 1cm
                    double W = node_ && node_->has_parameter("skeleton_window_m") ?
                        node_->get_parameter("skeleton_window_m").get_value<double>() : 0.03; // 軸方向±3cm

                    aspara_info.skeleton_points.clear();
                    for (int i = 0; i < skel_n; ++i) {
                        float alpha = (skel_n == 1) ? 0.0f : static_cast<float>(i) / static_cast<float>(skel_n - 1);
                        float ti = tmin + alpha * (tmax - tmin);
                        // 近傍収集
                        Eigen::Vector3f sum = Eigen::Vector3f::Zero();
                        int count = 0;
                        for (const auto& p : foreground->points) {
                            if (!std::isfinite(p.z) || p.z <= 0.0f) continue;
                            Eigen::Vector3f v(p.x, p.y, p.z);
                            float t = u.dot(v - c);
                            if (std::fabs(t - ti) > W) continue; // 軸方向窓
                            // 垂直距離
                            Eigen::Vector3f d = (v - c) - u * t;
                            if (d.norm() > R) continue;
                            sum += v; ++count;
                        }
                        Eigen::Vector3f pt;
                        if (count > 0) pt = sum / static_cast<float>(count); else pt = (c + u * ti);
                        SkeletonPoint sp;
                        sp.world_point.x = pt.x(); sp.world_point.y = pt.y(); sp.world_point.z = pt.z();
                        sp.distance_from_base = alpha;
                        // 2D投影（表示用）
                        if (camera_info) {
                            cv::Point2f uv = pointcloud_processor_->project3DTo2D(pcl::PointXYZRGB(pt.x(), pt.y(), pt.z()), *camera_info);
                            sp.image_point = uv;
                        }
                        aspara_info.skeleton_points.push_back(sp);
                    }

                    // 根本=最下端（yが最大）を採用
                    Eigen::Vector3f root3;
                    {
                        const auto& first = aspara_info.skeleton_points.front().world_point;
                        const auto& last  = aspara_info.skeleton_points.back().world_point;
                        if (last.y > first.y) { root3 = Eigen::Vector3f(last.x, last.y, last.z); }
                        else { root3 = Eigen::Vector3f(first.x, first.y, first.z); }
                    }
                    aspara_info.root_position_3d.x = root3.x();
                    aspara_info.root_position_3d.y = root3.y();
                    aspara_info.root_position_3d.z = root3.z();
                }
            }
        }
        // 右用点群をパブリッシュ（空なら従来のdenoised）
        bool disable_fb = false;
        try {
            if (node_->has_parameter("disable_filtered_fallback")) {
                disable_fb = node_->get_parameter("disable_filtered_fallback").get_value<bool>();
            }
        } catch (...) {}
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr to_publish =
            (foreground->points.empty() && !disable_fb) ? denoised_cloud : foreground;
        // 上限点数（単純サブサンプリング）。デフォルト0=無制限
        size_t max_points = 0;
        try {
            if (node_->has_parameter("filtered_points_max")) {
                int v = node_->get_parameter("filtered_points_max").get_value<int>();
                if (v > 0) max_points = static_cast<size_t>(v);
            }
        } catch (...) {}
        if (max_points > 0 && to_publish->points.size() > max_points) {
            size_t stride = (to_publish->points.size() + max_points - 1) / max_points;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr limited(new pcl::PointCloud<pcl::PointXYZRGB>);
            limited->points.reserve(max_points);
            for (size_t i = 0; i < to_publish->points.size(); i += stride) {
                limited->points.push_back(to_publish->points[i]);
            }
            limited->width = limited->points.size();
            limited->height = 1; limited->is_dense = false;
            to_publish = limited;
        }
        pointcloud_processor_->publishFilteredPointCloud(
            to_publish,
            frame_id_to_use, aspara_info.id, depth_image->header.stamp);

        // UI右パネル用に AsparaInfo.filtered_pointcloud を最新の前景へ更新
        try {
            sensor_msgs::msg::PointCloud2 filt_msg;
            filt_msg.header.stamp = depth_image->header.stamp;
            filt_msg.header.frame_id = frame_id_to_use;
            filt_msg.height = 1;
            filt_msg.width = static_cast<uint32_t>(to_publish->points.size());
            filt_msg.is_bigendian = false;
            filt_msg.is_dense = false;
            filt_msg.fields.clear();
            auto makeFieldF = [](const std::string& name, uint32_t offset) {
                sensor_msgs::msg::PointField f; f.name = name; f.offset = offset; f.datatype = 7; f.count = 1; return f; };
            filt_msg.fields.push_back(makeFieldF("x", 0));
            filt_msg.fields.push_back(makeFieldF("y", 4));
            filt_msg.fields.push_back(makeFieldF("z", 8));
            filt_msg.fields.push_back(makeFieldF("rgb", 12));
            filt_msg.point_step = 16;
            filt_msg.row_step = filt_msg.point_step * filt_msg.width;
            filt_msg.data.resize(filt_msg.row_step);
            uint8_t* dst_f = filt_msg.data.data();
            for (const auto& p : to_publish->points) {
                float x = p.x, y = p.y, z = p.z; std::memcpy(dst_f + 0, &x, 4); std::memcpy(dst_f + 4, &y, 4); std::memcpy(dst_f + 8, &z, 4);
                uint32_t rgb = (uint32_t(p.r) << 16) | (uint32_t(p.g) << 8) | uint32_t(p.b); float rgbf; std::memcpy(&rgbf, &rgb, 4); std::memcpy(dst_f + 12, &rgbf, 4);
                dst_f += filt_msg.point_step;
            }
            aspara_info.filtered_pointcloud = filt_msg;
        } catch (...) {}
        
        // 選択中のアスパラガスの点群をasparagus_pointcloudとして公開
        // フィルター前の生データ（ROI抽出後）を公開
        RCLCPP_WARN(rclcpp::get_logger("analyzer_thread"), 
            "[DEBUG] Checking pointcloud publish: selected_id=%d, current_id=%d, pub_valid=%d", 
            node_->selected_aspara_id_, aspara_info.id, (node_->selected_pointcloud_pub_ != nullptr));
        
        if (node_->selected_aspara_id_ == aspara_info.id && node_->selected_pointcloud_pub_) {
            // Viewer互換性のため、registered_pointsと同じフォーマット
            // x,y,z(float32) + rgb(float32) に統一する
            sensor_msgs::msg::PointCloud2 selected_cloud_msg;
            selected_cloud_msg.header.stamp = depth_image->header.stamp;
            selected_cloud_msg.header.frame_id = frame_id_to_use;
            selected_cloud_msg.height = 1;
            selected_cloud_msg.width = static_cast<uint32_t>(filtered_cloud->points.size());
            selected_cloud_msg.is_bigendian = false;
            selected_cloud_msg.is_dense = false;

            // フィールド定義: x,y,z(float32) + rgb(float32)
            selected_cloud_msg.fields.clear();
            auto makeField = [](const std::string& name, uint32_t offset, uint8_t datatype) {
                sensor_msgs::msg::PointField f; f.name = name; f.offset = offset; f.datatype = datatype; f.count = 1; return f; };
            selected_cloud_msg.fields.push_back(makeField("x", 0, 7));   // FLOAT32
            selected_cloud_msg.fields.push_back(makeField("y", 4, 7));   // FLOAT32
            selected_cloud_msg.fields.push_back(makeField("z", 8, 7));   // FLOAT32
            selected_cloud_msg.fields.push_back(makeField("rgb", 12, 7)); // FLOAT32

            selected_cloud_msg.point_step = 16; // 4*float
            selected_cloud_msg.row_step = selected_cloud_msg.point_step * selected_cloud_msg.width;
            selected_cloud_msg.data.resize(selected_cloud_msg.row_step);

            uint8_t* dst = selected_cloud_msg.data.data();
            for (const auto& p : filtered_cloud->points) {
                float x = p.x; float y = p.y; float z = p.z;
                std::memcpy(dst + 0, &x, 4);
                std::memcpy(dst + 4, &y, 4);
                std::memcpy(dst + 8, &z, 4);
                // Pack BGR -> RGB 8:8:8 into float32 as in registered_points
                uint32_t rgb = (uint32_t(p.r) << 16) | (uint32_t(p.g) << 8) | uint32_t(p.b);
                float rgbf;
                std::memcpy(&rgbf, &rgb, 4);
                std::memcpy(dst + 12, &rgbf, 4);
                dst += selected_cloud_msg.point_step;
            }

            node_->selected_pointcloud_pub_->publish(selected_cloud_msg);

            // AsparaInfo構造体にも点群を保存
            aspara_info.asparagus_pointcloud = selected_cloud_msg;

            RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"),
                "[POINTCLOUD] Published asparagus_pointcloud for ID:%d, Points:%zu (raw ROI data)",
                aspara_info.id, filtered_cloud->points.size());
        }
        
        // TFの代わりにMarkerで可視化
        // PCA主軸方向をMarker姿勢に反映
        geometry_msgs::msg::Vector3 axis_dir_msg; axis_dir_msg.x = 0; axis_dir_msg.y = 0; axis_dir_msg.z = 1;
        if (enable_pca_skeleton) {
        try {
            pcl::PCA<pcl::PointXYZRGB> pca_axis; pca_axis.setInputCloud(denoised_cloud);
            Eigen::Vector3f axis = pca_axis.getEigenVectors().col(0).normalized();
            axis_dir_msg.x = axis.x(); axis_dir_msg.y = axis.y(); axis_dir_msg.z = axis.z();
            } catch (...) {}
        }

        // 画像上の赤丸と一致させるため、帯ベース推定の根本座標を使用
        pointcloud_processor_->publishAsparaMarker(
            aspara_info.root_position_3d, frame_id_to_use, aspara_info.id, length, is_harvestable, depth_image->header.stamp, axis_dir_msg);
        
        // アスパラ情報を更新
        // 曲がり度: スケルトン中心線ベース（既定）/ PCA比 などを組合せ
        if (enable_metrics) {
            aspara_info.length = length;             // m
            aspara_info.straightness = straightness; // 0..1
            aspara_info.is_harvestable = is_harvestable;
            // グレード判定
            double len_a = 0.23, len_b = 0.20, len_ng = 0.18;    // [m]
            double st_a = 0.70, st_b = 0.60;                     // [-]
            double dia_a_min = 0.008, dia_a_max = 0.014;         // [m]
            double dia_b_min = 0.007, dia_b_max = 0.016;         // [m]
            try {
                if (node_->has_parameter("grade_length_a_min_m")) len_a = node_->get_parameter("grade_length_a_min_m").get_value<double>();
                if (node_->has_parameter("grade_length_b_min_m")) len_b = node_->get_parameter("grade_length_b_min_m").get_value<double>();
                if (node_->has_parameter("grade_length_ng_min_m")) len_ng = node_->get_parameter("grade_length_ng_min_m").get_value<double>();
                if (node_->has_parameter("grade_straightness_a_min")) st_a = node_->get_parameter("grade_straightness_a_min").get_value<double>();
                if (node_->has_parameter("grade_straightness_b_min")) st_b = node_->get_parameter("grade_straightness_b_min").get_value<double>();
                if (node_->has_parameter("grade_diameter_a_min_m")) dia_a_min = node_->get_parameter("grade_diameter_a_min_m").get_value<double>();
                if (node_->has_parameter("grade_diameter_a_max_m")) dia_a_max = node_->get_parameter("grade_diameter_a_max_m").get_value<double>();
                if (node_->has_parameter("grade_diameter_b_min_m")) dia_b_min = node_->get_parameter("grade_diameter_b_min_m").get_value<double>();
                if (node_->has_parameter("grade_diameter_b_max_m")) dia_b_max = node_->get_parameter("grade_diameter_b_max_m").get_value<double>();
            } catch (...) {}
            auto &info = aspara_info;
            // NG先判定
            if (info.length < len_ng || info.diameter < 0.006 || info.diameter > 0.018) {
                info.grade = AsparaguGrade::OUT_OF_SPEC;
            } else if (info.length >= len_a && info.straightness >= st_a && info.diameter >= dia_a_min && info.diameter <= dia_a_max) {
                info.grade = AsparaguGrade::A_GRADE;
            } else if (info.length >= len_b && info.straightness >= st_b && info.diameter >= dia_b_min && info.diameter <= dia_b_max) {
                info.grade = AsparaguGrade::B_GRADE;
            } else {
                info.grade = AsparaguGrade::C_GRADE;
            }
        } else {
            aspara_info.length = 0.0f;
            aspara_info.straightness = 0.0f;
            aspara_info.is_harvestable = false;
        }
        // root_position_3d は帯ベース推定で既に設定済み（上書きしない）
        
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
        it->skeleton_points = aspara_info.skeleton_points;  // 骨格ポイントも更新
        // ヒストグラム帯も転送
        it->depth_histogram_strip = aspara_info.depth_histogram_strip;
        
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