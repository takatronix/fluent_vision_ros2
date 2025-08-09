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

namespace {
// ユーティリティ: スケルトンから曲がり度を計算
static double computeCurvatureFromSkeleton(
    const std::vector<SkeletonPoint>& skel,
    double& out_arc_excess,
    double& out_rms_norm)
{
    out_arc_excess = 0.0;
    out_rms_norm = 0.0;
    if (skel.size() < 3) return 0.0;

    // Lchord
    const auto& p0 = skel.front().world_point;
    const auto& p1 = skel.back().world_point;
    Eigen::Vector3d a(p0.x, p0.y, p0.z);
    Eigen::Vector3d b(p1.x, p1.y, p1.z);
    double Lchord = (b - a).norm();
    if (Lchord <= 1e-6) return 0.0;

    // Lpoly
    double Lpoly = 0.0;
    for (size_t i = 1; i < skel.size(); ++i) {
        const auto& q0 = skel[i - 1].world_point;
        const auto& q1 = skel[i].world_point;
        Eigen::Vector3d v0(q0.x, q0.y, q0.z);
        Eigen::Vector3d v1(q1.x, q1.y, q1.z);
        Lpoly += (v1 - v0).norm();
    }
    out_arc_excess = std::max(0.0, (Lpoly - Lchord) / Lchord);

    // RMS perpendicular to chord
    Eigen::Vector3d dir = (b - a).normalized();
    double sum2 = 0.0; size_t n = 0;
    for (const auto& sp : skel) {
        Eigen::Vector3d p(sp.world_point.x, sp.world_point.y, sp.world_point.z);
        Eigen::Vector3d d = p - a;
        Eigen::Vector3d perp = d - dir * d.dot(dir);
        double dist = perp.norm();
        sum2 += dist * dist; ++n;
    }
    double rms = (n > 0) ? std::sqrt(sum2 / static_cast<double>(n)) : 0.0;
    out_rms_norm = rms / Lchord;
    return std::max(out_arc_excess, out_rms_norm);
}
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

        // 可能ならregistered_points（organized cloud）からROIスライス、なければ深度→点群変換
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

        // アスパラガス特性を計算
        fluent::utils::Stopwatch measurement_stopwatch;
        auto root_position = pointcloud_processor_->estimateRootPosition(denoised_cloud);
        float straightness = pointcloud_processor_->calculateStraightness(denoised_cloud);
        float length = pointcloud_processor_->calculateLength(denoised_cloud);
        aspara_info.processing_times.measurement_ms = measurement_stopwatch.elapsed_ms();
        
        // 可視化用PCAラインを生成 + 骨格エンドポイント推定
        fluent::utils::Stopwatch pca_stopwatch;
        auto pca_line = pointcloud_processor_->generatePCALine(denoised_cloud);
        aspara_info.processing_times.pca_calculation_ms = pca_stopwatch.elapsed_ms();

        // PCA主軸に沿った直線の端点（tip/root）を推定し、骨格点列を作成
        try {
            // 骨格生成には、実際にasparagus_pointcloudとして保存される点群（filtered_cloud）を使用
            // これはROI抽出後の生データで、2D投影が正しく機能している
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr skeleton_cloud = filtered_cloud;
            
            // デバッグ：filtered_cloudの内容確認
            RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"),
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
            
            RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"),
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
                    RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"),
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
                RCLCPP_INFO(rclcpp::get_logger("analyzer_thread"),
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
        bool is_harvestable = (length >= node_->harvest_min_length_ && 
                              length <= node_->harvest_max_length_ && 
                              straightness >= node_->straightness_threshold_);

        // 可視化処理時間の計測
        fluent::utils::Stopwatch vis_stopwatch;
        
        // ROI深度統計ログはfallback経路で実施済み

        // 右パネル用: ROI下部帯からz0推定 → |z-z0|<=±aspara_filter_distance で抽出 + 中央帯
        const std::string frame_id_to_use = frame_id;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr foreground(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (!denoised_cloud->points.empty()) {
            // パラメータ
            double band_alpha = node_ && node_->has_parameter("band_alpha") ?
                node_->get_parameter("band_alpha").get_value<double>() : 0.30; // 中央帯幅
            double bottom_ratio = node_ && node_->has_parameter("hist_band_bottom_ratio") ?
                node_->get_parameter("hist_band_bottom_ratio").get_value<double>() : 0.05;
            double top_ratio = node_ && node_->has_parameter("hist_band_top_ratio") ?
                node_->get_parameter("hist_band_top_ratio").get_value<double>() : 0.10;
            double percentile = node_ && node_->has_parameter("root_depth_percentile") ?
                node_->get_parameter("root_depth_percentile").get_value<double>() : 0.10;
            bool use_seg = node_ && node_->has_parameter("root_use_segmentation") ?
                node_->get_parameter("root_use_segmentation").get_value<bool>() : true;
            // HSV閾値
            double hmin = 35.0, hmax = 85.0, smin = 0.25, vmin = 0.20;
            if (node_) {
                if (node_->has_parameter("root_hsv_h_min")) hmin = node_->get_parameter("root_hsv_h_min").get_value<double>();
                if (node_->has_parameter("root_hsv_h_max")) hmax = node_->get_parameter("root_hsv_h_max").get_value<double>();
                if (node_->has_parameter("root_hsv_s_min")) smin = node_->get_parameter("root_hsv_s_min").get_value<double>();
                if (node_->has_parameter("root_hsv_v_min")) vmin = node_->get_parameter("root_hsv_v_min").get_value<double>();
            }
            double dist_window = node_ ? node_->aspara_filter_distance_ : 0.05; // ±距離窓

            // 画像データ（カラー/マスク）
            cv::Mat color_mat, mask_mat;
            {
                std::lock_guard<std::mutex> lk(node_->image_data_mutex_);
                if (node_->latest_color_image_) {
                    color_mat = cv_bridge::toCvCopy(node_->latest_color_image_, sensor_msgs::image_encodings::BGR8)->image;
                }
                if (!node_->latest_mask_.empty()) mask_mat = node_->latest_mask_.clone();
            }
            cv::Mat hsv;
            if (!color_mat.empty()) cv::cvtColor(color_mat, hsv, cv::COLOR_BGR2HSV);

            const cv::Rect& roi = aspara_info.bounding_box_2d;
            int y0 = roi.y + static_cast<int>(std::floor(roi.height * (1.0 - top_ratio)));
            int y1 = roi.y + static_cast<int>(std::floor(roi.height * (1.0 - bottom_ratio)));
            y0 = std::clamp(y0, roi.y, roi.y + roi.height - 1);
            y1 = std::clamp(y1, roi.y, roi.y + roi.height - 1);
            if (y1 < y0) std::swap(y0, y1);

            // 下部帯の有効depth収集（seg/緑優先）
            std::vector<float> band_depths;
            band_depths.reserve((y1 - y0 + 1) * roi.width);
            auto is_green = [&](int u, int v) -> bool {
                if (use_seg && !mask_mat.empty()) {
                    if (mask_mat.size()!=color_mat.size()) {
                        // サイズ差は最近傍で参照
                        int um = std::clamp(static_cast<int>(std::round(u * (mask_mat.cols / static_cast<double>(color_mat.cols)))), 0, mask_mat.cols-1);
                        int vm = std::clamp(static_cast<int>(std::round(v * (mask_mat.rows / static_cast<double>(color_mat.rows)))), 0, mask_mat.rows-1);
                        return mask_mat.at<uint8_t>(vm, um) > 127;
                    }
                    return mask_mat.at<uint8_t>(v, u) > 127;
                }
                if (hsv.empty()) return true;
                cv::Vec3b px = hsv.at<cv::Vec3b>(v, u);
                double H = (px[0] * 2.0);
                double S = px[1] / 255.0;
                double V = px[2] / 255.0;
                return (H>=hmin && H<=hmax && S>=smin && V>=vmin);
            };

            // depth画像（16U/32F）から帯の深度をサンプリング（カラーに整列前提）
            cv::Mat depth_mat;
            if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                depth_mat = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1)->image;
            } else if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                depth_mat = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1)->image;
            }
            double depth_scale = (depth_mat.type()==CV_16UC1) ? node_->depth_unit_m_16u_ : 1.0;
            for (int v = y0; v <= y1; ++v) {
                if (v<0 || v>=depth_mat.rows) continue;
                for (int u = roi.x; u < roi.x + roi.width; ++u) {
                    if (u<0 || u>=depth_mat.cols) continue;
                    if (!color_mat.empty() && !is_green(u,v)) continue;
                    float z = 0.0f;
                    if (depth_mat.type()==CV_16UC1) {
                        uint16_t mm = depth_mat.at<uint16_t>(v,u);
                        if (mm==0) continue;
                        z = static_cast<float>(mm) * static_cast<float>(depth_scale);
                    } else {
                        float m = depth_mat.at<float>(v,u);
                        if (!std::isfinite(m) || m<=0.0f) continue;
                        z = m;
                    }
                    band_depths.push_back(z);
                }
            }
            float z0 = 0.0f;
            if (!band_depths.empty()) {
                std::nth_element(band_depths.begin(), band_depths.begin() + static_cast<long>(percentile * (band_depths.size()-1)), band_depths.end());
                z0 = band_depths[static_cast<long>(percentile * (band_depths.size()-1))];
            } else {
                // フォールバック: 全体最浅
                z0 = std::numeric_limits<float>::max();
                for (const auto& p : denoised_cloud->points) if (std::isfinite(p.z) && p.z>0.0f) z0 = std::min(z0, p.z);
            }

            // 中央帯境界
            int band_half = static_cast<int>(std::round(roi.width * band_alpha * 0.5));
            int cx_px = roi.x + roi.width/2;
            int band_left = cx_px - band_half;
            int band_right = cx_px + band_half;

            // z0±dist_window かつ 中央帯 かつ セグメント/緑画素 に限定
            // さらに根本X周りの横方向半径で制限（背景面の混入抑制）
            double lateral_radius_m = node_ && node_->has_parameter("root_lateral_radius_m") ?
                node_->get_parameter("root_lateral_radius_m").get_value<double>() : 0.02; // 2cm
            for (const auto& p : denoised_cloud->points) {
                if (!std::isfinite(p.z) || p.z <= 0.0f) continue;
                if (std::fabs(p.z - z0) > dist_window) continue;
                // 横方向の半径制限（xのみ）。根本X中心から±半径以内のみ採用
                if (std::isfinite(aspara_info.root_position_3d.x) && lateral_radius_m > 0.0) {
                    if (std::fabs(static_cast<double>(p.x) - aspara_info.root_position_3d.x) > lateral_radius_m) continue;
                }
                cv::Point2f uv = pointcloud_processor_->project3DTo2D(p, *camera_info);
                if (!std::isfinite(uv.x) || !std::isfinite(uv.y)) continue;
                if (uv.x < roi.x || uv.x >= roi.x + roi.width || uv.y < roi.y || uv.y >= roi.y + roi.height) continue;
                if (uv.x < band_left || uv.x > band_right) continue;
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

            // 根本3D推定（xは帯の横ピーク、yは下辺、z=z0）
            // 横ヒストグラム（緑/seg限定）
            int bins = std::max(1, roi.width / 4);
            std::vector<int> hist(bins, 0);
            for (int v = y0; v <= y1; ++v) {
                for (int u = roi.x; u < roi.x + roi.width; ++u) {
                    if (!color_mat.empty() && !is_green(u,v)) continue;
                    int b = std::clamp((u - roi.x) * bins / std::max(1, roi.width), 0, bins-1);
                    hist[b]++;
                }
            }
            int best_b = std::distance(hist.begin(), std::max_element(hist.begin(), hist.end()));
            int root_u = roi.x + (best_b * roi.width) / bins + (roi.width / bins)/2;
            // Yは矩形の最下端から一定割合上を固定（深度ノイズで揺れないように）
            double y_offset_ratio = node_ && node_->has_parameter("root_y_offset_ratio") ?
                node_->get_parameter("root_y_offset_ratio").get_value<double>() : 0.05;
            int root_v = roi.y + roi.height - 1 - static_cast<int>(std::round(roi.height * y_offset_ratio));
            // 3D backproject
            if (camera_info) {
                double fx = camera_info->k[0], fy = camera_info->k[4];
                double cx = camera_info->k[2], cy = camera_info->k[5];
                aspara_info.root_position_3d.x = (static_cast<double>(root_u) - cx) * z0 / fx;
                aspara_info.root_position_3d.y = (static_cast<double>(root_v) - cy) * z0 / fy;
                aspara_info.root_position_3d.z = z0;
            }
        }
        // 右用点群をパブリッシュ（空なら従来のdenoised）
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr to_publish = (foreground->points.empty() ? denoised_cloud : foreground);
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
        geometry_msgs::msg::Vector3 axis_dir_msg;
        try {
            pcl::PCA<pcl::PointXYZRGB> pca_axis; pca_axis.setInputCloud(denoised_cloud);
            Eigen::Vector3f axis = pca_axis.getEigenVectors().col(0).normalized();
            axis_dir_msg.x = axis.x(); axis_dir_msg.y = axis.y(); axis_dir_msg.z = axis.z();
        } catch (...) { axis_dir_msg.x = 0; axis_dir_msg.y = 0; axis_dir_msg.z = 1; }

        // 画像上の赤丸と一致させるため、帯ベース推定の根本座標を使用
        pointcloud_processor_->publishAsparaMarker(
            aspara_info.root_position_3d, frame_id_to_use, aspara_info.id, length, is_harvestable, depth_image->header.stamp, axis_dir_msg);
        
        // アスパラ情報を更新
        // 曲がり度: スケルトン中心線ベース（既定）/ PCA比 などを組合せ
        double arc_excess = 0.0, rms_norm = 0.0;
        double curvature_skel = computeCurvatureFromSkeleton(aspara_info.skeleton_points, arc_excess, rms_norm); // 0..大
        // PCA直線性を補助に（1.0=直線）。曲がり度へ変換
        double curvature_pca = 1.0 - static_cast<double>(straightness);
        // ハイブリッド: 最大値（安全側）。将来YAMLで切替予定
        double curvature_hybrid = std::max(curvature_skel, curvature_pca);

        aspara_info.length = length;
        aspara_info.straightness = static_cast<float>(1.0 - curvature_hybrid);
        aspara_info.is_harvestable = is_harvestable;
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