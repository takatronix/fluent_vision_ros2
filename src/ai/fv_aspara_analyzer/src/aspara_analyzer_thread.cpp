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
            // PCLのPCAで主成分を取得
            pcl::PCA<pcl::PointXYZRGB> pca;
            pca.setInputCloud(denoised_cloud);
            Eigen::Matrix3f eigvec = pca.getEigenVectors();
            Eigen::Vector3f axis = eigvec.col(0).normalized(); // 主軸
            Eigen::Vector4f mean4 = pca.getMean();
            Eigen::Vector3f mean(mean4[0], mean4[1], mean4[2]);

            // 主軸方向のt最小/最大を求める（t = axis・(p-mean)）
            float tmin = std::numeric_limits<float>::max();
            float tmax = std::numeric_limits<float>::lowest();
            for (const auto& pt : denoised_cloud->points) {
                if (!std::isfinite(pt.z) || pt.z <= 0.0f) continue;
                Eigen::Vector3f p(pt.x, pt.y, pt.z);
                float t = axis.dot(p - mean);
                tmin = std::min(tmin, t);
                tmax = std::max(tmax, t);
            }
            // 3D端点
            Eigen::Vector3f end_root = mean + tmin * axis;
            Eigen::Vector3f end_tip  = mean + tmax * axis;

            // 骨格サンプル（5点）
            std::vector<SkeletonPoint> skel;
            const int NUM = 5;
            for (int i = 0; i < NUM; ++i) {
                float alpha = static_cast<float>(i) / (NUM - 1);
                Eigen::Vector3f p = end_root * (1.0f - alpha) + end_tip * alpha;
                pcl::PointXYZRGB pxyz; pxyz.x = p.x(); pxyz.y = p.y(); pxyz.z = p.z();
                cv::Point2f uv = pointcloud_processor_->project3DTo2D(pxyz, *camera_info);
                SkeletonPoint sp;
                sp.image_point = uv;
                sp.world_point.x = pxyz.x; sp.world_point.y = pxyz.y; sp.world_point.z = pxyz.z;
                sp.distance_from_base = alpha; // 0(root)→1(tip) の規格化距離
                skel.push_back(sp);
            }
            aspara_info.skeleton_points = skel;
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

        pointcloud_processor_->publishAsparaMarker(
            root_position, frame_id_to_use, aspara_info.id, length, is_harvestable, depth_image->header.stamp, axis_dir_msg);
        
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