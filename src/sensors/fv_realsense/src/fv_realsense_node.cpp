/**
 * @file fv_realsense_node.cpp
 * @brief Fluent Vision RealSenseカメラノードのメイン実装ファイル
 * @details Intel RealSenseカメラ（D415/D405）のカラー、深度、点群データ取得と配信
 * @author Takashi Otsuka
 * @date 2024
 * @version 1.0
 */

#include "fv_realsense/fv_realsense_node.hpp"
#include "fv_realsense/srv/get_distance.hpp"
#include "fv_realsense/srv/get_camera_info.hpp"
#include "fv_realsense/srv/set_mode.hpp"
// #include "fv_realsense/srv/generate_point_cloud.hpp"  // Removed: service deleted

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fluent_lib/cv_bridge_compat.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/qos.hpp>

#include <chrono>
#include <thread>

/**
 * @brief コンストラクタ
 * @param node_name ノード名
 * @details RealSenseカメラノードの初期化と設定
 * 
 * 初期化内容：
 * - パラメータの読み込み
 * - RealSenseカメラの初期化
 * - パブリッシャーの初期化
 * - サービスの初期化
 * - TF2座標変換の初期化
 * - 処理スレッドの開始
 */
FVDepthCameraNode::FVDepthCameraNode(const std::string& node_name)
    : Node(node_name), running_(false)
{
    RCLCPP_INFO(this->get_logger(), "🚀 FV Depth Camera starting...");
    RCLCPP_INFO(this->get_logger(), "📁 Node name: %s", node_name.c_str());
    
    try {
        // ===== Step 1: パラメータの読み込み =====
        RCLCPP_INFO(this->get_logger(), "📋 Step 1: Loading parameters...");
        loadParameters();
        
        // ===== Step 2: RealSenseの初期化（shared_from_thisより前） =====
        RCLCPP_INFO(this->get_logger(), "📷 Step 2: Initializing RealSense...");
        if (!initializeRealSense()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to initialize RealSense");
            return;
        }
        
        // ===== Step 3: パブリッシャーの初期化（RealSenseの後） =====
        RCLCPP_INFO(this->get_logger(), "📤 Step 3: Initializing publishers...");
        initializePublishers();
        
        // ===== Step 4: サービスの初期化 =====
        RCLCPP_INFO(this->get_logger(), "🔧 Step 4: Initializing services...");
        initializeServices();
        
        // ===== Step 5: TFの初期化 =====
        RCLCPP_INFO(this->get_logger(), "🔄 Step 5: Initializing TF...");
        initializeTF();
        
        // ===== Step 6: 処理スレッドの開始 =====
        RCLCPP_INFO(this->get_logger(), "🔄 Step 6: Starting processing thread...");
        running_ = true;
        processing_thread_ = std::thread(&FVDepthCameraNode::processingLoop, this);
        
        RCLCPP_INFO(this->get_logger(), "✅ FV Depth Camera started successfully");

        if (stats_report_enabled_) {
            const int period_ms = std::max(200, stats_report_period_ms_);
            stats_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(period_ms),
                [this, period_ms]() {
                    const uint64_t c_cb = color_cb_count_.load(std::memory_order_relaxed);
                    const uint64_t d_cb = depth_cb_count_.load(std::memory_order_relaxed);
                    const uint64_t c_pub = color_pub_count_.load(std::memory_order_relaxed);
                    const uint64_t d_pub = depth_pub_count_.load(std::memory_order_relaxed);

                    const uint64_t dc_cb = c_cb - last_stats_color_cb_;
                    const uint64_t dd_cb = d_cb - last_stats_depth_cb_;
                    const uint64_t dc_pub = c_pub - last_stats_color_pub_;
                    const uint64_t dd_pub = d_pub - last_stats_depth_pub_;

                    last_stats_color_cb_ = c_cb;
                    last_stats_depth_cb_ = d_cb;
                    last_stats_color_pub_ = c_pub;
                    last_stats_depth_pub_ = d_pub;

                    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                               std::chrono::steady_clock::now().time_since_epoch())
                                               .count();
                    const int64_t last_color_ns = last_color_recv_ns_.load(std::memory_order_relaxed);
                    const int64_t last_depth_ns = last_depth_recv_ns_.load(std::memory_order_relaxed);
                    const long color_stall_ms = (last_color_ns > 0) ? long((now_ns - last_color_ns) / 1000000) : -1;
                    const long depth_stall_ms = (last_depth_ns > 0) ? long((now_ns - last_depth_ns) / 1000000) : -1;

                    RCLCPP_INFO(this->get_logger(),
                                "📈 Stats(%dms): cb(color=%lu depth=%lu) pub(color=%lu depth=%lu) dropped(color=%lu depth=%lu) stall_ms(color=%ld depth=%ld)",
                                period_ms,
                                (unsigned long)dc_cb, (unsigned long)dd_cb,
                                (unsigned long)dc_pub, (unsigned long)dd_pub,
                                (unsigned long)dropped_color_frames_.load(std::memory_order_relaxed),
                                (unsigned long)dropped_depth_frames_.load(std::memory_order_relaxed),
                                color_stall_ms, depth_stall_ms);
                });
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Exception during initialization: %s", e.what());
    }
}

/**
 * @brief デストラクタ
 * @details RealSenseカメラノードの適切な終了処理
 * 
 * 終了処理：
 * - 処理スレッドの停止
 * - スレッドの結合
 * - リソースの解放
 */
FVDepthCameraNode::~FVDepthCameraNode()
{
    RCLCPP_INFO(this->get_logger(), "🛑 Shutting down FV Depth Camera...");
    
    // ===== 処理スレッドの停止 =====
    running_ = false;
    // Stop sensors to release USB resources quickly
    try {
        stopSensors();
    } catch (...) {
    }
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
}

/**
 * @brief パラメータの読み込み
 * @details ROS2パラメータを読み込み、カメラ設定を初期化
 * 
 * 読み込み内容：
 * - 設定ファイルパスの確認
 * - ノード名とネームスペースの確認
 * - 利用可能パラメータのデバッグ出力
 * - カメラ選択設定の読み込み
 */
void FVDepthCameraNode::loadParameters()
{
    RCLCPP_INFO(this->get_logger(), "📋 Loading parameters...");
    
    // ===== 設定ファイルパスの確認 =====
    RCLCPP_INFO(this->get_logger(), "📁 Loading config file...");
    
    // コマンドライン引数から設定ファイルパスを取得
    auto args = this->get_node_options().arguments();
    std::string config_file_path = "Unknown";
    
    // ノードオプションでパラメータファイルをチェック
    auto param_file_args = this->get_node_options().parameter_overrides();
    if (!param_file_args.empty()) {
        RCLCPP_INFO(this->get_logger(), "📁 Parameter overrides detected: %zu", param_file_args.size());
    }
    
    // ===== ノード名とネームスペースの確認 =====
    RCLCPP_INFO(this->get_logger(), "🏷️  Node name: %s", this->get_name());
    RCLCPP_INFO(this->get_logger(), "🏷️  Namespace: %s", this->get_namespace());
    
    // ===== 利用可能パラメータのデバッグ出力 =====
    RCLCPP_INFO(this->get_logger(), "🔍 All available parameters:");
    auto param_names = this->list_parameters({}, 10);
    for (const auto& name : param_names.names) {
        try {
            auto param = this->get_parameter(name);
            RCLCPP_INFO(this->get_logger(), "   - %s: %s", name.c_str(), param.value_to_string().c_str());
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "   - %s: <error reading>", name.c_str());
        }
    }
    
    // ===== カメラ選択設定の読み込み =====
    camera_selection_config_.selection_method = 
        this->declare_parameter("camera_selection.selection_method", "auto");
    camera_selection_config_.serial_number = 
        this->declare_parameter("camera_selection.serial_number", "");
    camera_selection_config_.device_name = 
        this->declare_parameter("camera_selection.device_name", "");
    camera_selection_config_.device_index = 
        this->declare_parameter("camera_selection.device_index", 0);
    
    // Debug: Print camera selection parameters
    RCLCPP_INFO(this->get_logger(), "🔍 Camera selection config:");
    RCLCPP_INFO(this->get_logger(), "   - Method: %s", camera_selection_config_.selection_method.c_str());
    RCLCPP_INFO(this->get_logger(), "   - Serial: %s", camera_selection_config_.serial_number.c_str());
    RCLCPP_INFO(this->get_logger(), "   - Name: %s", camera_selection_config_.device_name.c_str());
    RCLCPP_INFO(this->get_logger(), "   - Index: %d", camera_selection_config_.device_index);
    
    // Power management settings
    power_management_config_.startup_delay = 
        this->declare_parameter("power_management.startup_delay", 2.0);
    
    // Camera settings
    camera_config_.color_width = 
        this->declare_parameter("camera.color_width", 640);
    camera_config_.color_height = 
        this->declare_parameter("camera.color_height", 480);
    camera_config_.color_fps = 
        this->declare_parameter("camera.color_fps", 30);
    camera_config_.depth_width = 
        this->declare_parameter("camera.depth_width", 640);
    camera_config_.depth_height = 
        this->declare_parameter("camera.depth_height", 480);
    camera_config_.depth_fps = 
        this->declare_parameter("camera.depth_fps", 30);
    
    // 深度スケールのオーバーライド設定（設定ファイルから）
    config_depth_scale_ = this->declare_parameter("camera.depth_scale", -1.0);
    
    // Stream settings
    stream_config_.color_enabled = 
        this->declare_parameter("streams.color_enabled", true);
    stream_config_.depth_enabled = 
        this->declare_parameter("streams.depth_enabled", true);
    stream_config_.infrared_enabled = 
        this->declare_parameter("streams.infrared_enabled", false);
    stream_config_.pointcloud_enabled = 
        this->declare_parameter("streams.pointcloud_enabled", true);
    // Align option (depth aligned to color)
    // store align flag in a member via parameter (use topic_config_.pointcloud as temp holder if needed)
    align_to_color_ = this->declare_parameter("streams.align_to_color", false);
    // keep sync flag as existing behavior
    sync_enabled_ = this->declare_parameter("streams.sync_enabled", true);
    // Timestamping: map RealSense device timestamp to ROS time
    use_device_timestamp_ = this->declare_parameter("streams.use_device_timestamp", true);
    device_ts_reset_threshold_ms_ = this->declare_parameter("streams.device_ts_reset_threshold_ms", 1000.0);
    // Anti-drift guard for librealsense GLOBAL_TIME — re-anchor the
    // device→ROS time mapping whenever the computed stamp deviates from
    // this->now() by more than this many ms. 250 ms keeps frame-to-frame
    // relative timing from D405 (sub-ms jitter) while bounding the
    // absolute offset against ROS time so tf2 lookups don't drift past
    // the URDF horizon. Set to 0 to anchor every frame; set very large
    // to disable the guard (legacy behaviour).
    drift_resync_ms_ = this->declare_parameter("streams.device_ts_drift_resync_ms", 250.0);
    // Sync warning threshold (ms)
    sync_warn_ms_ = this->declare_parameter("streams.sync_warn_ms", 1.0);
    sync_max_skew_ms_ = this->declare_parameter("streams.sync_max_skew_ms", 20.0);
    sync_max_wait_ms_ = this->declare_parameter("streams.sync_max_wait_ms", 15);
    const int sync_q = this->declare_parameter<int>("streams.sync_queue_size", 5);
    sync_queue_size_ = static_cast<std::size_t>(std::max(1, sync_q));
    stream_config_.depth_colormap_enabled = 
        this->declare_parameter("streams.depth_colormap_enabled", true);
    // reuse the already-declared value to avoid double declaration
    stream_config_.sync_enabled = sync_enabled_;
    
    // Camera info settings
    camera_info_config_.enable_camera_info = 
        this->declare_parameter("camera_info.enable_camera_info", true);
    camera_info_config_.enable_compressed_topics = 
        this->declare_parameter("camera_info.enable_compressed_topics", true);
    camera_info_config_.compressed_quality = 
        this->declare_parameter("camera_info.compressed_quality", 85);
    camera_info_config_.enable_depth_compressed = 
        this->declare_parameter("camera_info.enable_depth_compressed", false);
    
    // QoS設定の読み込み
    int qos_queue_size = this->declare_parameter("qos.queue_size", 1);
    std::string qos_reliability = this->declare_parameter("qos.reliability", "best_effort");
    std::string qos_durability = this->declare_parameter("qos.durability", "volatile");
    std::string qos_history = this->declare_parameter("qos.history", "keep_last");
    
    // Services settings
    services_config_.get_distance_enabled = 
        this->declare_parameter("services.get_distance_enabled", true);
    services_config_.get_camera_info_enabled = 
        this->declare_parameter("services.get_camera_info_enabled", true);
    services_config_.set_mode_enabled = 
        this->declare_parameter("services.set_mode_enabled", false);
    
    // TF settings
    tf_config_.enabled = 
        this->declare_parameter("tf.enabled", true);
    tf_config_.base_frame = 
        this->declare_parameter("tf.base_frame", "base_link");
    tf_config_.camera_frame = 
        this->declare_parameter("tf.camera_frame", "camera_link");
    tf_config_.color_optical_frame = 
        this->declare_parameter("tf.color_optical_frame", "color_optical_frame");
    tf_config_.depth_optical_frame = 
        this->declare_parameter("tf.depth_optical_frame", "depth_optical_frame");
    
    // Topics
    topic_config_.color = 
        this->declare_parameter("topics.color", "color/image_raw");
    topic_config_.depth = 
        this->declare_parameter("topics.depth", "depth/image_rect_raw");
    topic_config_.color_compressed = 
        this->declare_parameter("topics.color_compressed", "color/image_raw/compressed");
    topic_config_.depth_colormap = 
        this->declare_parameter("topics.depth_colormap", "depth/colormap");
    topic_config_.pointcloud = 
        this->declare_parameter("topics.pointcloud", "depth/color/points");
    topic_config_.color_camera_info = 
        this->declare_parameter("topics.color_camera_info", "color/camera_info");
    topic_config_.depth_camera_info = 
        this->declare_parameter("topics.depth_camera_info", "depth/camera_info");
    topic_config_.registered_points = 
        this->declare_parameter("topics.registered_points", "");
    
    // Organized cloud parameters
    organized_pointcloud_enabled_ = this->declare_parameter("organized_pointcloud.enabled", false);
    organized_pointcloud_decimation_ = this->declare_parameter("organized_pointcloud.decimation", 1);
    organized_pointcloud_rgb_ = this->declare_parameter("organized_pointcloud.rgb", true);

    // Point cloud clipping distances (if provided in YAML under pointcloud.*)
    min_distance_m_ = this->declare_parameter("pointcloud.min_distance", 0.1);
    max_distance_m_ = this->declare_parameter("pointcloud.max_distance", 3.0);

    // Cache latest frames for service/callback access (avoid calling wait_for_frames in services)
    cache_latest_frames_enabled_ = this->declare_parameter("cache_latest_frames.enabled", false);

    // Frame wait / stall recovery tuning
    frame_wait_timeout_ms_ = this->declare_parameter("streams.frame_wait_timeout_ms", 1000);
    stall_warn_ms_ = this->declare_parameter("watchdog.warn_ms", 0);
    stall_restart_ms_ = this->declare_parameter("watchdog.restart_ms", 0);
    stats_report_enabled_ = this->declare_parameter("debug.stats_report_enabled", false);
    stats_report_period_ms_ = this->declare_parameter("debug.stats_report_period_ms", 1000);

    // Debug: Print all loaded parameters (after all declare_parameter calls)
    RCLCPP_INFO(this->get_logger(), "🔍 All loaded parameters:");
    RCLCPP_INFO(this->get_logger(), "   - Camera selection method: '%s'", camera_selection_config_.selection_method.c_str());
    RCLCPP_INFO(this->get_logger(), "   - Camera selection name: '%s'", camera_selection_config_.device_name.c_str());
    RCLCPP_INFO(this->get_logger(), "   - Camera selection serial: '%s'", camera_selection_config_.serial_number.c_str());
    RCLCPP_INFO(this->get_logger(), "   - Camera selection index: %d", camera_selection_config_.device_index);
    RCLCPP_INFO(this->get_logger(), "   - Color: %dx%d @ %dfps", camera_config_.color_width, camera_config_.color_height, camera_config_.color_fps);
    RCLCPP_INFO(this->get_logger(), "   - Depth: %dx%d @ %dfps", camera_config_.depth_width, camera_config_.depth_height, camera_config_.depth_fps);
    RCLCPP_INFO(this->get_logger(), "   - Topics: color=%s depth=%s", topic_config_.color.c_str(), topic_config_.depth.c_str());
    RCLCPP_INFO(this->get_logger(), "   - Streams: sync=%s use_device_timestamp=%s",
                stream_config_.sync_enabled ? "on" : "off",
                use_device_timestamp_ ? "on" : "off");

    RCLCPP_INFO(this->get_logger(), "✅ Parameters loaded successfully");
    RCLCPP_INFO(this->get_logger(), "📺 Color topic: %s", topic_config_.color.c_str());
    RCLCPP_INFO(this->get_logger(), "📺 Depth topic: %s", topic_config_.depth.c_str());

    // ===== 初期モードの設定（デフォルト: 2 フル機能） =====
    int initial_mode = this->declare_parameter("initial_mode", 2);
    current_mode_.store(initial_mode);
    RCLCPP_INFO(this->get_logger(), "🎛️ Initial mode set to: %d (0=off,1=color-only,2=full)", initial_mode);
}

bool FVDepthCameraNode::initializeRealSense()
{
    RCLCPP_INFO(this->get_logger(), "🔧 Initializing RealSense...");
    
    try {
        // Initialize context with error handling
        RCLCPP_INFO(this->get_logger(), "📋 Creating RealSense context...");
        ctx_ = std::make_unique<rs2::context>();
        
        // Device selection based on configuration
        RCLCPP_INFO(this->get_logger(), "🔍 Device selection...");
        if (!selectCamera()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to select camera");
            return false;
        }
        
        // Log requested streams (sensor-based streaming is configured in startSensors())
        if (stream_config_.color_enabled) {
            RCLCPP_INFO(this->get_logger(), "📹 Requested color: %dx%d @ %dfps",
                        camera_config_.color_width, camera_config_.color_height, camera_config_.color_fps);
        }
        if (stream_config_.depth_enabled) {
            RCLCPP_INFO(this->get_logger(), "📏 Requested depth: %dx%d @ %dfps",
                        camera_config_.depth_width, camera_config_.depth_height, camera_config_.depth_fps);
        }
        
        // 同期設定（方式A）: wait_for_framesでブロックせず、device timestampで近傍ペアリングする
        sync_enabled_ = (stream_config_.sync_enabled && stream_config_.color_enabled && stream_config_.depth_enabled);
        if (sync_enabled_) {
            RCLCPP_INFO(this->get_logger(), "🔗 Soft sync enabled: pair by device timestamp (max_skew=%.1fms wait=%dms)",
                        sync_max_skew_ms_, sync_max_wait_ms_);
        } else {
            RCLCPP_INFO(this->get_logger(), "⚠️ Soft sync disabled: will publish without pairing (depth/color independent)");
        }
        
        // Apply startup delay for power management
        if (power_management_config_.startup_delay > 0.0) {
            RCLCPP_INFO(this->get_logger(), "⏳ Waiting %.1f seconds for power stabilization...", 
                power_management_config_.startup_delay);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(power_management_config_.startup_delay * 1000)));
        }
        
        // Device is already selected (device_) and will be opened via rs2::sensor::open in startSensors().
        
        // Start sensors (non-blocking, decoupled streams)
        if (!startSensors()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to start RealSense sensors");
            return false;
        }
        
        // Get depth scale with error handling
        try {
            // Log device info before getting depth scale
            RCLCPP_INFO(this->get_logger(), "🔍 Getting depth scale from device: %s (SN: %s)",
                device_.get_info(RS2_CAMERA_INFO_NAME),
                device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
            
            // Get device name for later use
            std::string device_name = device_.get_info(RS2_CAMERA_INFO_NAME);
            
            // Get depth scale from depth sensor
            for (auto sensor : device_.query_sensors()) {
                if (sensor.is<rs2::depth_sensor>()) {
                    rs2::depth_sensor depth_sensor = sensor.as<rs2::depth_sensor>();
                    depth_scale_ = depth_sensor.get_depth_scale();
                    RCLCPP_INFO(this->get_logger(), "📏 Depth scale: %f", depth_scale_);
                    
                    // Double check with manual depth scale based on device model
                    if (device_name.find("D415") != std::string::npos) {
                        if (std::abs(depth_scale_ - 0.001f) > 0.0001f) {
                            RCLCPP_WARN(this->get_logger(), "⚠️ D415 depth scale mismatch: got %f, expected 0.001", depth_scale_);
                            depth_scale_ = 0.001f;  // Force correct value for D415
                        }
                    } else if (device_name.find("D405") != std::string::npos) {
                        // D405の正しいdepth_scaleは0.0001（設定ファイルの値を使用）
                        RCLCPP_INFO(this->get_logger(), "📏 D405 depth scale: %f (correct value)", depth_scale_);
                        // 設定ファイルからのオーバーライドがあれば使用
                        if (config_depth_scale_ > 0) {
                            RCLCPP_INFO(this->get_logger(), "📏 Using config depth scale: %f", config_depth_scale_);
                            depth_scale_ = config_depth_scale_;
                        }
                    }
                    break;
                }
            }
            // 設定ファイルからのオーバーライド（D405以外の場合）
            if (config_depth_scale_ > 0 && device_name.find("D405") == std::string::npos) {
                RCLCPP_INFO(this->get_logger(), "📏 Overriding depth scale from config: %f -> %f", depth_scale_, config_depth_scale_);
                depth_scale_ = config_depth_scale_;
            }
            
            RCLCPP_INFO(this->get_logger(), "📏 Final depth scale: %f", depth_scale_);
        } catch (const rs2::error& e) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Could not get depth scale: %s", e.what());
            depth_scale_ = (config_depth_scale_ > 0) ? config_depth_scale_ : 0.001f; // Use config or default
        }
        
        // Intrinsics are resolved from selected stream profiles in startSensors()
        
        RCLCPP_INFO(this->get_logger(), "✅ RealSense initialized successfully");
        return true;
        
    } catch (const rs2::error& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ RealSense error: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Exception during RealSense initialization: %s", e.what());
        return false;
    }
}

static rs2::stream_profile pickProfileSmart(const std::vector<rs2::stream_profile>& profiles,
                                           rs2_stream stream, rs2_format fmt,
                                           int width, int height, int fps) {
    // Strategy:
    // 1) Filter by stream+format
    // 2) Prefer exact width/height (if video)
    // 3) For fps: prefer exact; else prefer smallest fps above desired; else largest below desired
    rs2::stream_profile best;
    int best_w = 0, best_h = 0, best_fps = 0;
    bool best_has_res = false;

    auto better_fps = [&](int cand_fps, int cur_fps) -> bool {
        if (cur_fps == 0) return true;
        if (cand_fps == fps && cur_fps != fps) return true;
        if (cur_fps == fps && cand_fps != fps) return false;
        const bool cand_above = cand_fps > fps;
        const bool cur_above = cur_fps > fps;
        if (cand_above != cur_above) {
            // Prefer above desired (avoid accidentally selecting very low fps like 5fps).
            return cand_above;
        }
        if (cand_above) {
            // Both above: choose smallest above desired.
            return cand_fps < cur_fps;
        }
        // Both below (or equal handled above): choose largest below desired.
        return cand_fps > cur_fps;
    };

    for (const auto& sp : profiles) {
        if (sp.stream_type() != stream) continue;
        if (sp.format() != fmt) continue;
        const int sp_fps = sp.fps();

        int sp_w = 0, sp_h = 0;
        bool has_res = false;
        if (auto vp = sp.as<rs2::video_stream_profile>()) {
            sp_w = vp.width();
            sp_h = vp.height();
            has_res = true;
        }

        // Resolution preference: exact match is best. If no exact exists, keep first candidate and improve by fps.
        const bool res_exact = (!has_res) || (sp_w == width && sp_h == height);
        const bool best_res_exact = (!best_has_res) || (best_w == width && best_h == height);

        bool take = false;
        if (!best) {
            take = true;
        } else if (res_exact && !best_res_exact) {
            take = true;
        } else if (res_exact == best_res_exact) {
            // Same resolution class, pick by fps preference.
            take = better_fps(sp_fps, best_fps);
        }

        if (take) {
            best = sp;
            best_w = sp_w;
            best_h = sp_h;
            best_fps = sp_fps;
            best_has_res = has_res;
        }
    }
    return best;
}

bool FVDepthCameraNode::startSensors() {
    stopSensors();

    if (!device_) {
        RCLCPP_ERROR(this->get_logger(), "Device is not selected");
        return false;
    }

    // Reset queues
    {
        std::lock_guard<std::mutex> lk(sync_mutex_);
        color_queue_.clear();
        depth_queue_.clear();
    }
    last_color_recv_ns_.store(0, std::memory_order_relaxed);
    last_depth_recv_ns_.store(0, std::memory_order_relaxed);
    last_paired_publish_ns_.store(0, std::memory_order_relaxed);

    // Find sensors that provide color/depth streams
    rs2::sensor color_sensor;
    rs2::sensor depth_sensor;
    for (auto s : device_.query_sensors()) {
        for (const auto& sp : s.get_stream_profiles()) {
            if (!color_sensor && sp.stream_type() == RS2_STREAM_COLOR) {
                color_sensor = s;
            }
            if (!depth_sensor && sp.stream_type() == RS2_STREAM_DEPTH) {
                depth_sensor = s;
            }
        }
    }

    if (stream_config_.color_enabled && !color_sensor) {
        RCLCPP_ERROR(this->get_logger(), "No sensor provides RS2_STREAM_COLOR");
        return false;
    }
    if (stream_config_.depth_enabled && !depth_sensor) {
        RCLCPP_ERROR(this->get_logger(), "No sensor provides RS2_STREAM_DEPTH");
        return false;
    }

    // Select stream profiles
    if (stream_config_.color_enabled) {
        const auto profiles = color_sensor.get_stream_profiles();
        color_profile_ = pickProfileSmart(profiles, RS2_STREAM_COLOR, RS2_FORMAT_BGR8,
                                          camera_config_.color_width, camera_config_.color_height, camera_config_.color_fps);
        if (!color_profile_) {
            // Fallback to RGB8 if BGR8 is not available on this device
            color_profile_ = pickProfileSmart(profiles, RS2_STREAM_COLOR, RS2_FORMAT_RGB8,
                                              camera_config_.color_width, camera_config_.color_height, camera_config_.color_fps);
        }
        if (!color_profile_) {
            RCLCPP_ERROR(this->get_logger(),
                "❌ No color stream profile found for %dx%d. Available color profiles:",
                camera_config_.color_width, camera_config_.color_height);
            for (const auto& sp : profiles) {
                if (sp.stream_type() != RS2_STREAM_COLOR) continue;
                if (auto vp = sp.as<rs2::video_stream_profile>()) {
                    RCLCPP_ERROR(this->get_logger(), "   %dx%d @ %dfps (%s)",
                        vp.width(), vp.height(), sp.fps(), rs2_format_to_string(sp.format()));
                }
            }
            return false;
        }
        auto vp = color_profile_.as<rs2::video_stream_profile>();
        if (vp) {
            color_intrinsics_ = vp.get_intrinsics();
        }
        color_sensor_ = color_sensor;
        if (auto vp = color_profile_.as<rs2::video_stream_profile>()) {
            RCLCPP_INFO(this->get_logger(), "📹 Selected color profile: %dx%d @ %dfps fmt=%s",
                        vp.width(), vp.height(), color_profile_.fps(), rs2_format_to_string(color_profile_.format()));
        } else {
            RCLCPP_INFO(this->get_logger(), "📹 Selected color profile: @ %dfps fmt=%s",
                        color_profile_.fps(), rs2_format_to_string(color_profile_.format()));
        }
    }

    if (stream_config_.depth_enabled) {
        const auto profiles = depth_sensor.get_stream_profiles();
        depth_profile_ = pickProfileSmart(profiles, RS2_STREAM_DEPTH, RS2_FORMAT_Z16,
                                          camera_config_.depth_width, camera_config_.depth_height, camera_config_.depth_fps);
        if (!depth_profile_) {
            RCLCPP_ERROR(this->get_logger(),
                "❌ No depth stream profile found for %dx%d. Available depth profiles:",
                camera_config_.depth_width, camera_config_.depth_height);
            for (const auto& sp : profiles) {
                if (sp.stream_type() != RS2_STREAM_DEPTH) continue;
                if (auto vp = sp.as<rs2::video_stream_profile>()) {
                    RCLCPP_ERROR(this->get_logger(), "   %dx%d @ %dfps (%s)",
                        vp.width(), vp.height(), sp.fps(), rs2_format_to_string(sp.format()));
                }
            }
            return false;
        }
        auto vp = depth_profile_.as<rs2::video_stream_profile>();
        if (vp) {
            depth_intrinsics_ = vp.get_intrinsics();
        }
        depth_sensor_ = depth_sensor;
        if (auto vp = depth_profile_.as<rs2::video_stream_profile>()) {
            RCLCPP_INFO(this->get_logger(), "📏 Selected depth profile: %dx%d @ %dfps fmt=%s",
                        vp.width(), vp.height(), depth_profile_.fps(), rs2_format_to_string(depth_profile_.format()));
        } else {
            RCLCPP_INFO(this->get_logger(), "📏 Selected depth profile: @ %dfps fmt=%s",
                        depth_profile_.fps(), rs2_format_to_string(depth_profile_.format()));
        }
    }

    const bool same_sensor =
        stream_config_.color_enabled && stream_config_.depth_enabled &&
        color_sensor_ && depth_sensor_ && (color_sensor_.get() == depth_sensor_.get());

    auto dispatch_cb = [this](const rs2::frame& f) {
        if (!running_.load()) return;
        // Some devices may deliver a frameset when multiple profiles are opened together.
        if (f.is<rs2::frameset>()) {
            auto fs = f.as<rs2::frameset>();
            auto cf = fs.get_color_frame();
            auto df = fs.get_depth_frame();
            if (cf) onColorFrame(cf);
            if (df) onDepthFrame(df);
            return;
        }
        const auto st = f.get_profile().stream_type();
        if (st == RS2_STREAM_COLOR) onColorFrame(f);
        else if (st == RS2_STREAM_DEPTH) onDepthFrame(f);
    };

    // Open/start sensors (open multiple profiles together if they share the same UVC sensor)
    try {
        if (same_sensor) {
            std::vector<rs2::stream_profile> profiles;
            profiles.reserve(2);
            profiles.push_back(color_profile_);
            profiles.push_back(depth_profile_);
            color_sensor_.open(profiles);
            color_sensor_.start(dispatch_cb);
        } else {
            if (stream_config_.color_enabled) {
                color_sensor_.open(std::vector<rs2::stream_profile>{color_profile_});
                color_sensor_.start(dispatch_cb);
            }
            if (stream_config_.depth_enabled) {
                depth_sensor_.open(std::vector<rs2::stream_profile>{depth_profile_});
                depth_sensor_.start(dispatch_cb);
            }
        }
    } catch (const rs2::error& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open/start sensors: %s", e.what());
        try { stopSensors(); } catch (...) {}
        return false;
    }

    sensors_started_ = true;
    RCLCPP_INFO(this->get_logger(), "✅ Sensor callbacks started (color=%s depth=%s sync=%s skew=%.1fms wait=%dms q=%zu)",
                stream_config_.color_enabled ? "on" : "off",
                stream_config_.depth_enabled ? "on" : "off",
                sync_enabled_ ? "on" : "off",
                sync_max_skew_ms_, sync_max_wait_ms_, sync_queue_size_);
    return true;
}

bool FVDepthCameraNode::deepResetAndRestart() {
    // Step 1: tear down sensors against the (now-likely-invalid) device.
    // stopSensors() already swallows rs2::error internally so it's safe
    // to call after USB unplug.
    stopSensors();

    // Step 1b: explicitly drop EVERY cached rs2 handle that still
    // references the about-to-be-destroyed context. Anything left
    // referencing the old context (sensor, device, stream_profile,
    // frame, queued frame items) will be freed against an arena that
    // no longer exists once ctx_.reset() runs, producing
    // "free(): corrupted unsorted chunks" on the *next* deep reset
    // (the first one looks fine because the dangling pointers
    // haven't been touched yet).
    try { color_sensor_ = rs2::sensor(); } catch (...) {}
    try { depth_sensor_ = rs2::sensor(); } catch (...) {}
    try { device_ = rs2::device(); } catch (...) {}
    try { color_profile_ = rs2::stream_profile(); } catch (...) {}
    try { depth_profile_ = rs2::stream_profile(); } catch (...) {}
    {
        std::lock_guard<std::mutex> lk(latest_frame_mutex_);
        try { latest_color_frame_ = rs2::frame(); } catch (...) {}
        try { latest_depth_frame_ = rs2::frame(); } catch (...) {}
    }
    {
        std::lock_guard<std::mutex> lk(sync_mutex_);
        // FrameItem holds rs2::frame; clear queues to drop refs.
        color_queue_.clear();
        depth_queue_.clear();
    }

    // Step 2: drop the old rs2::context. rs2::device handles cached
    // inside it survive USB enumeration but point at the pre-unplug
    // device path; that's exactly the "No such device" the cheap
    // stop+start watchdog kept hitting.
    try {
        ctx_.reset();
    } catch (...) {}

    // Step 3: recreate context + re-enumerate. selectCamera() walks
    // ctx_->query_devices() and reassigns device_ to a fresh handle.
    // We retry up to ~3s because USB re-enumeration after plug-in is
    // not instantaneous on Tegra.
    constexpr int kMaxAttempts = 6;
    constexpr int kAttemptDelayMs = 500;
    for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
        try {
            ctx_ = std::make_unique<rs2::context>();
            if (selectCamera()) {
                if (startSensors()) {
                    RCLCPP_INFO(this->get_logger(),
                                "✅ Deep reset succeeded on attempt %d/%d",
                                attempt, kMaxAttempts);
                    return true;
                }
                RCLCPP_WARN(this->get_logger(),
                            "Deep reset attempt %d/%d: startSensors failed",
                            attempt, kMaxAttempts);
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "Deep reset attempt %d/%d: no device enumerated",
                            attempt, kMaxAttempts);
            }
        } catch (const rs2::error &e) {
            RCLCPP_WARN(this->get_logger(),
                        "Deep reset attempt %d/%d: %s",
                        attempt, kMaxAttempts, e.what());
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(),
                        "Deep reset attempt %d/%d: %s",
                        attempt, kMaxAttempts, e.what());
        }
        if (attempt < kMaxAttempts) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(kAttemptDelayMs));
        }
    }
    RCLCPP_ERROR(this->get_logger(),
                 "❌ Deep reset exhausted %d attempts; sensor stays down "
                 "until next watchdog cycle (operator may need to "
                 "verify USB cable / power).",
                 kMaxAttempts);
    return false;
}

void FVDepthCameraNode::stopSensors() {
    if (!sensors_started_) {
        return;
    }
    const bool same_sensor =
        stream_config_.color_enabled && stream_config_.depth_enabled &&
        color_sensor_ && depth_sensor_ && (color_sensor_.get() == depth_sensor_.get());
    try {
        if (same_sensor) {
            try { color_sensor_.stop(); } catch (...) {}
            try { color_sensor_.close(); } catch (...) {}
        } else {
            if (stream_config_.color_enabled) {
                try { color_sensor_.stop(); } catch (...) {}
                try { color_sensor_.close(); } catch (...) {}
            }
            if (stream_config_.depth_enabled) {
                try { depth_sensor_.stop(); } catch (...) {}
                try { depth_sensor_.close(); } catch (...) {}
            }
        }
    } catch (...) {
    }
    sensors_started_ = false;
}

void FVDepthCameraNode::onColorFrame(const rs2::frame& frame) {
    if (!running_.load()) {
        return;
    }
    color_cb_count_.fetch_add(1, std::memory_order_relaxed);
    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               std::chrono::steady_clock::now().time_since_epoch())
                               .count();
    last_color_recv_ns_.store(now_ns, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lk(latest_frame_mutex_);
        latest_color_frame_ = frame;
        latest_color_ts_ms_ = frame.get_timestamp();
    }

    FrameItem item;
    item.frame = frame;
    item.ts_ms = frame.get_timestamp();
    item.recv_tp = std::chrono::steady_clock::now();

    {
        std::lock_guard<std::mutex> lk(sync_mutex_);
        color_queue_.push_back(std::move(item));
        while (color_queue_.size() > sync_queue_size_) {
            color_queue_.pop_front();
            dropped_color_frames_.fetch_add(1, std::memory_order_relaxed);
        }
    }
    sync_cv_.notify_one();
}

void FVDepthCameraNode::onDepthFrame(const rs2::frame& frame) {
    if (!running_.load()) {
        return;
    }
    depth_cb_count_.fetch_add(1, std::memory_order_relaxed);
    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               std::chrono::steady_clock::now().time_since_epoch())
                               .count();
    last_depth_recv_ns_.store(now_ns, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lk(latest_frame_mutex_);
        latest_depth_frame_ = frame;
        latest_depth_ts_ms_ = frame.get_timestamp();
    }

    FrameItem item;
    item.frame = frame;
    item.ts_ms = frame.get_timestamp();
    item.recv_tp = std::chrono::steady_clock::now();

    {
        std::lock_guard<std::mutex> lk(sync_mutex_);
        depth_queue_.push_back(std::move(item));
        while (depth_queue_.size() > sync_queue_size_) {
            depth_queue_.pop_front();
            dropped_depth_frames_.fetch_add(1, std::memory_order_relaxed);
        }
    }
    sync_cv_.notify_one();
}

bool FVDepthCameraNode::selectCamera()
{
    auto devices = getAvailableDevices();

    // Crash guard: every branch below dereferences `devices` (most
    // notoriously the auto branch's `devices[0]`, which segfaults on
    // an empty vector). This used to be reachable only at boot when
    // ros2 launch already verified a device existed; the watchdog
    // deep-reset now calls selectCamera() while the USB might still
    // be unplugged, so we have to fail closed before any indexing.
    if (devices.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "🔍 selectCamera(): no RealSense devices enumerated; "
                    "will retry on the next watchdog cycle");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "🔍 selectCamera() - Method: %s, Name: %s",
        camera_selection_config_.selection_method.c_str(),
        camera_selection_config_.device_name.c_str());
    
    if (camera_selection_config_.selection_method == "serial" && 
        !camera_selection_config_.serial_number.empty()) {
        
        for (const auto& device : devices) {
            std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            if (serial == camera_selection_config_.serial_number) {
                device_ = device;
                RCLCPP_INFO(this->get_logger(), "📷 Selected camera by serial: %s", serial.c_str());
                // No need to call enable_device here - it's already called after selectCamera
                return true;
            }
        }
        RCLCPP_ERROR(this->get_logger(), "❌ Camera with serial %s not found", 
            camera_selection_config_.serial_number.c_str());
        return false;
        
    } else if (camera_selection_config_.selection_method == "name" && 
               !camera_selection_config_.device_name.empty()) {
        
        for (const auto& device : devices) {
            std::string name = device.get_info(RS2_CAMERA_INFO_NAME);
            std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            RCLCPP_INFO(this->get_logger(), "🔍 Checking device: %s (SN: %s) against target: %s", 
                name.c_str(), serial.c_str(), camera_selection_config_.device_name.c_str());
            if (name.find(camera_selection_config_.device_name) != std::string::npos) {
                device_ = device;
                RCLCPP_INFO(this->get_logger(), "📷 Selected camera by name: %s (SN: %s)", name.c_str(), serial.c_str());
                return true;
            }
        }
        RCLCPP_ERROR(this->get_logger(), "❌ Camera with name %s not found", 
            camera_selection_config_.device_name.c_str());
        return false;
        
    } else if (camera_selection_config_.selection_method == "index") {
        
        if (camera_selection_config_.device_index < static_cast<int>(devices.size())) {
            device_ = devices[camera_selection_config_.device_index];
            std::string name = device_.get_info(RS2_CAMERA_INFO_NAME);
            RCLCPP_INFO(this->get_logger(), "📷 Selected camera by index %d: %s", 
                camera_selection_config_.device_index, name.c_str());
            return true;
        }
        RCLCPP_ERROR(this->get_logger(), "❌ Camera index %d out of range", 
            camera_selection_config_.device_index);
        return false;
        
    } else {
        // Auto-select first available
        device_ = devices[0];
        std::string name = device_.get_info(RS2_CAMERA_INFO_NAME);
        std::string serial = device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        RCLCPP_INFO(this->get_logger(), "📷 Auto-selected camera: %s (Serial: %s)", 
            name.c_str(), serial.c_str());
        return true;
    }
}

void FVDepthCameraNode::initializePublishers()
{
    RCLCPP_INFO(this->get_logger(), "📤 Initializing publishers...");

    // Make relative topics private (~/...) so they include the node_name.
    // e.g. "color/image_raw" → "~/color/image_raw" → /fv/realsense_1/color/image_raw
    auto make_private = [](const std::string& topic) -> std::string {
        if (topic.empty() || topic[0] == '/' || topic[0] == '~') return topic;
        return "~/" + topic;
    };
    topic_config_.color = make_private(topic_config_.color);
    topic_config_.depth = make_private(topic_config_.depth);
    topic_config_.color_compressed = make_private(topic_config_.color_compressed);
    topic_config_.depth_colormap = make_private(topic_config_.depth_colormap);
    topic_config_.pointcloud = make_private(topic_config_.pointcloud);
    topic_config_.color_camera_info = make_private(topic_config_.color_camera_info);
    topic_config_.depth_camera_info = make_private(topic_config_.depth_camera_info);

    // QoS設定を構築（パラメータから読み込み）
    int qos_queue_size = this->get_parameter("qos.queue_size").as_int();
    std::string qos_reliability = this->get_parameter("qos.reliability").as_string();
    std::string qos_durability = this->get_parameter("qos.durability").as_string();
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_queue_size)).best_effort().durability_volatile();
    
    // Reliability設定
    if (qos_reliability == "best_effort") {
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    } else {
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    }
    
    // Durability設定
    if (qos_durability == "volatile") {
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    } else {
        qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    }
    
    qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    
    // Basic publishers
    if (stream_config_.color_enabled) {
        color_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            topic_config_.color, qos);
        RCLCPP_INFO(this->get_logger(), "📷 Color publisher created: %s, ptr: %p", 
            topic_config_.color.c_str(), static_cast<void*>(color_pub_.get()));
        
        if (camera_info_config_.enable_compressed_topics) {
            // Create compressed image publisher directly
            std::string compressed_topic = topic_config_.color + "/compressed";
            color_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                compressed_topic, qos);
            RCLCPP_INFO(this->get_logger(), "📷 Compressed publisher created: %s", 
                compressed_topic.c_str());
        }
    }
    
    if (stream_config_.depth_enabled) {
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            topic_config_.depth, qos);
    }
    
    if (stream_config_.depth_colormap_enabled) {
        depth_colormap_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            topic_config_.depth_colormap, qos);
    }
    
    if (stream_config_.pointcloud_enabled) {
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            topic_config_.pointcloud, qos);
    }

    if (organized_pointcloud_enabled_) {
        std::string reg_topic = topic_config_.registered_points;
        if (reg_topic.empty()) {
            // try to build from color topic prefix if absolute path not provided
            reg_topic = "/fv/d415/registered_points";
        }
        // QoS: BEST_EFFORT, small queue
        auto oqos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        registered_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(reg_topic, oqos);
        RCLCPP_INFO(this->get_logger(), "📌 Organized cloud publisher: %s", reg_topic.c_str());
    }
    
    // Camera info publishers
    if (camera_info_config_.enable_camera_info) {
        color_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            topic_config_.color_camera_info, qos);
        depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            topic_config_.depth_camera_info, qos);
        RCLCPP_INFO(this->get_logger(), "📋 Camera info publishers created: %s, %s", 
            topic_config_.color_camera_info.c_str(), 
            topic_config_.depth_camera_info.c_str());
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ Publishers initialized");
}

void FVDepthCameraNode::initializeServices()
{
    RCLCPP_INFO(this->get_logger(), "🔧 Initializing services...");
    
    if (services_config_.get_distance_enabled) {
        get_distance_service_ = this->create_service<fv_realsense::srv::GetDistance>(
            "~/get_distance",
            std::bind(&FVDepthCameraNode::handleGetDistance, this, 
                std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "📏 GetDistance service initialized");
    }
    
    if (services_config_.get_camera_info_enabled) {
        get_camera_info_service_ = this->create_service<fv_realsense::srv::GetCameraInfo>(
            "~/get_camera_info",
            std::bind(&FVDepthCameraNode::handleGetCameraInfo, this, 
                std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "📋 GetCameraInfo service initialized");
    }
    
    if (services_config_.set_mode_enabled) {
        set_mode_service_ = this->create_service<fv_realsense::srv::SetMode>(
            "~/set_mode",
            std::bind(&FVDepthCameraNode::handleSetMode, this, 
                std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "🎛️ SetMode service initialized");
    }
    
    // GeneratePointCloud service removed
    
    // Initialize subscribers
    initializeSubscribers();
}

void FVDepthCameraNode::initializeTF()
{
    if (!tf_config_.enabled) {
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "🔄 Initializing TF...");
    
    tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
    publishTF();
    
    RCLCPP_INFO(this->get_logger(), "✅ TF initialized");
}

void FVDepthCameraNode::initializeSubscribers()
{
    RCLCPP_INFO(this->get_logger(), "📥 Initializing subscribers...");
    
    // Click event subscriber for point marker
    click_event_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "click_event", 10,
        std::bind(&FVDepthCameraNode::clickEventCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "🖱️ Click event subscriber initialized");
}

void FVDepthCameraNode::processingLoop()
{
    RCLCPP_INFO(this->get_logger(), "🔄 Starting processing loop...");
    
    int frame_count = 0;
    int color_pub_count = 0;
    int depth_pub_count = 0;
    auto last_log_time = std::chrono::steady_clock::now();
    bool warned = false;
    
    while (running_ && rclcpp::ok()) {
        try {
            // モードに応じた処理
            int current_mode = current_mode_.load();
            
            switch (current_mode) {
                case 0: {  // 停止モード
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    break;
                }
                    
                case 1: {  // 基本動作モード
                    // カラーのみ: 最新colorフレームを待って配信
                    FrameItem color_item;
                    bool got_color = false;
                    {
                        std::unique_lock<std::mutex> lk(sync_mutex_);
                        sync_cv_.wait_for(lk, std::chrono::milliseconds(frame_wait_timeout_ms_), [&]() {
                            return !running_.load() || !color_queue_.empty();
                        });
                        if (!running_.load()) break;
                        if (!color_queue_.empty()) {
                            color_item = std::move(color_queue_.back());
                            color_queue_.clear();
                            got_color = true;
                        }
                    }

                    if (!got_color) {
                        // Stall detection based on last receive timestamp (from callbacks)
                        const int64_t last_ns = last_color_recv_ns_.load(std::memory_order_relaxed);
                        if (last_ns > 0) {
                            const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                       std::chrono::steady_clock::now().time_since_epoch())
                                                       .count();
                            const int64_t stall_ms = (now_ns - last_ns) / 1000000;
                            if (stall_warn_ms_ > 0 && stall_ms >= stall_warn_ms_ && !warned) {
                                RCLCPP_WARN(this->get_logger(), "⚠️ No color frames for %ldms (mode=1)", (long)stall_ms);
                                warned = true;
                            }
                            if (stall_restart_ms_ > 0 && stall_ms >= stall_restart_ms_) {
                                RCLCPP_ERROR(this->get_logger(), "🔁 Deep reset after %ldms color stall (mode=1)", (long)stall_ms);
                                deepResetAndRestart();
                                last_color_recv_ns_.store(
                                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch()).count(),
                                    std::memory_order_relaxed);
                                warned = false;
                            }
                        }
                        break;
                    }
                    warned = false;

                    frame_count++;
                    const rclcpp::Time stamp = stampFromDeviceTime(color_item.frame, color_item.ts_ms);
                    publishFrames(color_item.frame, rs2::frame(), stamp);
                    color_pub_count++;
                    last_paired_publish_ns_.store(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count(),
                        std::memory_order_relaxed);

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 1) {
                        const auto dropped_c = dropped_color_frames_.load(std::memory_order_relaxed);
                        const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                   std::chrono::steady_clock::now().time_since_epoch())
                                                   .count();
                        const int64_t last_color_ns = last_color_recv_ns_.load(std::memory_order_relaxed);
                        const int64_t last_pub_ns = last_paired_publish_ns_.load(std::memory_order_relaxed);
                        const long color_stall_ms = (last_color_ns > 0) ? long((now_ns - last_color_ns) / 1000000) : -1;
                        const long pub_stall_ms = (last_pub_ns > 0) ? long((now_ns - last_pub_ns) / 1000000) : -1;
                        RCLCPP_DEBUG(this->get_logger(), "📊 Mode 1: loop=%d color=%d dropped_color=%lu stall(color=%ldms pub=%ldms)",
                                     frame_count, color_pub_count, (unsigned long)dropped_c,
                                     color_stall_ms, pub_stall_ms);
                        frame_count = 0;
                        color_pub_count = 0;
                        last_log_time = now;
                    }
                    break;
                }
                    
                case 2: {  // フル機能モード
                    // color/depth を独立に配信（ペア待ちでブロックしない）
                    FrameItem color_item;
                    FrameItem depth_item;
                    bool got_color = false;
                    bool got_depth = false;
                    {
                        std::unique_lock<std::mutex> lk(sync_mutex_);
                        sync_cv_.wait_for(lk, std::chrono::milliseconds(frame_wait_timeout_ms_), [&]() {
                            return !running_.load() || !color_queue_.empty() || !depth_queue_.empty();
                        });
                        if (!running_.load()) break;
                        if (!color_queue_.empty()) {
                            color_item = std::move(color_queue_.back());
                            color_queue_.clear();
                            got_color = true;
                        }
                        if (!depth_queue_.empty()) {
                            depth_item = std::move(depth_queue_.back());
                            depth_queue_.clear();
                            got_depth = true;
                        }
                    }

                    if (!got_color && !got_depth) {
                        const int64_t last_ns = last_color_recv_ns_.load(std::memory_order_relaxed);
                        if (last_ns > 0) {
                            const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                       std::chrono::steady_clock::now().time_since_epoch())
                                                       .count();
                            const int64_t stall_ms = (now_ns - last_ns) / 1000000;
                            if (stall_warn_ms_ > 0 && stall_ms >= stall_warn_ms_ && !warned) {
                                const int64_t last_depth_ns = last_depth_recv_ns_.load(std::memory_order_relaxed);
                                int64_t depth_stall_ms = -1;
                                if (last_depth_ns > 0) depth_stall_ms = (now_ns - last_depth_ns) / 1000000;
                                RCLCPP_WARN(this->get_logger(),
                                            "⚠️ No frames for %ldms (mode=2) (color_stall=%ldms depth_stall=%ldms)",
                                            (long)stall_ms, (long)stall_ms, (long)depth_stall_ms);
                                warned = true;
                            }
                            if (stall_restart_ms_ > 0 && stall_ms >= stall_restart_ms_) {
                                RCLCPP_ERROR(this->get_logger(), "🔁 Deep reset after %ldms color stall (mode=2)", (long)stall_ms);
                                deepResetAndRestart();
                                last_color_recv_ns_.store(
                                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch()).count(),
                                    std::memory_order_relaxed);
                                warned = false;
                            }
                        }
                        break;
                    }
                    warned = false;

                    frame_count++;
                    if (got_color && got_depth && stream_config_.color_enabled && stream_config_.depth_enabled) {
                        const rclcpp::Time stamp = stampFromDeviceTime(color_item.frame, color_item.ts_ms);
                        publishFrames(color_item.frame, depth_item.frame, stamp);
                        color_pub_count++;
                        depth_pub_count++;
                    } else if (got_color && stream_config_.color_enabled) {
                        const rclcpp::Time cstamp = stampFromDeviceTime(color_item.frame, color_item.ts_ms);
                        publishFrames(color_item.frame, rs2::frame(), cstamp);
                        color_pub_count++;
                    } else if (got_depth && stream_config_.depth_enabled) {
                        const rclcpp::Time dstamp = stampFromDeviceTime(depth_item.frame, depth_item.ts_ms);
                        publishFrames(rs2::frame(), depth_item.frame, dstamp);
                        depth_pub_count++;
                    }
                    last_paired_publish_ns_.store(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count(),
                        std::memory_order_relaxed);

                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 1) {
                        const auto dropped_c = dropped_color_frames_.load(std::memory_order_relaxed);
                        const auto dropped_d = dropped_depth_frames_.load(std::memory_order_relaxed);
                        const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                   std::chrono::steady_clock::now().time_since_epoch())
                                                   .count();
                        const int64_t last_color_ns = last_color_recv_ns_.load(std::memory_order_relaxed);
                        const int64_t last_depth_ns = last_depth_recv_ns_.load(std::memory_order_relaxed);
                        const int64_t last_pub_ns = last_paired_publish_ns_.load(std::memory_order_relaxed);
                        const long color_stall_ms = (last_color_ns > 0) ? long((now_ns - last_color_ns) / 1000000) : -1;
                        const long depth_stall_ms = (last_depth_ns > 0) ? long((now_ns - last_depth_ns) / 1000000) : -1;
                        const long pub_stall_ms = (last_pub_ns > 0) ? long((now_ns - last_pub_ns) / 1000000) : -1;
                        RCLCPP_DEBUG(this->get_logger(),
                                     "📊 Mode 2: loop=%d color=%d depth=%d dropped_color=%lu dropped_depth=%lu stall(color=%ldms depth=%ldms pub=%ldms)",
                                     frame_count, color_pub_count, depth_pub_count,
                                     (unsigned long)dropped_c, (unsigned long)dropped_d,
                                     color_stall_ms, depth_stall_ms, pub_stall_ms);
                        frame_count = 0;
                        color_pub_count = 0;
                        depth_pub_count = 0;
                        last_log_time = now;
                    }
                    break;
                }
            }
            
        } catch (const rs2::error& e) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Frame processing error: %s", e.what());
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "🛑 Processing loop stopped");
}

rclcpp::Time FVDepthCameraNode::stampFromDeviceTime(const rs2::frame& frame, double device_ts_ms)
{
    if (!use_device_timestamp_) {
        return this->now();
    }

    const auto domain = frame ? frame.get_frame_timestamp_domain() : RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK;
    const rclcpp::Time now = this->now();

    std::lock_guard<std::mutex> lk(device_time_mutex_);
    if (!device_time_initialized_ || domain != device_time_domain_) {
        device_time_initialized_ = true;
        device_time_domain_ = domain;
        base_device_ts_ms_ = device_ts_ms;
        base_ros_stamp_ = now;
        last_device_ts_ms_ = device_ts_ms;
        last_ros_stamp_ = now;
        return now;
    }

    // Reset mapping if device timestamp jumps backwards significantly (device reset).
    if ((device_ts_ms + device_ts_reset_threshold_ms_) < last_device_ts_ms_) {
        base_device_ts_ms_ = device_ts_ms;
        base_ros_stamp_ = now;
        last_device_ts_ms_ = device_ts_ms;
        last_ros_stamp_ = now;
        return now;
    }

    const double dt_ms = device_ts_ms - base_device_ts_ms_;
    const int64_t dt_ns = static_cast<int64_t>(dt_ms * 1e6);
    rclcpp::Time stamp = base_ros_stamp_ + rclcpp::Duration(std::chrono::nanoseconds(dt_ns));

    // Drift guard: if the device-time-derived stamp diverges from ROS
    // time by more than drift_resync_ms_, re-anchor the baseline to the
    // present. Catches GLOBAL_TIME offset estimation slippage in
    // librealsense and prevents downstream tf2 future-extrapolation
    // failures.
    if (drift_resync_ms_ >= 0.0) {
        const rclcpp::Duration drift = stamp - now;
        const int64_t drift_ns = std::abs(drift.nanoseconds());
        const int64_t threshold_ns =
            static_cast<int64_t>(drift_resync_ms_ * 1e6);
        if (drift_ns > threshold_ns) {
            base_device_ts_ms_ = device_ts_ms;
            base_ros_stamp_ = now;
            stamp = now;
            last_device_ts_ms_ = device_ts_ms;
            last_ros_stamp_ = now;
            // Surface the event at most once per 10 s to avoid log spam.
            const auto steady_now = std::chrono::steady_clock::now();
            static auto s_last_log = steady_now - std::chrono::seconds(11);
            if (steady_now - s_last_log > std::chrono::seconds(10)) {
                s_last_log = steady_now;
                RCLCPP_INFO(
                    this->get_logger(),
                    "device->ROS time mapping re-anchored "
                    "(drift was %.3f s; threshold %.3f s)",
                    drift.seconds(), drift_resync_ms_ / 1000.0);
            }
            return stamp;
        }
    }

    if (stamp < last_ros_stamp_) {
        stamp = last_ros_stamp_;
    }
    last_device_ts_ms_ = device_ts_ms;
    last_ros_stamp_ = stamp;
    return stamp;
}

void FVDepthCameraNode::publishFrames(const rs2::frame& color_frame, const rs2::frame& depth_frame, const rclcpp::Time& stamp)
{
    const rclcpp::Time now = stamp;
    static int publish_count = 0;
    static auto last_publish_log = std::chrono::steady_clock::now();
    
    int current_mode = current_mode_.load();
    
    // モード0（停止）の場合は何も配信しない
    if (current_mode == 0) {
        return;
    }
    
    // Publish color frame (モード1と2で配信)
    if (stream_config_.color_enabled && color_frame && color_pub_) {
        cv::Mat color_image(cv::Size(color_intrinsics_.width, color_intrinsics_.height),
                           CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat bgr_image;
        // If stream provides RGB8, convert to BGR for ROS encoding consistency.
        if (color_frame.get_profile().format() == RS2_FORMAT_RGB8) {
            cv::cvtColor(color_image, bgr_image, cv::COLOR_RGB2BGR);
        } else {
            bgr_image = color_image;
        }
        
        // マーカーとHUDを描画（クリック時のみコピーして描画）
        bool marker_active = false;
        {
            std::lock_guard<std::mutex> lk(point_marker_mutex_);
            marker_active = point_marker_.active;
        }
        if (marker_active) {
            bgr_image = bgr_image.clone();
            drawMarker(bgr_image);
        }
        drawHUD(bgr_image);
        
        auto color_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgr_image).toImageMsg();
        color_msg->header.stamp = now;
        color_msg->header.frame_id = tf_config_.color_optical_frame;
        color_pub_->publish(*color_msg);
        color_pub_count_.fetch_add(1, std::memory_order_relaxed);
        publish_count++;
        
        // Debug: Check if actually published
        static int debug_count = 0;
        if (++debug_count % 30 == 0) {  // Log every 30 frames (1 second)
            RCLCPP_DEBUG(this->get_logger(), "🔍 Published color image to topic: %s", 
                topic_config_.color.c_str());
        }
        
        // Publish compressed color
        if (camera_info_config_.enable_compressed_topics && color_compressed_pub_) {
            // Create compressed image message
            auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
            compressed_msg->header = color_msg->header;
            compressed_msg->format = "jpeg";
            
            // Compress the image
            std::vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
            compression_params.push_back(camera_info_config_.compressed_quality);
            
            cv::imencode(".jpg", bgr_image, compressed_msg->data, compression_params);
            
            color_compressed_pub_->publish(std::move(compressed_msg));
        }

        if (cache_latest_frames_enabled_) {
            try {
                std::lock_guard<std::mutex> lk(latest_frame_mutex_);
                latest_color_frame_ = color_frame;
                latest_color_ts_ms_ = color_frame.get_timestamp();
                latest_color_stamp_ = now;
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to cache latest color frame: %s", e.what());
            }
        }
    }
    
    // Publish depth frame (モード2のみ配信)
    if (current_mode == 2 && stream_config_.depth_enabled && depth_frame && depth_pub_) {
        cv::Mat depth_image(cv::Size(depth_intrinsics_.width, depth_intrinsics_.height), 
                           CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        
        auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", depth_image).toImageMsg();
        depth_msg->header.stamp = now;
        depth_msg->header.frame_id = tf_config_.depth_optical_frame;
        depth_pub_->publish(*depth_msg);
        depth_pub_count_.fetch_add(1, std::memory_order_relaxed);

        if (cache_latest_frames_enabled_) {
            try {
                std::lock_guard<std::mutex> lk(latest_frame_mutex_);
                latest_depth_frame_ = depth_frame;
                latest_depth_ts_ms_ = depth_frame.get_timestamp();
                latest_depth_stamp_ = now;
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to cache latest depth frame: %s", e.what());
            }
        }
    }
    
    // Publish depth colormap (モード2のみ配信)
    if (current_mode == 2 && stream_config_.depth_colormap_enabled && depth_frame && depth_colormap_pub_) {
        cv::Mat colormap = createDepthColormap(depth_frame);
        auto colormap_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", colormap).toImageMsg();
        colormap_msg->header.stamp = now;
        colormap_msg->header.frame_id = tf_config_.depth_optical_frame;
        depth_colormap_pub_->publish(*colormap_msg);
    }

    // Publish organized registered_points (optional)
    if (current_mode == 2 && organized_pointcloud_enabled_ && registered_points_pub_ && depth_frame) {
        // Publish only if there are subscribers to reduce CPU
        if (registered_points_pub_->get_subscription_count() != 0) {
            try {
            int dw = depth_intrinsics_.width;
            int dh = depth_intrinsics_.height;
            int step = std::max(1, organized_pointcloud_decimation_);
            bool include_rgb = organized_pointcloud_rgb_ && color_frame;

            sensor_msgs::msg::PointCloud2 cloud_msg;
            cloud_msg.header.stamp = now;
            cloud_msg.header.frame_id = align_to_color_ ? tf_config_.color_optical_frame : tf_config_.depth_optical_frame;
            cloud_msg.width = dw / step;
            cloud_msg.height = dh / step;
            cloud_msg.is_bigendian = false;
            cloud_msg.is_dense = false;

            std::vector<sensor_msgs::msg::PointField> fields;
            auto addf = [&](const std::string& n, uint32_t off){ sensor_msgs::msg::PointField f; f.name=n; f.offset=off; f.datatype=sensor_msgs::msg::PointField::FLOAT32; f.count=1; fields.push_back(f); };
            addf("x",0); addf("y",4); addf("z",8);
            uint32_t point_step = 12;
            if (include_rgb) { sensor_msgs::msg::PointField f; f.name="rgb"; f.offset=12; f.datatype=sensor_msgs::msg::PointField::FLOAT32; f.count=1; fields.push_back(f); point_step=16; }
            cloud_msg.fields = fields;
            cloud_msg.point_step = point_step;
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
            cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);

            cv::Mat depth_image(cv::Size(depth_intrinsics_.width, depth_intrinsics_.height), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat color_image;
            if (include_rgb) {
                color_image = cv::Mat(cv::Size(color_intrinsics_.width, color_intrinsics_.height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            }

            uint8_t* dst = cloud_msg.data.data();
            for (int v = 0; v < dh; v += step) {
                for (int u = 0; u < dw; u += step) {
                    uint16_t d = depth_image.at<uint16_t>(v, u);
                    float z = (d > 0) ? d * depth_scale_ : std::numeric_limits<float>::quiet_NaN();
                    // Distance clipping (z-based)
                    if (d == 0 || z <= min_distance_m_ || z >= max_distance_m_) {
                        // write NaNs and continue to keep organized structure
                        float nanv = std::numeric_limits<float>::quiet_NaN();
                        memcpy(dst+0,&nanv,4); memcpy(dst+4,&nanv,4); memcpy(dst+8,&nanv,4);
                        if (include_rgb) { float nanrgb = std::numeric_limits<float>::quiet_NaN(); memcpy(dst+12,&nanrgb,4); }
                        dst += point_step;
                        continue;
                    }
                    float x = std::numeric_limits<float>::quiet_NaN();
                    float y = std::numeric_limits<float>::quiet_NaN();
                    if (d > 0) {
                        float pix[2] = {static_cast<float>(u), static_cast<float>(v)};
                        float pt[3];
                        rs2_deproject_pixel_to_point(pt, &depth_intrinsics_, pix, z);
                        x = pt[0]; y = pt[1];
                    }
                    memcpy(dst+0,&x,4); memcpy(dst+4,&y,4); memcpy(dst+8,&z,4);
                    if (include_rgb) {
                        float rgbf = std::numeric_limits<float>::quiet_NaN();
                        if (!color_image.empty()) {
                            cv::Vec3b c = color_image.at<cv::Vec3b>(v,u);
                            uint32_t rgb = (uint32_t(c[2])<<16) | (uint32_t(c[1])<<8) | uint32_t(c[0]);
                            memcpy(&rgbf,&rgb,4);
                        }
                        memcpy(dst+12,&rgbf,4);
                    }
                    dst += point_step;
                }
            }
            registered_points_pub_->publish(cloud_msg);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "organized cloud publish failed: %s", e.what());
            }
        }
    }
    
    // Publish camera info
    if (camera_info_config_.enable_camera_info) {
        if (color_info_pub_ && color_frame) {
            sensor_msgs::msg::CameraInfo color_info;
            color_info.header.stamp = now;
            color_info.header.frame_id = tf_config_.color_optical_frame;
            color_info.width = color_intrinsics_.width;
            color_info.height = color_intrinsics_.height;
            color_info.distortion_model = "plumb_bob";
            
            // Set intrinsic matrix
            color_info.k[0] = color_intrinsics_.fx;
            color_info.k[2] = color_intrinsics_.ppx;
            color_info.k[4] = color_intrinsics_.fy;
            color_info.k[5] = color_intrinsics_.ppy;
            color_info.k[8] = 1.0;
            
            // Set projection matrix
            color_info.p[0] = color_intrinsics_.fx;
            color_info.p[2] = color_intrinsics_.ppx;
            color_info.p[5] = color_intrinsics_.fy;
            color_info.p[6] = color_intrinsics_.ppy;
            color_info.p[10] = 1.0;
            
            // Set distortion coefficients
            color_info.d.resize(5);
            for (int i = 0; i < 5; i++) {
                color_info.d[i] = color_intrinsics_.coeffs[i];
            }
            
            color_info_pub_->publish(color_info);
        }
        
        if (depth_info_pub_ && depth_frame) {
            sensor_msgs::msg::CameraInfo depth_info;
            depth_info.header.stamp = now;
            depth_info.header.frame_id = tf_config_.depth_optical_frame;
            depth_info.width = depth_intrinsics_.width;
            depth_info.height = depth_intrinsics_.height;
            depth_info.distortion_model = "plumb_bob";
            
            // Set intrinsic matrix
            depth_info.k[0] = depth_intrinsics_.fx;
            depth_info.k[2] = depth_intrinsics_.ppx;
            depth_info.k[4] = depth_intrinsics_.fy;
            depth_info.k[5] = depth_intrinsics_.ppy;
            depth_info.k[8] = 1.0;
            
            // Set projection matrix
            depth_info.p[0] = depth_intrinsics_.fx;
            depth_info.p[2] = depth_intrinsics_.ppx;
            depth_info.p[5] = depth_intrinsics_.fy;
            depth_info.p[6] = depth_intrinsics_.ppy;
            depth_info.p[10] = 1.0;
            
            // Set distortion coefficients
            depth_info.d.resize(5);
            for (int i = 0; i < 5; i++) {
                depth_info.d[i] = depth_intrinsics_.coeffs[i];
            }
            
            depth_info_pub_->publish(depth_info);
        }
    }

    // Optional point cloud (requires both frames)
    if (current_mode == 2 && stream_config_.pointcloud_enabled && color_frame && depth_frame) {
        publishPointCloud(color_frame, depth_frame);
    }
    
    // Log publishing status
    auto current_time = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_publish_log).count() >= 1) {
        RCLCPP_DEBUG(this->get_logger(), "📤 Published %d frames in last second", publish_count);
        publish_count = 0;
        last_publish_log = current_time;
    }
}
void FVDepthCameraNode::drawHUD(cv::Mat& frame) const
{
    (void)frame; // HUD disabled per user request
}

void FVDepthCameraNode::publishPointCloud(const rs2::frame& color_frame, const rs2::frame& depth_frame)
{
    RCLCPP_DEBUG(this->get_logger(), "🔍 publishPointCloud called");
    
    // Point cloud requires both color and depth frames
    if (!color_frame || !depth_frame) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Missing frames - color: %s, depth: %s", 
            color_frame ? "✅" : "❌", depth_frame ? "✅" : "❌");
        return;
    }
    
    // Check if publisher is valid
    if (!pointcloud_pub_) {
        RCLCPP_ERROR(this->get_logger(), "❌ Point cloud publisher is null!");
        return;
    }
    
    // Check publisher status
    size_t sub_count = pointcloud_pub_->get_subscription_count();
    RCLCPP_DEBUG(this->get_logger(), "📊 Point cloud publisher - subscribers: %zu", sub_count);
    
    // Create point cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    
    // Get color and depth data
    cv::Mat color_image(cv::Size(color_intrinsics_.width, color_intrinsics_.height), 
                       CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat depth_image(cv::Size(depth_intrinsics_.width, depth_intrinsics_.height), 
                       CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    
    RCLCPP_DEBUG(this->get_logger(), "📐 Images - color: %dx%d, depth: %dx%d", 
        color_image.cols, color_image.rows, depth_image.cols, depth_image.rows);
    
    // Convert to point cloud
    int valid_points = 0;
    int skipped_points = 0;
    for (int y = 0; y < depth_intrinsics_.height; y += 2) {
        for (int x = 0; x < depth_intrinsics_.width; x += 2) {
            float depth = depth_image.at<uint16_t>(y, x) * depth_scale_;
            // Apply clipping based on configured distances
            if (depth > min_distance_m_ && depth < max_distance_m_) {
                float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
                float point[3];
                
                rs2_deproject_pixel_to_point(point, &depth_intrinsics_, pixel, depth);
                
                pcl::PointXYZRGB pcl_point;
                pcl_point.x = point[0];
                pcl_point.y = point[1];
                pcl_point.z = point[2];
                
                // Get color
                cv::Vec3b color = color_image.at<cv::Vec3b>(y, x);
                pcl_point.r = color[2];
                pcl_point.g = color[1];
                pcl_point.b = color[0];
                
                cloud.points.push_back(pcl_point);
                valid_points++;
            } else {
                skipped_points++;
            }
        }
    }
    
    RCLCPP_DEBUG(this->get_logger(), "☁️ Point cloud - valid: %d, skipped: %d, total: %zu", 
        valid_points, skipped_points, cloud.points.size());
    
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;
    
    // Publish
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    cloud_msg.header.frame_id = tf_config_.color_optical_frame;
    
    RCLCPP_DEBUG(this->get_logger(), "📤 Publishing point cloud with %zu points", cloud.points.size());
    pointcloud_pub_->publish(cloud_msg);
    RCLCPP_DEBUG(this->get_logger(), "✅ Point cloud published successfully");
}

cv::Mat FVDepthCameraNode::createDepthColormap(const rs2::frame& depth_frame)
{
    cv::Mat depth_image(cv::Size(depth_intrinsics_.width, depth_intrinsics_.height), 
                       CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    
    cv::Mat colormap;
    depth_image.convertTo(colormap, CV_8UC1, 255.0 / 65535.0);
    cv::applyColorMap(colormap, colormap, cv::COLORMAP_JET);
    
    return colormap;
}

void FVDepthCameraNode::publishTF()
{
    if (!tf_config_.enabled) {
        return;
    }
    
    auto now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    
    // Static transform from base_link to camera_link
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now;
    transform.header.frame_id = tf_config_.base_frame;
    transform.child_frame_id = tf_config_.camera_frame;
    
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    
    tf_static_broadcaster_->sendTransform(transform);
    
    // Static transform from camera_link to color_optical_frame
    geometry_msgs::msg::TransformStamped color_transform;
    color_transform.header.stamp = now;
    color_transform.header.frame_id = tf_config_.camera_frame;
    color_transform.child_frame_id = tf_config_.color_optical_frame;
    
    color_transform.transform.translation.x = 0.0;
    color_transform.transform.translation.y = 0.0;
    color_transform.transform.translation.z = 0.0;
    color_transform.transform.rotation.x = 0.0;
    color_transform.transform.rotation.y = 0.0;
    color_transform.transform.rotation.z = 0.0;
    color_transform.transform.rotation.w = 1.0;
    
    tf_static_broadcaster_->sendTransform(color_transform);
    
    // Static transform from camera_link to depth_optical_frame
    geometry_msgs::msg::TransformStamped depth_transform;
    depth_transform.header.stamp = now;
    depth_transform.header.frame_id = tf_config_.camera_frame;
    depth_transform.child_frame_id = tf_config_.depth_optical_frame;
    
    depth_transform.transform.translation.x = 0.0;
    depth_transform.transform.translation.y = 0.0;
    depth_transform.transform.translation.z = 0.0;
    depth_transform.transform.rotation.x = 0.0;
    depth_transform.transform.rotation.y = 0.0;
    depth_transform.transform.rotation.z = 0.0;
    depth_transform.transform.rotation.w = 1.0;
    
    tf_static_broadcaster_->sendTransform(depth_transform);
}

void FVDepthCameraNode::handleGetDistance(
    const std::shared_ptr<fv_realsense::srv::GetDistance::Request> request,
    std::shared_ptr<fv_realsense::srv::GetDistance::Response> response)
{
    float world_x, world_y, world_z;
    
    if (get3DCoordinate(request->x, request->y, world_x, world_y, world_z)) {
        response->success = true;
        response->x = world_x;
        response->y = world_y;
        response->z = world_z;
        response->frame_id = tf_config_.color_optical_frame;
        response->message = "Success";
    } else {
        response->success = false;
        response->x = 0.0;
        response->y = 0.0;
        response->z = 0.0;
        response->frame_id = tf_config_.color_optical_frame;
        response->message = "Failed to get 3D coordinate";
    }
}

void FVDepthCameraNode::handleGetCameraInfo(
    const std::shared_ptr<fv_realsense::srv::GetCameraInfo::Request> request,
    std::shared_ptr<fv_realsense::srv::GetCameraInfo::Response> response)
{
    (void)request; // suppress unused parameter warning
    try {
        response->success = true;
        response->camera_name = device_.get_info(RS2_CAMERA_INFO_NAME);
        response->serial_number = device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        response->device_name = device_.get_info(RS2_CAMERA_INFO_NAME);
        response->color_width = color_intrinsics_.width;
        response->color_height = color_intrinsics_.height;
        response->color_fps = camera_config_.color_fps;
        response->depth_width = depth_intrinsics_.width;
        response->depth_height = depth_intrinsics_.height;
        response->depth_fps = camera_config_.depth_fps;
        response->depth_scale = depth_scale_;
        response->message = "Success";
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Error: ") + e.what();
    }
}

void FVDepthCameraNode::handleSetMode(
    const std::shared_ptr<fv_realsense::srv::SetMode::Request> request,
    std::shared_ptr<fv_realsense::srv::SetMode::Response> response)
{
    try {
        int requested_mode = request->mode;
        
        // モード値の検証
        if (requested_mode < 0 || requested_mode > 2) {
            response->success = false;
            response->message = "Invalid mode. Must be 0, 1, or 2";
            response->current_mode = current_mode_.load();
            return;
        }
        
        // モードの設定
        current_mode_.store(requested_mode);
        
        // モード別の処理
        switch (requested_mode) {
            case 0:  // 表示なし
                RCLCPP_INFO(this->get_logger(), "🛑 Mode set to 0: NO DISPLAY");
                response->message = "Mode set to NO DISPLAY (0) - No marker shown";
                break;
                
            case 1:  // カーソルのみ
                RCLCPP_INFO(this->get_logger(), "🟢 Mode set to 1: CURSOR ONLY");
                response->message = "Mode set to CURSOR ONLY (1) - Green cursor for 10 seconds";
                break;
                
            case 2:  // カーソル + 座標 + 距離
                RCLCPP_INFO(this->get_logger(), "🔵 Mode set to 2: CURSOR + COORDINATES + DISTANCE");
                response->message = "Mode set to CURSOR + COORDINATES + DISTANCE (2) - Full info for 10 seconds";
                break;
        }
        
        response->success = true;
        response->current_mode = current_mode_.load();
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Error setting mode: ") + e.what();
        response->current_mode = current_mode_.load();
    }
}

// GeneratePointCloud handler removed

void FVDepthCameraNode::clickEventCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    try {
        // クリック座標を取得
        int x = static_cast<int>(msg->x);
        int y = static_cast<int>(msg->y);
        
        // 3D座標を取得
        float world_x, world_y, world_z;
        if (get3DCoordinate(x, y, world_x, world_y, world_z)) {
            // ポイントマーカーを更新
            {
                std::lock_guard<std::mutex> lk(point_marker_mutex_);
                point_marker_.point = cv::Point(x, y);
                point_marker_.start_time = this->now();
                point_marker_.active = true;
                point_marker_.mode = current_mode_.load();
                point_marker_.x = world_x;
                point_marker_.y = world_y;
                point_marker_.z = world_z;
            }
            
            RCLCPP_INFO(this->get_logger(), "🖱️ Click at (%d, %d) -> 3D: (%.3f, %.3f, %.3f)", 
                x, y, world_x, world_y, world_z);
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ Failed to get 3D coordinate for click at (%d, %d)", x, y);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Error in click event callback: %s", e.what());
    }
}

bool FVDepthCameraNode::get3DCoordinate(int x, int y, float& world_x, float& world_y, float& world_z)
{
    try {
        // NOTE:
        // - Do not call pipe_.wait_for_frames()/try_wait_for_frames() here (can block and/or open UVC twice).
        // - Use the latest depth frame captured by sensor callbacks.
        rs2::frame depth_frame;
        {
            std::lock_guard<std::mutex> lk(latest_frame_mutex_);
            depth_frame = latest_depth_frame_;
        }
        if (!depth_frame) {
            return false;
        }
        cv::Mat depth_mat(cv::Size(depth_intrinsics_.width, depth_intrinsics_.height),
                          CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        if (x < 0 || y < 0 || x >= depth_mat.cols || y >= depth_mat.rows) {
            return false;
        }

        // Get depth value (meters)
        const uint16_t raw = depth_mat.at<uint16_t>(y, x);
        float depth = static_cast<float>(raw) * depth_scale_;
        
        if (depth <= 0.0f) {
            return false;
        }
        
        // Convert to 3D coordinates
        float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
        float point[3];
        
        rs2_deproject_pixel_to_point(point, &depth_intrinsics_, pixel, depth);
        
        world_x = point[0];
        world_y = point[1];
        world_z = point[2];
        
        return true;
        
    } catch (const rs2::error& e) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Error getting 3D coordinate: %s", e.what());
        return false;
    }
}

void FVDepthCameraNode::drawMarker(cv::Mat& frame) const
{
    std::lock_guard<std::mutex> lk(point_marker_mutex_);
    if (!point_marker_.active) {
        return;
    }
    
    // 10秒経過したら非アクティブにする
    auto now = this->now();
    auto elapsed = now - point_marker_.start_time;
    if (elapsed.seconds() > 10.0) {
        point_marker_.active = false;
        return;
    }
    
    // モードに応じて表示
    switch (point_marker_.mode) {
        case 0:  // 表示なし
            return;
            
        case 1: {  // カーソルのみ
            cv::circle(frame, point_marker_.point, 10, cv::Scalar(0, 255, 0), 2);
            cv::circle(frame, point_marker_.point, 2, cv::Scalar(0, 255, 0), -1);
            break;
        }
            
        case 2: {  // カーソル + 座標 + 距離
            // カーソル描画
            cv::circle(frame, point_marker_.point, 10, cv::Scalar(0, 255, 0), 2);
            cv::circle(frame, point_marker_.point, 2, cv::Scalar(0, 255, 0), -1);
            
            // 座標テキスト
            std::string coord_text = cv::format("XY: (%d, %d)", 
                point_marker_.point.x, point_marker_.point.y);
            cv::putText(frame, coord_text, 
                cv::Point(point_marker_.point.x + 15, point_marker_.point.y - 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            
            // 3D座標テキスト
            std::string xyz_text = cv::format("XYZ: (%.2f, %.2f, %.2f)m", 
                point_marker_.x, point_marker_.y, point_marker_.z);
            cv::putText(frame, xyz_text, 
                cv::Point(point_marker_.point.x + 15, point_marker_.point.y),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            break;
        }
    }
}

std::vector<rs2::device> FVDepthCameraNode::getAvailableDevices()
{
    std::vector<rs2::device> devices;
    
    try {
        RCLCPP_INFO(this->get_logger(), "🔍 Querying RealSense devices...");
        if (!ctx_) {
            RCLCPP_ERROR(this->get_logger(), "❌ RealSense context is not initialized");
            return devices;
        }
        auto device_list = ctx_->query_devices();
        size_t device_count = device_list.size();
        
        RCLCPP_INFO(this->get_logger(), "🔍 Found %zu RealSense device(s)", device_count);
        
        // Limit the number of devices to prevent memory issues
        const size_t max_devices = 5;  // Reduced from 10 to 5
        if (device_count > max_devices) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Limiting devices to %zu (found %zu)", max_devices, device_count);
            device_count = max_devices;
        }
        
        for (size_t i = 0; i < device_count; ++i) {
            try {
                RCLCPP_INFO(this->get_logger(), "📷 Accessing device %zu...", i);
                auto device = device_list[i];
                
                // Get device info with error handling
                std::string name, serial;
                try {
                    name = device.get_info(RS2_CAMERA_INFO_NAME);
                } catch (const rs2::error& e) {
                    name = "Unknown";
                    RCLCPP_WARN(this->get_logger(), "⚠️ Could not get name for device %zu: %s", i, e.what());
                }
                
                try {
                    serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                } catch (const rs2::error& e) {
                    serial = "Unknown";
                    RCLCPP_WARN(this->get_logger(), "⚠️ Could not get serial for device %zu: %s", i, e.what());
                }
                
                RCLCPP_INFO(this->get_logger(), "📷 Device %zu: %s (SN: %s)", i, name.c_str(), serial.c_str());
                devices.push_back(device);
            } catch (const rs2::error& e) {
                RCLCPP_WARN(this->get_logger(), "⚠️ Error accessing device %zu: %s", i, e.what());
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "⚠️ Exception accessing device %zu: %s", i, e.what());
            }
        }
    } catch (const rs2::error& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Error querying devices: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Exception querying devices: %s", e.what());
    }
    
    return devices;
}

/**
 * @brief メイン関数
 * @param argc コマンドライン引数の数
 * @param argv コマンドライン引数の配列
 * @return int 終了コード
 * @details RealSenseカメラノードの初期化と実行
 * 
 * 実行内容：
 * - ROS2の初期化
 * - RealSenseカメラノードの作成
 * - ノードの実行（スピン）
 * - 適切な終了処理
 */
int main(int argc, char** argv)
{
    try {
        rclcpp::init(argc, argv);
        
        RCLCPP_INFO(rclcpp::get_logger("fv_realsense"), "🚀 Starting FV RealSense Node...");

        // ===== デフォルト名でノードを作成（launchファイルで必要に応じてリマップ） =====
        auto node = std::make_shared<FVDepthCameraNode>("fv_realsense");
        
        if (node) {
            RCLCPP_INFO(rclcpp::get_logger("fv_realsense"), "✅ Node created successfully");
            rclcpp::spin(node);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("fv_realsense"), "❌ Failed to create node");
            return 1;
        }
        
        rclcpp::shutdown();
        return 0;
        
    } catch (const std::bad_alloc& e) {
        RCLCPP_ERROR(rclcpp::get_logger("fv_realsense"),
            "❌ Memory allocation failed: %s. "
            "Check camera resolution parameters — the device may not support the requested resolution. "
            "Supported resolutions: 424x240, 480x270, 640x360, 640x480, 848x480, 1280x720", e.what());
        return 1;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("fv_realsense"),
            "❌ Exception in main: %s. "
            "If the camera was previously in use, try unplugging and reconnecting the USB cable.", e.what());
        return 1;
    }
} 
