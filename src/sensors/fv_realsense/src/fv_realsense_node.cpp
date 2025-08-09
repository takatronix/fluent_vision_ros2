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
#include <cv_bridge/cv_bridge.h>
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
    
    // Debug: Print all loaded parameters
    RCLCPP_INFO(this->get_logger(), "🔍 All loaded parameters:");
    RCLCPP_INFO(this->get_logger(), "   - Camera selection method: '%s'", camera_selection_config_.selection_method.c_str());
    RCLCPP_INFO(this->get_logger(), "   - Camera selection name: '%s'", camera_selection_config_.device_name.c_str());
    RCLCPP_INFO(this->get_logger(), "   - Camera selection serial: '%s'", camera_selection_config_.serial_number.c_str());
    RCLCPP_INFO(this->get_logger(), "   - Camera selection index: %d", camera_selection_config_.device_index);
    RCLCPP_INFO(this->get_logger(), "   - Color width: %d", camera_config_.color_width);
    RCLCPP_INFO(this->get_logger(), "   - Color height: %d", camera_config_.color_height);
    RCLCPP_INFO(this->get_logger(), "   - Color fps: %d", camera_config_.color_fps);
    RCLCPP_INFO(this->get_logger(), "   - Depth width: %d", camera_config_.depth_width);
    RCLCPP_INFO(this->get_logger(), "   - Depth height: %d", camera_config_.depth_height);
    RCLCPP_INFO(this->get_logger(), "   - Depth fps: %d", camera_config_.depth_fps);
    RCLCPP_INFO(this->get_logger(), "   - Color topic: %s", topic_config_.color.c_str());
    RCLCPP_INFO(this->get_logger(), "   - Depth topic: %s", topic_config_.depth.c_str());
    
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
        ctx_ = rs2::context();
        
        // Device selection based on configuration
        RCLCPP_INFO(this->get_logger(), "🔍 Device selection...");
        if (!selectCamera()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to select camera");
            return false;
        }
        
        // Configure pipeline with minimal settings FIRST
        if (stream_config_.color_enabled) {
            cfg_.enable_stream(RS2_STREAM_COLOR, 
                camera_config_.color_width, camera_config_.color_height, 
                RS2_FORMAT_BGR8, camera_config_.color_fps);
            RCLCPP_INFO(this->get_logger(), "📹 Enabled color stream: %dx%d @ %dfps", 
                camera_config_.color_width, camera_config_.color_height, camera_config_.color_fps);
        }
        
        // Wait 2 seconds like vision_ai does
        RCLCPP_INFO(this->get_logger(), "⏳ Waiting 2 seconds before enabling depth stream...");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Enable depth stream if configured
        if (stream_config_.depth_enabled) {
            cfg_.enable_stream(RS2_STREAM_DEPTH,
                camera_config_.depth_width, camera_config_.depth_height,
                RS2_FORMAT_Z16, camera_config_.depth_fps);
            RCLCPP_INFO(this->get_logger(), "📏 Enabled depth stream: %dx%d @ %dfps",
                camera_config_.depth_width, camera_config_.depth_height, camera_config_.depth_fps);
        }
        
        // 重要：同期設定を追加
        if (stream_config_.sync_enabled && stream_config_.color_enabled && stream_config_.depth_enabled) {
            // 真の同期設定を有効化
            RCLCPP_INFO(this->get_logger(), "🔗 Enabling true synchronization for color and depth streams");
            
            // 1. 同期オプションを設定
            rs2::config sync_config;
            sync_config.enable_stream(RS2_STREAM_COLOR, camera_config_.color_width, camera_config_.color_height, RS2_FORMAT_BGR8, camera_config_.color_fps);
            sync_config.enable_stream(RS2_STREAM_DEPTH, camera_config_.depth_width, camera_config_.depth_height, RS2_FORMAT_Z16, camera_config_.depth_fps);
            
            // 2. 同期設定（wait_for_framesが自動的に同期する）
            
            // 3. 同期設定を適用
            cfg_ = sync_config;
            
            // 4. パイプライン開始後に同期オプションを設定
            sync_enabled_ = true;  // フラグを設定
            
            RCLCPP_INFO(this->get_logger(), "🔗 Synchronization enabled - frames will be synchronized by wait_for_frames");
        } else {
            sync_enabled_ = false;
            RCLCPP_INFO(this->get_logger(), "⚠️ Synchronization disabled - streams may have timing differences");
        }
        
        // Apply startup delay for power management
        if (power_management_config_.startup_delay > 0.0) {
            RCLCPP_INFO(this->get_logger(), "⏳ Waiting %.1f seconds for power stabilization...", 
                power_management_config_.startup_delay);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(power_management_config_.startup_delay * 1000)));
        }
        
        // Configure specific device AFTER stream configuration (like vision_ai)
        if (device_) {
            std::string serial = device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            cfg_.enable_device(serial);
            RCLCPP_INFO(this->get_logger(), "📷 Pipeline configured for device: %s (SN: %s)", 
                device_.get_info(RS2_CAMERA_INFO_NAME), serial.c_str());
        }
        
        // Start pipeline with detailed error handling
        RCLCPP_INFO(this->get_logger(), "🚀 Starting RealSense pipeline...");
        try {
            profile_ = pipe_.start(cfg_);
            RCLCPP_INFO(this->get_logger(), "✅ Pipeline started successfully");
        } catch (const rs2::backend_error& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Backend error: %s", e.what());
            RCLCPP_ERROR(this->get_logger(), "📝 Device may be in use by another process");
            return false;
        } catch (const rs2::error& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ RealSense error: %s", e.what());
            return false;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Exception: %s", e.what());
            return false;
        }
        
        if (!profile_) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to start pipeline");
            return false;
        }
        
        // Wait for first frames to ensure pipeline is ready
        try {
            RCLCPP_INFO(this->get_logger(), "⏳ Waiting for initial frames...");
            auto initial_frames = pipe_.wait_for_frames(5000); // 5 second timeout
            auto color_frame = initial_frames.get_color_frame();
            auto depth_frame = initial_frames.get_depth_frame();
            
            if (color_frame) {
                RCLCPP_INFO(this->get_logger(), "✅ Got initial color frame: %dx%d", 
                    color_frame.get_width(), color_frame.get_height());
            }
            if (depth_frame) {
                RCLCPP_INFO(this->get_logger(), "✅ Got initial depth frame: %dx%d",
                    depth_frame.get_width(), depth_frame.get_height());
            }
        } catch (const rs2::error& e) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Could not get initial frames: %s", e.what());
        }
        
        device_ = profile_.get_device();
        if (!device_) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to get device from profile");
            return false;
        }
        
        // Get depth scale with error handling
        try {
            // Log device info before getting depth scale
            RCLCPP_INFO(this->get_logger(), "🔍 Getting depth scale from device: %s (SN: %s)",
                device_.get_info(RS2_CAMERA_INFO_NAME),
                device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
            
            // Get depth scale from depth sensor
            for (auto sensor : device_.query_sensors()) {
                if (sensor.is<rs2::depth_sensor>()) {
                    rs2::depth_sensor depth_sensor = sensor.as<rs2::depth_sensor>();
                    depth_scale_ = depth_sensor.get_depth_scale();
                    RCLCPP_INFO(this->get_logger(), "📏 Depth scale: %f", depth_scale_);
                    
                    // Double check with manual depth scale based on device model
                    std::string device_name = device_.get_info(RS2_CAMERA_INFO_NAME);
                    if (device_name.find("D415") != std::string::npos) {
                        if (std::abs(depth_scale_ - 0.001f) > 0.0001f) {
                            RCLCPP_WARN(this->get_logger(), "⚠️ D415 depth scale mismatch: got %f, expected 0.001", depth_scale_);
                            depth_scale_ = 0.001f;  // Force correct value for D415
                        }
                    } else if (device_name.find("D405") != std::string::npos) {
                        // D405は実際には0.0001を返すが、これは間違い。強制的に0.001に設定
                        RCLCPP_WARN(this->get_logger(), "⚠️ D405 depth scale override: got %f, forcing to 0.001", depth_scale_);
                        depth_scale_ = 0.001f;  // Force correct value for D405 (1mm per unit, same as D415)
                    }
                    break;
                }
            }
            // 設定ファイルからのオーバーライド
            if (config_depth_scale_ > 0) {
                RCLCPP_INFO(this->get_logger(), "📏 Overriding depth scale from config: %f -> %f", depth_scale_, config_depth_scale_);
                depth_scale_ = config_depth_scale_;
            }
            
            RCLCPP_INFO(this->get_logger(), "📏 Final depth scale: %f", depth_scale_);
        } catch (const rs2::error& e) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Could not get depth scale: %s", e.what());
            depth_scale_ = (config_depth_scale_ > 0) ? config_depth_scale_ : 0.001f; // Use config or default
        }
        
        // Get intrinsics with error handling
        try {
            if (stream_config_.color_enabled) {
                auto color_stream = profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
                color_intrinsics_ = color_stream.get_intrinsics();
                RCLCPP_INFO(this->get_logger(), "📹 Color intrinsics: %dx%d", 
                    color_intrinsics_.width, color_intrinsics_.height);
            }
            
            if (stream_config_.depth_enabled) {
                auto depth_stream = profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
                depth_intrinsics_ = depth_stream.get_intrinsics();
                RCLCPP_INFO(this->get_logger(), "📐 Depth intrinsics: %dx%d", 
                    depth_intrinsics_.width, depth_intrinsics_.height);
            }
        } catch (const rs2::error& e) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Could not get intrinsics: %s", e.what());
        }
        
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

bool FVDepthCameraNode::selectCamera()
{
    auto devices = getAvailableDevices();
    
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
    
    // Create image transport for compressed images
    // Note: ImageTransport will be created after the node is fully initialized
    
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
    auto last_log_time = std::chrono::steady_clock::now();
    
    while (running_ && rclcpp::ok()) {
        try {
            // モードに応じた処理
            int current_mode = current_mode_.load();
            
            switch (current_mode) {
                case 0: {  // 停止モード
                    // フレーム取得のみ（配信なし）
                    pipe_.wait_for_frames(1000);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    break;
                }
                    
                case 1: {  // 基本動作モード
                    // カラー画像のみ配信
                    rs2::frameset frames = pipe_.wait_for_frames(1000);
                    auto color_frame = frames.get_color_frame();
                    
                    if (color_frame) {
                        frame_count++;
                        publishFrames(color_frame, rs2::frame());  // 深度は配信しない
                        
                        // Log every second
                        auto now = std::chrono::steady_clock::now();
                        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 1) {
                            RCLCPP_DEBUG(this->get_logger(), "📊 Mode 1: %d frames, Color: %s", 
                                frame_count, color_frame ? "✅" : "❌");
                            frame_count = 0;
                            last_log_time = now;
                        }
                    }
                    break;
                }
                    
                case 2: {  // フル機能モード
                    // 同期が有効な場合は同期フレームセットを使用
                    rs2::frameset frames_full;
                    if (sync_enabled_) {
                        // 同期フレームセットを取得（深度とカラーが同じタイムスタンプ）
                        frames_full = pipe_.wait_for_frames(1000);
                        if (align_to_color_) {
                            static rs2::align align_color(RS2_STREAM_COLOR);
                            frames_full = align_color.process(frames_full);
                            // Use color intrinsics for aligned depth
                            depth_intrinsics_ = color_intrinsics_;
                        }
                        
                        // 同期チェック（タイムスタンプが同じか確認）
                        auto color_frame_full = frames_full.get_color_frame();
                        auto depth_frame_full = frames_full.get_depth_frame();
                        
                        if (color_frame_full && depth_frame_full) {
                            // タイムスタンプを比較（ミリ秒単位で）
                            double color_timestamp = color_frame_full.get_timestamp();
                            double depth_timestamp = depth_frame_full.get_timestamp();
                            double timestamp_diff = std::abs(color_timestamp - depth_timestamp);
                            
                            if (timestamp_diff > 1.0) {  // 1ms以上ずれている場合
                                RCLCPP_WARN(this->get_logger(), 
                                    "⚠️ Frame sync warning: color=%.3f, depth=%.3f, diff=%.3fms", 
                                    color_timestamp, depth_timestamp, timestamp_diff);
                            } else {
                                RCLCPP_DEBUG(this->get_logger(), 
                                    "🔗 Frames synchronized: color=%.3f, depth=%.3f, diff=%.3fms", 
                                    color_timestamp, depth_timestamp, timestamp_diff);
                            }
                        }
                    } else {
                        // 非同期モード（従来通り）
                        frames_full = pipe_.wait_for_frames(1000);
                        if (align_to_color_) {
                            static rs2::align align_color(RS2_STREAM_COLOR);
                            frames_full = align_color.process(frames_full);
                            depth_intrinsics_ = color_intrinsics_;
                        }
                    }
                    
                    auto color_frame_full = frames_full.get_color_frame();
                    auto depth_frame_full = frames_full.get_depth_frame();
                    
                    if (color_frame_full || depth_frame_full) {
                        frame_count++;
                        publishFrames(color_frame_full, depth_frame_full);
                        // Cache latest frames for service-safe access
            try {
                            std::lock_guard<std::mutex> lk(latest_frame_mutex_);
                // Use a consistent SYSTEM_TIME clock for all stamps to avoid mixed time sources
                latest_frame_stamp_ = rclcpp::Clock(RCL_SYSTEM_TIME).now();
                            if (color_frame_full) {
                                latest_color_image_mat_ = cv::Mat(cv::Size(color_intrinsics_.width, color_intrinsics_.height),
                                                                  CV_8UC3, (void*)color_frame_full.get_data(), cv::Mat::AUTO_STEP).clone();
                            } else {
                                latest_color_image_mat_.release();
                            }
                            if (depth_frame_full) {
                                latest_depth_image_mat_ = cv::Mat(cv::Size(depth_intrinsics_.width, depth_intrinsics_.height),
                                                                  CV_16UC1, (void*)depth_frame_full.get_data(), cv::Mat::AUTO_STEP).clone();
                            } else {
                                latest_depth_image_mat_.release();
                            }
                        } catch (const std::exception& e) {
                            RCLCPP_WARN(this->get_logger(), "Failed to cache latest frames: %s", e.what());
                        }
                        
                        // Log every second
                        auto now = std::chrono::steady_clock::now();
                        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 1) {
                            RCLCPP_DEBUG(this->get_logger(), "📊 Mode 2: %d frames, Color: %s, Depth: %s, Sync: %s", 
                                frame_count, 
                                color_frame_full ? "✅" : "❌",
                                depth_frame_full ? "✅" : "❌",
                                sync_enabled_ ? "🔗" : "⚠️");
                            frame_count = 0;
                            last_log_time = now;
                        }
                        
                        if (stream_config_.pointcloud_enabled) {
                            publishPointCloud(color_frame_full, depth_frame_full);
                        }
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

void FVDepthCameraNode::publishFrames(const rs2::frame& color_frame, const rs2::frame& depth_frame)
{
    // Use a consistent SYSTEM_TIME clock for all message stamps to avoid mixed time sources
    auto now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
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
        
        // マーカーとHUDを描画
        drawMarker(color_image);
        drawHUD(color_image);
        
        auto color_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_image).toImageMsg();
        color_msg->header.stamp = now;
        color_msg->header.frame_id = tf_config_.color_optical_frame;
        color_pub_->publish(*color_msg);
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
            
            cv::imencode(".jpg", color_image, compressed_msg->data, compression_params);
            
            color_compressed_pub_->publish(std::move(compressed_msg));
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
        if (registered_points_pub_->get_subscription_count() == 0) {
            // Skip generation when nobody listens
            return;
        }
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
    
    // Publish camera info
    if (camera_info_config_.enable_camera_info) {
        if (color_info_pub_) {
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
        
        if (depth_info_pub_) {
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
    // 左上にモード表示
    int mode = current_mode_.load();
    const char* mode_text = (mode == 0) ? "Mode: Off" : (mode == 1) ? "Mode: Cursor" : "Mode: Full";
    // 半透明黒背景
    cv::Rect bg(8, 8, 180, 26);
    cv::Mat roi = frame(bg);
    cv::Mat overlay; roi.copyTo(overlay);
    cv::rectangle(overlay, bg, cv::Scalar(0,0,0), -1);
    cv::addWeighted(overlay, 0.4, roi, 0.6, 0.0, roi);
    // テキスト
    cv::putText(frame, mode_text, cv::Point(16, 26), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 1, cv::LINE_AA);
}

void FVDepthCameraNode::publishPointCloud(const rs2::frame& color_frame, const rs2::frame& depth_frame)
{
    RCLCPP_INFO(this->get_logger(), "🔍 publishPointCloud called");
    
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
    RCLCPP_INFO(this->get_logger(), "📊 Point cloud publisher - subscribers: %zu", sub_count);
    
    // Create point cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    
    // Get color and depth data
    cv::Mat color_image(cv::Size(color_intrinsics_.width, color_intrinsics_.height), 
                       CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat depth_image(cv::Size(depth_intrinsics_.width, depth_intrinsics_.height), 
                       CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    
    RCLCPP_INFO(this->get_logger(), "📐 Images - color: %dx%d, depth: %dx%d", 
        color_image.cols, color_image.rows, depth_image.cols, depth_image.rows);
    
    // Convert to point cloud
    int valid_points = 0;
    int skipped_points = 0;
    for (int y = 0; y < depth_intrinsics_.height; y += 2) {
        for (int x = 0; x < depth_intrinsics_.width; x += 2) {
            float depth = depth_image.at<uint16_t>(y, x) * depth_scale_;
            
            if (depth > 0.1f && depth < 10.0f) {
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
    
    RCLCPP_INFO(this->get_logger(), "☁️ Point cloud - valid: %d, skipped: %d, total: %zu", 
        valid_points, skipped_points, cloud.points.size());
    
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;
    
    // Publish
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    cloud_msg.header.frame_id = tf_config_.color_optical_frame;
    
    RCLCPP_INFO(this->get_logger(), "📤 Publishing point cloud with %zu points", cloud.points.size());
    pointcloud_pub_->publish(cloud_msg);
    RCLCPP_INFO(this->get_logger(), "✅ Point cloud published successfully");
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
            point_marker_.point = cv::Point(x, y);
            point_marker_.start_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
            point_marker_.active = true;
            point_marker_.mode = current_mode_.load();
            point_marker_.x = world_x;
            point_marker_.y = world_y;
            point_marker_.z = world_z;
            
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
        // GPT5様よろしくお願いします - ここも同じパイプライン競合問題
        // 問題: pipe_.wait_for_frames()の競合
        // 修正案: ミューテックスまたはキャッシュフレーム使用
        // Get latest depth frame
        rs2::frameset frames = pipe_.wait_for_frames();
        auto depth_frame = frames.get_depth_frame();
        
        if (!depth_frame) {
            return false;
        }
        
        // Get depth value
        float depth = depth_frame.get_distance(x, y);
        
        if (depth <= 0.0f) {
            return false;
        }
        
        // Convert to 3D coordinates
        float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
        float point[3];
        
        rs2_deproject_pixel_to_point(point, &color_intrinsics_, pixel, depth);
        
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
    if (!point_marker_.active) {
        return;
    }
    
    // 10秒経過したら非アクティブにする
    auto now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
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
        auto device_list = ctx_.query_devices();
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
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("fv_realsense"), "❌ Exception in main: %s", e.what());
        return 1;
    }
} 