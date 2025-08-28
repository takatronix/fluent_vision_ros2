/**
 * @file fv_aspara_analyzer_node.cpp
 * @brief アスパラガス解析ノードのメイン実装ファイル
 * @details 3D点群データと2D検出結果を統合してアスパラガスの品質評価を行う
 * @author Takashi Otsuka
 * @date 2024
 * @version 1.0
 */

#include "fv_aspara_analyzer/fv_aspara_analyzer_node.hpp"
#include "fv_aspara_analyzer/aspara_analyzer_thread.hpp"
#include "fluent_lib/fluent.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <sstream>
#include <cstdio>

namespace fv_aspara_analyzer
{
// Shorten Fluent namespaces locally for readability
namespace fi = fluent_image;           // images
namespace flr = fluent_lib::ros;       // ROS helpers
namespace fu = fluent::utils;          // utils (FPS, Stopwatch)
// ====== 局所描画ユーティリティ ======
void FvAsparaAnalyzerNode::applySegOverlay(cv::Mat &output_image)
{
    if (!this->get_parameter("mask_overlay_enabled").as_bool()) return;
    try {
        cv::Mat mask = static_cast<cv::Mat&>(seg_mask_image_);
        if (mask.empty() || mask.size() != output_image.size()) return;
        cv::Mat bin;
        if (mask.type() != CV_8U) mask.convertTo(bin, CV_8U); else bin = mask;
        cv::threshold(bin, bin, 127, 255, cv::THRESH_BINARY);
        double alpha = std::clamp(mask_overlay_alpha_, 0.0, 1.0);
        cv::Mat tint(output_image.size(), output_image.type(), cv::Scalar(0, 255, 0));
        cv::Mat blended; cv::addWeighted(output_image, 1.0 - alpha, tint, alpha, 0.0, blended);
        blended.copyTo(output_image, bin);
    } catch (...) {}
}

/**
 * @brief コンストラクタ
 * @details ノードの初期化、パラメータ読み込み、トピックの設定を行う
 * 
 * 初期化内容：
 * - ROS2パラメータの宣言と取得
 * - サブスクライバー・パブリッシャーの作成
 * - TF2（座標変換）の初期化
 * - ログ出力の設定
 */
FvAsparaAnalyzerNode::FvAsparaAnalyzerNode() : Node("fv_aspara_analyzer")
{
    RCLCPP_WARN(this->get_logger(), "===== FV Aspara Analyzer Node Constructor START =====");
    
    // ===== パラメータ宣言 =====
    this->declare_parameter<double>("min_confidence", 0.5);                    // 最小信頼度閾値
    this->declare_parameter<int>("min_bbox_width", 20);                        // 最小矩形幅[px]
    this->declare_parameter<int>("min_bbox_height", 40);                       // 最小矩形高さ[px]
    this->declare_parameter<double>("pointcloud_distance_min", 0.1);           // 点群処理最小距離（10cm）
    this->declare_parameter<double>("pointcloud_distance_max", 2.0);           // 点群処理最大距離（2m）
    this->declare_parameter<double>("aspara_filter_distance", 0.05);           // アスパラガスフィルタ距離（5cm）
    this->declare_parameter<int>("noise_reduction_neighbors", 50);             // 統計フィルタの近傍点数
    this->declare_parameter<double>("noise_reduction_std_dev", 1.0);           // 統計フィルタの標準偏差
    this->declare_parameter<double>("voxel_leaf_size", 0.005);                 // ボクセルサイズ（5mm）
    this->declare_parameter<double>("harvest_min_length", 0.23);               // 収穫最小長さ（23cm）
    this->declare_parameter<double>("harvest_max_length", 0.50);               // 収穫最大長さ（50cm）
    this->declare_parameter<double>("straightness_threshold", 0.7);            // 真っ直ぐ度閾値

    // ===== 新パラメータスキーマ（ロジック別） =====
    // 長さ
    this->declare_parameter<std::string>("length.method", "auto");
    this->declare_parameter<double>("length.scale", 1.0);
    this->declare_parameter<double>("length.offset_m", 0.0);
    this->declare_parameter<bool>("length.clip.enable", true);
    this->declare_parameter<double>("length.clip.min_m", 0.05);
    this->declare_parameter<double>("length.clip.max_m", 0.50);
    this->declare_parameter<double>("length.pca.trim_low", 0.05);
    this->declare_parameter<double>("length.pca.trim_high", 0.95);

    // 曲率/直線度
    this->declare_parameter<std::string>("curvature.method", "hybrid_max");
    this->declare_parameter<double>("curvature.hybrid.weight_skeleton", 0.6);
    this->declare_parameter<double>("curvature.hybrid.weight_pca", 0.4);
    this->declare_parameter<double>("curvature.straightness.kappa_ref", 0.03);

    // 骨格（共通）
    this->declare_parameter<std::string>("skeleton.method", "iterative_local");
    this->declare_parameter<int>("skeleton.points_count", 9);
    this->declare_parameter<double>("skeleton.step_m", 0.02);
    this->declare_parameter<double>("skeleton.radius_m", 0.018);
    this->declare_parameter<double>("skeleton.window_m", 0.06);

    // UI
    this->declare_parameter<int>("ui.filtered_points_max", 0);
    this->declare_parameter<bool>("info_show_confidence", true);
    // Reference visuals
    this->declare_parameter<bool>("show_reference_line", true);   // 直線（弦）表示ON/OFF（既定ON）
    this->declare_parameter<bool>("show_reference_curve", false); // 参照曲線（骨格）表示ON/OFF（既定OFF）

    // 表示レンジ（未宣言だと has_parameter が false になり YAML が無視されるため明示宣言）
    this->declare_parameter<double>("display_length_min_m", 0.0);
    this->declare_parameter<double>("display_length_max_m", 1.0);
    this->declare_parameter<double>("display_diameter_min_m", 0.0);
    this->declare_parameter<double>("display_diameter_max_m", 1.0);
    this->declare_parameter<bool>("enable_pointcloud_processing", true);       // ポイントクラウド処理有効化
    this->declare_parameter<double>("depth_unit_m_16u", 0.001);               // 16UC1深度の単位(m/units)

    // ===== パラメータ取得 =====
    min_confidence_ = this->get_parameter("min_confidence").as_double();
    pointcloud_distance_min_ = this->get_parameter("pointcloud_distance_min").as_double();
    pointcloud_distance_max_ = this->get_parameter("pointcloud_distance_max").as_double();
    aspara_filter_distance_ = this->get_parameter("aspara_filter_distance").as_double();
    noise_reduction_neighbors_ = this->get_parameter("noise_reduction_neighbors").as_int();
    noise_reduction_std_dev_ = this->get_parameter("noise_reduction_std_dev").as_double();
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
    harvest_min_length_ = this->get_parameter("harvest_min_length").as_double();
    harvest_max_length_ = this->get_parameter("harvest_max_length").as_double();
    straightness_threshold_ = this->get_parameter("straightness_threshold").as_double();
    enable_pointcloud_processing_ = this->get_parameter("enable_pointcloud_processing").as_bool();
    depth_unit_m_16u_ = this->get_parameter("depth_unit_m_16u").as_double();

    // ===== トピック名パラメータ宣言 =====
    // デフォルト値は空の文字列にして、設定ファイルから読み込む
    this->declare_parameter<std::string>("detection_topic", "");
    this->declare_parameter<std::string>("pointcloud_topic", "");
    this->declare_parameter<std::string>("camera_info_topic", "");
    this->declare_parameter<std::string>("depth_camera_info_topic", "");
    this->declare_parameter<bool>("use_color_camera_info", false);  // アライン時はtrue
    this->declare_parameter<std::string>("mask_topic", "");
    this->declare_parameter<std::string>("camera_topic", "");
    this->declare_parameter<std::string>("depth_topic", "");
    this->declare_parameter<std::string>("output_filtered_pointcloud_topic", "");
    this->declare_parameter<std::string>("output_selected_pointcloud_topic", "");
    this->declare_parameter<std::string>("output_detected_all_pointcloud_topic", "");
    this->declare_parameter<bool>("enable_detected_all_points", false);
    this->declare_parameter<std::string>("output_annotated_image_topic", "");
    this->declare_parameter<bool>("debug_overlay", true);
    this->declare_parameter<bool>("mask_overlay_enabled", false);
    // 領域認識（色/セグ）ON/OFF
    this->declare_parameter<bool>("enable_region_recognition", false);
    // 左側プレビュー（Depth/PointCloud）表示ON/OFF
    this->declare_parameter<bool>("preview_panel_enabled", false); // 互換用（未使用）
    this->declare_parameter<bool>("left_panel_enabled", false);
    this->declare_parameter<bool>("right_panel_enabled", true);
    // ROI下部帯のDepthスキャン画像を表示
    this->declare_parameter<bool>("depth_scan_preview_enabled", true);
    // 曲がり度メソッドと重み
    this->declare_parameter<std::string>("curvature_method", "hybrid_max");
    this->declare_parameter<double>("curvature_weight_skeleton", 0.6);
    this->declare_parameter<double>("curvature_weight_pca", 0.4);
    // 長さ推定メソッド（auto: 骨格とPCAの大きい方 / skeleton / pca）
    this->declare_parameter<std::string>("length_method", "auto");
    // 参照用の直線（根本→先端を直線で結ぶ）の表示有無（既定ON）
    this->declare_parameter<bool>("show_pca_reference_line", true);
    // 根本判定を画像Yで行う（画面下=根本）か 3D Yで行うか（既定: 画像Y）
    this->declare_parameter<bool>("root_by_image_y", true);
    // 骨格抽出（宣言しないとスレッド側の has_parameter が false になり直線PCAにフォールバックする）
    this->declare_parameter<bool>("enable_pca_skeleton", true);
    this->declare_parameter<bool>("skeleton_enabled", true);
    this->declare_parameter<std::string>("skeleton_method", "iterative_local"); // pca_line / iterative_local / slice_centroid
    this->declare_parameter<int>("skeleton_points_count", 9);
    this->declare_parameter<double>("skeleton_step_m", 0.01);
    this->declare_parameter<double>("skeleton_radius_m", 0.015);
    this->declare_parameter<double>("skeleton_window_m", 0.06);
    this->declare_parameter<double>("skeleton_trim_low", 0.10);
    this->declare_parameter<double>("skeleton_trim_high", 0.90);
    // マスク描画のアルファ
    this->declare_parameter<double>("mask_overlay_alpha", 0.25);
    this->declare_parameter<double>("hud_alpha", 0.45);
    this->declare_parameter<double>("hud_font_scale", 0.45);
    this->declare_parameter<double>("foreground_depth_margin", 0.04); // 右点群の前景マージン[m]
    this->declare_parameter<bool>("disable_filtered_fallback", false); // フィルタ空時のフォールバック無効化
    // シンプルZ背面カット
    this->declare_parameter<bool>("simple_z_back_cut", true);
    this->declare_parameter<double>("z_back_cut_m", 0.15);
    this->declare_parameter<double>("detection_timeout_seconds", 3.0);  // デフォルト3秒
    this->declare_parameter<std::string>("camera_name", "Camera");  // カメラ名
    // ルート描画Y固定のオフセット比率（矩形下端からの%）
    this->declare_parameter<double>("root_y_offset_ratio", 0.05);

    // ===== トピック名取得 =====
    std::string detection_topic = this->get_parameter("detection_topic").as_string();
    std::string pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
    std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
    std::string depth_camera_info_topic = this->get_parameter("depth_camera_info_topic").as_string();
    bool use_color_camera_info = this->get_parameter("use_color_camera_info").as_bool();
    use_color_camera_info_flag_ = use_color_camera_info;
    std::string mask_topic = this->get_parameter("mask_topic").as_string();
    std::string camera_topic = this->get_parameter("camera_topic").as_string();
    std::string depth_topic = this->get_parameter("depth_topic").as_string();
    std::string output_filtered_pointcloud_topic = this->get_parameter("output_filtered_pointcloud_topic").as_string();
    std::string output_selected_pointcloud_topic = this->get_parameter("output_selected_pointcloud_topic").as_string();
    std::string output_annotated_image_topic = this->get_parameter("output_annotated_image_topic").as_string();
    std::string output_detected_all_pointcloud_topic = this->get_parameter("output_detected_all_pointcloud_topic").as_string();
    enable_detected_all_points_ = this->get_parameter("enable_detected_all_points").as_bool();
    debug_overlay_ = this->get_parameter("debug_overlay").as_bool();
    // 使用箇所では毎回 get_parameter で参照するため、一時変数は作らない
    // 曲がり度・マスク描画パラメータ
    curvature_method_ = this->get_parameter("curvature_method").as_string();
    curvature_weight_skeleton_ = this->get_parameter("curvature_weight_skeleton").as_double();
    curvature_weight_pca_ = this->get_parameter("curvature_weight_pca").as_double();
    mask_overlay_alpha_ = this->get_parameter("mask_overlay_alpha").as_double();
    // HUD
    hud_alpha_ = this->get_parameter("hud_alpha").as_double();
    hud_font_scale_ = this->get_parameter("hud_font_scale").as_double();

    // ===== 必須パラメータのバリデーション =====
    bool config_error = false;
    std::vector<std::string> missing_topics;

    if (detection_topic.empty()) {
        missing_topics.push_back("detection_topic");
        config_error = true;
    }
    if (pointcloud_topic.empty()) {
        missing_topics.push_back("pointcloud_topic");
        config_error = true;
    }
    if (camera_info_topic.empty()) {
        missing_topics.push_back("camera_info_topic");
        config_error = true;
    }
    if (depth_camera_info_topic.empty()) {
        missing_topics.push_back("depth_camera_info_topic");
        config_error = true;
    }
    if (camera_topic.empty()) {
        missing_topics.push_back("camera_topic");
        config_error = true;
    }
    if (depth_topic.empty()) {
        missing_topics.push_back("depth_topic");
        config_error = true;
    }
    if (output_annotated_image_topic.empty()) {
        missing_topics.push_back("output_annotated_image_topic");
        config_error = true;
    }

    if (config_error) {
        RCLCPP_FATAL(this->get_logger(), "");
        RCLCPP_FATAL(this->get_logger(), "========================================");
        RCLCPP_FATAL(this->get_logger(), "       設定ファイルエラー");
        RCLCPP_FATAL(this->get_logger(), "========================================");
        RCLCPP_FATAL(this->get_logger(), "");
        RCLCPP_FATAL(this->get_logger(), "必須パラメータが設定されていません:");
        for (const auto& topic : missing_topics) {
            RCLCPP_FATAL(this->get_logger(), "  - %s", topic.c_str());
        }
        RCLCPP_FATAL(this->get_logger(), "");
        RCLCPP_FATAL(this->get_logger(), "解決方法:");
        RCLCPP_FATAL(this->get_logger(), "  1. YAMLファイルでパラメータを設定してください");
        RCLCPP_FATAL(this->get_logger(), "  2. ノード起動時に --params-file オプションでYAMLファイルを指定してください");
        RCLCPP_FATAL(this->get_logger(), "");
        RCLCPP_FATAL(this->get_logger(), "起動例:");
        RCLCPP_FATAL(this->get_logger(), "  ros2 run fv_aspara_analyzer fv_aspara_analyzer_node \\");
        RCLCPP_FATAL(this->get_logger(), "    --ros-args --params-file /path/to/config.yaml");
        RCLCPP_FATAL(this->get_logger(), "");
        RCLCPP_FATAL(this->get_logger(), "参考設定ファイル:");
        RCLCPP_FATAL(this->get_logger(), "  - launch/fv_aspara_analyzer_d415.yaml");
        RCLCPP_FATAL(this->get_logger(), "  - launch/fv_aspara_analyzer_d405.yaml");
        RCLCPP_FATAL(this->get_logger(), "");
        RCLCPP_FATAL(this->get_logger(), "========================================");
        RCLCPP_FATAL(this->get_logger(), "       ノードを終了します");
        RCLCPP_FATAL(this->get_logger(), "========================================");
        
        // ノードを安全に終了
        rclcpp::shutdown();
        throw std::runtime_error("必須パラメータが設定されていません。設定ファイルを指定してください。");
    }

    // ===== トピック設定ログ出力 =====
    RCLCPP_WARN(this->get_logger(), "Topic configuration:");
    RCLCPP_WARN(this->get_logger(), "  Detection: %s", detection_topic.c_str());
    RCLCPP_WARN(this->get_logger(), "  Pointcloud: %s", pointcloud_topic.c_str());
    RCLCPP_WARN(this->get_logger(), "  Camera info (color): %s", camera_info_topic.c_str());
    RCLCPP_WARN(this->get_logger(), "  Camera info (depth): %s", depth_camera_info_topic.c_str());
    RCLCPP_WARN(this->get_logger(), "  use_color_camera_info: %s", use_color_camera_info ? "true" : "false");
    RCLCPP_WARN(this->get_logger(), "  Camera: %s", camera_topic.c_str());
    RCLCPP_WARN(this->get_logger(), "  Depth: %s", depth_topic.c_str());
    RCLCPP_WARN(this->get_logger(), "  Output annotated: %s", output_annotated_image_topic.c_str());
    if (enable_detected_all_points_) {
        RCLCPP_WARN(this->get_logger(), "  Output detected_all_points: %s", output_detected_all_pointcloud_topic.c_str());
    }

    // ===== サブスクライバー初期化 =====
    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        detection_topic, rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort),
        std::bind(&FvAsparaAnalyzerNode::detectionCallback, this, std::placeholders::_1));

    // 登録済みカラー点群（organized）を購読
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic, rclcpp::QoS(5).reliability(rclcpp::ReliabilityPolicy::BestEffort),
        std::bind(&FvAsparaAnalyzerNode::pointcloudCallback, this, std::placeholders::_1));

    // アライン時はカラーの内参を使用、それ以外は深度内参
    if (use_color_camera_info) {
        getCameraInfoOnce(camera_info_topic);
    } else {
        getCameraInfoOnce(depth_camera_info_topic);
    }

    mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        mask_topic, rclcpp::SensorDataQoS(),
        std::bind(&FvAsparaAnalyzerNode::maskCallback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic, rclcpp::SensorDataQoS(),
        std::bind(&FvAsparaAnalyzerNode::imageCallback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic, rclcpp::SensorDataQoS(),
        std::bind(&FvAsparaAnalyzerNode::depthCallback, this, std::placeholders::_1));
    
    // マウスクリックサブスクライバー（RQTからのカーソル位置設定用）
    mouse_click_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "mouse_click", 10,
        std::bind(&FvAsparaAnalyzerNode::mouseClickCallback, this, std::placeholders::_1));

    // ===== パブリッシャー初期化 =====
    filtered_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_filtered_pointcloud_topic, 10);
    
    // 選択中のアスパラガスの点群パブリッシャー（最終結果）
    selected_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_selected_pointcloud_topic, rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort));
    if (enable_detected_all_points_) {
        detected_all_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_detected_all_pointcloud_topic, rclcpp::QoS(5).reliability(rclcpp::ReliabilityPolicy::BestEffort));
    }
    
    // QoS設定（画像はSensorDataQoS=BestEffort既定。一般的なビューアと相性が良い）
    auto qos = rclcpp::SensorDataQoS();
    
    annotated_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        output_annotated_image_topic, qos);
    
    // 圧縮画像パブリッシャー（同じQoS設定）
    annotated_image_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        output_annotated_image_topic + "/compressed", qos);

    // ===== サービス初期化 =====
    next_asparagus_service_ = this->create_service<std_srvs::srv::Trigger>(
        "next_asparagus",
        std::bind(&FvAsparaAnalyzerNode::nextAsparaguService, this, std::placeholders::_1, std::placeholders::_2));
    
    prev_asparagus_service_ = this->create_service<std_srvs::srv::Trigger>(
        "prev_asparagus",
        std::bind(&FvAsparaAnalyzerNode::prevAsparaguService, this, std::placeholders::_1, std::placeholders::_2));
    
    // サービスクライアントは廃止（画像トピックからのローカル変換に統一）

    // ===== TF2（座標変換）初期化 =====
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
    // ===== データ初期化 =====
    selected_aspara_id_ = -1;  // 未選択状態
    selected_pointcloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    
    // selected_aspara_id_の初期化
    selected_aspara_id_ = -1;
    
    // カーソル初期化
    cursor_position_ = cv::Point(-1, -1);  // 非表示
    smooth_cursor_position_ = cv::Point(-1, -1);
    cursor_visible_ = false;
    cursor_auto_hide_ms_ = 5000;  // 5秒デフォルト
    last_detection_time_ = std::chrono::steady_clock::now();

    // ステータスタイムスタンプ初期化
    auto now_st = std::chrono::steady_clock::now();
    last_detection_msg_time_ = now_st;
    last_depth_msg_time_ = now_st;
    last_color_msg_time_ = now_st;

    // ===== フォント初期化 =====
    RCLCPP_INFO(this->get_logger(), "Japanese font support is now built into the new Fluent API");
    
    // FPS測定用メーターを初期化
    color_fps_meter_ = std::make_unique<fluent::utils::FPSMeter>(10);
    depth_fps_meter_ = std::make_unique<fluent::utils::FPSMeter>(10);
    detection_fps_meter_ = std::make_unique<fluent::utils::FPSMeter>(10);
    segmentation_fps_meter_ = std::make_unique<fluent::utils::FPSMeter>(10);
    pointcloud_fps_meter_ = std::make_unique<fluent::utils::FPSMeter>(10);
    
    // 検出処理時間計測用（StopWatchはデフォルトコンストラクタで初期化済み）
    // detection_stopwatch_は自動初期化されるのでここでの明示的な初期化は不要
    
    // ===== 非同期点群処理初期化 =====
    analyzer_thread_ = std::make_unique<AnalyzerThread>(this);
    RCLCPP_INFO(this->get_logger(), "Aspara analyzer thread initialized");
    
    // 短縮APIによるサブスク生成はコンストラクタ内のshared_from_this()起因の不具合を避けるため無効化
    // 既に上で標準create_subscriptionにより購読を作成済み

    // ===== 高頻度出力タイマー（15FPS for smooth animation）=====
    auto timer_callback = [this]() {
        if (latest_color_image_) {
            publishCurrentImage();
        }
    };
    animation_timer_ = flr::make_timer(this, std::chrono::milliseconds(33), timer_callback);
    RCLCPP_WARN(this->get_logger(), "Animation timer created (30 FPS smooth animation)");
    
    // ===== 初期化完了ログ =====
    RCLCPP_WARN(this->get_logger(), "All subscribers created successfully");
    RCLCPP_WARN(this->get_logger(), "All publishers created successfully");
    RCLCPP_WARN(this->get_logger(), "===== FV Aspara Analyzer Node Constructor END =====");
    RCLCPP_INFO(this->get_logger(), "FV Aspara Analyzer Node initialized");
}

/**
 * @brief デストラクタ
 * @details リソースの適切な解放を行う
 */
FvAsparaAnalyzerNode::~FvAsparaAnalyzerNode() {}

/**
 * @brief 2D検出結果のコールバック関数
 * @param msg 検出結果の配列メッセージ
 * @details YOLO等の物体検出結果を受信し、アスパラガス情報を更新
 * 
 * 処理内容：
 * - 信頼度フィルタリング
 * - バウンディングボックス抽出
 * - アスパラガス情報の作成
 * - 最高信頼度のアスパラガスを選択して処理
 */
void FvAsparaAnalyzerNode::detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    // ノード生存監視（検出ノード）
    detection_node_seen_ = true;
    last_detection_msg_time_ = std::chrono::steady_clock::now();

    // 全体処理時間計測開始
    auto callback_start = std::chrono::high_resolution_clock::now();
    detection_stopwatch_.reset();
    
    // FPS計測
    if (detection_fps_meter_) {
        detection_fps_meter_->tick(this->now());
    }
    
    // 深度/CameraInfoが未到着でも2D検出の処理は継続する（描画のため）
    if (!latest_depth_image_ || !latest_camera_info_) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Required data not available yet. Waiting for depth and camera info...");
    }

    // 新しい検出結果を準備（アスパラ本体のみ、クラスID=0）
    std::vector<std::pair<cv::Rect, float>> new_detections;
    
    for (const auto& detection : msg->detections) {
        if (detection.results.empty()) continue;
        
        // class_idをintに変換（文字列の場合があるため）
        int class_id = 0;
        try {
            if (!detection.results[0].hypothesis.class_id.empty()) {
                class_id = std::stoi(detection.results[0].hypothesis.class_id);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to parse class_id: %s", e.what());
            continue;
        }
        float confidence = detection.results[0].hypothesis.score;
        
        // 信頼度フィルタリング
        if (confidence < min_confidence_) continue;
        
        // アスパラ本体（クラスID=0）のみを処理
        if (class_id == 0) {
            cv::Rect bbox(
                static_cast<int>(detection.bbox.center.position.x - detection.bbox.size_x / 2),
                static_cast<int>(detection.bbox.center.position.y - detection.bbox.size_y / 2),
                static_cast<int>(detection.bbox.size_x),
                static_cast<int>(detection.bbox.size_y)
            );
            // 最小矩形フィルタリング
            int min_w = this->get_parameter("min_bbox_width").as_int();
            int min_h = this->get_parameter("min_bbox_height").as_int();
            if (bbox.width < min_w || bbox.height < min_h) continue;
            
            new_detections.push_back(std::make_pair(bbox, confidence));
        }
    }
    
    // 検出データ処理時間
    auto parse_end = std::chrono::high_resolution_clock::now();
    double parse_ms = std::chrono::duration<double, std::milli>(parse_end - callback_start).count();

    // AsparaSelectionでID管理とスムーズアニメーションを実行
    auto update_start = std::chrono::high_resolution_clock::now();
    {
        auto t_wait_start = std::chrono::steady_clock::now();
        RCLCPP_DEBUG(this->get_logger(), "[LOCK WAIT] detection/updateAsparaList");
        std::lock_guard<std::mutex> lock(aspara_list_mutex_);
        auto t_acq = std::chrono::steady_clock::now();
        double wait_ms = std::chrono::duration<double, std::milli>(t_acq - t_wait_start).count();
        size_t before_sz = aspara_list_.size();
        RCLCPP_DEBUG(this->get_logger(), "[LOCK ACQ] detection/updateAsparaList waited=%.3fms size_before=%zu", wait_ms, before_sz);
        aspara_list_ = aspara_selection_.updateAsparaList(new_detections, aspara_list_);
        auto t_rel = std::chrono::steady_clock::now();
        double hold_ms = std::chrono::duration<double, std::milli>(t_rel - t_acq).count();
        RCLCPP_DEBUG(this->get_logger(), "[LOCK HOLD] detection/updateAsparaList hold=%.3fms size_after=%zu", hold_ms, aspara_list_.size());
    }
    auto update_end = std::chrono::high_resolution_clock::now();
    double update_ms = std::chrono::duration<double, std::milli>(update_end - update_start).count();

    // 最高信頼度のアスパラガスを処理
    auto process_start = std::chrono::high_resolution_clock::now();
    {
        auto t_wait_start = std::chrono::steady_clock::now();
        RCLCPP_DEBUG(this->get_logger(), "[LOCK WAIT] detection/processSelected");
        std::lock_guard<std::mutex> lock(aspara_list_mutex_);
        auto t_acq = std::chrono::steady_clock::now();
        double wait_ms = std::chrono::duration<double, std::milli>(t_acq - t_wait_start).count();
        RCLCPP_DEBUG(this->get_logger(), "[LOCK ACQ] detection/processSelected waited=%.3fms size=%zu", wait_ms, aspara_list_.size());
        if (!aspara_list_.empty()) {
        // 距離優先、同じなら信頼度でソート
        std::sort(aspara_list_.begin(), aspara_list_.end(),
                  [](const AsparaInfo& a, const AsparaInfo& b) {
                      // 矩形面積で距離を推定（大きいほど近い）
                      int area_a = a.bounding_box_2d.width * a.bounding_box_2d.height;
                      int area_b = b.bounding_box_2d.width * b.bounding_box_2d.height;
                      
                      // 距離優先（面積が大きい = 距離が近い）
                      if (area_a != area_b) {
                          return area_a > area_b;  // 面積大きい方を優先
                      }
                      
                      // 距離が同じなら信頼度で比較
                      return a.confidence > b.confidence;
                  });
        
        // AsparaSelectionで最適な候補を選択／維持
        int current_selected = aspara_selection_.getSelectedAsparaId();
        if (current_selected == -1) {
            // 初回選択
            current_selected = aspara_selection_.selectBestCandidate(aspara_list_);
            aspara_selection_.setSelectedAsparaId(current_selected);
        } else {
            // 現在の選択IDがリストに存在しない場合は再選択
            bool found = false;
            for (const auto& a : aspara_list_) {
                if (a.id == current_selected) { found = true; break; }
            }
            if (!found) {
                current_selected = aspara_selection_.selectBestCandidate(aspara_list_);
                aspara_selection_.setSelectedAsparaId(current_selected);
            }
        }
        selected_aspara_id_ = current_selected;

        // 選択対象を処理（存在する場合）
        if (selected_aspara_id_ != -1) {
            for (auto& aspara : aspara_list_) {
                if (aspara.id == selected_aspara_id_) {
                    associateAsparagusParts(msg, aspara);
                    if (analyzer_thread_) {
                        analyzer_thread_->enqueueAnalysis(aspara);
                    }
                    break;
                }
            }
        }
        // if (!aspara_list_.empty()) の終了
        auto t_rel = std::chrono::steady_clock::now();
        double hold_ms = std::chrono::duration<double, std::milli>(t_rel - t_acq).count();
        RCLCPP_DEBUG(this->get_logger(), "[LOCK HOLD] detection/processSelected hold=%.3fms", hold_ms);
    }  // if (!aspara_list_.empty())の終了
    }  // ロックスコープの終了
    
    auto process_end = std::chrono::high_resolution_clock::now();
    double process_ms = std::chrono::duration<double, std::milli>(process_end - process_start).count();
    
    // 全検出ROIの生点群をまとめて発行（オプション）
    if (enable_detected_all_points_ && detected_all_pointcloud_pub_) {
        sensor_msgs::msg::Image::SharedPtr depth_image;
        sensor_msgs::msg::Image::SharedPtr color_image;
        sensor_msgs::msg::CameraInfo::SharedPtr camera_info;
        {
            std::lock_guard<std::mutex> lk(image_data_mutex_);
            depth_image = latest_depth_image_;
            color_image = latest_color_image_;
            camera_info = latest_camera_info_;
        }
        if (depth_image && color_image && camera_info && !aspara_list_.empty()) {
            try {
                // OpenCVへ変換
                cv::Mat depth_mat;
                if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                    depth_mat = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1)->image;
                } else if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                    depth_mat = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1)->image;
                }
                cv::Mat color_mat = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8)->image;
                if (!depth_mat.empty() && depth_mat.size() == color_mat.size()) {
                    // 深度スケール（D405のみ0.1mm/Unitへの自動上書きを許可）
                    double dmin, dmax; cv::minMaxLoc(depth_mat, &dmin, &dmax);
                    float depth_scale_m = 0.001f; // D415既定は1mm/Unit
                    if (depth_mat.type() != CV_16UC1) {
                        depth_scale_m = 1.0f; // 32FC1は[m]
                    } else {
                        bool allow_override = false;
                        try {
                            if (this->has_parameter("camera_name")) {
                                auto cam = this->get_parameter("camera_name").as_string();
                                if (cam == std::string("D405")) allow_override = true;
                            }
                            if (!allow_override && this->has_parameter("depth_unit_auto_override")) {
                                allow_override = this->get_parameter("depth_unit_auto_override").as_bool();
                            }
                        } catch (...) {}
                        if (allow_override && dmax < 200) {
                            depth_scale_m = 0.0001f;
                        }
                    }
                    // 結合点群
                    pcl::PointCloud<pcl::PointXYZRGB> all_cloud;
                    std::vector<AsparaInfo> asparas_copy;
                    {
                        std::lock_guard<std::mutex> lk2(aspara_list_mutex_);
                        asparas_copy = aspara_list_;
                    }
                    for (const auto& a : asparas_copy) {
                        cv::Rect img_rect(0, 0, depth_mat.cols, depth_mat.rows);
                        cv::Rect roi = a.bounding_box_2d & img_rect;
                        if (roi.area() <= 0) continue;
                        auto roi_cloud = fluent_cloud::io::DepthToCloud::convertAsparagusROI(
                            depth_mat, color_mat, roi, *camera_info, depth_scale_m);
                        if (roi_cloud && !roi_cloud->points.empty()) {
                            all_cloud.points.insert(all_cloud.points.end(), roi_cloud->points.begin(), roi_cloud->points.end());
                        }
                    }
                    if (!all_cloud.points.empty()) {
                        all_cloud.width = all_cloud.points.size(); all_cloud.height = 1; all_cloud.is_dense = false;
                        sensor_msgs::msg::PointCloud2 msg_pc2;
                        pcl::toROSMsg(all_cloud, msg_pc2);
                        msg_pc2.header.stamp = depth_image->header.stamp;
                        msg_pc2.header.frame_id = depth_image->header.frame_id;
                        detected_all_pointcloud_pub_->publish(msg_pc2);
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "detected_all_points publish error: %s", e.what());
            }
        }
    }
    
    // 全体処理時間
    auto callback_end = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(callback_end - callback_start).count();
    
    // 分析時間を保存
    last_analysis_time_ms_ = total_ms;
    
    // 詳細ログ出力（1秒に1回に制限）
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[DETECTION] Total:%.2fms (Parse:%.2fms, Update:%.2fms, Process:%.2fms) Detections:%zu",
        total_ms, parse_ms, update_ms, process_ms, msg->detections.size());
}

/**
 * @brief アスパラガス本体と穂の関連付け処理
 * @param detections 全検出結果
 * @param aspara_info アスパラガス情報（更新対象）
 * @details 本体（クラスID 0）と穂（クラスID 1）を空間的関連性で関連付け
 */
void FvAsparaAnalyzerNode::associateAsparagusParts(
    const vision_msgs::msg::Detection2DArray::SharedPtr& detections,
    AsparaInfo& aspara_info)
{
    // 本体部分の検出（クラスID 0）
    for (const auto& detection : detections->detections) {
        if (detection.results.empty()) continue;
        
        // class_idをintに変換（文字列の場合があるため）
        int class_id = 0;
        try {
            if (!detection.results[0].hypothesis.class_id.empty()) {
                class_id = std::stoi(detection.results[0].hypothesis.class_id);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to parse class_id: %s", e.what());
            continue;
        }
        float confidence = detection.results[0].hypothesis.score;
        
        // バウンディングボックスを取得
        cv::Rect bbox(
            static_cast<int>(detection.bbox.center.position.x - detection.bbox.size_x / 2),
            static_cast<int>(detection.bbox.center.position.y - detection.bbox.size_y / 2),
            static_cast<int>(detection.bbox.size_x),
            static_cast<int>(detection.bbox.size_y)
        );
        
        if (class_id == 0) {  // 本体
            // 現在のアスパラと重複度チェック
            float overlap = aspara_selection_.calculateOverlap(bbox, aspara_info.bounding_box_2d);
            if (overlap > 0.5f) {  // 50%以上重複していれば同じアスパラの本体
                aspara_info.body_part.class_id = class_id;
                aspara_info.body_part.bounding_box_2d = bbox;
                aspara_info.body_part.confidence = confidence;
                aspara_info.body_part.is_valid = true;
            }
        }
        else if (class_id == 1) {  // 穂
            // 穂がどの矩形に含まれるかチェック（複数の場合は密集度を考慮）
            std::vector<int> overlapping_aspara_indices;
            
            // 穂の中心点がどのアスパラ矩形内にあるかチェック
            cv::Point2f spike_center(bbox.x + bbox.width / 2.0f, bbox.y + bbox.height / 2.0f);
            
            for (size_t i = 0; i < aspara_list_.size(); ++i) {
                // 穂の中心点がアスパラ矩形内にあるかチェック
                if (aspara_list_[i].bounding_box_2d.contains(cv::Point(spike_center.x, spike_center.y))) {
                    overlapping_aspara_indices.push_back(i);
                }
            }
            
            // 現在のアスパラとの関連性をチェック
            bool is_related_to_current = false;
            for (int idx : overlapping_aspara_indices) {
                if (aspara_list_[idx].id == aspara_info.id) {
                    is_related_to_current = true;
                    break;
                }
            }
            
            if (is_related_to_current && aspara_info.body_part.is_valid && 
                isAssociatedSpike(aspara_info.body_part.bounding_box_2d, bbox)) {
                
                AsparagusPart spike_part;
                spike_part.class_id = class_id;
                spike_part.bounding_box_2d = bbox;
                
                // 密集度による信頼度調整
                if (overlapping_aspara_indices.size() > 1) {
                    // 複数のアスパラと重複している場合は信頼度を下げる
                    float density_penalty = 1.0f / overlapping_aspara_indices.size();
                    spike_part.confidence = confidence * density_penalty;
                    RCLCPP_DEBUG(this->get_logger(), 
                        "Spike overlaps with %zu asparagus, confidence reduced from %.3f to %.3f",
                        overlapping_aspara_indices.size(), confidence, spike_part.confidence);
                } else {
                    spike_part.confidence = confidence;
                }
                
                spike_part.is_valid = true;
                aspara_info.spike_parts.push_back(spike_part);
            }
        }
    }
}

/**
 * @brief アスパラガス本体と穂の空間的関連性を判定
 * @param body_bbox 本体のバウンディングボックス
 * @param spike_bbox 穂のバウンディングボックス
 * @return 関連性ありかどうか
 */
bool FvAsparaAnalyzerNode::isAssociatedSpike(const cv::Rect& body_bbox, const cv::Rect& spike_bbox)
{
    // 穂は本体の上部に位置するべき
    if (spike_bbox.y > body_bbox.y + body_bbox.height * 0.3) {
        return false;  // 穂が本体の下部にある場合は関連なし
    }
    
    // 水平方向の重複チェック
    int left_overlap = std::max(body_bbox.x, spike_bbox.x);
    int right_overlap = std::min(body_bbox.x + body_bbox.width, spike_bbox.x + spike_bbox.width);
    
    if (left_overlap >= right_overlap) {
        return false;  // 水平方向に重複なし
    }
    
    // 重複度を計算
    int overlap_width = right_overlap - left_overlap;
    int min_width = std::min(body_bbox.width, spike_bbox.width);
    
    float horizontal_overlap_ratio = static_cast<float>(overlap_width) / static_cast<float>(min_width);
    
    // 30%以上水平方向に重複していれば関連ありと判定
    return horizontal_overlap_ratio > 0.3f;
}

/**
 * @brief 3D点群データのコールバック関数
 * @param msg 点群データメッセージ
 * @details RealSense等からの3D点群を受信し、解析用データとして保存
 */
// 点群コールバック（registered_points購読）
void FvAsparaAnalyzerNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_WARN_ONCE(this->get_logger(), "Pointcloud callback received first pointcloud");
    latest_pointcloud_ = msg;
    // organized: height > 1
    last_pointcloud_was_organized_.store(msg->height > 1);
}

/**
 * @brief カメラ情報のコールバック関数
 * @param msg カメラキャリブレーション情報
 * @details カメラの内部パラメータを受信し、3D-2D変換に使用
 */


/**
 * @brief マスク画像のコールバック関数
 * @param msg セグメンテーションマスク画像
 * @details アスパラガス領域のマスクを受信し、精度向上に使用
 */
void FvAsparaAnalyzerNode::maskCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 受信時にcv::Matへ変換し、カラー画像サイズに合わせてリサイズ→mono8で保持
    try {
        fluent_image::Image seg_in(*msg);
        cv::Mat seg_mat = static_cast<cv::Mat&>(seg_in);
        // 目標サイズ（最新カラーがあれば合わせる）
        int target_w = 0, target_h = 0;
        {
            std::lock_guard<std::mutex> lk(image_data_mutex_);
            if (latest_color_image_) {
                target_w = static_cast<int>(latest_color_image_->width);
                target_h = static_cast<int>(latest_color_image_->height);
            }
        }
        if (target_w > 0 && target_h > 0 && (seg_mat.cols != target_w || seg_mat.rows != target_h)) {
            cv::Mat resized; cv::resize(seg_mat, resized, cv::Size(target_w, target_h), 0, 0, cv::INTER_NEAREST);
            seg_mat = resized;
        }
        // mono8へ
        if (seg_mat.type() != CV_8UC1) {
            cv::Mat gray;
            if (seg_mat.type() == CV_8UC3) cv::cvtColor(seg_mat, gray, cv::COLOR_BGR2GRAY);
            else if (seg_mat.type() == CV_8UC4) cv::cvtColor(seg_mat, gray, cv::COLOR_BGRA2GRAY);
            else seg_mat.convertTo(gray, CV_8U);
            seg_mask_image_ = fluent_image::make(gray, "mono8");
        } else {
            seg_mask_image_ = fluent_image::make(seg_mat, "mono8");
        }
    } catch (...) {
        // 無視（描画時に存在チェック）
    }
    if (segmentation_fps_meter_) segmentation_fps_meter_->tick(this->now());
}

/**
 * @brief カラー画像のコールバック関数
 * @param msg カラー画像メッセージ
 * @details 可視化用のカラー画像を受信
 */
void FvAsparaAnalyzerNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_WARN_ONCE(this->get_logger(), "Image callback received first image");
    // ノード生存監視（カメラノード）
    camera_node_seen_ = true;
    last_color_msg_time_ = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(image_data_mutex_);  // 分離したmutexを使用
        latest_color_image_ = msg;
    }
    
    // FPS計測
    if (color_fps_meter_) {
        color_fps_meter_->tick(this->now());
    }
    
    // タイマーで30FPS出力するので、ここでは出力しない
    // publishCurrentImage(); // 削除
}

/**
 * @brief 深度画像のコールバック関数
 * @param msg 深度画像メッセージ
 * @details 深度画像を保存して効率的な処理に使用
 */
void FvAsparaAnalyzerNode::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_WARN_ONCE(this->get_logger(), "Depth callback received first depth image");
    // ノード生存監視（深度ノード）
    depth_node_seen_ = true;
    last_depth_msg_time_ = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(image_data_mutex_);  // 分離したmutexを使用
        latest_depth_image_ = msg;
    }
    
    // FPS計測
    if (depth_fps_meter_) {
        depth_fps_meter_->tick(this->now());
    }
}

/**
 * @brief マウスクリックコールバック関数
 * @param msg クリック座標メッセージ（x, y座標）
 * @details RQTからのマウスクリックイベントを受信してカーソル位置を設定
 */
void FvAsparaAnalyzerNode::mouseClickCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    // カーソル位置を設定
    cursor_position_.x = static_cast<int>(msg->x);
    cursor_position_.y = static_cast<int>(msg->y);
    
    // カーソルを表示
    cursor_visible_ = true;
    
    // アニメーション開始時刻を更新
    cursor_animation_start_ = std::chrono::steady_clock::now();
    
    RCLCPP_INFO(this->get_logger(), "Mouse click received: (%d, %d)", 
                cursor_position_.x, cursor_position_.y);
}

/**
 * @brief カーソル位置を設定
 * @param x カーソルのX座標
 * @param y カーソルのY座標
 */
void FvAsparaAnalyzerNode::setCursor(int x, int y)
{
    cursor_position_.x = x;
    cursor_position_.y = y;
    cursor_visible_ = true;
    cursor_animation_start_ = std::chrono::steady_clock::now();
}

/**
 * @brief カーソル位置を取得
 * @param x カーソルのX座標（出力）
 * @param y カーソルのY座標（出力）
 */
void FvAsparaAnalyzerNode::getCursor(int& x, int& y) const
{
    x = cursor_position_.x;
    y = cursor_position_.y;
}

/**
 * @brief カーソル位置のアスパラを選択
 * @return 選択成功したらtrue
 */
bool FvAsparaAnalyzerNode::selectAsparaAtCursor()
{
    if (!cursor_visible_ || cursor_position_.x < 0 || cursor_position_.y < 0) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(aspara_list_mutex_);
    for (const auto& aspara : aspara_list_) {
        if (aspara.bounding_box_2d.contains(cursor_position_)) {
            selected_aspara_id_ = aspara.id;
            aspara_selection_.setSelectedAsparaId(aspara.id);
            return true;
        }
    }
    return false;
}

/**
 * @brief 次のアスパラへカーソル移動
 * @return 移動成功したらtrue
 */
bool FvAsparaAnalyzerNode::moveCursorToNext()
{
    std::lock_guard<std::mutex> lock(aspara_list_mutex_);
    if (aspara_list_.empty()) {
        return false;
    }
    
    // 次のアスパラを選択
    int next_id = aspara_selection_.selectNextAsparagus(aspara_list_);
    
    // 選択されたアスパラの中心にカーソル移動
    for (const auto& aspara : aspara_list_) {
        if (aspara.id == next_id) {
            cursor_position_.x = aspara.bounding_box_2d.x + aspara.bounding_box_2d.width / 2;
            cursor_position_.y = aspara.bounding_box_2d.y + aspara.bounding_box_2d.height / 2;
            cursor_visible_ = true;
            selected_aspara_id_ = next_id;
            return true;
        }
    }
    return false;
}

/**
 * @brief 前のアスパラへカーソル移動
 * @return 移動成功したらtrue
 */
bool FvAsparaAnalyzerNode::moveCursorToPrev()
{
    std::lock_guard<std::mutex> lock(aspara_list_mutex_);
    if (aspara_list_.empty()) {
        return false;
    }
    
    // 前のアスパラを選択
    int prev_id = aspara_selection_.selectPrevAsparagus(aspara_list_);
    
    // 選択されたアスパラの中心にカーソル移動
    for (const auto& aspara : aspara_list_) {
        if (aspara.id == prev_id) {
            cursor_position_.x = aspara.bounding_box_2d.x + aspara.bounding_box_2d.width / 2;
            cursor_position_.y = aspara.bounding_box_2d.y + aspara.bounding_box_2d.height / 2;
            cursor_visible_ = true;
            selected_aspara_id_ = prev_id;
            return true;
        }
    }
    return false;
}
































/**
 * @brief 現在の画像を出力（検出結果がある場合はオーバーレイ付き）
 * @details 常に画像を出力し、検出結果がある場合はオーバーレイを追加
 */
void FvAsparaAnalyzerNode::publishCurrentImage()
{
    static int publish_count = 0;
    static auto last_publish_log = std::chrono::steady_clock::now();
    publish_count++;
    
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_publish_log).count() > 1.0) {
        // RCLCPP_INFO(this->get_logger(), "[PUBLISH] Rate: %d publishes/sec", publish_count);  // ログ削減
        publish_count = 0;
        last_publish_log = now;
    }
    
    fu::Stopwatch total_sw;
    
    if (!latest_color_image_) {
        return;
    }
    
    // カラー画像をBGR8に正規化（FluentImage）
    fu::Stopwatch cvt_sw;
    fi::Image color_fi(*latest_color_image_);
    color_fi = color_fi.to_bgr8();
    cv::Mat color_image = static_cast<cv::Mat&>(color_fi);
    double cvt_ms = cvt_sw.elapsed_ms();
    
    // オフスクリーンキャンバス（永続）に描画（排他制御で一貫性を担保）
    // ダブルバッファ: 描画は draw へ、publishは pub から。最後にswap。
    if (static_cast<cv::Mat&>(canvas_draw_).empty() ||
        static_cast<cv::Mat&>(canvas_draw_).cols != color_image.cols ||
        static_cast<cv::Mat&>(canvas_draw_).rows != color_image.rows) {
        canvas_draw_ = fi::Image(color_image.clone(), "bgr8");
    } else {
        static_cast<cv::Mat&>(canvas_draw_) = color_image.clone();
    }
    fi::Image &canvas = canvas_draw_;
    cv::Mat &output_image = static_cast<cv::Mat&>(canvas);

    // セグメンテーションマスクの薄色オーバーレイ（🟢緑）: 受信時にBGR8サイズへ揃えてあるので即合成
    if (this->get_parameter("mask_overlay_enabled").as_bool()) {
        try {
            cv::Mat mask = static_cast<cv::Mat&>(seg_mask_image_);
            if (!mask.empty() && mask.size() == output_image.size()) {
                cv::Mat bin;
                if (mask.type() != CV_8U) mask.convertTo(bin, CV_8U); else bin = mask;
                cv::threshold(bin, bin, 127, 255, cv::THRESH_BINARY);
                double alpha = std::clamp(mask_overlay_alpha_, 0.0, 1.0);
                cv::Mat tint(output_image.size(), output_image.type(), cv::Scalar(0, 255, 0));
                cv::Mat blended; cv::addWeighted(output_image, 1.0 - alpha, tint, alpha, 0.0, blended);
                blended.copyTo(output_image, bin);
            }
        } catch (...) {}
    }
    
    // FPS計算用（FPSMeterを使用）
    static auto last_time = std::chrono::high_resolution_clock::now();
    static auto last_detection_time = std::chrono::high_resolution_clock::now();
    
    // 現在時刻取得
    auto current_time = std::chrono::high_resolution_clock::now();
    auto delta_time = std::chrono::duration<float>(current_time - last_time).count();
    last_time = current_time;
    
    // アニメーション更新（実際のFPSに基づいて調整）
    // delta_timeを使って、フレームレート非依存のスムージングを実現
    const float target_smoothing_time = 0.2f;  // 200msで目標値に収束（より滑らか）
    const float animation_speed = std::min(1.0f, delta_time / target_smoothing_time);
    
    // 検出結果の描画
    // try_lockを使用してロック競合を回避
    fu::Stopwatch snap_sw;
    std::vector<AsparaInfo> snapshot_list;
    int snapshot_selected_id = -1;
    
    // 静的変数で前回の検出結果を常に保持
    static std::vector<AsparaInfo> persistent_list;
    static int persistent_selected_id = -1;
    static auto last_detection_update = std::chrono::steady_clock::now();
    
    // try_lockで非ブロッキングアクセス
    std::unique_lock<std::mutex> lock(aspara_list_mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
        // ロック取得成功 - 最新データをチェック
        if (!aspara_list_.empty() && latest_camera_info_) {
            // 新しい検出がある場合のみ更新
            // 選択IDの整合性チェック
            if (selected_aspara_id_ == -1 ||
                std::find_if(aspara_list_.begin(), aspara_list_.end(),
                    [this](const AsparaInfo& info) { return info.id == selected_aspara_id_; }) == aspara_list_.end()) {
                selected_aspara_id_ = aspara_list_[0].id;
            }
            // 永続リストを更新
            persistent_list = aspara_list_;
            persistent_selected_id = selected_aspara_id_;
            last_detection_update = std::chrono::steady_clock::now();
        }
        lock.unlock();  // 早めに解放
    }
    
    // 常に永続リストを使用（検出がなくても前の位置を保持）
    snapshot_list = persistent_list;
    snapshot_selected_id = persistent_selected_id;
    
    // カーソル自動管理
    auto cursor_time_now = std::chrono::steady_clock::now();
    if (!snapshot_list.empty()) {
        // アスパラが検出されたらlast_detection_timeを更新
        last_detection_time_ = cursor_time_now;
        
        // カーソルが非表示の場合、自動で表示ON
        if (!cursor_visible_) {
            cursor_visible_ = true;
            // 最初のアスパラの中心にカーソルを設定
            if (cursor_position_.x == -1 && cursor_position_.y == -1) {
                const auto& first_aspara = snapshot_list[0];
                cursor_position_.x = first_aspara.bounding_box_2d.x + first_aspara.bounding_box_2d.width / 2;
                cursor_position_.y = first_aspara.bounding_box_2d.y + first_aspara.bounding_box_2d.height / 2;
                smooth_cursor_position_ = cursor_position_;
            }
        }

        // 検出が1つしかない場合の自動追従（設定でONのとき）
        bool auto_select_single = true;
        if (this->has_parameter("auto_select_single")) {
            auto_select_single = this->get_parameter("auto_select_single").as_bool();
        }
        if (auto_select_single && snapshot_list.size() == 1) {
            const auto& only_aspara = snapshot_list[0];
            cursor_position_.x = only_aspara.bounding_box_2d.x + only_aspara.bounding_box_2d.width / 2;
            cursor_position_.y = only_aspara.bounding_box_2d.y + only_aspara.bounding_box_2d.height / 2;
            cursor_visible_ = true;
        }
    } else {
        // アスパラが検出されない場合、5秒後に自動でカーソルOFF
        auto time_since_detection = std::chrono::duration_cast<std::chrono::milliseconds>(
            cursor_time_now - last_detection_time_).count();
        if (time_since_detection > cursor_auto_hide_ms_ && cursor_visible_) {
            cursor_visible_ = false;
            // カーソル位置はリセットしない（記憶する）
        }
    }
    
    // カーソルがONの場合、カーソル位置のアスパラを選択
    if (cursor_visible_ && cursor_position_.x >= 0 && cursor_position_.y >= 0) {
        // カーソル位置にヒットするアスパラを探す
        int hit_aspara_id = -1;
        for (const auto& aspara : snapshot_list) {
            if (aspara.bounding_box_2d.contains(cursor_position_)) {
                hit_aspara_id = aspara.id;
                break;
            }
        }
        
        // ヒットしたアスパラがあれば選択
        if (hit_aspara_id != -1) {
            snapshot_selected_id = hit_aspara_id;
            persistent_selected_id = hit_aspara_id;
            selected_aspara_id_ = hit_aspara_id;
        }
    }
    
    // タイムアウト処理（設定値後に自動的に消去）
    static double detection_timeout = this->get_parameter("detection_timeout_seconds").as_double();
    auto time_now = std::chrono::steady_clock::now();
    auto time_since_update = std::chrono::duration<double>(time_now - last_detection_update).count();
    if (time_since_update > detection_timeout && !persistent_list.empty()) {
        // 3秒後に検出結果をクリア
        persistent_list.clear();
        persistent_selected_id = -1;
    }
    
    double snap_ms = snap_sw.elapsed_ms();
    
    // 2) スムージング状態を静的変数で保持
    static std::map<int, cv::Rect> smooth_bbox_map;
    static std::map<int, float> animation_alpha_map;
    
    // 古いIDをクリーンアップ
    std::set<int> current_ids;
    for (const auto& info : snapshot_list) {
        current_ids.insert(info.id);
    }
    for (auto it = smooth_bbox_map.begin(); it != smooth_bbox_map.end(); ) {
        if (current_ids.find(it->first) == current_ids.end()) {
            it = smooth_bbox_map.erase(it);
        } else {
            ++it;
        }
    }
    
    // スムージング計算
    for (auto& aspara_info : snapshot_list) {
        if (smooth_bbox_map.find(aspara_info.id) == smooth_bbox_map.end()) {
            // 新規
            smooth_bbox_map[aspara_info.id] = aspara_info.bounding_box_2d;
            animation_alpha_map[aspara_info.id] = 0.0f;
            aspara_info.smooth_bbox = aspara_info.bounding_box_2d;
            aspara_info.animation_alpha = 0.0f;
        } else {
            // 既存 - スムージング（より滑らかに）
            auto lerp = [](float a, float b, float t) { return a + (b - a) * t; };
            cv::Rect& smooth = smooth_bbox_map[aspara_info.id];
            // 選択中は遅延なく追従（ID処理の影響で枠が動かない問題を回避）
            if (aspara_info.id == snapshot_selected_id) {
                smooth = aspara_info.bounding_box_2d;
            } else {
                // 非選択は滑らかに追従
                float smooth_factor = std::min(1.0f, animation_speed * 1.5f);
                smooth.x = static_cast<int>(lerp(static_cast<float>(smooth.x), static_cast<float>(aspara_info.bounding_box_2d.x), smooth_factor));
                smooth.y = static_cast<int>(lerp(static_cast<float>(smooth.y), static_cast<float>(aspara_info.bounding_box_2d.y), smooth_factor));
                smooth.width = static_cast<int>(lerp(static_cast<float>(smooth.width), static_cast<float>(aspara_info.bounding_box_2d.width), smooth_factor));
                smooth.height = static_cast<int>(lerp(static_cast<float>(smooth.height), static_cast<float>(aspara_info.bounding_box_2d.height), smooth_factor));
            }
            aspara_info.smooth_bbox = smooth;
            
            float& alpha = animation_alpha_map[aspara_info.id];
            alpha = std::min(1.0f, alpha + delta_time * 3.0f);
            aspara_info.animation_alpha = alpha;
        }
        aspara_info.frame_count++;
    }

    if (!snapshot_list.empty()) {
        last_detection_time = current_time;
        
        // 各検出されたアスパラガスを描画
        for (auto& aspara_info : snapshot_list) {
            bool is_selected = (aspara_info.id == snapshot_selected_id);
            
            // グレード別色分け
            auto gradeToColor = [](AsparaguGrade g) -> cv::Scalar {
                switch (g) {
                    case AsparaguGrade::A_GRADE: return cv::Scalar(0, 255, 0);      // 緑 (BGR)
                    case AsparaguGrade::B_GRADE: return cv::Scalar(0, 255, 255);    // 黄
                    case AsparaguGrade::C_GRADE: return cv::Scalar(0, 128, 255);    // オレンジ
                    case AsparaguGrade::OUT_OF_SPEC: return cv::Scalar(0, 0, 255);  // 赤
                    case AsparaguGrade::UNKNOWN:
                    default: return cv::Scalar(0, 255, 0);
                }
            };
            cv::Scalar grade_color = gradeToColor(aspara_info.grade);
            // 未選択時は彩度を落とす
            if (!is_selected) {
                grade_color = cv::Scalar(grade_color[0] * 0.5, grade_color[1] * 0.5, grade_color[2] * 0.5);
            }
            int thickness = is_selected ? 2 : 1;
            
            // バウンディングボックス描画（グレード色）
            cv::rectangle(output_image, aspara_info.smooth_bbox, grade_color, thickness);

            // [可視化] ルート推定の帯スキャン位置をROI上に可視化（横線2本＋中央線、赤丸は静止表示）
            if (this->get_parameter("depth_scan_preview_enabled").as_bool()) {
                // 赤玉が左右に流れて見えないよう、帯スキャンはスムーズ矩形ではなく瞬時の矩形を使用
                const cv::Rect &bbox = aspara_info.bounding_box_2d & cv::Rect(0,0,output_image.cols, output_image.rows);
                if (bbox.width > 1 && bbox.height > 1) {
                    // パラメータ（帯の下端/上端比率）
                    double bottom_ratio = this->has_parameter("hist_band_bottom_ratio") ? this->get_parameter("hist_band_bottom_ratio").as_double() : 0.05;
                    double top_ratio    = this->has_parameter("hist_band_top_ratio")    ? this->get_parameter("hist_band_top_ratio").as_double()    : 0.10;
                    int y0 = bbox.y + static_cast<int>(std::floor(bbox.height * (1.0 - top_ratio)));
                    int y1 = bbox.y + static_cast<int>(std::floor(bbox.height * (1.0 - bottom_ratio)));
                    y0 = std::clamp(y0, bbox.y, bbox.y + bbox.height - 1);
                    y1 = std::clamp(y1, bbox.y, bbox.y + bbox.height - 1);
                    if (y1 < y0) std::swap(y0, y1);
                    int ymid = (y0 + y1) / 2;
                    // 横線（シアン）
                    cv::line(output_image, cv::Point(bbox.x, y0), cv::Point(bbox.x + bbox.width - 1, y0), cv::Scalar(255, 255, 0), 1);
                    cv::line(output_image, cv::Point(bbox.x, y1), cv::Point(bbox.x + bbox.width - 1, y1), cv::Scalar(255, 255, 0), 1);
                    cv::line(output_image, cv::Point(bbox.x, ymid), cv::Point(bbox.x + bbox.width - 1, ymid), cv::Scalar(80, 255, 80), 1);
                    // 赤い丸（推定z0の現在位置）: ヒストグラムから受け取った正規化位置を静止表示
                    bool drew = false;
                    if (aspara_info.z0_norm >= 0.0f) {
                        int px = bbox.x + 1 + static_cast<int>(std::clamp(aspara_info.z0_norm, 0.0f, 1.0f) * std::max(1, bbox.width - 2));
                        cv::circle(output_image, cv::Point(px, ymid), 6, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
                        drew = true;
                    }
                    // フォールバック: 3D根本投影x位置が使えるならそこに赤丸
                    if (!drew && latest_camera_info_) {
                        const auto &rp = aspara_info.root_position_3d;
                        if (std::isfinite(rp.z) && rp.z > 0.0) {
                            double fx = latest_camera_info_->k[0];
                            double cx = latest_camera_info_->k[2];
                            int u = static_cast<int>(std::round(fx * (rp.x / rp.z) + cx));
                            int px = std::clamp(u, bbox.x + 1, bbox.x + bbox.width - 2);
                            cv::circle(output_image, cv::Point(px, ymid), 6, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
                        }
                    }
                }
            }

            // ヒストグラム帯（高さ8px）を矩形下に貼り付け（常時表示）
            try {
                if (!aspara_info.depth_histogram_strip.empty()) {
                    const int strip_h = 8;
                    // 帯スキャンと同様に静止表示のため瞬時の矩形で配置
                    const cv::Rect &bbox = aspara_info.bounding_box_2d;
                    if (bbox.width > 1 && bbox.height > 1) {
                        cv::Mat strip_resized;
                        cv::resize(aspara_info.depth_histogram_strip, strip_resized, cv::Size(bbox.width, strip_h), 0, 0, cv::INTER_NEAREST);
                        if (strip_resized.type() != CV_8UC1) {
                            cv::Mat tmp; strip_resized.convertTo(tmp, CV_8UC1); strip_resized = tmp;
                        }
                        cv::Mat strip_bgr; cv::cvtColor(strip_resized, strip_bgr, cv::COLOR_GRAY2BGR);
                        int sx = std::clamp(bbox.x, 0, std::max(0, output_image.cols - 1));
                        int sy = std::clamp(bbox.y + bbox.height + 4, 0, std::max(0, output_image.rows - 1));
                        int sw = std::min(strip_bgr.cols, output_image.cols - sx);
                        int sh = std::min(strip_bgr.rows, output_image.rows - sy);
                        if (sw > 0 && sh > 0) {
                            strip_bgr(cv::Rect(0, 0, sw, sh)).copyTo(output_image(cv::Rect(sx, sy, sw, sh)));
                        }

                        // 追加: 参照しているDepth帯の「実際の画像」もヒストグラムの下に貼り付け
                        // 近いほど明るくなる正規化（16Uは>0マスク）で 16px 高に整形
                        if (this->get_parameter("depth_scan_preview_enabled").as_bool()) {
                            sensor_msgs::msg::Image::SharedPtr depth_copy;
                            sensor_msgs::msg::Image::SharedPtr color_copy;
                            {
                                std::lock_guard<std::mutex> lk(image_data_mutex_);
                                depth_copy = latest_depth_image_;
                                color_copy = latest_color_image_;
                            }
                            if (depth_copy) {
                                try {
                                    cv::Mat depth_mat;
                                    bool depth_is_16u = false;
                                    if (depth_copy->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                                        depth_mat = cv_bridge::toCvCopy(depth_copy, sensor_msgs::image_encodings::TYPE_16UC1)->image;
                                        depth_is_16u = true;
                                    } else if (depth_copy->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                                        depth_mat = cv_bridge::toCvCopy(depth_copy, sensor_msgs::image_encodings::TYPE_32FC1)->image;
                                    }
                                    if (!depth_mat.empty()) {
                                        int cw = color_copy ? static_cast<int>(color_copy->width) : output_image.cols;
                                        int ch = color_copy ? static_cast<int>(color_copy->height) : output_image.rows;
                                        int dw = depth_mat.cols, dh = depth_mat.rows;
                                        double sx_scale = cw > 0 ? static_cast<double>(dw) / cw : 1.0;
                                        double sy_scale = ch > 0 ? static_cast<double>(dh) / ch : 1.0;
                                        cv::Rect droi(
                                            std::clamp(static_cast<int>(std::round(bbox.x * sx_scale)), 0, dw-1),
                                            std::clamp(static_cast<int>(std::round(bbox.y * sy_scale)), 0, dh-1),
                                            std::clamp(static_cast<int>(std::round(bbox.width * sx_scale)), 1, dw),
                                            std::clamp(static_cast<int>(std::round(bbox.height * sy_scale)), 1, dh)
                                        );
                                        droi.width = std::min(droi.width, dw - droi.x);
                                        droi.height = std::min(droi.height, dh - droi.y);
                                        // 帯の上下境界（ノードパラメータ）
                                        double bottom_ratio = this->get_parameter("hist_band_bottom_ratio").as_double();
                                        double top_ratio    = this->get_parameter("hist_band_top_ratio").as_double();
                                        int y0d = droi.y + static_cast<int>(std::floor(droi.height * (1.0 - top_ratio)));
                                        int y1d = droi.y + static_cast<int>(std::floor(droi.height * (1.0 - bottom_ratio)));
                                        y0d = std::clamp(y0d, droi.y, droi.y + droi.height - 1);
                                        y1d = std::clamp(y1d, droi.y, droi.y + droi.height - 1);
                                        if (y1d < y0d) std::swap(y0d, y1d);
                                        cv::Mat band = depth_mat(cv::Rect(droi.x, y0d, droi.width, std::max(1, y1d - y0d + 1))).clone();
                                        // 正規化（近距離ほど明）
                                        cv::Mat band_u8;
                                        if (depth_is_16u) {
                                            cv::Mat mask = band > 0;
                                            double dmin=0.0, dmax=0.0; cv::minMaxLoc(band, &dmin, &dmax, nullptr, nullptr, mask);
                                            if (!(dmax > dmin)) { dmin = 0.0; dmax = 10000.0; }
                                            cv::Mat band_f; band.convertTo(band_f, CV_32F);
                                            cv::Mat norm = (band_f - static_cast<float>(dmin)) * (255.0f / static_cast<float>(dmax - dmin + 1e-6f));
                                            norm.setTo(0, ~mask);
                                            norm.convertTo(band_u8, CV_8U);
                                        } else {
                                            double dmin=0.0, dmax=0.0; cv::minMaxLoc(band, &dmin, &dmax);
                                            if (!(dmax > dmin)) { dmin = 0.0; dmax = 2.0; }
                                            cv::Mat norm = (band - dmin) * (255.0 / (dmax - dmin + 1e-6));
                                            norm.convertTo(band_u8, CV_8U);
                                        }
                                        // 近距離ほど明くする（上下反転ではなくレンジに依存）
                                        cv::Mat band_bgr; cv::cvtColor(band_u8, band_bgr, cv::COLOR_GRAY2BGR);
                                        cv::Mat band_resized; cv::resize(band_bgr, band_resized, cv::Size(sw, 16), 0, 0, cv::INTER_NEAREST);
                                        int sy2 = sy + sh + 2;
                                        if (sy2 + band_resized.rows <= output_image.rows) {
                                            band_resized.copyTo(output_image(cv::Rect(sx, sy2, band_resized.cols, band_resized.rows)));
                                        }
                                    }
                                } catch (...) {}
                            }
                        }
                    }
                }
            } catch (...) {}

            // 推定根本2D位置（analyzer側の推定z0とヒストで得たXに相当）を可視化
            // 既に3Dの根本は aspara_info.root_position_3d に格納。2Dへ投影して赤丸を描画
            if (latest_camera_info_) {
                const auto& rp = aspara_info.root_position_3d;
                if (std::isfinite(rp.z) && rp.z > 0.0) {
                    double fx = latest_camera_info_->k[0];
                    double fy = latest_camera_info_->k[4];
                    double cx = latest_camera_info_->k[2];
                    double cy = latest_camera_info_->k[5];
                    int u = static_cast<int>(std::round(fx * (rp.x / rp.z) + cx));
                    int v = static_cast<int>(std::round(fy * (rp.y / rp.z) + cy));
                    if (u >= 0 && u < output_image.cols && v >= 0 && v < output_image.rows) {
                        cv::circle(output_image, cv::Point(u, v), 6, cv::Scalar(0, 0, 255), -1);
                        // 根本座標テキスト表示は無し（赤丸のみ）
                    }
                }
            }
            
            // 上部の緑バック黒字ラベルは非表示にする（ユーザー指示）

            // 矩形内詳細情報（選択中のみ）
            if (is_selected) {
                auto gradeToLabel = [](AsparaguGrade g){
                    switch (g) {
                        case AsparaguGrade::A_GRADE: return "A";
                        case AsparaguGrade::B_GRADE: return "B";
                        case AsparaguGrade::C_GRADE: return "C";
                        case AsparaguGrade::OUT_OF_SPEC: return "NG";
                        default: return "-";
                    }
                };
                // 計測値
                const int raw_pts  = static_cast<int>(aspara_info.asparagus_pointcloud.width);
                const int filt_pts = static_cast<int>(aspara_info.filtered_pointcloud.width);
                const double len_cm = aspara_info.length * 100.0;
                const double dia_mm = aspara_info.diameter * 1000.0;
                const double str_pct = aspara_info.straightness * 100.0;
                const double t_ms = aspara_info.processing_times.total_ms;
                const int spikes = static_cast<int>(aspara_info.spike_parts.size());
                // 距離（根本3D座標からの距離）
                double dist_m = 0.0;
                if (std::isfinite(aspara_info.root_position_3d.x) && std::isfinite(aspara_info.root_position_3d.y) && std::isfinite(aspara_info.root_position_3d.z)) {
                    dist_m = std::sqrt(
                        aspara_info.root_position_3d.x * aspara_info.root_position_3d.x +
                        aspara_info.root_position_3d.y * aspara_info.root_position_3d.y +
                        aspara_info.root_position_3d.z * aspara_info.root_position_3d.z);
                }
                // 分析状態
                bool analyzing = (t_ms <= 0.0) && analyzer_thread_ && analyzer_thread_->isProcessing();
                // 表示行
                std::vector<std::string> lines;
                lines.push_back(cv::format("ID:%d [選択中]", aspara_info.id));
                lines.push_back(cv::format("信頼: %.0f%%", aspara_info.confidence * 100.0));
                if (dist_m > 0.0) lines.push_back(cv::format("距離: %.2fm", dist_m));
                // 要望: 点群数は先頭の生ROI点数のみ表示
                lines.push_back(cv::format("点群: %d", raw_pts));
                if (analyzing) {
                    lines.push_back("分析中...");
                } else {
                    lines.push_back(cv::format("分析: %.1fms", t_ms));
                }
                // 表示レンジも考慮して長さ表示をフィルタ
                double disp_len_min = this->has_parameter("display_length_min_m") ? this->get_parameter("display_length_min_m").as_double() : 0.0;
                double disp_len_max = this->has_parameter("display_length_max_m") ? this->get_parameter("display_length_max_m").as_double() : std::numeric_limits<double>::infinity();
                bool len_ok = aspara_info.length_valid && aspara_info.length >= disp_len_min && aspara_info.length <= disp_len_max;
                if (t_ms > 0.0) {
                    lines.push_back(len_ok ? cv::format("長さ: %.1fcm", len_cm) : "長さ: --");
                }
                if (t_ms > 0.0) {
                    if (dia_mm > 0.0) lines.push_back(cv::format("太さ: %.0fmm", dia_mm));
                    lines.push_back(cv::format("真直: %.0f%%", str_pct));
                    lines.push_back(cv::format("グレード: %s", gradeToLabel(aspara_info.grade)));
                }
                if (spikes > 0) lines.push_back(cv::format("穂: %d個", spikes));

                // 情報パネルは右プレビュー描画の後に最前面で重ね描きするため、この段階では描画しない
            }
        }

        // === 選択アスパラのボクセルプレビュー（左隣に半透明の黒枠パネル） ===
        bool left_on  = this->has_parameter("left_panel_enabled") ? this->get_parameter("left_panel_enabled").as_bool() : false;
        bool right_on = this->has_parameter("right_panel_enabled") ? this->get_parameter("right_panel_enabled").as_bool() : true;
        if (snapshot_selected_id != -1 && (left_on || right_on)) {
            // 深度・カメラ情報を取得
            sensor_msgs::msg::Image::SharedPtr depth_copy;
            sensor_msgs::msg::Image::SharedPtr color_copy_for_panel;
            sensor_msgs::msg::CameraInfo::SharedPtr caminfo_copy;
            {
                std::lock_guard<std::mutex> lk(image_data_mutex_);
                depth_copy = latest_depth_image_;
                color_copy_for_panel = latest_color_image_;
                caminfo_copy = latest_camera_info_;
            }
            if (depth_copy && caminfo_copy) {
                // 対象ROI
                const auto it_sel = std::find_if(snapshot_list.begin(), snapshot_list.end(),
                    [&](const AsparaInfo& a){ return a.id == snapshot_selected_id; });
                if (it_sel != snapshot_list.end()) {
                    const cv::Rect roi = it_sel->smooth_bbox & cv::Rect(0,0,output_image.cols, output_image.rows);
                    if (roi.area() > 0) {
                        try {
                            // 深度画像をOpenCVに
                            cv::Mat depth_mat;
                            bool depth_is_16u = false;
                            if (depth_copy->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                                depth_mat = cv_bridge::toCvCopy(depth_copy, sensor_msgs::image_encodings::TYPE_16UC1)->image;
                                depth_is_16u = true;
                            } else if (depth_copy->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                                depth_mat = cv_bridge::toCvCopy(depth_copy, sensor_msgs::image_encodings::TYPE_32FC1)->image;
                            }
                            if (!depth_mat.empty()) {
                                // ROI領域を深度座標系に合わせる（カラーと深度の解像度差を考慮）
                                int cw = static_cast<int>(latest_color_image_ ? latest_color_image_->width : output_image.cols);
                                int ch = static_cast<int>(latest_color_image_ ? latest_color_image_->height : output_image.rows);
                                int dw = static_cast<int>(depth_copy->width);
                                int dh = static_cast<int>(depth_copy->height);
                                double sx = (cw > 0) ? static_cast<double>(dw) / cw : 1.0;
                                double sy = (ch > 0) ? static_cast<double>(dh) / ch : 1.0;
                                cv::Rect droi(
                                    std::clamp(static_cast<int>(std::round(roi.x * sx)), 0, dw-1),
                                    std::clamp(static_cast<int>(std::round(roi.y * sy)), 0, dh-1),
                                    std::clamp(static_cast<int>(std::round(roi.width * sx)), 1, dw),
                                    std::clamp(static_cast<int>(std::round(roi.height * sy)), 1, dh)
                                );
                                droi.width = std::min(droi.width, dw - droi.x);
                                droi.height = std::min(droi.height, dh - droi.y);

                                // 3Dポイント収集（間引き）+ 元カラー保持
                                std::vector<cv::Point3f> points;
                                std::vector<cv::Vec3b>  colors;
                                points.reserve(4000);
                                colors.reserve(4000);
                                double fx = caminfo_copy->k[0], fy = caminfo_copy->k[4];
                                double cx = caminfo_copy->k[2], cy = caminfo_copy->k[5];
                                // 内参の解像度差を補正
                                int kiw = static_cast<int>(caminfo_copy->width);
                                int kih = static_cast<int>(caminfo_copy->height);
                                if (kiw > 0 && kih > 0) {
                                    double sfx = static_cast<double>(dw) / kiw;
                                    double sfy = static_cast<double>(dh) / kih;
                                    fx *= sfx; cx *= sfx; fy *= sfy; cy *= sfy;
                                }
                                // 深度スケール
                                double depth_scale = depth_is_16u ? depth_unit_m_16u_ : 1.0;
                                const int step = 3; // 間引き
                                cv::Mat color_mat;
                                if (color_copy_for_panel) {
                                    try { color_mat = cv_bridge::toCvCopy(color_copy_for_panel, sensor_msgs::image_encodings::BGR8)->image; } catch (...) {}
                                }
                                for (int v = droi.y; v < droi.y + droi.height; v += step) {
                                    const uint16_t* row16 = depth_is_16u ? depth_mat.ptr<uint16_t>(v) : nullptr;
                                    const float* row32 = !depth_is_16u ? depth_mat.ptr<float>(v) : nullptr;
                                    for (int u = droi.x; u < droi.x + droi.width; u += step) {
                                        float z = 0.0f;
                                        if (depth_is_16u) {
                                            uint16_t d = row16[u];
                                            if (d == 0) continue;
                                            z = static_cast<float>(d) * static_cast<float>(depth_scale);
                                        } else {
                                            float d = row32[u];
                                            if (!std::isfinite(d) || d <= 0.0f) continue;
                                            z = d;
                                        }
                                        // 距離範囲
                                        if (z < pointcloud_distance_min_ || z > pointcloud_distance_max_) continue;
                                        float x = (static_cast<float>(u) - static_cast<float>(cx)) * z / static_cast<float>(fx);
                                        float y = (static_cast<float>(v) - static_cast<float>(cy)) * z / static_cast<float>(fy);
                                        points.emplace_back(x, y, z);
                                        // 元カラー（カラー画像に合わせて座標スケールを戻す）
                                        if (!color_mat.empty()) {
                                            int uc = static_cast<int>(std::round(u / sx));
                                            int vc = static_cast<int>(std::round(v / sy));
                                            uc = std::clamp(uc, 0, color_mat.cols - 1);
                                            vc = std::clamp(vc, 0, color_mat.rows - 1);
                                            colors.emplace_back(color_mat.at<cv::Vec3b>(vc, uc));
                                        } else {
                                            colors.emplace_back(cv::Vec3b(0,255,0));
                                        }
                                        if (points.size() > 8000) break;
                                    }
                                    if (points.size() > 8000) break;
                                }

                                if (points.size() >= 20) {
                                    // PCAで主成分に整列（上向きを固定）
                                    cv::Mat data(static_cast<int>(points.size()), 3, CV_32F);
                                    for (int i = 0; i < data.rows; ++i) {
                                        data.at<float>(i,0) = points[i].x;
                                        data.at<float>(i,1) = points[i].y;
                                        data.at<float>(i,2) = points[i].z;
                                    }
                                    cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
                                    // 第一主成分を縦軸、第二主成分を横軸に
                                    cv::Mat proj;
                                    pca.project(data, proj); // Nx3 -> Nx3（0=PC1,1=PC2,2=PC3）

                                    // 上下向きの決定（PC1の中央値が上に行くように符号調整）
                                    std::vector<float> pc1(proj.rows);
                                    for (int i = 0; i < proj.rows; ++i) pc1[i] = proj.at<float>(i,0);
                                    std::nth_element(pc1.begin(), pc1.begin()+pc1.size()/2, pc1.end());
                                    float med = pc1[pc1.size()/2];
                                    float sgn = (med < 0.0f) ? -1.0f : 1.0f; // 上向きを正に

                                    // パネル領域（左）
                                    const int panel_w = std::min(160, roi.width); // ROI幅に合わせる
                                    const int panel_h = roi.height;
                                    int panel_x = std::max(5, roi.x - panel_w - 8);
                                    int panel_y = std::max(5, roi.y);
                                    cv::Rect panel(panel_x, panel_y, panel_w, std::min(panel_h, output_image.rows - panel_y - 5));
                                    // 左側に十分な空きがある場合のみ描画（画面外やROIと重なるなら非表示）
                                    bool left_ok = left_on &&
                                                   panel_x >= 0 && panel_y >= 0 &&
                                                   (panel_x + panel_w) <= std::max(0, roi.x - 2) &&
                                                   (panel_y + panel.height) <= output_image.rows - 1;
                                    if (left_ok) {
                                        cv::Mat roi_img = output_image(panel);
                                        cv::Mat overlay_panel; roi_img.copyTo(overlay_panel);
                                        cv::rectangle(overlay_panel, cv::Rect(0,0,panel.width,panel.height), cv::Scalar(0,0,0), -1);
                                        cv::addWeighted(overlay_panel, 0.5, roi_img, 0.5, 0.0, roi_img);
                                    }

                                    // Depth ROI (Jet) preview in left panel background
                                    if (left_ok) try {
                                        cv::Mat depth_roi = depth_mat(droi).clone();
                                        if (!depth_roi.empty()) {
                                            cv::Mat depth_u8;
                                            // Build mask for valid (>0) pixels when 16U
                                            if (depth_is_16u) {
                                                cv::Mat mask = depth_roi > 0;
                                                double dmin=0.0, dmax=0.0;
                                                cv::minMaxLoc(depth_roi, &dmin, &dmax, nullptr, nullptr, mask);
                                                if (!(dmax > dmin)) { dmin = 0.0; dmax = 10000.0; }
                                                cv::Mat depth_f; depth_roi.convertTo(depth_f, CV_32F);
                                                cv::Mat depth_norm;
                                                depth_norm = (depth_f - static_cast<float>(dmin)) * (255.0f / static_cast<float>(dmax - dmin + 1e-6f));
                                                depth_norm.setTo(0, ~mask);
                                                depth_norm.convertTo(depth_u8, CV_8U);
                                            } else {
                                                // 32F meters -> normalize within ROI
                                                double dmin=0.0, dmax=0.0; cv::minMaxLoc(depth_roi, &dmin, &dmax);
                                                if (!(dmax > dmin)) { dmin = 0.0; dmax = 2.0; }
                                                cv::Mat depth_norm = (depth_roi - dmin) * (255.0 / (dmax - dmin + 1e-6));
                                                depth_norm.convertTo(depth_u8, CV_8U);
                                            }
                                            cv::Mat depth_color;
                                            cv::applyColorMap(depth_u8, depth_color, cv::COLORMAP_JET);
                                            cv::Mat depth_resized;
                                            cv::resize(depth_color, depth_resized, cv::Size(panel.width, panel.height), 0, 0, cv::INTER_NEAREST);
                                            // Lightly blend to keep overlays readable
                                            cv::Mat roi_img = output_image(panel);
                                            cv::addWeighted(depth_resized, 0.6, roi_img, 0.4, 0.0, roi_img);
                                        }
                                    } catch (...) {
                                        // ignore preview errors
                                    }

                                    // 画像座標系で投影して描画する関数（PCA不使用）
                                    auto drawCloudToPanel = [&](const sensor_msgs::msg::PointCloud2& pc2,
                                                               const cv::Rect& roi_rect,
                                                               const cv::Rect& pnl_rect,
                                                               int& out_count) {
                                        out_count = 0;
                                        if (!caminfo_copy || pc2.data.empty()) return;
                                        double fx = caminfo_copy->k[0];
                                        double fy = caminfo_copy->k[4];
                                        double cx = caminfo_copy->k[2];
                                        double cy = caminfo_copy->k[5];
                                        bool has_rgb = false;
                                        for (const auto& f : pc2.fields) if (f.name == "rgb") { has_rgb = true; break; }
                                        const uint8_t* data_ptr = pc2.data.data();
                                        size_t step = pc2.point_step;
                                        size_t n = pc2.width * pc2.height;
                                        auto putDot = [&](int px, int py, const cv::Vec3b& c){
                                            for (int dy=-1; dy<=1; ++dy) {
                                                for (int dx=-1; dx<=1; ++dx) {
                                                    int xx = px+dx, yy = py+dy;
                                                    if (xx > pnl_rect.x && xx < pnl_rect.x + pnl_rect.width-1 &&
                                                        yy > pnl_rect.y && yy < pnl_rect.y + pnl_rect.height-1) {
                                                        output_image.at<cv::Vec3b>(yy, xx) = c;
                                                    }
                                                }
                                            }
                                        };
                                        for (size_t i = 0; i < n; ++i) {
                                            const uint8_t* pt = data_ptr + i * step;
                                            float x, y, z; std::memcpy(&x, pt + 0, 4); std::memcpy(&y, pt + 4, 4); std::memcpy(&z, pt + 8, 4);
                                            if (!std::isfinite(z) || z <= 0.0f) continue;
                                            double u = fx * (static_cast<double>(x) / static_cast<double>(z)) + cx;
                                            double v = fy * (static_cast<double>(y) / static_cast<double>(z)) + cy;
                                            if (u < roi_rect.x || u >= roi_rect.x + roi_rect.width ||
                                                v < roi_rect.y || v >= roi_rect.y + roi_rect.height) continue;
                                            int px = pnl_rect.x + 1 + static_cast<int>((u - roi_rect.x) / roi_rect.width * (pnl_rect.width - 2));
                                            int py = pnl_rect.y + 1 + static_cast<int>((v - roi_rect.y) / roi_rect.height * (pnl_rect.height - 2));
                                            cv::Vec3b c(0,255,0);
                                            if (has_rgb && step >= 16) {
                                                float rgbf; std::memcpy(&rgbf, pt + 12, 4);
                                                uint32_t rgb; std::memcpy(&rgb, &rgbf, 4);
                                                uint8_t r = (rgb >> 16) & 0xff, g = (rgb >> 8) & 0xff, b = rgb & 0xff;
                                                c = cv::Vec3b(b,g,r);
                                            }
                                            putDot(px, py, c);
                                            ++out_count;
                                        }
                                    };

                                    if (left_ok) {
                                        // 左枠: RAW（asparagus_pointcloud）を投影描画
                                    int raw_drawn = 0;
                                    if (it_sel != snapshot_list.end() && !it_sel->asparagus_pointcloud.data.empty()) {
                                        drawCloudToPanel(it_sel->asparagus_pointcloud, roi, panel, raw_drawn);
                                    }
                                        // 左枠ラベル（総点数）
                                        {
                                            int raw_total = 0;
                                            if (it_sel != snapshot_list.end() && !it_sel->asparagus_pointcloud.data.empty()) {
                                                raw_total = static_cast<int>(it_sel->asparagus_pointcloud.width * it_sel->asparagus_pointcloud.height);
                                            }
                                            std::string lbl = cv::format("RAW: %d", raw_total);
                                            int ty = std::max(12, panel_y - 6);
                                            fluent::text::draw(output_image, lbl, cv::Point(panel_x+2, ty), cv::Scalar(255,255,255), 0.5, 1);
                                        }
                                    }

                                    // 左パネルに下部帯ヒストグラム（left_ok時のみ）
                                    if (left_ok) try {
                                        if (this->get_parameter("depth_scan_preview_enabled").as_bool() && color_copy_for_panel) {
                                            cv::Mat color_mat_h; 
                                            try { color_mat_h = cv_bridge::toCvCopy(color_copy_for_panel, sensor_msgs::image_encodings::BGR8)->image; } catch (...) {}
                                            if (!color_mat_h.empty()) {
                                                cv::Mat hsv; cv::cvtColor(color_mat_h, hsv, cv::COLOR_BGR2HSV);
                                                double bottom_ratio = this->get_parameter("hist_band_bottom_ratio").as_double();
                                                double top_ratio = this->get_parameter("hist_band_top_ratio").as_double();
                                                double hmin = this->has_parameter("root_hsv_h_min") ? this->get_parameter("root_hsv_h_min").as_double() : 35.0;
                                                double hmax = this->has_parameter("root_hsv_h_max") ? this->get_parameter("root_hsv_h_max").as_double() : 85.0;
                                                double smin = this->has_parameter("root_hsv_s_min") ? this->get_parameter("root_hsv_s_min").as_double() : 0.25;
                                                double vmin = this->has_parameter("root_hsv_v_min") ? this->get_parameter("root_hsv_v_min").as_double() : 0.20;
                                                auto roi_c = roi & cv::Rect(0,0,color_mat_h.cols, color_mat_h.rows);
                                                int y0c = roi_c.y + static_cast<int>(std::floor(roi_c.height * (1.0 - top_ratio)));
                                                int y1c = roi_c.y + static_cast<int>(std::floor(roi_c.height * (1.0 - bottom_ratio)));
                                                y0c = std::clamp(y0c, roi_c.y, roi_c.y + roi_c.height - 1);
                                                y1c = std::clamp(y1c, roi_c.y, roi_c.y + roi_c.height - 1);
                                                if (y1c < y0c) std::swap(y0c, y1c);
                                                int bins = std::max(4, roi_c.width / 4);
                                                std::vector<int> hist(bins, 0);
                                                for (int v = y0c; v <= y1c; ++v) {
                                                    for (int u = roi_c.x; u < roi_c.x + roi_c.width; ++u) {
                                                        cv::Vec3b px = hsv.at<cv::Vec3b>(v,u);
                                                        double H = (px[0] * 2.0);
                                                        double S = px[1] / 255.0; double V = px[2] / 255.0;
                                                        if (!(H>=hmin && H<=hmax && S>=smin && V>=vmin)) continue;
                                                        int b = std::clamp((u - roi_c.x) * bins / std::max(1, roi_c.width), 0, bins-1);
                                                        hist[b]++;
                                                    }
                                                }
                                                int maxv = *std::max_element(hist.begin(), hist.end());
                                                if (maxv > 0) {
                                                    int gh = std::min(40, panel.height/4);
                                                    int gy = panel.y + panel.height - gh - 2;
                                                    int gx = panel.x + 2;
                                                    int gw = panel.width - 4;
                                                    for (int i = 0; i < bins; ++i) {
                                                        int x0 = gx + i * gw / bins;
                                                        int x1 = gx + (i+1) * gw / bins - 1;
                                                        int h = static_cast<int>(std::round((hist[i] / (double)maxv) * gh));
                                                        cv::rectangle(output_image, cv::Rect(x0, gy + gh - h, x1 - x0 + 1, h), cv::Scalar(0,255,0), -1);
                                                    }
                                                     fluent::text::draw(output_image, "帯スキャン", cv::Point(panel.x+4, gy-4), cv::Scalar(0,255,0), 0.5, 1);
                                                     // HUDインジケータは非表示
                                                }
                                            }
                                        }
                                    } catch (...) {}

                                    // 右側プレビュー：フィルタ済み点群（denoised）を表示
                                    int panel_x_r = std::min(output_image.cols - panel_w - 5, roi.x + roi.width + 8);
                                    // 右側に十分な空きがある場合のみ描画（画面外やROIと重なるなら非表示）
                                    bool right_ok = right_on &&
                                                    (panel_x_r >= std::min(output_image.cols - panel_w - 5, roi.x + roi.width + 8)) &&
                                                    (panel_x_r >= roi.x + roi.width + 2) &&
                                                    (panel_x_r + panel_w <= output_image.cols - 1);
                                    if (right_ok) {
                                        cv::Rect panel_r(panel_x_r, panel_y, panel_w, std::min(panel_h, output_image.rows - panel_y - 5));
                                        // 半透明黒オーバーレイは残す（テキストは描画しない）
                                        {
                                            cv::Mat roi_r = output_image(panel_r);
                                            cv::Mat ov_r; roi_r.copyTo(ov_r);
                                            cv::rectangle(ov_r, cv::Rect(0,0,panel_r.width,panel_r.height), cv::Scalar(0,0,0), -1);
                                            cv::addWeighted(ov_r, 0.5, roi_r, 0.5, 0.0, roi_r);
                                        }
                                        // 右枠: FILTERED（filtered_pointcloud）の点群のみ描画
                                        int filt_drawn = 0;
                                        if (it_sel != snapshot_list.end() && !it_sel->filtered_pointcloud.data.empty()) {
                                            drawCloudToPanel(it_sel->filtered_pointcloud, roi, panel_r, filt_drawn);
                                        }

                                        // 情報ウィンドウは点群を描いた“後”に重ねて描画（見えなくならないよう最前面）
                                        try {
                                            auto paramb = [this](const char* key, bool defv){return this->has_parameter(key)? this->get_parameter(key).as_bool() : defv;};
                                            bool show_id = paramb("info_show_id", true);
                                            bool show_conf = paramb("info_show_confidence", true);
                                            bool show_length = paramb("info_show_length", true);
                                            bool show_diameter = paramb("info_show_diameter", true);
                                            bool show_straightness = paramb("info_show_straightness", true);
                                            bool show_grade = paramb("info_show_grade", true);
                                            bool show_harvest = paramb("info_show_harvestable", true);
                                            bool show_distance = paramb("info_show_distance", true);
                                            bool show_debug = paramb("info_show_debug_points", true);

                                            std::vector<std::string> lines;
                                            if (show_id) lines.push_back(cv::format("ID: %d", it_sel->id));
                                            if (show_conf) lines.push_back(cv::format("信頼度: %.0f%%", it_sel->confidence * 100.0));
                                            // 表示制限（長さ/太さ）を尊重
                                            double disp_len_min = this->has_parameter("display_length_min_m") ? this->get_parameter("display_length_min_m").as_double() : 0.0;
                                            double disp_len_max = this->has_parameter("display_length_max_m") ? this->get_parameter("display_length_max_m").as_double() : std::numeric_limits<double>::infinity();
                                            double disp_dia_min = this->has_parameter("display_diameter_min_m") ? this->get_parameter("display_diameter_min_m").as_double() : 0.0;
                                            double disp_dia_max = this->has_parameter("display_diameter_max_m") ? this->get_parameter("display_diameter_max_m").as_double() : std::numeric_limits<double>::infinity();

                                            bool len_ok = it_sel->length_valid && it_sel->length >= disp_len_min && it_sel->length <= disp_len_max;
                                            bool dia_ok = (it_sel->diameter >= disp_dia_min && it_sel->diameter <= disp_dia_max);

                                            if (show_length) {
                                                if (len_ok) lines.push_back(cv::format("長さ: %.1fcm", it_sel->length * 100.0));
                                                else        lines.push_back("長さ: --");
                                            }
                                            if (show_diameter) {
                                                if (dia_ok && it_sel->diameter > 0) lines.push_back(cv::format("太さ: %.0fmm", it_sel->diameter * 1000.0));
                                                else                               lines.push_back("太さ: --");
                                            }
                                            if (show_straightness) lines.push_back(cv::format("曲がり度: %.0f", it_sel->straightness * 100.0));
                                            // デバッグ表示が有効なとき、直線参照線（弦長）を表示
                                            if (show_debug) {
                                                double chord_cm = 0.0;
                                                if (!it_sel->skeleton_points.empty()) {
                                                    const auto &wf = it_sel->skeleton_points.front().world_point;
                                                    const auto &wl = it_sel->skeleton_points.back().world_point;
                                                    double dx = static_cast<double>(wf.x) - static_cast<double>(wl.x);
                                                    double dy = static_cast<double>(wf.y) - static_cast<double>(wl.y);
                                                    double dz = static_cast<double>(wf.z) - static_cast<double>(wl.z);
                                                    chord_cm = std::sqrt(dx*dx + dy*dy + dz*dz) * 100.0;
                                                }
                                                lines.push_back(cv::format("参照直線: %.1fcm", chord_cm));
                                            }
                                            if (show_grade) {
                                                const char* gl = (it_sel->grade==AsparaguGrade::A_GRADE?"A": it_sel->grade==AsparaguGrade::B_GRADE?"B": it_sel->grade==AsparaguGrade::C_GRADE?"C":"NG");
                                                lines.push_back(std::string("グレード: ") + gl);
                                            }
                                            if (show_harvest) lines.push_back(std::string("収穫可否: ") + (it_sel->is_harvestable?"YES":"NO"));
                                            if (show_distance) {
                                                double dm = 0.0; const auto &rp = it_sel->root_position_3d;
                                                if (std::isfinite(rp.x) && std::isfinite(rp.y) && std::isfinite(rp.z)) dm = std::sqrt(rp.x*rp.x + rp.y*rp.y + rp.z*rp.z);
                                                if (dm > 0.0) lines.push_back(cv::format("距離: %.2fm", dm));
                                            }
                                            if (show_debug) {
                                                int raw_pts = static_cast<int>(it_sel->asparagus_pointcloud.width * it_sel->asparagus_pointcloud.height);
                                                int filt_pts = static_cast<int>(it_sel->filtered_pointcloud.width * it_sel->filtered_pointcloud.height);
                                                lines.push_back("[debug]");
                                                lines.push_back(cv::format("点群数: %d/%d", raw_pts, filt_pts));
                                            }

                                            // テキストに合わせてサイズ調整＋配置（選択枠基準のアンカー＆オフセット）
                                            const int pad = 6; const int lh = 18; int panel_text_w = 0; int base = 0;
                                            for (const auto &ln : lines) { cv::Size ts = cv::getTextSize(ln, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base); panel_text_w = std::max(panel_text_w, ts.width); }
                                            int text_w = panel_text_w + pad*2;
                                            int text_h = static_cast<int>(lines.size())*lh + pad*2;

                                            // パラメータ: アンカー(top_left|top_right)とオフセット
                                            auto params = [this](const char* key, int defv){return this->has_parameter(key)? this->get_parameter(key).as_int() : defv;};
                                            std::string anchor = this->has_parameter("info_anchor") ? this->get_parameter("info_anchor").as_string() : std::string("top_right");
                                            int offx = params("info_offset_x", 8);
                                            int offy = params("info_offset_y", 8);

                                            // 選択枠基準座標
                                            int anchor_x = (anchor == "top_left") ? roi.x : (roi.x + roi.width);
                                            int anchor_y = roi.y;
                                            int px_i = anchor_x + offx;
                                            int py_i = anchor_y + offy;

                                            // 画像範囲に収める
                                            px_i = std::clamp(px_i, 0, std::max(0, output_image.cols - text_w - 1));
                                            py_i = std::clamp(py_i, 0, std::max(0, output_image.rows - text_h - 1));
                                            cv::Rect info(px_i, py_i, text_w, text_h);

                                            // 背景と白枠
                                            {
                                                cv::Mat roi_r = output_image(info); cv::Mat ov_r; roi_r.copyTo(ov_r);
                                                cv::rectangle(ov_r, cv::Rect(0,0,info.width,info.height), cv::Scalar(0,0,0), -1);
                                                cv::addWeighted(ov_r, 0.45, roi_r, 0.55, 0.0, roi_r);
                                            }
                                            cv::rectangle(output_image, info, cv::Scalar(255,255,255), 1);

                                            int tx = info.x + pad; int ty = info.y + pad + 12;
                                            for (const auto &ln : lines) { fluent::text::draw(output_image, ln, cv::Point(tx, ty), cv::Scalar(255,255,255), 0.5, 1); ty += lh; }
                                        } catch (...) {}

                                    }
                                }
                            }
                        } catch (...) {
                            // 失敗しても無視（描画だけ）
                        }
                    }
                }
            }
        }
    } else {
        // 未検出の場合 - 表示なし
        selected_aspara_id_ = -1; // 選択解除
    }
    
    // カーソルのスムーズアニメーション更新と描画
    if (cursor_visible_ && cursor_position_.x >= 0 && cursor_position_.y >= 0) {
        // スムーズアニメーション（200msで収束）
        const float cursor_smoothing_time = 0.2f;
        float cursor_smooth_factor = std::min(1.0f, delta_time / cursor_smoothing_time);
        
        auto lerp = [](float a, float b, float t) { return a + (b - a) * t; };
        smooth_cursor_position_.x = lerp(smooth_cursor_position_.x, cursor_position_.x, cursor_smooth_factor);
        smooth_cursor_position_.y = lerp(smooth_cursor_position_.y, cursor_position_.y, cursor_smooth_factor);
        
        // 緑の十字カーソルを描画
        cv::Scalar cursor_color(0, 255, 0);  // 緑色
        int cursor_size = 20;
        int cursor_thickness = 2;
        
        // 横線
        cv::line(output_image, 
                cv::Point(smooth_cursor_position_.x - cursor_size, smooth_cursor_position_.y),
                cv::Point(smooth_cursor_position_.x + cursor_size, smooth_cursor_position_.y),
                cursor_color, cursor_thickness);
        
        // 縦線
        cv::line(output_image,
                cv::Point(smooth_cursor_position_.x, smooth_cursor_position_.y - cursor_size),
                cv::Point(smooth_cursor_position_.x, smooth_cursor_position_.y + cursor_size),
                cursor_color, cursor_thickness);
        
        // カーソル位置の距離・座標表示（(x,y,z) と 距離[cm]）
        double distance_m = 0.0;
        bool has_distance = false;
        double x_m = 0.0, y_m = 0.0, z_m = 0.0;
        {
            // 深度・カメラ情報を取得
            sensor_msgs::msg::Image::SharedPtr depth_copy;
            sensor_msgs::msg::Image::SharedPtr color_copy;
            sensor_msgs::msg::CameraInfo::SharedPtr caminfo_copy;
            {
                std::lock_guard<std::mutex> lk(image_data_mutex_);
                depth_copy = latest_depth_image_;
                color_copy = latest_color_image_;
                caminfo_copy = latest_camera_info_;
            }
            if (depth_copy && color_copy) {
                try {
                    // 変換
                    cv::Mat depth_mat;
                    if (depth_copy->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                        depth_mat = cv_bridge::toCvCopy(depth_copy, sensor_msgs::image_encodings::TYPE_16UC1)->image;
                    } else if (depth_copy->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                        depth_mat = cv_bridge::toCvCopy(depth_copy, sensor_msgs::image_encodings::TYPE_32FC1)->image;
                    }
                    // カラー画像サイズ
                    int cw = static_cast<int>(color_copy->width);
                    int ch = static_cast<int>(color_copy->height);
                    int dw = static_cast<int>(depth_copy->width);
                    int dh = static_cast<int>(depth_copy->height);
                    // カーソル座標を深度座標にスケール
                    double sx = (cw > 0) ? static_cast<double>(dw) / cw : 1.0;
                    double sy = (ch > 0) ? static_cast<double>(dh) / ch : 1.0;
                    int xd = static_cast<int>(std::round(smooth_cursor_position_.x * sx));
                    int yd = static_cast<int>(std::round(smooth_cursor_position_.y * sy));
                    // 近傍3x3で中央値
                    std::vector<double> vals;
                    for (int vy = std::max(0, yd - 1); vy <= std::min(dh - 1, yd + 1); ++vy) {
                        for (int ux = std::max(0, xd - 1); ux <= std::min(dw - 1, xd + 1); ++ux) {
                            if (depth_mat.type() == CV_16UC1) {
                                uint16_t mm = depth_mat.at<uint16_t>(vy, ux);
                                if (mm > 0) vals.push_back(static_cast<double>(mm) * depth_unit_m_16u_);
                            } else if (depth_mat.type() == CV_32FC1) {
                                float m = depth_mat.at<float>(vy, ux);
                                if (std::isfinite(m) && m > 0.0f) vals.push_back(static_cast<double>(m));
                            }
                        }
                    }
                    if (!vals.empty()) {
                        std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
                        distance_m = vals[vals.size() / 2];
                        z_m = distance_m;
                        // 3D座標計算（カメラ座標系）
                        if (caminfo_copy) {
                            double fx = caminfo_copy->k[0];
                            double fy = caminfo_copy->k[4];
                            double cx = caminfo_copy->k[2];
                            double cy = caminfo_copy->k[5];
                            // カメラ内参が別解像度の場合はスケール調整
                            int kiw = static_cast<int>(caminfo_copy->width);
                            int kih = static_cast<int>(caminfo_copy->height);
                            if (kiw > 0 && kih > 0) {
                                double sfx = static_cast<double>(dw) / kiw;
                                double sfy = static_cast<double>(dh) / kih;
                                fx *= sfx; cx *= sfx; fy *= sfy; cy *= sfy;
                            }
                            if (fx > 0.0 && fy > 0.0) {
                                x_m = (static_cast<double>(xd) - cx) * z_m / fx;
                                y_m = (static_cast<double>(yd) - cy) * z_m / fy;
                            }
                        }
                        has_distance = true;
                    }
                } catch (...) {
                    has_distance = false;
                }
            }
        }
        // カーソルの座標・距離テキストは表示しない
    }

    // 骨格オーバーレイ（全アスパラ表示。選択は太線で強調）
    int selected_skeleton_points = 0;
    for (const auto& a : snapshot_list) {
        if (a.skeleton_points.empty()) continue;
        bool is_selected = (a.id == snapshot_selected_id);
        if (is_selected) {
            selected_skeleton_points = static_cast<int>(a.skeleton_points.size());
            // デバッグ：各骨格点の座標を表示
            for (size_t i = 0; i < a.skeleton_points.size(); ++i) {
                const auto& sp = a.skeleton_points[i];
                RCLCPP_DEBUG(this->get_logger(), 
                    "[SKELETON_DRAW] Point %zu: image(%.1f,%.1f) world(%.3f,%.3f,%.3f)",
                    i, sp.image_point.x, sp.image_point.y,
                    sp.world_point.x, sp.world_point.y, sp.world_point.z);
            }
        }
        cv::Scalar bone_color = is_selected ? cv::Scalar(255, 0, 255) : cv::Scalar(180, 120, 200);
        // 要望: ピンクの線は半分の細さに（選択時も1px）
        int thickness = 1;
        
        // 骨格曲線の表示はフラグで切替
        bool show_curve = this->has_parameter("show_reference_curve") ? this->get_parameter("show_reference_curve").as_bool() : false;
        if (show_curve) {
            // 全ての骨格点を個別に描画（小さな青丸）
            for (size_t i = 0; i < a.skeleton_points.size(); ++i) {
                const auto& p = a.skeleton_points[i].image_point;
                cv::circle(output_image, p, 2, cv::Scalar(255, 0, 0), -1);
            }
            // 線で結ぶ
            for (size_t i = 1; i < a.skeleton_points.size(); ++i) {
                const auto& p0 = a.skeleton_points[i-1].image_point;
                const auto& p1 = a.skeleton_points[i].image_point;
                cv::line(output_image, p0, p1, bone_color, thickness, cv::LINE_AA);
            }
        }
        
        // 始点(根本)と終点(先端)を強調
        if (a.skeleton_points.size() >= 2) {
            int r = is_selected ? 6 : 4;
            // 根本=frontを赤、先端=backを黄に変更
            cv::circle(output_image, a.skeleton_points.front().image_point, r, cv::Scalar(0,0,255),   -1);
            cv::circle(output_image, a.skeleton_points.back().image_point,  r, cv::Scalar(0,255,255), -1);

            // 根本→先端の直線参照はパラメータで表示切替
            bool show_ref = this->has_parameter("show_reference_line") ? this->get_parameter("show_reference_line").as_bool() : true;
            if (show_ref) {
                const auto& p_root = a.skeleton_points.front().image_point;
                const auto& p_tip  = a.skeleton_points.back().image_point;
                // 参照直線は長さの基準にも使用するため、色は白に変更
                cv::line(output_image, p_root, p_tip, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
                // [DEBUG] 直線参照線の長さ（3D弦長）を出力
                const auto& w_root = a.skeleton_points.front().world_point;
                const auto& w_tip  = a.skeleton_points.back().world_point;
                double chord = std::sqrt(
                    (w_root.x - w_tip.x)*(w_root.x - w_tip.x) +
                    (w_root.y - w_tip.y)*(w_root.y - w_tip.y) +
                    (w_root.z - w_tip.z)*(w_root.z - w_tip.z));
                RCLCPP_DEBUG(this->get_logger(), "[REF_LINE] ID:%d chord_length=%.3fm (root-tip)", a.id, chord);
            }
        }
    }
    
    
    // 選択中のアスパラガスの点群を2D投影して描画
    int drawn_points = 0;
    int total_points = 0;
    for (const auto& a : snapshot_list) {
        if (a.id != snapshot_selected_id) continue;  // 選択中のアスパラのみ
        if (a.asparagus_pointcloud.data.empty()) continue;
        
        // カメラ情報を取得
        sensor_msgs::msg::CameraInfo::SharedPtr caminfo_for_pc;
        {
            std::lock_guard<std::mutex> lock(aspara_list_mutex_);
            caminfo_for_pc = latest_camera_info_;
        }
        if (!caminfo_for_pc) continue;
        
        // PointCloud2をPCLに変換
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        try {
            // フィールドの確認とデータ解析
            bool has_rgb = false;
            for (const auto& field : a.asparagus_pointcloud.fields) {
                if (field.name == "rgb") {
                    has_rgb = true;
                    break;
                }
            }
            
            // データから点群を取り出す
            const uint8_t* data_ptr = a.asparagus_pointcloud.data.data();
            size_t point_step = a.asparagus_pointcloud.point_step;
            size_t num_points = a.asparagus_pointcloud.width * a.asparagus_pointcloud.height;
            total_points = static_cast<int>(num_points);
            
            for (size_t i = 0; i < num_points; ++i) {
                const uint8_t* pt = data_ptr + i * point_step;
                
                // xyz座標を取得 (float32)
                float x, y, z;
                std::memcpy(&x, pt + 0, 4);
                std::memcpy(&y, pt + 4, 4);
                std::memcpy(&z, pt + 8, 4);
                
                // 無効な点はスキップ
                if (!std::isfinite(z) || z <= 0.0f) continue;
                
                // カメラ内部パラメータで2D投影
                if (caminfo_for_pc) {
                    double fx = caminfo_for_pc->k[0];
                    double fy = caminfo_for_pc->k[4];
                    double cx = caminfo_for_pc->k[2];
                    double cy = caminfo_for_pc->k[5];
                    
                    // 3D点を2D画像座標に投影
                    double u = fx * (x / z) + cx;
                    double v = fy * (y / z) + cy;
                    
                    // 画像範囲内チェック
                    if (u >= 0 && u < output_image.cols && v >= 0 && v < output_image.rows) {
                        // 色情報を取得（あれば）
                        cv::Scalar pt_color(100, 255, 100);  // デフォルト：緑
                        if (has_rgb && point_step >= 16) {
                            float rgb_float;
                            std::memcpy(&rgb_float, pt + 12, 4);
                            uint32_t rgb;
                            std::memcpy(&rgb, &rgb_float, 4);
                            uint8_t r = (rgb >> 16) & 0xff;
                            uint8_t g = (rgb >> 8) & 0xff;
                            uint8_t b = rgb & 0xff;
                            pt_color = cv::Scalar(b, g, r);
                        }
                        
                        // 点を描画（選択中なので少し大きめに）
                        cv::circle(output_image, cv::Point(static_cast<int>(u), static_cast<int>(v)), 
                                  2, pt_color, -1, cv::LINE_AA);
                        drawn_points++;
                    }
                }
            }
            
            // 点群描画数のデバッグログ
            RCLCPP_INFO(this->get_logger(), 
                "[POINTCLOUD_DRAW] Drew %d/%d points for aspara ID %d", 
                drawn_points, total_points, a.id);
                
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), 
                "[POINTCLOUD_DRAW] Failed to project pointcloud for aspara %d: %s", 
                a.id, e.what());
        }
    }
    
    // ポイント数を画面に表示（左側パネル）
    if (total_points > 0) {
        std::string points_info = cv::format("Points: %d/%d", drawn_points, total_points);
        cv::Point text_pos(10, output_image.rows - 40);
        fluent::text::draw(output_image, points_info, text_pos, cv::Scalar(0, 255, 255), 0.6, 2);
    }
    
    // FPS表示を簡略化（重い処理を削除）
    // static変数で1秒ごとに更新
    static auto last_fps_update = std::chrono::high_resolution_clock::now();
    static float display_fps = 0.0f;
    static int frame_count = 0;
    frame_count++;
    
    auto fps_delta = std::chrono::duration<float>(current_time - last_fps_update).count();
    if (fps_delta > 1.0f) {
        display_fps = frame_count / fps_delta;
        frame_count = 0;
        last_fps_update = current_time;
        
        // 1秒ごとにコンソール出力（画像描画はしない）
        if (color_fps_meter_ && detection_fps_meter_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "FPS - Camera:%.0f Detection:%.0f Output:%.0f",
                color_fps_meter_->getCurrentFPS(),
                detection_fps_meter_->getCurrentFPS(),
                display_fps);
        }
    }
    
    // ===== 画面左上に情報表示 =====
    // 背景パネルは削除（ユーザー要望により）
    int hud_x = 6;  // 左端に寄せる
    int hud_y = 6;  // 上端に寄せる（さらに上）
    // デバッグオーバーレイは削除（ユーザー要望により）
    
    // 入力ノードの生存ステータス（ノードの接続状態を表示）
    // 左上に直接表示（背景なし）
    auto now_steady2 = std::chrono::steady_clock::now();
    auto alive_within = std::chrono::milliseconds(1500); // 1.5秒以内に受信していれば生存
    bool det_alive = detection_node_seen_ && (now_steady2 - last_detection_msg_time_ <= alive_within);
    bool depth_alive = depth_node_seen_ && (now_steady2 - last_depth_msg_time_ <= alive_within);
    bool cam_alive = camera_node_seen_ && (now_steady2 - last_color_msg_time_ <= alive_within);

    int status_y = hud_y + 22 + 14;  // 黒帯の下に配置
    int status_x = 10;  // 画面左端近く
    auto draw_status = [&](const std::string& label_jp, bool alive) {
        // ●を左側に描画
        cv::Scalar color = alive ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255); // 緑/赤
        cv::circle(output_image, cv::Point(status_x, status_y-5), 6, color, -1);
        // テキストは●の右側に
        fluent::text::draw(output_image, label_jp, cv::Point(status_x + 14, status_y), cv::Scalar(255,255,255), 0.5, 1);
        status_x += 60; // 次の項目へ余白（半分に詰める）
    };
    draw_status("検出", det_alive);
    draw_status("深度", depth_alive);
    draw_status("カメラ", cam_alive);

    // 点群（organized）受信状態を表示（organizedでなければ🔴）
    bool organized_cloud_ok = false;
    bool fallback_active = false;
    bool have_pc = static_cast<bool>(latest_pointcloud_);
    if (have_pc) {
        bool is_organized = latest_pointcloud_->height > 1;
        rclcpp::Time now_rcl = this->now();
        bool is_recent = (now_rcl - latest_pointcloud_->header.stamp).seconds() <= 1.5;
        organized_cloud_ok = is_organized && is_recent;
        fallback_active = (!is_organized || !is_recent);
    }
    draw_status("点群", organized_cloud_ok);

    // フレーム番号（常に更新される値）
    static uint64_t total_frame_count = 0;
    total_frame_count++;
    
    // FPS情報取得
    float color_fps = color_fps_meter_ ? color_fps_meter_->getCurrentFPS() : 0.0f;
    float depth_fps = depth_fps_meter_ ? depth_fps_meter_->getCurrentFPS() : 0.0f;
    float detection_fps = detection_fps_meter_ ? detection_fps_meter_->getCurrentFPS() : 0.0f;
    // float segmentation_fps = segmentation_fps_meter_ ? segmentation_fps_meter_->getCurrentFPS() : 0.0f;  // 現在未使用
    
    // テキスト描画
    cv::Scalar text_color(255, 255, 255);  // 白色
    cv::Scalar fps_good(0, 255, 0);      // 緑（良好）
    cv::Scalar fps_warn(0, 255, 255);    // 黄色（警告）
    cv::Scalar fps_bad(0, 0, 255);       // 赤（問題）
    
    // カメラ名を取得
    static std::string camera_name = this->get_parameter("camera_name").as_string();
    
    // 1行目: 背景の半透明黒パネル（上部の一帯）
    int hud_w = std::min(420, output_image.cols - 12);
    int hud_h = 22; // 一行分の黒帯のみ
    cv::Rect hud_rect(hud_x, hud_y, hud_w, hud_h);
    {
        cv::Mat roi = output_image(hud_rect);
        cv::Mat ov; roi.copyTo(ov);
        cv::rectangle(ov, cv::Rect(0,0,hud_rect.width,hud_rect.height), cv::Scalar(0,0,0), -1);
        double a = std::clamp(hud_alpha_, 0.0, 1.0);
        cv::addWeighted(ov, a, roi, 1.0 - a, 0.0, roi);
    }



    
    // 黒帯の中に小さめでカメラ名+FPS
    std::string fps_line = cv::format("[%s] FPS C:%.0f D:%.0f Det:%.0f Out:%.0f",
                                       camera_name.c_str(), color_fps, depth_fps, detection_fps, display_fps);
    int line_y = hud_y + hud_h - 6;
    fluent::text::draw(output_image, fps_line, cv::Point(hud_x + 8, line_y), text_color, std::max(0.35, hud_font_scale_ * 0.85), 1);

    // 以降の行は黒帯の下から積む
    int y_offset = hud_y + hud_h + 6;
    
    // 2行目: フレーム番号、検出数、分析時間
    y_offset += 20  ;
    std::string info_line;
    double analysis_ms = last_analysis_time_ms_.load();
    if (snapshot_list.empty()) {
        info_line = cv::format("フレーム: %lu | 未検出 | 分析: %.1fms", 
                              (unsigned long)total_frame_count, analysis_ms);
    } else {
        info_line = cv::format("フレーム: %lu | 検出数: %zu | 分析: %.1fms", 
                              (unsigned long)total_frame_count, snapshot_list.size(), analysis_ms);
    }
    cv::Scalar info_color = snapshot_list.empty() ? cv::Scalar(128, 128, 128) : cv::Scalar(0, 255, 0);
    fluent::text::draw(output_image, info_line, cv::Point(hud_x + 8, y_offset),
                info_color, hud_font_scale_, 1);
    
    // 3行目: 点群処理情報
    y_offset += 16;
    float pointcloud_fps = pointcloud_fps_meter_ ? pointcloud_fps_meter_->getCurrentFPS() : 0.0f;
    double pointcloud_ms = last_pointcloud_time_ms_.load();
    std::string pointcloud_line = cv::format("Pointcloud: %.1f FPS | %.1fms", pointcloud_fps, pointcloud_ms);
    if (have_pc && fallback_active) {
        pointcloud_line += " [fallback]";
    }
    fluent::text::draw(output_image, pointcloud_line, cv::Point(hud_x + 8, y_offset),
                text_color, hud_font_scale_, 1);

    // 4行目: システム負荷情報（CPU/MEM のみ。GPUは非表示）
    y_offset += 16;
    static fluent::utils::SystemMonitor sysmon;
    static auto last_sys_update = std::chrono::steady_clock::now();
    static fluent::utils::SystemStats last_stats;
    auto now_sys = std::chrono::steady_clock::now();
    if (now_sys - last_sys_update > std::chrono::milliseconds(1000)) {
        last_sys_update = now_sys;
        last_stats = sysmon.sample();
    }
    // 描画テキスト生成
    std::string sys_line = cv::format("CPU: %.0f%% | MEM: %d/%d MB",
                               last_stats.cpu_usage_pct, last_stats.mem_used_mb, last_stats.mem_total_mb);
    fluent::text::draw(output_image, sys_line, cv::Point(hud_x + 8, y_offset), text_color, hud_font_scale_, 1);
    
    // FPS値に応じて色付け（オプション）
    if (display_fps < 15.0f) {
        // 出力FPSが低い場合の警告
        fluent::text::draw(output_image, "!", cv::Point(580, 25),
                    fps_bad, 0.7, 2);
    }
    
    // スワップしてから publish（出力用は読み取り専用）
    {
        std::lock_guard<std::mutex> lk(canvas_mutex_);
        std::swap(canvas_pub_, canvas_draw_);
    }
    fi::Image &pub_canvas = canvas_pub_;
    // 画像をパブリッシュ（同一pubキャンバスから非圧縮/圧縮を出力）
    fu::Stopwatch pub_sw;
    try {
        if (annotated_image_pub_ && annotated_image_pub_->get_subscription_count() > 0) {
            flr::publish(annotated_image_pub_, pub_canvas, latest_color_image_->header);
        }
        if (annotated_image_compressed_pub_ && annotated_image_compressed_pub_->get_subscription_count() > 0) {
            flr::publish_compressed(annotated_image_compressed_pub_, pub_canvas, latest_color_image_->header, 85, "jpeg");
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception in publishCurrentImage: %s", e.what());
    }
    double pub_ms = pub_sw.elapsed_ms();
    double total_ms = total_sw.elapsed_ms();
    
    // 処理時間が10ms以上の場合のみログ出力
    if (total_ms > 10.0) {
        RCLCPP_WARN(this->get_logger(), 
            "[PUBLISH] Total:%.2fms (Convert:%.2fms, Snapshot:%.2fms, Publish:%.2fms)",
            total_ms, cvt_ms, snap_ms, pub_ms);
    }
}

/**
 * @brief 次のアスパラガスを選択（サービス）
 */
void FvAsparaAnalyzerNode::nextAsparaguService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;  // 未使用パラメータの警告回避
    
    // カーソル移動を使用
    if (moveCursorToNext()) {
        response->success = true;
        response->message = "Selected next asparagus ID: " + std::to_string(selected_aspara_id_);
        RCLCPP_INFO(this->get_logger(), "Next asparagus selected: ID %d", selected_aspara_id_);
    } else {
        response->success = false;
        response->message = "No asparagus detected or failed to select next";
    }
}

/**
 * @brief 前のアスパラガスを選択（サービス）
 */
void FvAsparaAnalyzerNode::prevAsparaguService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;  // 未使用パラメータの警告回避
    
    // カーソル移動を使用
    if (moveCursorToPrev()) {
        response->success = true;
        response->message = "Selected previous asparagus ID: " + std::to_string(selected_aspara_id_);
        RCLCPP_INFO(this->get_logger(), "Previous asparagus selected: ID %d", selected_aspara_id_);
    } else {
        response->success = false;
        response->message = "No asparagus detected or failed to select previous";
    }
}

/**
 * @brief カメラ情報を起動時に1回だけ取得
 * @param camera_info_topic カメラ情報トピック名
 */
void FvAsparaAnalyzerNode::getCameraInfoOnce(const std::string& camera_info_topic)
{
    RCLCPP_WARN(this->get_logger(), "カメラ情報を取得中: %s", camera_info_topic.c_str());
    
    // カメラ情報取得用のワンタイムサブスクライバー
    auto camera_info_received = false;
    
    auto camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), 
        [this, &camera_info_received](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            {
                std::lock_guard<std::mutex> lock(this->aspara_list_mutex_);
                this->latest_camera_info_ = msg;
            }
            camera_info_received = true;
            
            RCLCPP_WARN(this->get_logger(), "カメラ情報を取得しました:");
            RCLCPP_WARN(this->get_logger(), "  Image size: %dx%d", msg->width, msg->height);
            RCLCPP_WARN(this->get_logger(), "  Focal length: fx=%.2f, fy=%.2f", msg->k[0], msg->k[4]);
            RCLCPP_WARN(this->get_logger(), "  Principal point: cx=%.2f, cy=%.2f", msg->k[2], msg->k[5]);
            RCLCPP_WARN(this->get_logger(), "  Distortion model: %s", msg->distortion_model.c_str());
        });
    
    // カメラ情報が取得できるまで待機
    auto start_time = this->now();
    while (!camera_info_received && rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        auto elapsed = (this->now() - start_time).seconds();
        if (elapsed > 1.0) {
            RCLCPP_WARN(this->get_logger(), "カメラノードの起動を待っています... (%.1fs)", elapsed);
            start_time = this->now(); // リセットして1秒ごとにメッセージ
        }
    }
    
    RCLCPP_WARN(this->get_logger(), "カメラ情報の取得が完了しました");
}



} // namespace fv_aspara_analyzer

/**
 * @brief メイン関数
 * @param argc コマンドライン引数の数
 * @param argv コマンドライン引数の配列
 * @return 終了コード
 * @details ROS2ノードの初期化と実行
 * 
 * 実行内容：
 * - ROS2の初期化
 * - アスパラガス解析ノードの作成
 * - ノードの実行（スピン）
 * - 適切な終了処理
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto logger = rclcpp::get_logger("fv_aspara_analyzer_main");
    RCLCPP_WARN(logger, "===== Starting FV Aspara Analyzer Node Main =====");
    
    try {
        auto node = std::make_shared<fv_aspara_analyzer::FvAsparaAnalyzerNode>();
        RCLCPP_WARN(logger, "Node created, starting spin...");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Exception in main: %s", e.what());
    }
    
    RCLCPP_WARN(logger, "===== Shutting down FV Aspara Analyzer Node =====");
    rclcpp::shutdown();
    return 0;
}