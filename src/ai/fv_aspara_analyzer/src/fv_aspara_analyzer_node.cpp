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
#include <fluent.hpp>

namespace fv_aspara_analyzer
{

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
    this->declare_parameter<double>("pointcloud_distance_min", 0.1);           // 点群処理最小距離（10cm）
    this->declare_parameter<double>("pointcloud_distance_max", 2.0);           // 点群処理最大距離（2m）
    this->declare_parameter<double>("aspara_filter_distance", 0.05);           // アスパラガスフィルタ距離（5cm）
    this->declare_parameter<int>("noise_reduction_neighbors", 50);             // 統計フィルタの近傍点数
    this->declare_parameter<double>("noise_reduction_std_dev", 1.0);           // 統計フィルタの標準偏差
    this->declare_parameter<double>("voxel_leaf_size", 0.005);                 // ボクセルサイズ（5mm）
    this->declare_parameter<double>("harvest_min_length", 0.23);               // 収穫最小長さ（23cm）
    this->declare_parameter<double>("harvest_max_length", 0.50);               // 収穫最大長さ（50cm）
    this->declare_parameter<double>("straightness_threshold", 0.7);            // 真っ直ぐ度閾値
    this->declare_parameter<bool>("enable_pointcloud_processing", true);       // ポイントクラウド処理有効化

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

    // ===== トピック名パラメータ宣言 =====
    // デフォルト値は空の文字列にして、設定ファイルから読み込む
    this->declare_parameter<std::string>("detection_topic", "");
    this->declare_parameter<std::string>("pointcloud_topic", "");
    this->declare_parameter<std::string>("camera_info_topic", "");
    this->declare_parameter<std::string>("mask_topic", "");
    this->declare_parameter<std::string>("camera_topic", "");
    this->declare_parameter<std::string>("depth_topic", "");
    this->declare_parameter<std::string>("output_filtered_pointcloud_topic", "");
    this->declare_parameter<std::string>("output_selected_pointcloud_topic", "");
    this->declare_parameter<std::string>("output_annotated_image_topic", "");

    // ===== トピック名取得 =====
    std::string detection_topic = this->get_parameter("detection_topic").as_string();
    std::string pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
    std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
    std::string mask_topic = this->get_parameter("mask_topic").as_string();
    std::string camera_topic = this->get_parameter("camera_topic").as_string();
    std::string depth_topic = this->get_parameter("depth_topic").as_string();
    std::string output_filtered_pointcloud_topic = this->get_parameter("output_filtered_pointcloud_topic").as_string();
    std::string output_selected_pointcloud_topic = this->get_parameter("output_selected_pointcloud_topic").as_string();
    std::string output_annotated_image_topic = this->get_parameter("output_annotated_image_topic").as_string();

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
    RCLCPP_WARN(this->get_logger(), "  Camera info: %s", camera_info_topic.c_str());
    RCLCPP_WARN(this->get_logger(), "  Camera: %s", camera_topic.c_str());
    RCLCPP_WARN(this->get_logger(), "  Depth: %s", depth_topic.c_str());
    RCLCPP_WARN(this->get_logger(), "  Output annotated: %s", output_annotated_image_topic.c_str());

    // ===== サブスクライバー初期化 =====
    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        detection_topic, 10, 
        std::bind(&FvAsparaAnalyzerNode::detectionCallback, this, std::placeholders::_1));

    // 点群サブスクリプションは実際には使用されていない（深度画像から生成するため）
    // pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     pointcloud_topic, 10,
    //     std::bind(&FvAsparaAnalyzerNode::pointcloudCallback, this, std::placeholders::_1));

    // カメラ情報を起動時に1回だけ取得（毎フレーム受信は無駄）
    getCameraInfoOnce(camera_info_topic);

    mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        mask_topic, 10,
        std::bind(&FvAsparaAnalyzerNode::maskCallback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic, 10,
        std::bind(&FvAsparaAnalyzerNode::imageCallback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic, 10,
        std::bind(&FvAsparaAnalyzerNode::depthCallback, this, std::placeholders::_1));

    // ===== パブリッシャー初期化 =====
    filtered_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_filtered_pointcloud_topic, 10);
    
    // 選択中のアスパラガスの点群パブリッシャー（最終結果）
    selected_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_selected_pointcloud_topic, 10);
    
    annotated_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        output_annotated_image_topic, 10);

    // ===== サービス初期化 =====
    next_asparagus_service_ = this->create_service<std_srvs::srv::Trigger>(
        "next_asparagus",
        std::bind(&FvAsparaAnalyzerNode::nextAsparaguService, this, std::placeholders::_1, std::placeholders::_2));
    
    prev_asparagus_service_ = this->create_service<std_srvs::srv::Trigger>(
        "prev_asparagus",
        std::bind(&FvAsparaAnalyzerNode::prevAsparaguService, this, std::placeholders::_1, std::placeholders::_2));

    // ===== TF2（座標変換）初期化 =====
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
    // ===== データ初期化 =====
    selected_aspara_id_ = -1;  // 未選択状態
    selected_pointcloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    
    // selected_aspara_id_の初期化
    selected_aspara_id_ = -1;

    // ===== フォント初期化 =====
    RCLCPP_INFO(this->get_logger(), "Japanese font support is now built into the new Fluent API");
    
    // FPS測定用メーターを初期化
    color_fps_meter_ = std::make_unique<fluent::utils::FPSMeter>(10);
    depth_fps_meter_ = std::make_unique<fluent::utils::FPSMeter>(10);
    detection_fps_meter_ = std::make_unique<fluent::utils::FPSMeter>(10);
    
    // 検出処理時間計測用（StopWatchはデフォルトコンストラクタで初期化済み）
    // detection_stopwatch_は自動初期化されるのでここでの明示的な初期化は不要
    
    // ===== 非同期点群処理初期化 =====
    analyzer_thread_ = std::make_unique<AnalyzerThread>(this);
    RCLCPP_INFO(this->get_logger(), "Aspara analyzer thread initialized");
    
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
    // 全体処理時間計測開始
    auto callback_start = std::chrono::high_resolution_clock::now();
    detection_stopwatch_.reset();
    
    // FPS計測
    if (detection_fps_meter_) {
        detection_fps_meter_->tick(this->now());
    }
    
    // 必要なデータの可用性チェック
    if (!latest_depth_image_ || !latest_camera_info_) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Required data not available yet. Waiting for depth and camera info...");
        return;
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
        
        // AsparaSelectionで最適な候補を選択
        if (aspara_selection_.getSelectedAsparaId() == -1) {
            // 初回選択時は最適な候補を自動選択
            int best_id = aspara_selection_.selectBestCandidate(aspara_list_);
            aspara_selection_.setSelectedAsparaId(best_id);
        }
        selected_aspara_id_ = aspara_selection_.getSelectedAsparaId();
        
        // 選択中アスパラのみ穂情報を関連付け（ロック時間短縮）
        if (selected_aspara_id_ != -1) {
            for (auto& aspara : aspara_list_) {
                if (aspara.id == selected_aspara_id_) {
                    associateAsparagusParts(msg, aspara);
                    // 非同期処理に送信
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
    
    // 全体処理時間
    auto callback_end = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(callback_end - callback_start).count();
    
    // 詳細ログ出力
    RCLCPP_INFO(this->get_logger(), 
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
// 点群コールバック（現在未使用 - 深度画像から直接生成）
// void FvAsparaAnalyzerNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
// {
//     RCLCPP_WARN_ONCE(this->get_logger(), "Pointcloud callback received first pointcloud");
//     latest_pointcloud_ = msg;
// }

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
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        latest_mask_ = cv_ptr->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
    }
}

/**
 * @brief カラー画像のコールバック関数
 * @param msg カラー画像メッセージ
 * @details 可視化用のカラー画像を受信
 */
void FvAsparaAnalyzerNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_WARN_ONCE(this->get_logger(), "Image callback received first image");
    {
        std::lock_guard<std::mutex> lock(image_data_mutex_);  // 分離したmutexを使用
        latest_color_image_ = msg;
    }
    
    // FPS計測
    if (color_fps_meter_) {
        color_fps_meter_->tick(this->now());
    }
    
    // 重い点群処理はAnalyzerThreadで非同期実行される
    // imageCallbackでは軽い画像出力のみ実行
    
    // 常に画像を出力する
    // 検出結果がある場合はオーバーレイ付き、ない場合は元画像をそのまま出力
    publishCurrentImage();
}

/**
 * @brief 深度画像のコールバック関数
 * @param msg 深度画像メッセージ
 * @details 深度画像を保存して効率的な処理に使用
 */
void FvAsparaAnalyzerNode::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_WARN_ONCE(this->get_logger(), "Depth callback received first depth image");
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
 * @brief 現在の画像を出力（検出結果がある場合はオーバーレイ付き）
 * @details 常に画像を出力し、検出結果がある場合はオーバーレイを追加
 */
void FvAsparaAnalyzerNode::publishCurrentImage()
{
    auto pub_start = std::chrono::high_resolution_clock::now();
    
    if (!latest_color_image_) {
        return;
    }
    
    // カラー画像をcv::Matに変換
    auto cvt_start = std::chrono::high_resolution_clock::now();
    cv::Mat color_image;
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(latest_color_image_, sensor_msgs::image_encodings::BGR8);
        color_image = cv_ptr->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        return;
    }
    auto cvt_end = std::chrono::high_resolution_clock::now();
    double cvt_ms = std::chrono::duration<double, std::milli>(cvt_end - cvt_start).count();
    
    cv::Mat output_image = color_image.clone();
    
    // FPS計算用（FPSMeterを使用）
    static auto last_time = std::chrono::high_resolution_clock::now();
    static auto last_detection_time = std::chrono::high_resolution_clock::now();
    
    // 現在時刻取得
    auto current_time = std::chrono::high_resolution_clock::now();
    auto delta_time = std::chrono::duration<float>(current_time - last_time).count();
    last_time = current_time;
    
    // アニメーション更新（実際のFPSに基づいて調整）
    // delta_timeを使って、フレームレート非依存のスムージングを実現
    const float target_smoothing_time = 0.1f;  // 100msで目標値に収束
    const float animation_speed = std::min(1.0f, delta_time / target_smoothing_time);
    
    // 検出結果の描画
    // 1) ロック下でスナップショット取得（最小限）
    auto snap_start = std::chrono::high_resolution_clock::now();
    std::vector<AsparaInfo> snapshot_list;
    int snapshot_selected_id = -1;
    {
        std::lock_guard<std::mutex> lock(aspara_list_mutex_);
        if (!aspara_list_.empty() && latest_camera_info_) {
            // 選択IDの整合性チェックのみ
            if (selected_aspara_id_ == -1 ||
                std::find_if(aspara_list_.begin(), aspara_list_.end(),
                    [this](const AsparaInfo& info) { return info.id == selected_aspara_id_; }) == aspara_list_.end()) {
                selected_aspara_id_ = aspara_list_[0].id;
            }
            // スナップショット取得（コピー）
            snapshot_list = aspara_list_;
            snapshot_selected_id = selected_aspara_id_;
        }
    }  // ロック解放
    auto snap_end = std::chrono::high_resolution_clock::now();
    double snap_ms = std::chrono::duration<double, std::milli>(snap_end - snap_start).count();
    
    // 2) ロック外でスムージング計算
    for (auto& aspara_info : snapshot_list) {
        if (aspara_info.is_new) {
            aspara_info.smooth_bbox = aspara_info.bounding_box_2d;
            aspara_info.animation_alpha = 0.0f;
            aspara_info.is_new = false;
        } else {
            auto lerp = [](float a, float b, float t) { return a + (b - a) * t; };
            aspara_info.smooth_bbox.x = lerp(aspara_info.smooth_bbox.x, aspara_info.bounding_box_2d.x, animation_speed);
            aspara_info.smooth_bbox.y = lerp(aspara_info.smooth_bbox.y, aspara_info.bounding_box_2d.y, animation_speed);
            aspara_info.smooth_bbox.width = lerp(aspara_info.smooth_bbox.width, aspara_info.bounding_box_2d.width, animation_speed);
            aspara_info.smooth_bbox.height = lerp(aspara_info.smooth_bbox.height, aspara_info.bounding_box_2d.height, animation_speed);
            aspara_info.animation_alpha = std::min(1.0f, aspara_info.animation_alpha + delta_time * 3.0f);
        }
        aspara_info.frame_count++;
    }

    if (!snapshot_list.empty() && latest_camera_info_) {
        last_detection_time = current_time;
        
        // 各検出されたアスパラガスを描画
        for (auto& aspara_info : snapshot_list) {
            bool is_selected = (aspara_info.id == snapshot_selected_id);
            
            // 描画設定
            cv::Scalar color = is_selected ? cv::Scalar(0, 255, 0) : cv::Scalar(128, 128, 128);
            int thickness = is_selected ? 2 : 1;
            
            // バウンディングボックス描画
            cv::rectangle(output_image, aspara_info.smooth_bbox, color, thickness);
            
            // ラベル描画（選択中のみ詳細表示）
            if (is_selected) {
                std::string label = cv::format("#%d (%.0f%%)", aspara_info.id, aspara_info.confidence * 100);
                int baseline;
                cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseline);
                
                // 背景ボックス
                cv::Point text_pos(aspara_info.smooth_bbox.x, aspara_info.smooth_bbox.y - 5);
                cv::rectangle(output_image, 
                    cv::Rect(text_pos.x, text_pos.y - text_size.height - 3, text_size.width + 6, text_size.height + 6),
                    color, -1);
                
                // テキスト
                cv::putText(output_image, label, cv::Point(text_pos.x + 3, text_pos.y), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
            }
        }
    } else {
        // 未検出の場合
        cv::putText(output_image, "未検出", cv::Point(10, 250), 
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(128, 128, 128), 2);
        selected_aspara_id_ = -1; // 選択解除
    }
    
    // FPS情報を左上に表示
    auto img = Image::from(output_image);  // 一度だけImage作成
    int y_pos = 40;
    
    // 出力FPS計測（static変数で管理）
    static auto last_output_time = std::chrono::high_resolution_clock::now();
    static int output_frame_count = 0;
    static float output_fps = 0.0f;
    output_frame_count++;
    auto output_delta = std::chrono::duration<float>(current_time - last_output_time).count();
    if (output_delta > 1.0f) {
        output_fps = output_frame_count / output_delta;
        output_frame_count = 0;
        last_output_time = current_time;
    }
    
    // カメラFPS（FPSMeterから実測値を取得）
    float actual_color_fps = color_fps_meter_ ? color_fps_meter_->getCurrentFPS() : 0.0f;
    float actual_depth_fps = depth_fps_meter_ ? depth_fps_meter_->getCurrentFPS() : 0.0f;
    float actual_detection_fps = detection_fps_meter_ ? detection_fps_meter_->getCurrentFPS() : 0.0f;
    
    // FPS情報表示
    img.text(cv::format("Color: %.0fFPS", actual_color_fps), 15, y_pos);
    y_pos += 40;
    img.text(cv::format("Depth: %.0fFPS", actual_depth_fps), 15, y_pos);
    y_pos += 40;
    img.text(cv::format("Output: %.1fFPS", output_fps), 15, y_pos);  // 実際の出力FPS
    y_pos += 40;
    img.text(cv::format("物体検知: %.1fFPS", actual_detection_fps), 15, y_pos);
    y_pos += 40;
    
    // 検出処理時間
    double detection_time_ms = detection_stopwatch_.elapsed_ms();
    img.text(cv::format("処理時間: %.1fms", detection_time_ms), 15, y_pos);
    y_pos += 40;
    
    // 検出数の詳細（スナップショットを使用）
    size_t aspara_count = snapshot_list.size();
    size_t spike_count = 0;
    for (const auto& aspara : snapshot_list) {
        spike_count += aspara.spike_parts.size();
    }
    
    std::string detection_text = (aspara_count > 0 || spike_count > 0) 
        ? cv::format("検出数: アスパラ:%zu 穂:%zu", aspara_count, spike_count)
        : "検出数: なし";
    img.text(detection_text, 15, y_pos);
    
    // 選択中のアスパラの分析時間を表示
    if (snapshot_selected_id != -1 && enable_pointcloud_processing_) {
        y_pos += 40;
        for (const auto& aspara : snapshot_list) {
            if (aspara.id == snapshot_selected_id) {
                img.text(cv::format("アスパラ#%d分析: %.1fms", aspara.id, aspara.processing_times.total_ms), 15, y_pos);
                break;
            }
        }
    }
    
    // 画像をパブリッシュ
    auto pub_msg_start = std::chrono::high_resolution_clock::now();
    try {
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
            latest_color_image_->header,
            sensor_msgs::image_encodings::BGR8,
            output_image).toImageMsg();
        
        annotated_image_pub_->publish(*msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception in publishCurrentImage: %s", e.what());
    }
    
    auto pub_end = std::chrono::high_resolution_clock::now();
    double pub_ms = std::chrono::duration<double, std::milli>(pub_end - pub_msg_start).count();
    double total_ms = std::chrono::duration<double, std::milli>(pub_end - pub_start).count();
    
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
    
    std::lock_guard<std::mutex> lock(aspara_list_mutex_);
    if (aspara_list_.empty()) {
        response->success = false;
        response->message = "No asparagus detected";
        return;
    }
    
    // AsparaSelectionで次のアスパラを選択
    int next_id = aspara_selection_.selectNextAsparagus(aspara_list_);
    selected_aspara_id_ = next_id;
    
    if (next_id != -1) {
        response->success = true;
        response->message = "Selected next asparagus ID: " + std::to_string(next_id);
        RCLCPP_INFO(this->get_logger(), "Next asparagus selected: ID %d", next_id);
    } else {
        response->success = false;
        response->message = "Failed to select next asparagus";
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
    
    std::lock_guard<std::mutex> lock(aspara_list_mutex_);
    if (aspara_list_.empty()) {
        response->success = false;
        response->message = "No asparagus detected";
        return;
    }
    
    // AsparaSelectionで前のアスパラを選択
    int prev_id = aspara_selection_.selectPrevAsparagus(aspara_list_);
    selected_aspara_id_ = prev_id;
    
    if (prev_id != -1) {
        response->success = true;
        response->message = "Selected previous asparagus ID: " + std::to_string(prev_id);
        RCLCPP_INFO(this->get_logger(), "Previous asparagus selected: ID %d", prev_id);
    } else {
        response->success = false;
        response->message = "Failed to select previous asparagus";
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
        camera_info_topic, 1, 
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