/**
 * @file fv_aspara_analyzer_node.cpp
 * @brief アスパラガス解析ノードのメイン実装ファイル
 * @details 3D点群データと2D検出結果を統合してアスパラガスの品質評価を行う
 * @author Takashi Otsuka
 * @date 2024
 * @version 1.0
 */

#include "fv_aspara_analyzer/fv_aspara_analyzer_node.hpp"
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

    // ===== CallbackGroup初期化（マルチスレッド対応）=====
    image_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    detection_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    RCLCPP_INFO(this->get_logger(), "Callback groups created for multithreading");

    // ===== サブスクライバー初期化 =====
    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        detection_topic, 10, 
        std::bind(&FvAsparaAnalyzerNode::detectionCallback, this, std::placeholders::_1));

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic, 10,
        std::bind(&FvAsparaAnalyzerNode::pointcloudCallback, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, 10,
        std::bind(&FvAsparaAnalyzerNode::cameraInfoCallback, this, std::placeholders::_1));

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
    shutdown_flag_ = false;
    processing_in_progress_ = false;
    
    // ワーカースレッド開始
    pointcloud_worker_thread_ = std::thread(&FvAsparaAnalyzerNode::pointcloudWorkerLoop, this);
    RCLCPP_INFO(this->get_logger(), "Pointcloud processing worker thread started");
    
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
FvAsparaAnalyzerNode::~FvAsparaAnalyzerNode() 
{
    // ワーカースレッド終了処理
    shutdown_flag_ = true;
    pointcloud_cv_.notify_all();  // 待機中のスレッドを起こす
    
    if (pointcloud_worker_thread_.joinable()) {
        pointcloud_worker_thread_.join();
        RCLCPP_INFO(this->get_logger(), "Pointcloud processing worker thread terminated");
    }
}

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
    RCLCPP_WARN(this->get_logger(), "===== Detection callback START: %zu detections =====", msg->detections.size());
    
    // 処理時間計測開始
    detection_stopwatch_.reset();
    
    // FPS計測
    if (detection_fps_meter_) {
        detection_fps_meter_->tick(this->now());
    }
    
    // 深度画像とカメラ情報の可用性チェック
    if (!latest_depth_image_ || !latest_camera_info_) {
        RCLCPP_WARN(this->get_logger(), "Depth image or camera info not available yet. Depth:%s, CI:%s", 
                   latest_depth_image_ ? "OK" : "NULL", latest_camera_info_ ? "OK" : "NULL");
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

    // AsparaSelectionでID管理とスムーズアニメーションを実行
    aspara_list_ = aspara_selection_.updateAsparaList(new_detections, aspara_list_);

    // 最高信頼度のアスパラガスを処理
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
        
        // 各アスパラに穂情報を関連付け
        for (auto& aspara : aspara_list_) {
            associateAsparagusParts(msg, aspara);
        }
        
        // 検出情報更新時は点群処理はスキップ（imageCallbackで毎フレーム実行）
    }
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
void FvAsparaAnalyzerNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_WARN_ONCE(this->get_logger(), "Pointcloud callback received first pointcloud");
    latest_pointcloud_ = msg;
}

/**
 * @brief カメラ情報のコールバック関数
 * @param msg カメラキャリブレーション情報
 * @details カメラの内部パラメータを受信し、3D-2D変換に使用
 */
void FvAsparaAnalyzerNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    RCLCPP_WARN_ONCE(this->get_logger(), "Camera info callback received first camera info");
    latest_camera_info_ = msg;
}

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
    latest_color_image_ = msg;
    
    // FPS計測
    if (color_fps_meter_) {
        color_fps_meter_->tick(this->now());
    }
    
    // アスパラの非同期点群処理をキューに追加（メインスレッドは軽量な処理のみ）
    if (enable_pointcloud_processing_ && !aspara_list_.empty()) {
        for (const auto& aspara : aspara_list_) {
            // 選択中のアスパラは必ず分析、その他は余裕があれば分析
            bool should_analyze = (aspara.id == selected_aspara_id_) || (selected_aspara_id_ == -1);
            
            if (should_analyze) {
                // 重い点群処理は非同期キューに追加（カクつき回避）
                enqueueAsparaguProcessing(aspara);
                
                RCLCPP_DEBUG(this->get_logger(), "Enqueued asparagus ID %d for async processing", aspara.id);
            }
        }
    }
    
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
    latest_depth_image_ = msg;
    
    // FPS計測
    if (depth_fps_meter_) {
        depth_fps_meter_->tick(this->now());
    }
}

/**
 * @brief 深度画像から効率的に点群を生成
 * @param bbox 2Dバウンディングボックス
 * @return フィルタリング済み点群
 * @details バウンディングボックス内の深度ピクセルのみを3D点に変換
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr FvAsparaAnalyzerNode::extractPointCloudFromDepth(
    const cv::Rect& bbox)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    if (!latest_depth_image_ || !latest_camera_info_ || !latest_color_image_) {
        return cloud;
    }
    
    // 深度画像とカラー画像をOpenCV形式に変換
    cv::Mat depth_image;
    cv::Mat color_image;
    
    try {
        // 深度画像の変換
        cv_bridge::CvImagePtr depth_ptr;
        if (latest_depth_image_->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            depth_ptr = cv_bridge::toCvCopy(latest_depth_image_, sensor_msgs::image_encodings::TYPE_16UC1);
            depth_image = depth_ptr->image;
        } else if (latest_depth_image_->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            depth_ptr = cv_bridge::toCvCopy(latest_depth_image_, sensor_msgs::image_encodings::TYPE_32FC1);
            depth_ptr->image.convertTo(depth_image, CV_16UC1, 1000.0); // メートルをミリメートルに変換
        } else {
            RCLCPP_WARN(this->get_logger(), "Unsupported depth encoding: %s", latest_depth_image_->encoding.c_str());
            return cloud;
        }
        
        // カラー画像の変換
        cv_bridge::CvImagePtr color_ptr = cv_bridge::toCvCopy(latest_color_image_, sensor_msgs::image_encodings::BGR8);
        color_image = color_ptr->image;
        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception in extractPointCloudFromDepth: %s", e.what());
        return cloud;
    }
    
    // 画像サイズの一致確認
    if (depth_image.rows != color_image.rows || depth_image.cols != color_image.cols) {
        RCLCPP_WARN(this->get_logger(), 
                    "Depth and color image size mismatch: depth(%dx%d) vs color(%dx%d)",
                    depth_image.cols, depth_image.rows, color_image.cols, color_image.rows);
        return cloud;
    }
    
    // カメラパラメータの取得と有効性チェック
    double fx = latest_camera_info_->k[0];
    double fy = latest_camera_info_->k[4];
    double cx = latest_camera_info_->k[2];
    double cy = latest_camera_info_->k[5];
    
    // 0除算の回避
    if (fx <= 0.0 || fy <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid camera parameters: fx=%.3f, fy=%.3f", fx, fy);
        return cloud;
    }
    
    // バウンディングボックスの有効性確認と範囲クリップ
    if (bbox.width <= 0 || bbox.height <= 0) {
        RCLCPP_WARN(this->get_logger(), "Invalid bounding box size: %dx%d", bbox.width, bbox.height);
        return cloud;
    }
    
    int x_start = std::max(0, bbox.x);
    int y_start = std::max(0, bbox.y);
    int x_end = std::min(depth_image.cols, bbox.x + bbox.width);
    int y_end = std::min(depth_image.rows, bbox.y + bbox.height);
    
    // クリップ後のサイズ確認
    if (x_start >= x_end || y_start >= y_end) {
        RCLCPP_WARN(this->get_logger(), 
                    "Bounding box outside image bounds: bbox(%d,%d,%d,%d), image(%dx%d)",
                    bbox.x, bbox.y, bbox.width, bbox.height, depth_image.cols, depth_image.rows);
        return cloud;
    }
    
    // バウンディングボックス内のピクセルのみを処理
    for (int v = y_start; v < y_end; v++) {
        for (int u = x_start; u < x_end; u++) {
            // 深度値を取得（単位：mm）
            uint16_t depth_mm = depth_image.at<uint16_t>(v, u);
            
            // 無効な深度値をスキップ
            if (depth_mm == 0) continue;
            
            // メートルに変換
            float depth_m = depth_mm / 1000.0f;
            
            // 距離フィルタリング
            if (depth_m < pointcloud_distance_min_ || depth_m > pointcloud_distance_max_) {
                continue;
            }
            
            // 3D座標を計算（カメラ座標系）
            pcl::PointXYZRGB point;
            point.z = depth_m;
            point.x = (u - cx) * depth_m / fx;
            point.y = (v - cy) * depth_m / fy;
            
            // カラー情報を追加
            cv::Vec3b color = color_image.at<cv::Vec3b>(v, u);
            point.b = color[0];
            point.g = color[1];
            point.r = color[2];
            
            cloud->points.push_back(point);
        }
    }
    
    // 点群のメタデータを設定
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    
    RCLCPP_DEBUG(this->get_logger(), "Extracted %zu points from depth image bbox", cloud->points.size());
    
    return cloud;
}

/**
 * @brief 3D点を2D画像座標に投影
 * @param point 3D点
 * @param camera_info カメラ情報
 * @return 2D画像座標
 * @details カメラの内部パラメータを使用して3D-2D変換
 * 
 * 変換処理：
 * - カメラ内部パラメータ（fx, fy, cx, cy）を使用
 * - レンズ歪み補正を適用（k1, k2, p1, p2, k3）
 * - 正規化座標での歪み補正
 */
cv::Point2f FvAsparaAnalyzerNode::project3DTo2D(
    const pcl::PointXYZRGB& point,
    const sensor_msgs::msg::CameraInfo& camera_info)
{
    // カメラ内部パラメータ
    double fx = camera_info.k[0];  // K[0,0] - 焦点距離X
    double fy = camera_info.k[4];  // K[1,1] - 焦点距離Y
    double cx = camera_info.k[2];  // K[0,2] - 光学中心X
    double cy = camera_info.k[5];  // K[1,2] - 光学中心Y

    // 3D点を2Dに投影
    double u = (point.x * fx / point.z) + cx;
    double v = (point.y * fy / point.z) + cy;

    // レンズ歪み補正（必要に応じて）
    if (!camera_info.d.empty()) {
        double k1 = camera_info.d[0];  // 半径方向歪み係数1
        double k2 = camera_info.d[1];  // 半径方向歪み係数2
        double p1 = camera_info.d[2];  // 接線方向歪み係数1
        double p2 = camera_info.d[3];  // 接線方向歪み係数2
        double k3 = (camera_info.d.size() > 4) ? camera_info.d[4] : 0.0;  // 半径方向歪み係数3

        // 正規化座標
        double x_norm = (u - cx) / fx;
        double y_norm = (v - cy) / fy;
        
        double r2 = x_norm * x_norm + y_norm * y_norm;  // 半径の2乗
        double r4 = r2 * r2;                            // 半径の4乗
        double r6 = r4 * r2;                            // 半径の6乗

        // 半径方向歪み
        double radial_distortion = 1 + k1 * r2 + k2 * r4 + k3 * r6;

        // 接線方向歪み
        double dx = 2 * p1 * x_norm * y_norm + p2 * (r2 + 2 * x_norm * x_norm);
        double dy = p1 * (r2 + 2 * y_norm * y_norm) + 2 * p2 * x_norm * y_norm;

        // 歪み補正を適用
        x_norm = x_norm * radial_distortion + dx;
        y_norm = y_norm * radial_distortion + dy;

        // ピクセル座標に戻す
        u = x_norm * fx + cx;
        v = y_norm * fy + cy;
    }

    return cv::Point2f(static_cast<float>(u), static_cast<float>(v));
}

/**
 * @brief バウンディングボックスで点群をフィルタリング
 * @param input_cloud 入力点群
 * @param bbox 2Dバウンディングボックス
 * @param camera_info カメラ情報
 * @return フィルタリング済み点群
 * @details 2D検出領域に対応する3D点群を抽出
 * 
 * フィルタリング処理：
 * - 無効点の除去（NaN、無限大値）
 * - 距離フィルタリング（設定範囲内）
 * - 3D-2D投影によるバウンディングボックス内の点抽出
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr FvAsparaAnalyzerNode::filterPointCloudByBoundingBox(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
    const cv::Rect& bbox,
    const sensor_msgs::msg::CameraInfo& camera_info)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (const auto& point : input_cloud->points) {
        // 無効点をスキップ
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }

        // 距離フィルタリング
        float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (distance < pointcloud_distance_min_ || distance > pointcloud_distance_max_) {
            continue;
        }

        // 2Dに投影
        cv::Point2f projected = project3DTo2D(point, camera_info);

        // バウンディングボックス内かチェック
        if (bbox.contains(cv::Point(static_cast<int>(projected.x), static_cast<int>(projected.y)))) {
            filtered_cloud->points.push_back(point);
        }
    }

    // 点群のメタデータを設定
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = false;

    return filtered_cloud;
}

/**
 * @brief 点群のノイズ除去処理
 * @param input_cloud 入力点群
 * @return ノイズ除去済み点群
 * @details 統計的外れ値除去とボクセルグリッドフィルタを適用
 * 
 * ノイズ除去処理：
 * - ボクセルグリッドフィルタ（ダウンサンプリング）
 * - 統計的外れ値除去（ノイズ除去）
 * - 点群密度に応じた適応的処理
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr FvAsparaAnalyzerNode::applyNoiseReduction(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud)
{
    // 点群サイズチェック
    if (input_cloud->points.size() < 10) {
        RCLCPP_WARN(this->get_logger(), "Too few points for noise reduction");
        return input_cloud;
    }

    // ステップ1: ボクセルグリッドフィルタ（ダウンサンプリング）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_filter.filter(*voxel_filtered);

    if (voxel_filtered->points.size() < 10) {
        RCLCPP_WARN(this->get_logger(), "Too few points after voxel filtering");
        return voxel_filtered;
    }

    // ステップ2: 統計的外れ値除去
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr denoised_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
    statistical_filter.setInputCloud(voxel_filtered);
    statistical_filter.setMeanK(noise_reduction_neighbors_);
    statistical_filter.setStddevMulThresh(noise_reduction_std_dev_);
    statistical_filter.filter(*denoised_cloud);

    // デバッグログ
    RCLCPP_DEBUG(this->get_logger(), 
                 "Noise reduction: %zu -> %zu -> %zu points",
                 input_cloud->points.size(),
                 voxel_filtered->points.size(),
                 denoised_cloud->points.size());

    return denoised_cloud;
}

/**
 * @brief アスパラガスの根元位置を推定
 * @param aspara_cloud アスパラガス点群
 * @return 根元の3D座標
 * @details 点群の下端部から根元位置を推定
 * 
 * 推定方法：
 * - Z座標の最小値を根元位置として使用
 * - 地面との交点計算は将来的に実装予定
 */
geometry_msgs::msg::Point FvAsparaAnalyzerNode::estimateRootPosition(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aspara_cloud)
{
    geometry_msgs::msg::Point root_position;
    
    if (aspara_cloud->points.empty()) {
        return root_position;
    }

    // Z座標の最小値を根元位置として使用
    float min_z = std::numeric_limits<float>::max();
    pcl::PointXYZRGB root_point;
    
    for (const auto& point : aspara_cloud->points) {
        if (point.z < min_z) {
            min_z = point.z;
            root_point = point;
        }
    }

    root_position.x = root_point.x;
    root_position.y = root_point.y;
    root_position.z = root_point.z;

    return root_position;
}

/**
 * @brief アスパラガスの真っ直ぐ度を計算
 * @param aspara_cloud アスパラガス点群
 * @return 真っ直ぐ度（0.0-1.0）
 * @details PCAの第1主成分の分散から直線性を評価
 * 
 * 計算方法：
 * - 主成分分析（PCA）を実行
 * - 第1主成分の固有値の比率を計算
 * - 高値ほど直線的（真っ直ぐ）
 */
float FvAsparaAnalyzerNode::calculateStraightness(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aspara_cloud)
{
    if (aspara_cloud->points.size() < 3) {
        return 0.0f;
    }

    // PCAによる真っ直ぐ度計算
    pcl::PCA<pcl::PointXYZRGB> pca;
    pca.setInputCloud(aspara_cloud);
    
    Eigen::Vector3f eigenvalues = pca.getEigenValues();
    
    // 真っ直ぐ度 = 最大固有値 / 全固有値の和
    if (eigenvalues[1] == 0.0f) {
        return 1.0f;  // 完全な直線
    }
    
    float straightness = eigenvalues[0] / (eigenvalues[0] + eigenvalues[1] + eigenvalues[2]);
    return std::min(1.0f, straightness);
}

/**
 * @brief アスパラガスの長さを計算
 * @param aspara_cloud アスパラガス点群
 * @return 長さ（メートル）
 * @details 点群の主軸方向の最大距離を計算
 * 
 * 計算方法：
 * - Z軸方向の最大・最小値を取得
 * - その差を長さとして使用
 * - 将来的には曲線長の計算を実装予定
 */
float FvAsparaAnalyzerNode::calculateLength(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aspara_cloud)
{
    if (aspara_cloud->points.size() < 2) {
        return 0.0f;
    }

    // Z軸方向の最大・最小値を取得
    float min_val = std::numeric_limits<float>::max();
    float max_val = std::numeric_limits<float>::lowest();

    for (const auto& point : aspara_cloud->points) {
        float z = point.z;  // Z軸を主軸として仮定
        if (z < min_val) min_val = z;
        if (z > max_val) max_val = z;
    }

    return std::abs(max_val - min_val);
}

/**
 * @brief PCA直線を生成
 * @param aspara_cloud アスパラガス点群
 * @return PCA直線の点群
 * @details 主成分分析による直線近似を生成
 * 
 * 生成処理：
 * - PCAで主成分軸を計算
 * - データの投影範囲を取得
 * - 直線上に等間隔でポイントを生成
 * - 可視化用のデバッグデータとして使用
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr FvAsparaAnalyzerNode::generatePCALine(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aspara_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pca_line(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (aspara_cloud->points.size() < 3) {
        return pca_line;
    }

    // PCA計算
    pcl::PCA<pcl::PointXYZRGB> pca;
    pca.setInputCloud(aspara_cloud);
    
    // 主成分軸とデータの範囲を取得
    Eigen::Vector4f centroid = pca.getMean();
    Eigen::Vector3f principal_axis = pca.getEigenVectors().col(0); // 第1主成分
    
    // データの投影範囲を計算
    float min_proj = std::numeric_limits<float>::max();
    float max_proj = std::numeric_limits<float>::lowest();
    
    for (const auto& point : aspara_cloud->points) {
        Eigen::Vector3f p(point.x, point.y, point.z);
        Eigen::Vector3f centered = p - centroid.head<3>();
        float projection = centered.dot(principal_axis);
        
        if (projection < min_proj) min_proj = projection;
        if (projection > max_proj) max_proj = projection;
    }
    
    // PCAライン上に等間隔でポイントを生成
    int num_points = 50; // ライン解像度
    for (int i = 0; i < num_points; ++i) {
        float t = min_proj + (max_proj - min_proj) * i / (num_points - 1);
        Eigen::Vector3f line_point = centroid.head<3>() + t * principal_axis;
        
        pcl::PointXYZ point;
        point.x = line_point.x();
        point.y = line_point.y();
        point.z = line_point.z();
        
        pca_line->points.push_back(point);
    }
    
    // 点群のメタデータを設定
    pca_line->width = pca_line->points.size();
    pca_line->height = 1;
    pca_line->is_dense = false;
    
    return pca_line;
}

/**
 * @brief アスパラガス情報の処理
 * @param aspara_info アスパラガス情報
 * @details 品質評価と収穫判定の実行
 * 
 * 処理フロー：
 * - 点群フィルタリング
 * - ノイズ除去
 * - 品質評価（長さ、真っ直ぐ度）
 * - 収穫適性判定
 * - 結果の可視化とパブリッシュ
 */
void FvAsparaAnalyzerNode::processAsparagus(const AsparaInfo& aspara_info)
{
    RCLCPP_WARN(this->get_logger(), "Processing asparagus %d with confidence %.3f", aspara_info.id, aspara_info.confidence);
    
    try {
        // 全体処理時間の計測開始
        fluent::utils::Stopwatch total_stopwatch;
    
    // mutableなコピーを作成して処理時間を記録できるようにする
    AsparaInfo mutable_aspara_info = aspara_info;
    
    if (!latest_color_image_) {
        RCLCPP_WARN(this->get_logger(), "Color image not available yet");
        return;
    }

    // 深度画像から効率的に点群を抽出（バウンディングボックス内のみ）
    fluent::utils::Stopwatch filter_stopwatch;
    auto filtered_cloud = extractPointCloudFromDepth(aspara_info.bounding_box_2d);
    mutable_aspara_info.processing_times.filter_bbox_ms = filter_stopwatch.elapsed_ms();
    
    if (filtered_cloud->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "No points found in bounding box");
        return;
    }

    // ノイズ除去
    fluent::utils::Stopwatch noise_stopwatch;
    auto denoised_cloud = applyNoiseReduction(filtered_cloud);
    mutable_aspara_info.processing_times.noise_reduction_ms = noise_stopwatch.elapsed_ms();

    if (denoised_cloud->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "No points left after noise reduction");
        return;
    }

    // アスパラガス特性を計算
    fluent::utils::Stopwatch measurement_stopwatch;
    auto root_position = estimateRootPosition(denoised_cloud);
    float straightness = calculateStraightness(denoised_cloud);
    float length = calculateLength(denoised_cloud);
    mutable_aspara_info.processing_times.measurement_ms = measurement_stopwatch.elapsed_ms();
    
    // 可視化用PCAラインを生成
    fluent::utils::Stopwatch pca_stopwatch;
    auto pca_line = generatePCALine(denoised_cloud);
    mutable_aspara_info.processing_times.pca_calculation_ms = pca_stopwatch.elapsed_ms();

    // 収穫適性を判定
    bool is_harvestable = (length >= harvest_min_length_ && 
                          length <= harvest_max_length_ && 
                          straightness >= straightness_threshold_);

    // カラー画像をcv::Matに変換
    cv::Mat color_image;
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(latest_color_image_, sensor_msgs::image_encodings::BGR8);
        color_image = cv_ptr->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        return;
    }

    // 可視化処理時間の計測
    fluent::utils::Stopwatch vis_stopwatch;
    
    // 結果をパブリッシュ
    publishFilteredPointCloud(denoised_cloud, latest_pointcloud_->header.frame_id, aspara_info.id);
    publishRootTF(root_position, latest_pointcloud_->header.frame_id, aspara_info.id);
    
    // 全体処理時間を記録
    mutable_aspara_info.processing_times.total_ms = total_stopwatch.elapsed_ms();
    mutable_aspara_info.processing_times.visualization_ms = vis_stopwatch.elapsed_ms();
    
    publishAnnotatedImage(color_image, mutable_aspara_info, denoised_cloud, pca_line, length, straightness, is_harvestable);

    // 結果ログ出力（処理時間含む）
    RCLCPP_INFO(this->get_logger(), 
                "Aspara ID:%d, Confidence:%.2f, Length:%.3fm, Straightness:%.2f, Harvestable:%s, Total:%.1fms (Filter:%.1fms, Noise:%.1fms, PCA:%.1fms)",
                aspara_info.id, aspara_info.confidence, length, straightness, 
                is_harvestable ? "YES" : "NO",
                mutable_aspara_info.processing_times.total_ms,
                mutable_aspara_info.processing_times.filter_bbox_ms,
                mutable_aspara_info.processing_times.noise_reduction_ms,
                mutable_aspara_info.processing_times.pca_calculation_ms);
                
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception in processAsparagus (ID:%d): %s", aspara_info.id, e.what());
        return;
    } catch (const pcl::PCLException& e) {
        RCLCPP_ERROR(this->get_logger(), "PCL exception in processAsparagus (ID:%d): %s", aspara_info.id, e.what());
        return;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Standard exception in processAsparagus (ID:%d): %s", aspara_info.id, e.what());
        return;
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown exception in processAsparagus (ID:%d)", aspara_info.id);
        return;
    }
}

/**
 * @brief フィルタリング済み点群をパブリッシュ
 * @param cloud 点群データ
 * @param frame_id 座標系ID
 * @param aspara_id アスパラガスID
 * @details デバッグ用の可視化データを出力
 */
void FvAsparaAnalyzerNode::publishFilteredPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const std::string& frame_id,
    int /*aspara_id*/)
{
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    output_msg.header.stamp = this->now();
    output_msg.header.frame_id = frame_id;
    
    filtered_pointcloud_pub_->publish(output_msg);
}

/**
 * @brief 根元位置のTF座標をパブリッシュ
 * @param root_position 根元位置
 * @param frame_id 座標系ID
 * @param aspara_id アスパラガスID
 * @details ロボット制御用の座標変換情報を出力
 */
void FvAsparaAnalyzerNode::publishRootTF(
    const geometry_msgs::msg::Point& root_position,
    const std::string& frame_id,
    int aspara_id)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = "aspara_" + std::to_string(aspara_id) + "_root";
    
    transform_stamped.transform.translation.x = root_position.x;
    transform_stamped.transform.translation.y = root_position.y;
    transform_stamped.transform.translation.z = root_position.z;
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;
    
    tf_broadcaster_->sendTransform(transform_stamped);
}

/**
 * @brief 現在の画像を出力（検出結果がある場合はオーバーレイ付き）
 * @details 常に画像を出力し、検出結果がある場合はオーバーレイを追加
 */
void FvAsparaAnalyzerNode::publishCurrentImage()
{
    if (!latest_color_image_) {
        return;
    }
    
    // カラー画像をcv::Matに変換
    cv::Mat color_image;
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(latest_color_image_, sensor_msgs::image_encodings::BGR8);
        color_image = cv_ptr->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        return;
    }
    
    cv::Mat output_image = color_image.clone();
    
    // FPS計算用（static変数で状態保持）
    static auto last_time = std::chrono::high_resolution_clock::now();
    static auto last_detection_time = std::chrono::high_resolution_clock::now();
    static float publish_fps = 0.0f;
    static int frame_counter = 0;
    static std::chrono::time_point<std::chrono::high_resolution_clock> fps_update_time = std::chrono::high_resolution_clock::now();
    
    // 現在時刻取得
    auto current_time = std::chrono::high_resolution_clock::now();
    auto delta_time = std::chrono::duration<float>(current_time - last_time).count();
    last_time = current_time;
    
    // FPS計算（1秒ごとに更新）
    frame_counter++;
    auto fps_elapsed = std::chrono::duration<float>(current_time - fps_update_time).count();
    if (fps_elapsed >= 1.0f) {
        publish_fps = frame_counter / fps_elapsed;
        frame_counter = 0;
        fps_update_time = current_time;
    }
    
    // アニメーション更新（60FPS = 16.67ms）
    const float animation_speed = 0.15f; // スムージング係数
    
    // 検出結果の描画
    if (!aspara_list_.empty() && latest_camera_info_) {
        last_detection_time = current_time;
        
        // 選択中のIDが未設定または無効な場合、最初のアスパラガスを選択
        if (selected_aspara_id_ == -1 || 
            std::find_if(aspara_list_.begin(), aspara_list_.end(), 
                [this](const AsparaInfo& info) { return info.id == selected_aspara_id_; }) == aspara_list_.end()) {
            selected_aspara_id_ = aspara_list_[0].id;
        }
        
        // 各検出されたアスパラガスを描画
        for (auto& aspara_info : aspara_list_) {
            bool is_selected = (aspara_info.id == selected_aspara_id_);
            
            // スムーズアニメーション計算
            if (aspara_info.is_new) {
                aspara_info.smooth_bbox = aspara_info.bounding_box_2d;
                aspara_info.animation_alpha = 0.0f;
                aspara_info.is_new = false;
            } else {
                // バウンディングボックスのスムージング（線形補間）
                aspara_info.smooth_bbox.x = aspara_info.smooth_bbox.x + 
                    (aspara_info.bounding_box_2d.x - aspara_info.smooth_bbox.x) * animation_speed;
                aspara_info.smooth_bbox.y = aspara_info.smooth_bbox.y + 
                    (aspara_info.bounding_box_2d.y - aspara_info.smooth_bbox.y) * animation_speed;
                aspara_info.smooth_bbox.width = aspara_info.smooth_bbox.width + 
                    (aspara_info.bounding_box_2d.width - aspara_info.smooth_bbox.width) * animation_speed;
                aspara_info.smooth_bbox.height = aspara_info.smooth_bbox.height + 
                    (aspara_info.bounding_box_2d.height - aspara_info.smooth_bbox.height) * animation_speed;
                
                // フェードインアニメーション
                if (aspara_info.animation_alpha < 1.0f) {
                    aspara_info.animation_alpha = std::min(1.0f, aspara_info.animation_alpha + delta_time * 3.0f);
                }
            }
            
            // 選択中のアスパラガスの点群を保存
            if (is_selected && aspara_info.filtered_pointcloud.data.size() > 0) {
                // 点群をPCLフォーマットに変換
                pcl::fromROSMsg(aspara_info.filtered_pointcloud, *selected_pointcloud_);
                
                // 選択中の点群をパブリッシュ
                sensor_msgs::msg::PointCloud2 selected_msg;
                pcl::toROSMsg(*selected_pointcloud_, selected_msg);
                selected_msg.header = latest_color_image_->header;
                selected_pointcloud_pub_->publish(selected_msg);
            }
            
            // 半透明オーバーレイ用の一時画像
            cv::Mat overlay = output_image.clone();
            
            // 色とアルファ値の設定
            cv::Scalar bbox_color;
            float alpha;
            
            if (is_selected) {
                // 選択中：緑色、半透明
                bbox_color = cv::Scalar(0, 255, 0); // 緑色（BGR）
                alpha = 0.3f * aspara_info.animation_alpha; // より透明に
                
                // バウンディングボックス塗りつぶし（半透明）
                cv::rectangle(overlay, aspara_info.smooth_bbox, bbox_color, -1);
                cv::addWeighted(overlay, alpha, output_image, 1.0 - alpha, 0, output_image);
                
                // 枠線（不透明）
                cv::rectangle(output_image, aspara_info.smooth_bbox, bbox_color, 2);
                
                // クラス名と信頼度を緑色で表示
                int y_offset = aspara_info.smooth_bbox.y - 10;
                if (y_offset < 30) {
                    y_offset = aspara_info.smooth_bbox.y + aspara_info.smooth_bbox.height + 30;
                }
                
                std::string class_name = cv::format("Asparagus #%d", aspara_info.id);
                std::string confidence_text = cv::format("%.1f%%", aspara_info.confidence * 100);
                
                // テキスト描画（影付き、背景なし、サイズ大きめ）
                cv::Point shadow_offset(2, 2);
                
                // クラス名
                cv::putText(output_image, class_name,
                    cv::Point(aspara_info.smooth_bbox.x, y_offset) + shadow_offset, 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 1);
                cv::putText(output_image, class_name,
                    cv::Point(aspara_info.smooth_bbox.x, y_offset), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, bbox_color, 1);
                    
                // 信頼度
                cv::putText(output_image, confidence_text,
                    cv::Point(aspara_info.smooth_bbox.x, y_offset + 30) + shadow_offset, 
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 1);
                cv::putText(output_image, confidence_text,
                    cv::Point(aspara_info.smooth_bbox.x, y_offset + 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, bbox_color, 1);
                    
            } else {
                // 選択されていない：モノクロ、80%透過
                alpha = 0.2f * aspara_info.animation_alpha; // 80%透過 = 20%不透明
                
                // モノクロ変換用のROI
                cv::Rect roi = aspara_info.smooth_bbox;
                // 画像境界チェック
                roi.x = std::max(0, roi.x);
                roi.y = std::max(0, roi.y);
                roi.width = std::min(roi.width, output_image.cols - roi.x);
                roi.height = std::min(roi.height, output_image.rows - roi.y);
                
                if (roi.width > 0 && roi.height > 0) {
                    cv::Mat roi_color = output_image(roi);
                    cv::Mat roi_gray;
                    cv::cvtColor(roi_color, roi_gray, cv::COLOR_BGR2GRAY);
                    cv::cvtColor(roi_gray, roi_gray, cv::COLOR_GRAY2BGR);
                    
                    // モノクロ領域を80%透過でブレンド
                    cv::addWeighted(roi_gray, alpha, roi_color, 1.0 - alpha, 0, roi_color);
                    
                    // グレーの枠線
                    cv::rectangle(output_image, aspara_info.smooth_bbox, cv::Scalar(128, 128, 128), 1);
                }
            }
            
            aspara_info.frame_count++;
        }
    } else {
        // 未検出の場合
        Image::from(output_image).text("未検出", 10, 250);
        selected_aspara_id_ = -1; // 選択解除
    }
    
    // FPS情報を左上に表示（位置を調整して重ならないように）
    cv::Point text_pos(15, 40);  // 左に余白、上にも余裕を持たせる
    
    // カメラFPS（FPSMeterから実測値を取得）
    float actual_color_fps = color_fps_meter_ ? color_fps_meter_->getCurrentFPS() : 0.0f;
    float actual_depth_fps = depth_fps_meter_ ? depth_fps_meter_->getCurrentFPS() : 0.0f;
    float actual_detection_fps = detection_fps_meter_ ? detection_fps_meter_->getCurrentFPS() : 0.0f;
    
    Image::from(output_image).text(cv::format("Color: %.0fFPS", actual_color_fps), text_pos.x, text_pos.y);
    
    text_pos.y += 40;  // 行間を広げる
    Image::from(output_image).text(cv::format("Depth: %.0fFPS", actual_depth_fps), text_pos.x, text_pos.y);
    
    // 出力FPS
    text_pos.y += 40;
    Image::from(output_image).text(cv::format("Output: %.1fFPS", publish_fps), text_pos.x, text_pos.y);
    
    // 物体検知FPS
    text_pos.y += 40;
    Image::from(output_image).text(cv::format("物体検知: %.1fFPS", actual_detection_fps), text_pos.x, text_pos.y);
        
    // 検出処理時間
    text_pos.y += 40;
    double detection_time_ms = detection_stopwatch_.elapsed_ms();
    Image::from(output_image).text(cv::format("処理時間: %.1fms", detection_time_ms), text_pos.x, text_pos.y);
    
    // 検出数の詳細（アスパラと稂を分けて表示）
    text_pos.y += 40;
    
    // アスパラガス本体と穂の数を数える
    size_t aspara_count = aspara_list_.size();
    size_t spike_count = 0;
    
    // 各アスパラの穂情報から穂数を計算
    for (const auto& aspara : aspara_list_) {
        spike_count += aspara.spike_parts.size();
    }
    
    std::string detection_text;
    if (aspara_count > 0 || spike_count > 0) {
        detection_text = cv::format("検出数: アスパラ:%zu 穂:%zu", aspara_count, spike_count);
    } else {
        detection_text = "検出数: なし";
    }
    
    Image::from(output_image).text(detection_text, text_pos.x, text_pos.y);
    
    // 選択中のアスパラの分析時間を表示
    if (selected_aspara_id_ != -1 && enable_pointcloud_processing_) {
        text_pos.y += 40;
        for (const auto& aspara : aspara_list_) {
            if (aspara.id == selected_aspara_id_) {
                double analysis_time = aspara.processing_times.total_ms;
                std::string analysis_text = cv::format("アスパラ#%d分析: %.1fms", aspara.id, analysis_time);
                Image::from(output_image).text(analysis_text, text_pos.x, text_pos.y);
                break;
            }
        }
    }
    
    // 画像をパブリッシュ
    try {
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
            latest_color_image_->header,
            sensor_msgs::image_encodings::BGR8,
            output_image).toImageMsg();
        
        annotated_image_pub_->publish(*msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception in publishCurrentImage: %s", e.what());
    }
}

/**
 * @brief 注釈付き画像をパブリッシュ
 * @param image 元画像
 * @param aspara_info アスパラガス情報
 * @param filtered_cloud フィルタリング済み点群
 * @param pca_line_cloud PCA直線点群
 * @param length 長さ
 * @param straightness 真っ直ぐ度
 * @param is_harvestable 収穫可能フラグ
 * @details デバッグ用の可視化画像を生成・出力
 * 
 * 可視化内容：
 * - バウンディングボックス（収穫可能：緑、不可：赤）
 * - フィルタリング済み点群（シアン色）
 * - PCA直線（ピンク色）
 * - 情報テキスト（ID、信頼度、長さ、真っ直ぐ度、収穫適性）
 */
void FvAsparaAnalyzerNode::publishAnnotatedImage(
    const cv::Mat& image,
    const AsparaInfo& aspara_info,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pca_line_cloud,
    float length,
    float straightness,
    bool is_harvestable)
{
    RCLCPP_WARN(this->get_logger(), "Publishing annotated image for aspara %d", aspara_info.id);
    cv::Mat annotated_image = image.clone();
    
    // バウンディングボックスを描画
    cv::Scalar bbox_color = is_harvestable ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255); // 収穫可能：緑、不可：赤
    cv::rectangle(annotated_image, aspara_info.bounding_box_2d, bbox_color, 3);
    
    // フィルタリング済み点群を画像上に描画
    if (!filtered_cloud->points.empty() && latest_camera_info_) {
        for (const auto& point : filtered_cloud->points) {
            cv::Point2f projected = project3DTo2D(point, *latest_camera_info_);
            if (projected.x >= 0 && projected.x < image.cols && projected.y >= 0 && projected.y < image.rows) {
                cv::circle(annotated_image, cv::Point(projected.x, projected.y), 2, cv::Scalar(255, 255, 0), -1); // シアン色の点
            }
        }
    }
    
    // PCA直線をピンク色で描画
    if (!pca_line_cloud->points.empty() && latest_camera_info_ && pca_line_cloud->points.size() > 1) {
        std::vector<cv::Point> line_points_2d;
        
        // 全PCA直線点を2Dに投影
        for (const auto& point : pca_line_cloud->points) {
            // pcl::PointXYZをpcl::PointXYZRGBに変換（投影関数用）
            pcl::PointXYZRGB rgb_point;
            rgb_point.x = point.x;
            rgb_point.y = point.y; 
            rgb_point.z = point.z;
            
            cv::Point2f projected = project3DTo2D(rgb_point, *latest_camera_info_);
            if (projected.x >= 0 && projected.x < image.cols && projected.y >= 0 && projected.y < image.rows) {
                line_points_2d.push_back(cv::Point(projected.x, projected.y));
            }
        }
        
        // 接続されたピンク色の線分を描画
        cv::Scalar pink_color(255, 0, 255); // ピンク色（BGR形式）
        for (size_t i = 1; i < line_points_2d.size(); ++i) {
            cv::line(annotated_image, line_points_2d[i-1], line_points_2d[i], pink_color, 3);
        }
    }
    
    // テキスト情報を準備（日本語）
    std::string status_text = is_harvestable ? "収穫可能" : "未成熟";
    cv::Scalar text_color = is_harvestable ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    
    // 情報テキストを描画
    int y_offset = aspara_info.bounding_box_2d.y - 10;
    if (y_offset < 30) y_offset = aspara_info.bounding_box_2d.y + aspara_info.bounding_box_2d.height + 30;
    
    // 基本的なテキスト描画のみ
    std::string info_text = cv::format("ID:%d %.2f", aspara_info.id, aspara_info.confidence);
    cv::putText(annotated_image, info_text, cv::Point(aspara_info.bounding_box_2d.x, y_offset), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2);
    
    std::string status_info = cv::format("L:%.1fcm S:%.2f %s", length * 100, straightness, status_text.c_str());
    cv::putText(annotated_image, status_info, cv::Point(aspara_info.bounding_box_2d.x, y_offset + 25), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2);
    
    // 処理時間情報（簡略化）
    std::string timing_info = cv::format("Points:%zu Time:%.1fms", filtered_cloud->points.size(), aspara_info.processing_times.total_ms);
    cv::putText(annotated_image, timing_info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    
    // 注釈付き画像をパブリッシュ
    try {
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
            latest_color_image_->header,
            sensor_msgs::image_encodings::BGR8,
            annotated_image).toImageMsg();
        
        annotated_image_pub_->publish(*msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception in publishAnnotatedImage: %s", e.what());
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
 * @brief アスパラガス処理をキューに追加
 * @param aspara_info 処理対象アスパラガス情報
 */
void FvAsparaAnalyzerNode::enqueueAsparaguProcessing(const AsparaInfo& aspara_info)
{
    std::lock_guard<std::mutex> lock(pointcloud_mutex_);
    
    // キューサイズ制限（メモリリーク防止）
    if (processing_queue_.size() >= 10) {
        // 古いタスクを削除
        processing_queue_.pop();
        RCLCPP_WARN(this->get_logger(), "Processing queue full, dropping oldest task");
    }
    
    processing_queue_.push(aspara_info);
    pointcloud_cv_.notify_one();  // ワーカースレッドに通知
}

/**
 * @brief ワーカースレッドのメインループ
 */
void FvAsparaAnalyzerNode::pointcloudWorkerLoop()
{
    RCLCPP_INFO(this->get_logger(), "Pointcloud worker thread started");
    
    while (!shutdown_flag_) {
        std::unique_lock<std::mutex> lock(pointcloud_mutex_);
        
        // キューにタスクがあるまで待機
        pointcloud_cv_.wait(lock, [this]() {
            return !processing_queue_.empty() || shutdown_flag_;
        });
        
        if (shutdown_flag_) {
            break;
        }
        
        if (!processing_queue_.empty()) {
            // キューからタスクを取得
            AsparaInfo aspara_copy = processing_queue_.front();
            processing_queue_.pop();
            lock.unlock();  // ロック解除してから重い処理を実行
            
            // 処理中フラグを設定
            processing_in_progress_ = true;
            
            try {
                RCLCPP_DEBUG(this->get_logger(), "Starting async analysis for asparagus ID %d", aspara_copy.id);
                
                // 分析時間計測開始
                auto start_time = std::chrono::high_resolution_clock::now();
                
                // アスパラの点群分析を実行（重い処理）
                processAsparagus(aspara_copy);
                
                // 分析時間計測終了・記録
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                
                // 結果をメインスレッドのデータに反映（スレッドセーフ）
                {
                    std::lock_guard<std::mutex> result_lock(pointcloud_mutex_);
                    for (auto& aspara : aspara_list_) {
                        if (aspara.id == aspara_copy.id) {
                            aspara.processing_times.total_ms = duration.count();
                            aspara.length = aspara_copy.length;
                            aspara.straightness = aspara_copy.straightness;
                            aspara.is_harvestable = aspara_copy.is_harvestable;
                            // 他の分析結果も更新...
                            break;
                        }
                    }
                }
                
                RCLCPP_DEBUG(this->get_logger(), "Async asparagus %d analysis completed: %.1fms", 
                           aspara_copy.id, duration.count());
                           
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in async asparagus %d analysis: %s", 
                           aspara_copy.id, e.what());
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Unknown exception in async asparagus %d analysis", 
                           aspara_copy.id);
            }
            
            processing_in_progress_ = false;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Pointcloud worker thread terminated");
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
        RCLCPP_WARN(logger, "Node created, starting MultiThreadedExecutor...");
        
        // マルチスレッドエグゼキューター使用（カクつき解決）
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Exception in main: %s", e.what());
    }
    
    RCLCPP_WARN(logger, "===== Shutting down FV Aspara Analyzer Node =====");
    rclcpp::shutdown();
    return 0;
}