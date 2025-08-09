/**
 * @file fv_object_detector_node.cpp
 * @brief Fluent Vision 物体検出ノードのメイン実装ファイル
 * @details YOLOv10を使用したリアルタイム物体検出とオブジェクトトラッキング
 * @author Takashi Otsuka
 * @date 2024
 * @version 1.0
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include "fv_object_detector/ai_model.hpp"
#include "fv_object_detector/yolov10_model.hpp"
#include "fv_object_detector/object_tracker.hpp"
#include "fv_object_detector/detection_data.hpp"
// #include "fv_object_detector/srv/set_detection_state.hpp"
// #include "fv_object_detector/srv/get_detection_stats.hpp"

#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <rclcpp/qos.hpp>

/**
 * @class FVObjectDetectorNode
 * @brief Fluent Vision 物体検出メインノードクラス
 * @details YOLOv10を使用したリアルタイム物体検出とオブジェクトトラッキングを提供
 * 
 * 主な機能：
 * - リアルタイム物体検出（YOLOv10 + OpenVINO）
 * - オブジェクトトラッキング（フレーム間ID追跡）
 * - 検出結果の可視化とアノテーション
 * - 統計情報の収集と表示
 * - 設定可能な処理頻度とパラメータ
 */
class FVObjectDetectorNode : public rclcpp::Node
{
public:
    /**
     * @brief コンストラクタ
     * @details ノードの初期化、パラメータ読み込み、AIモデル設定を行う
     * 
     * 初期化内容：
     * - ROS2パラメータの宣言と取得
     * - AIモデルの読み込みと初期化
     * - サブスクライバー・パブリッシャーの作成
     * - オブジェクトトラッカーの初期化
     * - 処理タイマーの設定
     */
    FVObjectDetectorNode() : Node("fv_object_detector_node"), 
                            detection_enabled_(true),
                            frame_count_(0),
                            last_fps_time_(std::chrono::steady_clock::now())
    {
        RCLCPP_INFO(this->get_logger(), "FV Object Detector Node starting...");
        
        // ===== パラメータ宣言 =====
        this->declare_parameter("input_image_topic", "/camera/color/image_raw");        // 入力画像トピック
        this->declare_parameter("output_image_topic", "/object_detection/annotated_image"); // 出力画像トピック
        this->declare_parameter("output_detections_topic", "/object_detection/detections"); // 検出結果トピック
        this->declare_parameter("processing_frequency", 10.0);                          // 処理頻度（Hz）
        this->declare_parameter("enable_tracking", true);                               // トラッキング有効化
        this->declare_parameter("enable_visualization", true);                          // 可視化有効化
        this->declare_parameter("show_stats_on_image", true);                           // 統計情報表示
        this->declare_parameter("publish_annotated_image", true);                       // アノテーション画像出力
        
        // パラメータを取得
        input_topic_ = this->get_parameter("input_image_topic").as_string();
        output_image_topic_ = this->get_parameter("output_image_topic").as_string();
        output_detections_topic_ = this->get_parameter("output_detections_topic").as_string();
        processing_frequency_ = this->get_parameter("processing_frequency").as_double();
        enable_tracking_ = this->get_parameter("enable_tracking").as_bool();
        enable_visualization_ = this->get_parameter("enable_visualization").as_bool();
        show_stats_on_image_ = this->get_parameter("show_stats_on_image").as_bool();
        publish_annotated_image_ = this->get_parameter("publish_annotated_image").as_bool();

        // QoS設定を読み込み
        int qos_queue_size = this->declare_parameter("qos.queue_size", 10);
        std::string qos_reliability = this->declare_parameter("qos.reliability", std::string("best_effort"));
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(qos_queue_size));
        if (qos_reliability == "best_effort") {
            qos.best_effort();
        } else {
            qos.reliable();
        }
        
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output image topic: %s", output_image_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output detections topic: %s", output_detections_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Processing frequency: %.1f Hz", processing_frequency_);
        RCLCPP_INFO(this->get_logger(), "Tracking enabled: %s", enable_tracking_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Visualization enabled: %s", enable_visualization_ ? "true" : "false");
        
        // モデル設定を読み込み
        loadModelConfig();
        
        // オブジェクトトラッカーを初期化
        if (enable_tracking_) {
            tracker_ = std::make_unique<fv_object_detector::ObjectTracker>();
        }
        
        // サブスクライバーを設定（QoSはパラメータから）
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic_, qos,
            std::bind(&FVObjectDetectorNode::imageCallback, this, std::placeholders::_1));
        
        // パブリッシャーを設定
        if (enable_visualization_) {
            image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                output_image_topic_, qos);
        }
        
        detections_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
            output_detections_topic_, qos);
        
        // サービスを設定（一時的に無効化）
        // set_detection_state_srv_ = this->create_service<fv_object_detector::srv::SetDetectionState>(
        //     "set_detection_state",
        //     std::bind(&FVObjectDetectorNode::setDetectionStateCallback, this, 
        //              std::placeholders::_1, std::placeholders::_2));
        
        // get_detection_stats_srv_ = this->create_service<fv_object_detector::srv::GetDetectionStats>(
        //     "get_detection_stats",
        //     std::bind(&FVObjectDetectorNode::getDetectionStatsCallback, this, 
        //              std::placeholders::_1, std::placeholders::_2));
        
        // 処理タイマーを設定
        if (processing_frequency_ > 0) {
            timer_ = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / processing_frequency_),
                std::bind(&FVObjectDetectorNode::processTimer, this));
        }
        
        RCLCPP_INFO(this->get_logger(), "FV Object Detector Node started successfully");
    }

private:
    /**
     * @brief AIモデル設定の読み込み
     * @details ROS2パラメータからYOLOv10モデルの設定を読み込み、AIモデルを初期化
     * 
     * 設定内容：
     * - モデルタイプ（yolov10）
     * - モデルファイルパス
     * - 推論デバイス（CPU/GPU）
     * - 入力画像サイズ
     * - 信頼度閾値
     * - NMS閾値
     * - 最小面積フィルタ
     * - クラス名リスト
     */
    void loadModelConfig()
    {
        try {
            // ===== ROS2パラメータからモデル設定を取得 =====
            auto model_type = this->declare_parameter("model.type", "yolov10");                    // モデルタイプ
            auto model_path = this->declare_parameter("model.model_path", "/models/v2_nano_best_fp16_dynamic.xml"); // モデルファイルパス
            auto device = this->declare_parameter("model.device", "");                             // 推論デバイス
            auto input_width = this->declare_parameter("model.input_width", 640);                  // 入力画像幅
            auto input_height = this->declare_parameter("model.input_height", 640);                // 入力画像高さ
            auto confidence_threshold = this->declare_parameter("model.confidence_threshold", 0.5); // 信頼度閾値
            auto nms_threshold = this->declare_parameter("model.nms_threshold", 0.45);             // NMS閾値
            auto min_area = this->declare_parameter("model.min_area", 100.0);                      // 最小面積
            
            // クラス名を取得
            auto class_names = this->declare_parameter("class_names", std::vector<std::string>());
            
            // 設定JSONを構築
            nlohmann::json config_json;
            config_json["model"]["type"] = model_type;
            config_json["model"]["name"] = "YOLOv10";
            config_json["model"]["path"] = model_path;
            config_json["model"]["device"] = device;
            config_json["model"]["input_width"] = input_width;
            config_json["model"]["input_height"] = input_height;
            config_json["model"]["confidence_threshold"] = confidence_threshold;
            config_json["model"]["nms_threshold"] = nms_threshold;
            config_json["model"]["min_area"] = min_area;
            config_json["classes"] = class_names;
            
            // AIモデルを初期化
            if (model_type == "yolov10") {
                model_ = fv_object_detector::AIModel::createFromConfig(config_json);
            } else {
                throw std::runtime_error("Unknown model type: " + model_type);
            }
            
            RCLCPP_INFO(this->get_logger(), "Model loaded successfully: %s", model_->getModelName().c_str());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load model: %s", e.what());
            throw;
        }
    }
    
    /**
     * @brief 画像コールバック関数
     * @param msg 受信した画像メッセージ
     * @details 入力画像を受信し、最新画像として保存
     * 
     * 処理内容：
     * - 画像メッセージの受信
     * - スレッドセーフな最新画像の更新
     * - 画像受信フラグの設定
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 最新の画像を保存（スレッドセーフ）
        std::lock_guard<std::mutex> lock(image_mutex_);
        latest_image_ = msg;
        image_received_ = true;
    }
    
    /**
     * @brief 処理タイマーコールバック関数
     * @details 定期的に物体検出処理を実行するメインループ
     * 
     * 処理フロー：
     * - 画像の可用性チェック
     * - OpenCV形式への変換
     * - YOLOv10による物体検出
     * - オブジェクトトラッキング
     * - 結果の可視化とパブリッシュ
     * - 統計情報の更新
     */
    void processTimer()
    {
        // 画像受信と検出有効性のチェック
        if (!image_received_ || !detection_enabled_) {
            return;
        }
        
        // 最新画像の取得（スレッドセーフ）
        sensor_msgs::msg::Image::SharedPtr image_msg;
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            image_msg = latest_image_;
        }
        
        if (!image_msg) {
            return;
        }
        
        // ===== 画像をOpenCV形式に変換 =====
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        cv::Mat image = cv_ptr->image;
        
        // ===== 処理開始時間を記録 =====
        auto processing_start = std::chrono::high_resolution_clock::now();
        
        // ===== 物体検出を実行 =====
        std::vector<fv_object_detector::DetectionData> detections;
        try {
            detections = model_->infer(image);
            RCLCPP_DEBUG(this->get_logger(), "Detection completed: %zu objects", detections.size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Detection failed: %s", e.what());
            return;
        }
        
        // ===== 処理終了時間を記録 =====
        auto processing_end = std::chrono::high_resolution_clock::now();
        auto processing_duration = std::chrono::duration_cast<std::chrono::microseconds>(processing_end - processing_start);
        
        // ===== 統計情報を更新 =====
        updateStats(detections.size(), processing_duration.count() / 1000.0);
        
        // ===== オブジェクトトラッキングを適用 =====
        if (enable_tracking_ && tracker_) {
            tracker_->assignObjectIds(detections);
        }
        
        // ===== 検出結果をパブリッシュ =====
        publishDetections(detections, image_msg->header);
        
        // ===== アノテーション画像をパブリッシュ =====
        if (enable_visualization_ && publish_annotated_image_) {
            cv::Mat annotated_image = drawDetections(image, detections);
            publishAnnotatedImage(annotated_image, image_msg->header);
        }
    }
    
    /**
     * @brief 検出結果の可視化描画
     * @param image 元画像
     * @param detections 検出結果の配列
     * @return cv::Mat 描画済み画像
     * @details 検出結果を画像上に描画し、可視化用のアノテーション画像を生成
     * 
     * 描画内容：
     * - バウンディングボックス（クラス別色分け）
     * - クラス名と信頼度
     * - オブジェクトID（トラッキング時）
     * - 統計情報（設定時）
     */
    cv::Mat drawDetections(const cv::Mat& image, const std::vector<fv_object_detector::DetectionData>& detections)
    {
        cv::Mat result = image.clone();
        
        // ===== 統計情報を画像に描画 =====
        if (show_stats_on_image_) {
            drawStatsOnImage(result);
        }
        
        // ===== 各検出結果を描画 =====
        for (const auto& det : detections) {
            // バウンディングボックスを描画
            cv::Rect bbox(det.bbox.x, det.bbox.y, det.bbox.width, det.bbox.height);
            cv::Scalar color = getColorForClass(det.class_id);
            cv::rectangle(result, bbox, color, 2);
            
            // ラベルテキストを構築
            std::string label = det.class_name + " " + std::to_string(static_cast<int>(det.confidence * 100)) + "%";
            if (det.object_id >= 0) {
                label += " ID:" + std::to_string(det.object_id);
            }
            
            // テキストサイズを計算
            int baseline = 0;
            cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            cv::Point text_pos(bbox.x, bbox.y - 5);
            
            // ラベル背景を描画
            cv::rectangle(result, 
                         cv::Point(text_pos.x, text_pos.y - text_size.height - baseline),
                         cv::Point(text_pos.x + text_size.width, text_pos.y + baseline),
                         color, -1);
            
            // ラベルテキストを描画
            cv::putText(result, label, text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        }
        
        return result;
    }
    
    /**
     * @brief クラスIDに基づく色の取得
     * @param class_id クラスID
     * @return cv::Scalar クラスに対応する色（BGR形式）
     * @details クラスIDに基づいて一意の色を生成し、可視化時の識別を容易にする
     * 
     * 色の割り当て：
     * - クラスID 0: 青 (255, 0, 0)
     * - クラスID 1: 緑 (0, 255, 0)
     * - クラスID 2: 赤 (0, 0, 255)
     * - クラスID 3: シアン (255, 255, 0)
     * - クラスID 4: マゼンタ (255, 0, 255)
     * - クラスID 5: 黄色 (0, 255, 255)
     * - それ以外: 循環的に色を割り当て
     */
    cv::Scalar getColorForClass(int class_id)
    {
        // クラスIDに基づいて色を生成（BGR形式）
        static std::vector<cv::Scalar> colors = {
            cv::Scalar(255, 0, 0),    // 青
            cv::Scalar(0, 255, 0),    // 緑
            cv::Scalar(0, 0, 255),    // 赤
            cv::Scalar(255, 255, 0),  // シアン
            cv::Scalar(255, 0, 255),  // マゼンタ
            cv::Scalar(0, 255, 255),  // 黄色
        };
        
        return colors[class_id % colors.size()];
    }
    
    /**
     * @brief 検出結果のパブリッシュ
     * @param detections 検出結果の配列
     * @param header メッセージヘッダー
     * @details 検出結果をROS2標準メッセージ形式でパブリッシュ
     * 
     * 変換内容：
     * - DetectionData → vision_msgs::Detection2D
     * - バウンディングボックス座標変換
     * - クラス情報と信頼度の設定
     * - メッセージヘッダーの設定
     */
    void publishDetections(const std::vector<fv_object_detector::DetectionData>& detections, 
                          const std_msgs::msg::Header& header)
    {
        vision_msgs::msg::Detection2DArray detections_msg;
        detections_msg.header = header;
        
        // ===== 各検出結果をROS2メッセージ形式に変換 =====
        for (const auto& det : detections) {
            vision_msgs::msg::Detection2D detection_msg;
            
            // バウンディングボックス座標設定
            detection_msg.bbox.center.position.x = det.bbox.x + det.bbox.width / 2.0;
            detection_msg.bbox.center.position.y = det.bbox.y + det.bbox.height / 2.0;
            detection_msg.bbox.size_x = det.bbox.width;
            detection_msg.bbox.size_y = det.bbox.height;
            
            // クラス情報と信頼度設定
            detection_msg.results.resize(1);
            detection_msg.results[0].hypothesis.class_id = std::to_string(det.class_id);
            detection_msg.results[0].hypothesis.score = det.confidence;
            
            detections_msg.detections.push_back(detection_msg);
        }
        
        // 検出結果をパブリッシュ
        detections_pub_->publish(detections_msg);
    }
    
    /**
     * @brief アノテーション画像のパブリッシュ
     * @param image 描画済み画像
     * @param header メッセージヘッダー
     * @details 検出結果を描画した画像をROS2メッセージ形式でパブリッシュ
     * 
     * 変換内容：
     * - cv::Mat → sensor_msgs::Image
     * - BGR8エンコーディング設定
     * - メッセージヘッダーの設定
     */
    void publishAnnotatedImage(const cv::Mat& image, const std_msgs::msg::Header& header)
    {
        cv_bridge::CvImage cv_image;
        cv_image.header = header;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_image.image = image;
        
        // アノテーション画像をパブリッシュ
        image_pub_->publish(*cv_image.toImageMsg());
    }
    
    /**
     * @brief 統計情報を画像に描画
     * @param image 描画対象画像
     * @details パフォーマンス統計情報を画像の上部に描画
     * 
     * 表示内容：
     * - 使用デバイス（CPU/GPU）
     * - 処理FPS
     * - 推論時間
     * - 総処理時間
     * - 検出数
     * - 検出状態（ON/OFF）
     */
    void drawStatsOnImage(cv::Mat& image)
    {
        // 統計情報テキストを構築
        std::string stats_text = "Device: " + stats_.device_used;
        stats_text += " | FPS: " + std::to_string(static_cast<int>(stats_.fps));
        stats_text += " | Inference: " + std::to_string(static_cast<int>(stats_.inference_time_ms)) + "ms";
        stats_text += " | Total: " + std::to_string(static_cast<int>(stats_.total_processing_time_ms)) + "ms";
        stats_text += " | Detections: " + std::to_string(stats_.filtered_detections);
        stats_text += " | Status: " + std::string(detection_enabled_ ? "ON" : "OFF");
        
        // 統計情報を画像上部に描画（緑色）
        cv::putText(image, stats_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }
    
    /**
     * @brief 統計情報の更新
     * @param detection_count 検出数
     * @param processing_time_ms 処理時間（ミリ秒）
     * @details パフォーマンス統計情報を更新し、FPS計算を実行
     * 
     * 更新内容：
     * - 検出数
     * - 総処理時間
     * - 推論時間
     * - 使用デバイス
     * - タイムスタンプ
     * - FPS（1秒ごとに更新）
     */
    void updateStats(int detection_count, double processing_time_ms)
    {
        // ===== 基本統計情報の更新 =====
        stats_.filtered_detections = detection_count;
        stats_.total_processing_time_ms = processing_time_ms;
        stats_.inference_time_ms = model_->getLastInferTime();
        stats_.device_used = model_->getDevice();
        stats_.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        
        // ===== FPS計算 =====
        frame_count_++;
        auto current_time = std::chrono::steady_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_fps_time_).count();
        
        if (time_diff >= 1000) { // 1秒ごとにFPS更新
            stats_.fps = frame_count_ * 1000.0 / time_diff;
            frame_count_ = 0;
            last_fps_time_ = current_time;
        }
    }
    
    // 検出状態設定サービスコールバック（一時的に無効化）
    // void setDetectionStateCallback(
    //     const std::shared_ptr<fv_object_detector::srv::SetDetectionState::Request> request,
    //     std::shared_ptr<fv_object_detector::srv::SetDetectionState::Response> response)
    // {
    //     detection_enabled_ = request->enable_detection;
    //     response->success = true;
    //     response->message = "Detection " + (detection_enabled_ ? "enabled" : "disabled");
    //     RCLCPP_INFO(this->get_logger(), "Detection state changed: %s", response->message.c_str());
    // }
    
    // 統計情報取得サービスコールバック（一時的に無効化）
    // void getDetectionStatsCallback(
    //     const std::shared_ptr<fv_object_detector::srv::GetDetectionStats::Request> request,
    //     std::shared_ptr<fv_object_detector::srv::GetDetectionStats::Response> response)
    // {
    //     response->inference_time_ms = stats_.inference_time_ms;
    //     response->total_processing_time_ms = stats_.total_processing_time_ms;
    //     response->device_used = stats_.device_used;
    //     response->total_detections = stats_.total_detections;
    //     response->filtered_detections = stats_.filtered_detections;
    //     response->fps = stats_.fps;
    //     response->timestamp = stats_.timestamp;
    //     response->detection_enabled = detection_enabled_;
    // }
    
    // ===== パラメータ群 =====
    std::string input_topic_;                    ///< 入力画像トピック
    std::string output_image_topic_;             ///< 出力画像トピック
    std::string output_detections_topic_;        ///< 検出結果トピック
    double processing_frequency_;                ///< 処理頻度（Hz）
    bool enable_tracking_;                       ///< トラッキング有効化フラグ
    bool enable_visualization_;                  ///< 可視化有効化フラグ
    bool show_stats_on_image_;                   ///< 統計情報表示フラグ
    bool publish_annotated_image_;               ///< アノテーション画像出力フラグ
    
    // ===== メンバー変数群 =====
    std::unique_ptr<fv_object_detector::AIModel> model_;           ///< AIモデル
    std::unique_ptr<fv_object_detector::ObjectTracker> tracker_;   ///< オブジェクトトラッカー
    
    // ===== ROS2サブスクライバー =====
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;  ///< 画像サブスクライバー
    
    // ===== ROS2パブリッシャー =====
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;              ///< 画像パブリッシャー
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_; ///< 検出結果パブリッシャー
    
    // ===== ROS2サービス（一時的に無効化） =====
    // rclcpp::Service<fv_object_detector::srv::SetDetectionState>::SharedPtr set_detection_state_srv_;
    // rclcpp::Service<fv_object_detector::srv::GetDetectionStats>::SharedPtr get_detection_stats_srv_;
    
    // ===== ROS2タイマー =====
    rclcpp::TimerBase::SharedPtr timer_;  ///< 処理タイマー
    
    // ===== 画像処理用変数群 =====
    std::mutex image_mutex_;                                    ///< 画像アクセス用ミューテックス
    sensor_msgs::msg::Image::SharedPtr latest_image_;          ///< 最新画像
    bool image_received_ = false;                              ///< 画像受信フラグ
    
    // ===== 統計情報群 =====
    fv_object_detector::DetectionStats stats_;                 ///< 統計情報
    bool detection_enabled_;                                   ///< 検出有効化フラグ
    int frame_count_;                                          ///< フレームカウンタ
    std::chrono::steady_clock::time_point last_fps_time_;      ///< 最後のFPS計算時刻
};

/**
 * @brief メイン関数
 * @param argc コマンドライン引数の数
 * @param argv コマンドライン引数の配列
 * @return int 終了コード
 * @details ROS2ノードの初期化と実行
 * 
 * 実行内容：
 * - ROS2の初期化
 * - 物体検出ノードの作成
 * - ノードの実行（スピン）
 * - 適切な終了処理
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<FVObjectDetectorNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("fv_object_detector"), "Node failed: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
} 