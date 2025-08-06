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

class FVObjectDetectorNode : public rclcpp::Node
{
public:
    FVObjectDetectorNode() : Node("fv_object_detector_node"), 
                            detection_enabled_(true),
                            frame_count_(0),
                            last_fps_time_(std::chrono::steady_clock::now())
    {
        RCLCPP_INFO(this->get_logger(), "FV Object Detector Node starting...");
        
        // パラメータを宣言
        this->declare_parameter("input_image_topic", "/camera/color/image_raw");
        this->declare_parameter("output_image_topic", "/object_detection/annotated_image");
        this->declare_parameter("output_detections_topic", "/object_detection/detections");
        this->declare_parameter("processing_frequency", 10.0);
        this->declare_parameter("enable_tracking", true);
        this->declare_parameter("enable_visualization", true);
        this->declare_parameter("show_stats_on_image", true);
        this->declare_parameter("publish_annotated_image", true);
        
        // パラメータを取得
        input_topic_ = this->get_parameter("input_image_topic").as_string();
        output_image_topic_ = this->get_parameter("output_image_topic").as_string();
        output_detections_topic_ = this->get_parameter("output_detections_topic").as_string();
        processing_frequency_ = this->get_parameter("processing_frequency").as_double();
        enable_tracking_ = this->get_parameter("enable_tracking").as_bool();
        enable_visualization_ = this->get_parameter("enable_visualization").as_bool();
        show_stats_on_image_ = this->get_parameter("show_stats_on_image").as_bool();
        publish_annotated_image_ = this->get_parameter("publish_annotated_image").as_bool();
        
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
        
        // サブスクライバーを設定
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic_, 10,
            std::bind(&FVObjectDetectorNode::imageCallback, this, std::placeholders::_1));
        
        // パブリッシャーを設定
        if (enable_visualization_) {
            image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                output_image_topic_, 10);
        }
        
        detections_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
            output_detections_topic_, 10);
        
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
    void loadModelConfig()
    {
        try {
            // ROS2パラメータからモデル設定を取得
            auto model_type = this->declare_parameter("model.type", "yolov10");
            auto model_path = this->declare_parameter("model.model_path", "/home/takatronix/FluentVision/models/yolov10n.pt");
            auto device = this->declare_parameter("model.device", "CPU");
            auto input_width = this->declare_parameter("model.input_width", 640);
            auto input_height = this->declare_parameter("model.input_height", 640);
            auto confidence_threshold = this->declare_parameter("model.confidence_threshold", 0.5);
            auto nms_threshold = this->declare_parameter("model.nms_threshold", 0.45);
            auto min_area = this->declare_parameter("model.min_area", 100.0);
            
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
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 最新の画像を保存
        std::lock_guard<std::mutex> lock(image_mutex_);
        latest_image_ = msg;
        image_received_ = true;
    }
    
    void processTimer()
    {
        if (!image_received_ || !detection_enabled_) {
            return;
        }
        
        sensor_msgs::msg::Image::SharedPtr image_msg;
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            image_msg = latest_image_;
        }
        
        if (!image_msg) {
            return;
        }
        
        // 画像をOpenCV形式に変換
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        cv::Mat image = cv_ptr->image;
        
        // 処理開始時間を記録
        auto processing_start = std::chrono::high_resolution_clock::now();
        
        // 物体検出を実行
        std::vector<fv_object_detector::DetectionData> detections;
        try {
            detections = model_->infer(image);
            RCLCPP_DEBUG(this->get_logger(), "Detection completed: %zu objects", detections.size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Detection failed: %s", e.what());
            return;
        }
        
        // 処理終了時間を記録
        auto processing_end = std::chrono::high_resolution_clock::now();
        auto processing_duration = std::chrono::duration_cast<std::chrono::microseconds>(processing_end - processing_start);
        
        // 統計情報を更新
        updateStats(detections.size(), processing_duration.count() / 1000.0);
        
        // オブジェクトトラッキングを適用
        if (enable_tracking_ && tracker_) {
            tracker_->assignObjectIds(detections);
        }
        
        // 検出結果をパブリッシュ
        publishDetections(detections, image_msg->header);
        
        // アノテーション画像をパブリッシュ
        if (enable_visualization_ && publish_annotated_image_) {
            cv::Mat annotated_image = drawDetections(image, detections);
            publishAnnotatedImage(annotated_image, image_msg->header);
        }
    }
    
    cv::Mat drawDetections(const cv::Mat& image, const std::vector<fv_object_detector::DetectionData>& detections)
    {
        cv::Mat result = image.clone();
        
        // 統計情報を画像に描画
        if (show_stats_on_image_) {
            drawStatsOnImage(result);
        }
        
        for (const auto& det : detections) {
            // バウンディングボックスを描画
            cv::Rect bbox(det.bbox.x, det.bbox.y, det.bbox.width, det.bbox.height);
            cv::Scalar color = getColorForClass(det.class_id);
            cv::rectangle(result, bbox, color, 2);
            
            // ラベルを描画
            std::string label = det.class_name + " " + std::to_string(static_cast<int>(det.confidence * 100)) + "%";
            if (det.object_id >= 0) {
                label += " ID:" + std::to_string(det.object_id);
            }
            
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
    
    cv::Scalar getColorForClass(int class_id)
    {
        // クラスIDに基づいて色を生成
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
    
    void publishDetections(const std::vector<fv_object_detector::DetectionData>& detections, 
                          const std_msgs::msg::Header& header)
    {
        vision_msgs::msg::Detection2DArray detections_msg;
        detections_msg.header = header;
        
        for (const auto& det : detections) {
            vision_msgs::msg::Detection2D detection_msg;
            
            // バウンディングボックス
            detection_msg.bbox.center.position.x = det.bbox.x + det.bbox.width / 2.0;
            detection_msg.bbox.center.position.y = det.bbox.y + det.bbox.height / 2.0;
            detection_msg.bbox.size_x = det.bbox.width;
            detection_msg.bbox.size_y = det.bbox.height;
            
            // クラス情報
            detection_msg.results.resize(1);
            detection_msg.results[0].hypothesis.class_id = det.class_id;
            detection_msg.results[0].hypothesis.score = det.confidence;
            
            detections_msg.detections.push_back(detection_msg);
        }
        
        detections_pub_->publish(detections_msg);
    }
    
    void publishAnnotatedImage(const cv::Mat& image, const std_msgs::msg::Header& header)
    {
        cv_bridge::CvImage cv_image;
        cv_image.header = header;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_image.image = image;
        
        image_pub_->publish(*cv_image.toImageMsg());
    }
    
    // 統計情報を画像に描画
    void drawStatsOnImage(cv::Mat& image)
    {
        std::string stats_text = "Device: " + stats_.device_used;
        stats_text += " | FPS: " + std::to_string(static_cast<int>(stats_.fps));
        stats_text += " | Inference: " + std::to_string(static_cast<int>(stats_.inference_time_ms)) + "ms";
        stats_text += " | Total: " + std::to_string(static_cast<int>(stats_.total_processing_time_ms)) + "ms";
        stats_text += " | Detections: " + std::to_string(stats_.filtered_detections);
        stats_text += " | Status: " + std::string(detection_enabled_ ? "ON" : "OFF");
        
        cv::putText(image, stats_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }
    
    // 統計情報を更新
    void updateStats(int detection_count, double processing_time_ms)
    {
        stats_.filtered_detections = detection_count;
        stats_.total_processing_time_ms = processing_time_ms;
        stats_.inference_time_ms = model_->getLastInferTime();
        stats_.device_used = model_->getDevice();
        stats_.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        
        // FPS計算
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
    
    // パラメータ
    std::string input_topic_;
    std::string output_image_topic_;
    std::string output_detections_topic_;
    double processing_frequency_;
    bool enable_tracking_;
    bool enable_visualization_;
    bool show_stats_on_image_;
    bool publish_annotated_image_;
    
    // メンバー変数
    std::unique_ptr<fv_object_detector::AIModel> model_;
    std::unique_ptr<fv_object_detector::ObjectTracker> tracker_;
    
    // サブスクライバー
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    
    // パブリッシャー
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
    
    // サービス（一時的に無効化）
    // rclcpp::Service<fv_object_detector::srv::SetDetectionState>::SharedPtr set_detection_state_srv_;
    // rclcpp::Service<fv_object_detector::srv::GetDetectionStats>::SharedPtr get_detection_stats_srv_;
    
    // タイマー
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 画像処理用
    std::mutex image_mutex_;
    sensor_msgs::msg::Image::SharedPtr latest_image_;
    bool image_received_ = false;
    
    // 統計情報
    fv_object_detector::DetectionStats stats_;
    bool detection_enabled_;
    int frame_count_;
    std::chrono::steady_clock::time_point last_fps_time_;
};

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