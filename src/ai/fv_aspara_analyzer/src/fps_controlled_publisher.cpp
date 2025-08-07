/**
 * @file fps_controlled_publisher.cpp
 * @brief FPS制御可能なアスパラガス検出結果公開ノード
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class FpsControlledPublisher : public rclcpp::Node {
public:
    FpsControlledPublisher() : Node("fps_controlled_publisher") {
        RCLCPP_INFO(this->get_logger(), "===== FPS制御アスパラガス公開ノード =====");
        
        // FPS設定パラメータ
        this->declare_parameter<double>("bbox_fps", 30.0);      // 枠表示FPS（デフォルト30）
        this->declare_parameter<double>("preview_fps", 10.0);   // プレビュー点群FPS
        this->declare_parameter<double>("detailed_fps", 2.0);   // 詳細点群FPS
        this->declare_parameter<bool>("adaptive_fps", true);    // 負荷に応じて自動調整
        this->declare_parameter<double>("target_cpu_usage", 30.0); // 目標CPU使用率
        
        // 初期FPS取得
        bbox_fps_ = this->get_parameter("bbox_fps").as_double();
        preview_fps_ = this->get_parameter("preview_fps").as_double();
        detailed_fps_ = this->get_parameter("detailed_fps").as_double();
        adaptive_fps_ = this->get_parameter("adaptive_fps").as_bool();
        
        // サブスクライバー
        detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/fv/d415/object_detection/detections", 10,
            std::bind(&FpsControlledPublisher::detectionCallback, this, std::placeholders::_1));
            
        color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/fv/d415/color/image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                color_msg_ = msg;
            });
            
        // パブリッシャー
        bbox_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/asparagus/bbox/image", 10);
            
        bbox_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/asparagus/bbox/markers", 10);
            
        preview_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/asparagus/preview/points", 10);
            
        detailed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/asparagus/detailed/points", 10);
            
        fps_info_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/asparagus/fps_info", 10);
            
        // タイマー（FPSに基づいて設定）
        updateTimers();
        
        // FPS調整タイマー（1秒ごと）
        fps_adjustment_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&FpsControlledPublisher::adjustFPS, this));
            
        // パラメータ変更コールバック
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&FpsControlledPublisher::parametersCallback, this, std::placeholders::_1));
    }
    
private:
    // FPS設定
    double bbox_fps_;
    double preview_fps_;
    double detailed_fps_;
    bool adaptive_fps_;
    
    // 実測FPS
    struct FPSCounter {
        int count = 0;
        std::chrono::steady_clock::time_point last_reset = std::chrono::steady_clock::now();
        double current_fps = 0.0;
        
        void tick() {
            count++;
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_reset).count();
            
            if (elapsed >= 1000) {  // 1秒経過
                current_fps = count * 1000.0 / elapsed;
                count = 0;
                last_reset = now;
            }
        }
        
        double getFPS() const { return current_fps; }
    };
    
    FPSCounter bbox_fps_counter_;
    FPSCounter preview_fps_counter_;
    FPSCounter detailed_fps_counter_;
    
    // 処理時間計測
    std::chrono::steady_clock::time_point last_bbox_time_;
    std::chrono::steady_clock::time_point last_preview_time_;
    std::chrono::steady_clock::time_point last_detailed_time_;
    
    // データ
    vision_msgs::msg::Detection2DArray::SharedPtr latest_detections_;
    sensor_msgs::msg::Image::SharedPtr color_msg_;
    std::mutex data_mutex_;
    
    // ROS2インターフェース
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bbox_image_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr preview_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detailed_cloud_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fps_info_pub_;
    
    rclcpp::TimerBase::SharedPtr bbox_timer_;
    rclcpp::TimerBase::SharedPtr preview_timer_;
    rclcpp::TimerBase::SharedPtr detailed_timer_;
    rclcpp::TimerBase::SharedPtr fps_adjustment_timer_;
    
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_detections_ = msg;
    }
    
    /**
     * @brief バウンディングボックス更新（高FPS）
     */
    void updateBoundingBoxes() {
        auto start = std::chrono::high_resolution_clock::now();
        
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!latest_detections_ || !color_msg_) return;
        
        // FPSチェック（スキップ判定）
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_bbox_time_).count();
        if (elapsed < (1000.0 / bbox_fps_)) {
            return;  // FPS制限
        }
        last_bbox_time_ = now;
        
        // 画像に枠を描画
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(color_msg_, "bgr8");
        } catch (cv_bridge::Exception& e) {
            return;
        }
        
        // 枠描画とマーカー生成
        visualization_msgs::msg::MarkerArray markers;
        int id = 0;
        
        for (const auto& det : latest_detections_->detections) {
            if (det.results.empty()) continue;
            
            // バウンディングボックス描画
            cv::Rect bbox(
                det.bbox.center.position.x - det.bbox.size_x / 2,
                det.bbox.center.position.y - det.bbox.size_y / 2,
                det.bbox.size_x,
                det.bbox.size_y
            );
            
            // 距離推定
            float distance = estimateDistance(bbox);
            
            // 色（距離に応じて）
            cv::Scalar color = distance < 1.0 ? cv::Scalar(0, 255, 0) :   // 近い：緑
                              distance < 2.0 ? cv::Scalar(0, 255, 255) : // 中間：黄
                                              cv::Scalar(0, 0, 255);      // 遠い：赤
            
            cv::rectangle(cv_ptr->image, bbox, color, 2);
            
            // 距離表示
            std::string text = cv::format("%.1fm", distance);
            cv::putText(cv_ptr->image, text, 
                       cv::Point(bbox.x, bbox.y - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            
            // 3Dマーカー
            markers.markers.push_back(create3DBBoxMarker(bbox, distance, id++));
        }
        
        // 画像公開
        bbox_image_pub_->publish(cv_ptr->toImageMsg());
        
        // マーカー公開
        bbox_marker_pub_->publish(markers);
        
        // FPSカウント
        bbox_fps_counter_.tick();
        
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start).count() / 1000.0;
        
        RCLCPP_DEBUG(this->get_logger(), 
            "BBox更新: %.1fms (目標FPS: %.1f, 実測FPS: %.1f)",
            elapsed_ms, bbox_fps_, bbox_fps_counter_.getFPS());
    }
    
    /**
     * @brief プレビュー点群更新（中FPS）
     */
    void updatePreviewCloud() {
        // TODO: 低解像度点群生成
        preview_fps_counter_.tick();
    }
    
    /**
     * @brief 詳細点群更新（低FPS）
     */
    void updateDetailedCloud() {
        // TODO: 高解像度点群生成
        detailed_fps_counter_.tick();
    }
    
    /**
     * @brief FPS自動調整
     */
    void adjustFPS() {
        if (!adaptive_fps_) {
            publishFPSInfo();
            return;
        }
        
        // CPU使用率を取得（簡易版）
        double cpu_usage = estimateCPUUsage();
        double target_cpu = this->get_parameter("target_cpu_usage").as_double();
        
        // FPS調整ロジック
        if (cpu_usage > target_cpu * 1.2) {  // 20%超過
            // FPSを下げる
            bbox_fps_ = std::max(10.0, bbox_fps_ * 0.8);
            preview_fps_ = std::max(5.0, preview_fps_ * 0.8);
            detailed_fps_ = std::max(1.0, detailed_fps_ * 0.8);
            
            RCLCPP_WARN(this->get_logger(), 
                "CPU使用率高（%.1f%%） - FPS削減: BBox=%.1f, Preview=%.1f",
                cpu_usage, bbox_fps_, preview_fps_);
                
        } else if (cpu_usage < target_cpu * 0.8) {  // 20%余裕
            // FPSを上げる
            bbox_fps_ = std::min(60.0, bbox_fps_ * 1.1);
            preview_fps_ = std::min(30.0, preview_fps_ * 1.1);
            detailed_fps_ = std::min(10.0, detailed_fps_ * 1.1);
            
            RCLCPP_INFO(this->get_logger(), 
                "CPU余裕あり（%.1f%%） - FPS増加: BBox=%.1f",
                cpu_usage, bbox_fps_);
        }
        
        // タイマー更新
        updateTimers();
        publishFPSInfo();
    }
    
    /**
     * @brief タイマー更新
     */
    void updateTimers() {
        // 既存タイマーをキャンセル
        if (bbox_timer_) bbox_timer_->cancel();
        if (preview_timer_) preview_timer_->cancel();
        if (detailed_timer_) detailed_timer_->cancel();
        
        // 新しいタイマー作成
        auto bbox_period = std::chrono::microseconds(static_cast<int64_t>(1000000.0 / bbox_fps_));
        bbox_timer_ = this->create_wall_timer(
            bbox_period, 
            std::bind(&FpsControlledPublisher::updateBoundingBoxes, this));
            
        auto preview_period = std::chrono::microseconds(static_cast<int64_t>(1000000.0 / preview_fps_));
        preview_timer_ = this->create_wall_timer(
            preview_period,
            std::bind(&FpsControlledPublisher::updatePreviewCloud, this));
            
        auto detailed_period = std::chrono::microseconds(static_cast<int64_t>(1000000.0 / detailed_fps_));
        detailed_timer_ = this->create_wall_timer(
            detailed_period,
            std::bind(&FpsControlledPublisher::updateDetailedCloud, this));
    }
    
    /**
     * @brief FPS情報公開
     */
    void publishFPSInfo() {
        std_msgs::msg::String msg;
        msg.data = cv::format(
            "FPS設定 [BBox: %.1f/%.1f] [Preview: %.1f/%.1f] [Detailed: %.1f/%.1f]",
            bbox_fps_counter_.getFPS(), bbox_fps_,
            preview_fps_counter_.getFPS(), preview_fps_,
            detailed_fps_counter_.getFPS(), detailed_fps_
        );
        fps_info_pub_->publish(msg);
    }
    
    /**
     * @brief パラメータ変更コールバック
     */
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters) {
        
        for (const auto& param : parameters) {
            if (param.get_name() == "bbox_fps") {
                bbox_fps_ = param.as_double();
            } else if (param.get_name() == "preview_fps") {
                preview_fps_ = param.as_double();
            } else if (param.get_name() == "detailed_fps") {
                detailed_fps_ = param.as_double();
            } else if (param.get_name() == "adaptive_fps") {
                adaptive_fps_ = param.as_bool();
            }
        }
        
        updateTimers();
        
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }
    
    float estimateDistance(const cv::Rect& bbox) {
        return (615.92f * 0.015f) / bbox.width;  // 簡易推定
    }
    
    double estimateCPUUsage() {
        // 簡易推定（実際はシステムコールで取得）
        return 25.0;  // TODO: 実装
    }
    
    visualization_msgs::msg::Marker create3DBBoxMarker(
        const cv::Rect& bbox, float distance, int id) {
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "fv/d415/depth_optical_frame";
        marker.header.stamp = this->now();
        marker.ns = "asparagus_bbox";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 3D位置計算（簡易版）
        float cx = bbox.x + bbox.width / 2.0f;
        float cy = bbox.y + bbox.height / 2.0f;
        float x = (cx - 321.84f) * distance / 615.92f;
        float y = (cy - 237.25f) * distance / 615.58f;
        
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = distance;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.002;  // 線の太さ
        
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        
        return marker;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), 
        "\n===== FPS制御可能な公開ノード =====\n"
        "動的パラメータ設定:\n"
        "- ros2 param set /fps_controlled_publisher bbox_fps 60.0\n"
        "- ros2 param set /fps_controlled_publisher preview_fps 15.0\n"
        "- ros2 param set /fps_controlled_publisher adaptive_fps true\n"
        "\n特徴:\n"
        "- 枠表示: 10-60 FPS\n"
        "- プレビュー: 5-30 FPS\n"
        "- 詳細: 1-10 FPS\n"
        "- CPU負荷に応じて自動調整");
    
    rclcpp::spin(std::make_shared<FpsControlledPublisher>());
    rclcpp::shutdown();
    return 0;
}