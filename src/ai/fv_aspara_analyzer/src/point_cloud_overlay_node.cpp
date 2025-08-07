/**
 * @file point_cloud_overlay_node.cpp
 * @brief 映像に点群をオーバーレイ表示するノード
 * @details 検出したアスパラガス領域の点群を2D映像に投影して色付き表示
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fluent_cloud/io/depth_to_cloud.hpp>

class PointCloudOverlayNode : public rclcpp::Node {
public:
    PointCloudOverlayNode() : Node("point_cloud_overlay") {
        RCLCPP_INFO(this->get_logger(), "===== 点群オーバーレイノード起動 =====");
        
        // パラメータ
        this->declare_parameter<float>("point_size", 2.0);
        this->declare_parameter<float>("alpha", 0.7);  // 透明度
        this->declare_parameter<bool>("show_depth_color", true);
        this->declare_parameter<bool>("show_bbox", true);
        this->declare_parameter<int>("point_skip", 2);  // 表示用間引き
        this->declare_parameter<float>("near_distance", 0.3);  // 近距離閾値
        this->declare_parameter<float>("far_distance", 2.0);   // 遠距離閾値
        
        // カメラ情報取得
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/fv/d415/color/camera_info", 1,
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                if (!camera_info_received_) {
                    camera_info_ = *msg;
                    camera_info_received_ = true;
                    RCLCPP_INFO(this->get_logger(), 
                        "カメラ情報取得: %dx%d, fx=%.2f, fy=%.2f", 
                        msg->width, msg->height, msg->k[0], msg->k[4]);
                }
            });
        
        // サブスクライバー
        color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/fv/d415/color/image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                color_msg_ = msg;
            });
            
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/fv/d415/depth/image_rect_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                depth_msg_ = msg;
            });
            
        detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/fv/d415/object_detection/detections", 10,
            std::bind(&PointCloudOverlayNode::detectionCallback, this, std::placeholders::_1));
            
        // パブリッシャー
        overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/asparagus/overlay/image", 10);
            
        annotated_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/asparagus/overlay/points", 10);
            
        // 処理タイマー
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20Hz
            std::bind(&PointCloudOverlayNode::processOverlay, this));
    }
    
private:
    // データ
    sensor_msgs::msg::CameraInfo camera_info_;
    bool camera_info_received_ = false;
    sensor_msgs::msg::Image::SharedPtr color_msg_;
    sensor_msgs::msg::Image::SharedPtr depth_msg_;
    vision_msgs::msg::Detection2DArray::SharedPtr latest_detections_;
    std::mutex data_mutex_;
    
    // パラメータ
    float point_size_;
    float alpha_;
    bool show_depth_color_;
    bool show_bbox_;
    int point_skip_;
    float near_distance_;
    float far_distance_;
    
    // ROS2インターフェース
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr annotated_cloud_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_detections_ = msg;
    }
    
    void processOverlay() {
        if (!camera_info_received_) {
            RCLCPP_WARN_ONCE(this->get_logger(), "カメラ情報待機中...");
            return;
        }
        
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!color_msg_ || !depth_msg_ || !latest_detections_) return;
        if (latest_detections_->detections.empty()) return;
        
        auto start = std::chrono::high_resolution_clock::now();
        
        // パラメータ更新
        point_size_ = this->get_parameter("point_size").as_double();
        alpha_ = this->get_parameter("alpha").as_double();
        show_depth_color_ = this->get_parameter("show_depth_color").as_bool();
        show_bbox_ = this->get_parameter("show_bbox").as_bool();
        point_skip_ = this->get_parameter("point_skip").as_int();
        near_distance_ = this->get_parameter("near_distance").as_double();
        far_distance_ = this->get_parameter("far_distance").as_double();
        
        // 画像変換
        cv::Mat color_image, depth_image;
        try {
            color_image = cv_bridge::toCvCopy(color_msg_, "bgr8")->image;
            depth_image = cv_bridge::toCvCopy(depth_msg_, depth_msg_->encoding)->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "画像変換エラー: %s", e.what());
            return;
        }
        
        // オーバーレイ画像作成
        cv::Mat overlay_image = color_image.clone();
        
        // 全体の点群（公開用）
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_points(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        // 各検出に対して処理
        for (size_t i = 0; i < latest_detections_->detections.size(); ++i) {
            const auto& det = latest_detections_->detections[i];
            if (det.results.empty()) continue;
            
            // バウンディングボックス
            cv::Rect bbox(
                det.bbox.center.position.x - det.bbox.size_x / 2,
                det.bbox.center.position.y - det.bbox.size_y / 2,
                det.bbox.size_x,
                det.bbox.size_y
            );
            
            // 画像範囲チェック
            bbox.x = std::max(0, bbox.x);
            bbox.y = std::max(0, bbox.y);
            bbox.width = std::min(color_image.cols - bbox.x, bbox.width);
            bbox.height = std::min(color_image.rows - bbox.y, bbox.height);
            
            // 枠表示
            if (show_bbox_) {
                cv::rectangle(overlay_image, bbox, cv::Scalar(0, 255, 0), 2);
                cv::putText(overlay_image, 
                    "Asparagus #" + std::to_string(i),
                    cv::Point(bbox.x, bbox.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }
            
            // ROI内の点群をオーバーレイ
            overlayROIPointCloud(depth_image, color_image, overlay_image, 
                                bbox, i, all_points);
        }
        
        // オーバーレイ画像公開
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = this->now();
        out_msg.header.frame_id = color_msg_->header.frame_id;
        out_msg.encoding = "bgr8";
        out_msg.image = overlay_image;
        overlay_pub_->publish(out_msg.toImageMsg());
        
        // 点群も公開
        if (!all_points->empty()) {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*all_points, cloud_msg);
            cloud_msg.header.stamp = this->now();
            cloud_msg.header.frame_id = "fv/d415/depth_optical_frame";
            annotated_cloud_pub_->publish(cloud_msg);
        }
        
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start).count() / 1000.0;
        
        RCLCPP_DEBUG(this->get_logger(), 
            "オーバーレイ処理: %.1fms (%zu検出, %zu点)",
            elapsed, latest_detections_->detections.size(), all_points->size());
    }
    
    /**
     * @brief ROI内の点群を2D画像にオーバーレイ
     */
    void overlayROIPointCloud(
        const cv::Mat& depth_image,
        const cv::Mat& color_image,
        cv::Mat& overlay_image,
        const cv::Rect& bbox,
        int detection_id,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& all_points) {
        
        // カメラパラメータ
        float fx = camera_info_.k[0];
        float fy = camera_info_.k[4];
        float cx = camera_info_.k[2];
        float cy = camera_info_.k[5];
        
        // 深度スケール（RealSenseは通常0.001）
        float depth_scale = 0.001f;
        
        // 色設定（検出IDごとに異なる色）
        cv::Scalar point_color = getColorForDetection(detection_id);
        
        // ROI内の各ピクセルを処理
        for (int y = bbox.y; y < bbox.y + bbox.height; y += point_skip_) {
            for (int x = bbox.x; x < bbox.x + bbox.width; x += point_skip_) {
                // 深度値取得
                float depth_value = 0;
                if (depth_image.type() == CV_16UC1) {
                    depth_value = depth_image.at<uint16_t>(y, x) * depth_scale;
                } else if (depth_image.type() == CV_32FC1) {
                    depth_value = depth_image.at<float>(y, x);
                }
                
                // 有効な深度値のみ処理
                if (depth_value > 0.1f && depth_value < 10.0f) {
                    // 3D座標計算
                    float z = depth_value;
                    float x_3d = (x - cx) * z / fx;
                    float y_3d = (y - cy) * z / fy;
                    
                    // 点群に追加
                    pcl::PointXYZRGB point;
                    point.x = x_3d;
                    point.y = y_3d;
                    point.z = z;
                    
                    cv::Vec3b bgr = color_image.at<cv::Vec3b>(y, x);
                    point.b = bgr[0];
                    point.g = bgr[1];
                    point.r = bgr[2];
                    
                    all_points->points.push_back(point);
                    
                    // 2D画像にオーバーレイ
                    cv::Scalar overlay_color;
                    if (show_depth_color_) {
                        // 深度に応じた色
                        overlay_color = getDepthColor(z);
                    } else {
                        // 検出IDに応じた色
                        overlay_color = point_color;
                    }
                    
                    // 円として描画（アルファブレンディング）
                    cv::Point2f center(x, y);
                    drawTransparentCircle(overlay_image, center, 
                                        point_size_, overlay_color, alpha_);
                }
            }
        }
    }
    
    /**
     * @brief 深度に応じた色を取得
     */
    cv::Scalar getDepthColor(float depth) {
        float normalized;
        
        if (depth < near_distance_) {
            // 近距離：青→緑
            normalized = depth / near_distance_;
            return cv::Scalar(255 * (1 - normalized), 255 * normalized, 0);
        } else if (depth < far_distance_) {
            // 中距離：緑→黄→赤
            normalized = (depth - near_distance_) / (far_distance_ - near_distance_);
            if (normalized < 0.5) {
                // 緑→黄
                return cv::Scalar(0, 255, 255 * (normalized * 2));
            } else {
                // 黄→赤
                return cv::Scalar(0, 255 * (2 - normalized * 2), 255);
            }
        } else {
            // 遠距離：赤
            return cv::Scalar(0, 0, 255);
        }
    }
    
    /**
     * @brief 検出IDに応じた色を取得
     */
    cv::Scalar getColorForDetection(int id) {
        const std::vector<cv::Scalar> colors = {
            cv::Scalar(255, 0, 0),    // 青
            cv::Scalar(0, 255, 0),    // 緑
            cv::Scalar(0, 0, 255),    // 赤
            cv::Scalar(255, 255, 0),  // シアン
            cv::Scalar(255, 0, 255),  // マゼンタ
            cv::Scalar(0, 255, 255),  // 黄
            cv::Scalar(128, 255, 0),  // 黄緑
            cv::Scalar(255, 128, 0),  // オレンジ
        };
        
        return colors[id % colors.size()];
    }
    
    /**
     * @brief 透明度付きの円を描画
     */
    void drawTransparentCircle(cv::Mat& img, const cv::Point2f& center,
                               float radius, const cv::Scalar& color, float alpha) {
        // 描画範囲計算
        int x_min = std::max(0, static_cast<int>(center.x - radius));
        int x_max = std::min(img.cols - 1, static_cast<int>(center.x + radius));
        int y_min = std::max(0, static_cast<int>(center.y - radius));
        int y_max = std::min(img.rows - 1, static_cast<int>(center.y + radius));
        
        float radius_sq = radius * radius;
        
        // ピクセルごとにアルファブレンド
        for (int y = y_min; y <= y_max; ++y) {
            for (int x = x_min; x <= x_max; ++x) {
                float dx = x - center.x;
                float dy = y - center.y;
                float dist_sq = dx * dx + dy * dy;
                
                if (dist_sq <= radius_sq) {
                    cv::Vec3b& pixel = img.at<cv::Vec3b>(y, x);
                    pixel[0] = pixel[0] * (1 - alpha) + color[0] * alpha;
                    pixel[1] = pixel[1] * (1 - alpha) + color[1] * alpha;
                    pixel[2] = pixel[2] * (1 - alpha) + color[2] * alpha;
                }
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), 
        "\n===== 点群オーバーレイノード =====\n"
        "検出したアスパラガス領域の点群を\n"
        "リアルタイムで映像に重ね描き\n"
        "\n【表示モード】\n"
        "- 深度カラー: 近い=青、遠い=赤\n"
        "- 検出別カラー: 各アスパラを異なる色で\n"
        "\n【パラメータ設定】\n"
        "ros2 param set /point_cloud_overlay show_depth_color true\n"
        "ros2 param set /point_cloud_overlay point_size 3.0\n"
        "ros2 param set /point_cloud_overlay alpha 0.6");
    
    rclcpp::spin(std::make_shared<PointCloudOverlayNode>());
    rclcpp::shutdown();
    return 0;
}