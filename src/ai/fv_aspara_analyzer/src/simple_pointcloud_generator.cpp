/**
 * @file simple_pointcloud_generator.cpp
 * @brief シンプルな点群生成ノード（最小構成）
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class SimplePointCloudGenerator : public rclcpp::Node {
public:
    SimplePointCloudGenerator() : Node("simple_pointcloud_generator") {
        RCLCPP_INFO(this->get_logger(), "シンプル点群生成ノード起動");
        
        // カメラパラメータ（D415）
        fx_ = 615.92f;
        fy_ = 615.58f;
        cx_ = 321.84f;
        cy_ = 237.25f;
        
        // サブスクライバー
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/fv/d415/depth/image_rect_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                depth_msg_ = msg;
            });
            
        color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/fv/d415/color/image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                color_msg_ = msg;
            });
            
        detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/fv/d415/object_detection/detections", 10,
            std::bind(&SimplePointCloudGenerator::detectionCallback, this, std::placeholders::_1));
            
        // パブリッシャー
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/asparagus/points", 10);
    }
    
private:
    float fx_, fy_, cx_, cy_;
    sensor_msgs::msg::Image::SharedPtr depth_msg_;
    sensor_msgs::msg::Image::SharedPtr color_msg_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    
    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        if (!depth_msg_ || !color_msg_ || msg->detections.empty()) return;
        
        // 最初の検出のみ処理
        const auto& det = msg->detections[0];
        
        // ROI計算
        int cx = det.bbox.center.position.x;
        int cy = det.bbox.center.position.y;
        int w = det.bbox.size_x;
        int h = det.bbox.size_y;
        
        // 点群生成
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        
        cv_bridge::CvImageConstPtr depth_cv = cv_bridge::toCvShare(depth_msg_);
        cv_bridge::CvImageConstPtr color_cv = cv_bridge::toCvShare(color_msg_, "bgr8");
        
        // ROI内をスキャン
        for (int y = cy - h/2; y < cy + h/2; y += 2) {
            for (int x = cx - w/2; x < cx + w/2; x += 2) {
                if (x < 0 || x >= depth_cv->image.cols || 
                    y < 0 || y >= depth_cv->image.rows) continue;
                
                float depth = depth_cv->image.at<uint16_t>(y, x) * 0.001f;
                if (depth > 0.1f && depth < 5.0f) {
                    pcl::PointXYZRGB point;
                    point.z = depth;
                    point.x = (x - cx_) * depth / fx_;
                    point.y = (y - cy_) * depth / fy_;
                    
                    cv::Vec3b bgr = color_cv->image.at<cv::Vec3b>(y, x);
                    point.b = bgr[0];
                    point.g = bgr[1];
                    point.r = bgr[2];
                    
                    cloud->points.push_back(point);
                }
            }
        }
        
        // 公開
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header = depth_msg_->header;
        cloud_msg.header.frame_id = "fv/d415/depth_optical_frame";
        cloud_pub_->publish(cloud_msg);
        
        RCLCPP_INFO(this->get_logger(), "点群生成: %zu点", cloud->size());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePointCloudGenerator>());
    rclcpp::shutdown();
    return 0;
}