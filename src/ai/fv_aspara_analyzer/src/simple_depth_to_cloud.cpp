/**
 * @file simple_depth_to_cloud.cpp
 * @brief 深度画像から直接ポイントクラウドを生成する簡単な例
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class SimpleDepthToCloud : public rclcpp::Node {
public:
    SimpleDepthToCloud() : Node("simple_depth_to_cloud") {
        // サブスクライバー
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/fv/d415/depth/image_rect_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                depth_msg_ = msg;
                processIfReady();
            });
            
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/fv/d415/depth/camera_info", 10,
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                // カメラパラメータを保存
                fx_ = msg->k[0];
                fy_ = msg->k[4];
                cx_ = msg->k[2];
                cy_ = msg->k[5];
                
                RCLCPP_INFO_ONCE(this->get_logger(), 
                    "カメラパラメータ取得: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f",
                    fx_, fy_, cx_, cy_);
                
                camera_info_received_ = true;
            });
            
        // パブリッシャー
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/simple/points", 10);
    }
    
private:
    // カメラパラメータ
    float fx_, fy_, cx_, cy_;
    bool camera_info_received_ = false;
    
    // メッセージ
    sensor_msgs::msg::Image::SharedPtr depth_msg_;
    
    // ROS2インターフェース
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    
    void processIfReady() {
        if (!camera_info_received_ || !depth_msg_) return;
        
        // 深度画像をOpenCVに変換
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(depth_msg_, depth_msg_->encoding);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        // ポイントクラウド作成
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // 深度画像の各ピクセルを3D点に変換
        for (int v = 0; v < cv_ptr->image.rows; v += 4) {  // 4ピクセル飛ばしで高速化
            for (int u = 0; u < cv_ptr->image.cols; u += 4) {
                // 深度値取得（単位：mm）
                uint16_t depth_mm = cv_ptr->image.at<uint16_t>(v, u);
                
                // 0は無効値
                if (depth_mm == 0) continue;
                
                // メートルに変換
                float z = depth_mm * 0.001f;
                
                // 有効範囲チェック
                if (z < 0.1 || z > 3.0) continue;
                
                // ピンホールカメラモデルで3D座標計算
                float x = (u - cx_) * z / fx_;
                float y = (v - cy_) * z / fy_;
                
                // 点を追加
                pcl::PointXYZ point;
                point.x = x;
                point.y = y;
                point.z = z;
                cloud->push_back(point);
            }
        }
        
        // ROS2メッセージに変換してパブリッシュ
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header = depth_msg_->header;
        cloud_pub_->publish(output);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "ポイントクラウド生成: %zu点", cloud->size());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), 
        "===== シンプルな深度→ポイントクラウド変換 =====\n"
        "必要なデータ:\n"
        "1. 深度画像 (/fv/d415/depth/image_rect_raw)\n"
        "2. カメラ情報 (/fv/d415/depth/camera_info)\n"
        "これだけでポイントクラウドが作れます！");
    
    rclcpp::spin(std::make_shared<SimpleDepthToCloud>());
    rclcpp::shutdown();
    return 0;
}

/**
 * ポイント：
 * 1. RealSense APIは一切使っていない
 * 2. 必要なのは深度画像とカメラパラメータだけ
 * 3. 数学的な変換で3D座標を計算
 * 
 * カメラパラメータの意味：
 * - fx, fy: レンズの焦点距離（ピクセル単位）
 * - cx, cy: 画像の中心座標（主点）
 * - これらはカメラの工場出荷時に校正済み
 */