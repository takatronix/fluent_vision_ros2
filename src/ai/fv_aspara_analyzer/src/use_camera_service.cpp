/**
 * @file use_camera_service.cpp
 * @brief RealSenseのサービスを使ってカメラ情報を取得
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/srv/get_camera_info.hpp>

class CameraServiceExample : public rclcpp::Node {
public:
    CameraServiceExample() : Node("camera_service_example") {
        // サービスクライアント作成
        camera_info_client_ = this->create_client<sensor_msgs::srv::GetCameraInfo>(
            "/fv/d415/get_camera_info");
            
        // タイマーで定期的に取得
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&CameraServiceExample::getCameraInfo, this));
            
        RCLCPP_INFO(this->get_logger(), "カメラ情報サービスクライアント起動");
    }
    
private:
    rclcpp::Client<sensor_msgs::srv::GetCameraInfo>::SharedPtr camera_info_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    void getCameraInfo() {
        if (!camera_info_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "カメラ情報サービスが利用できません");
            return;
        }
        
        // リクエスト作成（GetCameraInfoは空のリクエスト）
        auto request = std::make_shared<sensor_msgs::srv::GetCameraInfo::Request>();
        
        // 非同期呼び出し
        camera_info_client_->async_send_request(request,
            [this](rclcpp::Client<sensor_msgs::srv::GetCameraInfo>::SharedFuture future) {
                try {
                    auto response = future.get();
                    const auto& info = response->camera_info;
                    
                    RCLCPP_INFO(this->get_logger(), 
                        "\n===== カメラ情報取得成功 =====\n"
                        "解像度: %dx%d\n"
                        "焦点距離: fx=%.1f, fy=%.1f\n"
                        "主点: cx=%.1f, cy=%.1f\n"
                        "歪み係数: D=[%.3f, %.3f, %.3f, %.3f, %.3f]",
                        info.width, info.height,
                        info.k[0], info.k[4],  // fx, fy
                        info.k[2], info.k[5],  // cx, cy
                        info.d.size() > 0 ? info.d[0] : 0.0,
                        info.d.size() > 1 ? info.d[1] : 0.0,
                        info.d.size() > 2 ? info.d[2] : 0.0,
                        info.d.size() > 3 ? info.d[3] : 0.0,
                        info.d.size() > 4 ? info.d[4] : 0.0);
                        
                    // これでポイントクラウド生成に必要な全データが取得できた！
                    processWithCameraInfo(info);
                    
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "エラー: %s", e.what());
                }
            });
    }
    
    void processWithCameraInfo(const sensor_msgs::msg::CameraInfo& info) {
        // ここでカメラ情報を使った処理
        float fx = info.k[0];
        float fy = info.k[4];
        float cx = info.k[2];
        float cy = info.k[5];
        
        RCLCPP_INFO(this->get_logger(), 
            "この情報で深度画像から3D座標を計算できます！\n"
            "X = (u - %.1f) * Z / %.1f\n"
            "Y = (v - %.1f) * Z / %.1f",
            cx, fx, cy, fy);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), 
        "===== RealSenseサービス使用例 =====\n"
        "メリット:\n"
        "1. camera_infoトピックを常時購読する必要がない\n"
        "2. 必要な時だけ取得できる\n"
        "3. 解像度変更後も正しい値が取得できる");
    
    rclcpp::spin(std::make_shared<CameraServiceExample>());
    rclcpp::shutdown();
    return 0;
}