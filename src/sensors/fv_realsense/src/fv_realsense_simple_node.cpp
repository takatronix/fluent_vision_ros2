/**
 * @file fv_realsense_simple_node.cpp
 * @brief Fluent Vision RealSenseシンプルノードの実装ファイル
 * @details テスト用のシンプルなRealSenseノード（実際のカメラなしでテスト画像を配信）
 * @author Takashi Otsuka
 * @date 2024
 * @version 1.0
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

/**
 * @class FVRealSenseSimpleNode
 * @brief Fluent Vision RealSenseシンプルノードクラス
 * @details 実際のRealSenseカメラなしでテスト画像を配信するシンプルなノード
 * 
 * 主な機能：
 * - テスト画像の生成（グラデーション背景、動く円、テキスト）
 * - 指定FPSでの画像配信
 * - パラメータによる画像サイズとFPSの設定
 */
class FVRealSenseSimpleNode : public rclcpp::Node
{
public:
    /**
     * @brief コンストラクタ
     * @details シンプルRealSenseノードの初期化と設定
     * 
     * 初期化内容：
     * - パラメータの読み込み
     * - パブリッシャーの初期化
     * - テスト画像パブリッシャーの作成
     */
    FVRealSenseSimpleNode() : Node("fv_realsense_simple")
    {
        RCLCPP_INFO(this->get_logger(), "🚀 FV RealSense Simple Node starting...");
        
        try {
            // ===== Step 1: パラメータの読み込み =====
            RCLCPP_INFO(this->get_logger(), "📋 Step 1: Loading parameters...");
            loadParameters();
            
            // ===== Step 2: パブリッシャーの初期化 =====
            RCLCPP_INFO(this->get_logger(), "📤 Step 2: Initializing publishers...");
            initializePublishers();
            
            // ===== Step 3: テスト画像パブリッシャーの作成 =====
            RCLCPP_INFO(this->get_logger(), "🖼️ Step 3: Creating test image publisher...");
            createTestImagePublisher();
            
            RCLCPP_INFO(this->get_logger(), "✅ FV RealSense Simple Node started successfully");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Exception during initialization: %s", e.what());
        }
    }
    
    /**
     * @brief デストラクタ
     * @details シンプルRealSenseノードの適切な終了処理
     */
    ~FVRealSenseSimpleNode()
    {
        RCLCPP_INFO(this->get_logger(), "🛑 Shutting down FV RealSense Simple Node...");
    }

private:
    /**
     * @brief パラメータの読み込み
     * @details カメラ設定パラメータを読み込み
     * 
     * 読み込み内容：
     * - カラー画像の幅と高さ
     * - フレームレート（FPS）
     */
    void loadParameters()
    {
        // ===== カメラ設定 =====
        color_width_ = this->declare_parameter("camera.color_width", 640);   // カラー画像幅
        color_height_ = this->declare_parameter("camera.color_height", 480); // カラー画像高さ
        color_fps_ = this->declare_parameter("camera.color_fps", 30);        // フレームレート
        
        RCLCPP_INFO(this->get_logger(), "📷 Camera settings: %dx%d @ %dfps", 
            color_width_, color_height_, color_fps_);
    }
    
    /**
     * @brief パブリッシャーの初期化
     * @details 画像パブリッシャーを作成
     * 
     * 作成内容：
     * - カラー画像パブリッシャー
     */
    void initializePublishers()
    {
        // ===== 画像パブリッシャーの作成 =====
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/fv_realsense/color/image_raw", 10);  // カラー画像トピック
        
        RCLCPP_INFO(this->get_logger(), "📤 Image publisher created: /fv_realsense/color/image_raw");
    }
    
    /**
     * @brief テスト画像パブリッシャーの作成
     * @details 指定FPSでテスト画像を配信するタイマーを作成
     * 
     * 作成内容：
     * - テスト画像配信用タイマー
     * - FPSに基づく配信間隔の設定
     */
    void createTestImagePublisher()
    {
        // ===== テスト画像配信用タイマーの作成 =====
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / color_fps_),  // FPSをミリ秒に変換
            std::bind(&FVRealSenseSimpleNode::publishTestImage, this));
        
        RCLCPP_INFO(this->get_logger(), "⏰ Test image timer created");
    }
    
    /**
     * @brief テスト画像の配信
     * @details カラフルなテスト画像を生成して配信
     * 
     * 生成内容：
     * - グラデーション背景
     * - 色付き矩形
     * - テキスト表示
     * - タイムスタンプ
     * - 動く円
     */
    void publishTestImage()
    {
        try {
            // ===== カラフルなテスト画像の作成（グラデーション背景付き） =====
            cv::Mat test_image = cv::Mat::zeros(color_height_, color_width_, CV_8UC3);
            
            // ===== グラデーション背景の作成 =====
            for (int y = 0; y < color_height_; y++) {
                for (int x = 0; x < color_width_; x++) {
                    int blue = (x * 255) / color_width_;   // X座標に基づく青成分
                    int green = (y * 255) / color_height_; // Y座標に基づく緑成分
                    int red = 128;                         // 固定の赤成分
                    test_image.at<cv::Vec3b>(y, x) = cv::Vec3b(blue, green, red);
                }
            }
            
            // ===== 色付き矩形の追加 =====
            cv::rectangle(test_image, cv::Point(50, 50), cv::Point(590, 430), cv::Scalar(0, 255, 0), 5);
            
            // ===== テキストの追加 =====
            cv::putText(test_image, "RealSense Test Image", cv::Point(150, 240), 
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 3);
            
            // ===== タイムスタンプの追加 =====
            auto now = this->now();
            std::string timestamp = std::to_string(now.seconds());
            cv::putText(test_image, "Time: " + timestamp, cv::Point(50, 50), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
            
            // ===== 動く円の追加 =====
            int circle_x = (static_cast<int>(now.seconds()) % 10) * 60 + 50;
            cv::circle(test_image, cv::Point(circle_x, 100), 20, cv::Scalar(255, 0, 255), -1);
            
            // ===== ROSメッセージへの変換 =====
            auto ros_image = cv_bridge::CvImage();
            ros_image.image = test_image;
            ros_image.encoding = "bgr8";
            ros_image.header.stamp = now;
            ros_image.header.frame_id = "camera_frame";
            
            // ===== 配信 =====
            image_pub_->publish(*ros_image.toImageMsg());
            
            RCLCPP_INFO(this->get_logger(), "📤 Published colorful test image at time: %s", timestamp.c_str());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Error publishing test image: %s", e.what());
        }
    }
    
    // ===== パラメータ =====
    int color_width_;   // カラー画像幅
    int color_height_;  // カラー画像高さ
    int color_fps_;     // フレームレート
    
    // ===== パブリッシャー =====
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;  // 画像パブリッシャー
    
    // ===== タイマー =====
    rclcpp::TimerBase::SharedPtr timer_;  // テスト画像配信用タイマー
};

/**
 * @brief メイン関数
 * @param argc コマンドライン引数の数
 * @param argv コマンドライン引数の配列
 * @return int 終了コード
 * @details シンプルRealSenseノードの初期化と実行
 * 
 * 実行内容：
 * - ROS2の初期化
 * - シンプルRealSenseノードの作成
 * - ノードの実行（スピン）
 * - 適切な終了処理
 */
int main(int argc, char** argv)
{
    try {
        rclcpp::init(argc, argv);
        
        RCLCPP_INFO(rclcpp::get_logger("fv_realsense_simple"), "🚀 Starting FV RealSense Simple Node...");
        
        auto node = std::make_shared<FVRealSenseSimpleNode>();
        
        if (node) {
            RCLCPP_INFO(rclcpp::get_logger("fv_realsense_simple"), "✅ Node created successfully");
            rclcpp::spin(node);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("fv_realsense_simple"), "❌ Failed to create node");
            return 1;
        }
        
        rclcpp::shutdown();
        return 0;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("fv_realsense_simple"), "❌ Exception in main: %s", e.what());
        return 1;
    }
} 