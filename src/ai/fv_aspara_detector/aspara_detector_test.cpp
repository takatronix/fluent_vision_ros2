/**
 * @file aspara_detector_test.cpp
 * @brief アスパラガス検出のテストプログラム
 * @details FluentCloudライブラリを使用した改良版
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fluent_cloud/fluent_cloud.hpp>
#include <fluent_cloud/segmentation/asparagus_segmentation.hpp>
#include <fluent_cloud/features/asparagus_analyzer.hpp>
#include <chrono>

class AsparagusDetectorTest : public rclcpp::Node {
public:
    AsparagusDetectorTest() : Node("asparagus_detector_test") {
        RCLCPP_INFO(this->get_logger(), "===== アスパラガス検出テスト開始 =====");
        
        // パラメータ
        this->declare_parameter<std::string>("pointcloud_topic", "/fv/d415/points");
        this->declare_parameter<std::string>("detection_topic", "/fv/d415/object_detection/detections");
        this->declare_parameter<std::string>("mask_topic", "/fv/d415/segmentation_mask/image");
        this->declare_parameter<std::string>("output_topic", "/asparagus/detected");
        
        // サブスクライバー
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("pointcloud_topic").as_string(), 10,
            std::bind(&AsparagusDetectorTest::pointcloudCallback, this, std::placeholders::_1));
            
        detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            this->get_parameter("detection_topic").as_string(), 10,
            std::bind(&AsparagusDetectorTest::detectionCallback, this, std::placeholders::_1));
            
        mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            this->get_parameter("mask_topic").as_string(), 10,
            std::bind(&AsparagusDetectorTest::maskCallback, this, std::placeholders::_1));
            
        // パブリッシャー
        output_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("output_topic").as_string(), 10);
            
        result_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/asparagus/analysis_result", 10);
    }
    
private:
    // 最新データ保存
    sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
    vision_msgs::msg::Detection2DArray::SharedPtr latest_detections_;
    cv::Mat latest_mask_;
    
    // ROS2インターフェース
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
    
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        latest_pointcloud_ = msg;
        processIfReady();
    }
    
    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        latest_detections_ = msg;
        RCLCPP_INFO(this->get_logger(), "検出数: %zu", msg->detections.size());
    }
    
    void maskCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            latest_mask_ = cv_ptr->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "マスク変換エラー: %s", e.what());
        }
    }
    
    void processIfReady() {
        if (!latest_pointcloud_ || !latest_detections_) {
            return;
        }
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // PCLに変換
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*latest_pointcloud_, *cloud);
        
        RCLCPP_INFO(this->get_logger(), "点群サイズ: %zu点", cloud->size());
        
        // FluentCloudで処理
        using namespace fluent_cloud;
        
        // 1. 基本的なフィルタリング
        auto filtered = FluentCloud<pcl::PointXYZRGB>::from(cloud)
            .filterByDistance(0.1, 2.0)  // 10cm-2m
            .removeOutliers(50, 1.0)     // 統計的外れ値除去
            .downsample(0.005);          // 5mmボクセル
            
        RCLCPP_INFO(this->get_logger(), "フィルタリング後: %zu点", filtered.size());
        
        // 2. アスパラガス検出（複数の方法を試す）
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> asparagus_candidates;
        
        // 方法1: YOLOの検出結果を使用
        for (const auto& detection : latest_detections_->detections) {
            if (detection.results.empty()) continue;
            
            // クラス名チェック（アスパラガスかどうか）
            std::string class_name = detection.results[0].hypothesis.class_id;
            if (class_name != "asparagus" && class_name != "0") continue;  // クラスID確認
            
            // バウンディングボックス内の点群を抽出
            cv::Rect bbox(
                detection.bbox.center.position.x - detection.bbox.size_x / 2,
                detection.bbox.center.position.y - detection.bbox.size_y / 2,
                detection.bbox.size_x,
                detection.bbox.size_y
            );
            
            // TODO: カメラ情報を使った正確な抽出
            // 簡易版：高さベースでフィルタ
            auto candidate = filtered.clone()
                .filterByHeight(0.0, 0.6)  // 地面から60cmまで
                .cluster(0.02, 50, 5000)   // クラスタリング
                .toPointCloud();
                
            if (candidate->size() > 100) {
                asparagus_candidates.push_back(candidate);
            }
        }
        
        // 方法2: マスク画像を使用（利用可能な場合）
        if (!latest_mask_.empty()) {
            // マスクベースセグメンテーション
            segmentation::AsparagusSegmentation<pcl::PointXYZRGB> seg;
            // TODO: カメラ情報を追加
            // auto mask_result = seg.segmentWithMask(filtered.toPointCloud(), latest_mask_, camera_info);
        }
        
        // 方法3: 色ベース検出（フォールバック）
        if (asparagus_candidates.empty()) {
            auto color_result = filtered.clone()
                .extractAsparagus()  // 緑色抽出
                .cluster(0.02, 100, 5000)
                .toPointCloud();
                
            if (color_result->size() > 100) {
                asparagus_candidates.push_back(color_result);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "アスパラガス候補数: %zu", asparagus_candidates.size());
        
        // 3. 各候補の解析
        std::stringstream result_msg;
        result_msg << "=== アスパラガス解析結果 ===\n";
        
        for (size_t i = 0; i < asparagus_candidates.size(); ++i) {
            features::AsparagusAnalyzer<pcl::PointXYZRGB> analyzer;
            auto features = analyzer.analyze(asparagus_candidates[i]);
            
            result_msg << "\n候補 #" << (i+1) << ":\n";
            result_msg << "  長さ: " << std::fixed << std::setprecision(1) 
                      << features.length * 100 << " cm\n";
            result_msg << "  真っ直ぐ度: " << std::fixed << std::setprecision(0)
                      << features.straightness * 100 << " %\n";
            result_msg << "  直径: " << std::fixed << std::setprecision(1)
                      << features.diameter * 1000 << " mm\n";
            result_msg << "  収穫適性: " << (features.harvestable ? "可" : "不可") 
                      << " (" << features.harvest_reason << ")\n";
            
            // 収穫可能なものは緑、不可なものは赤で色付け
            if (features.harvestable) {
                FluentCloud<pcl::PointXYZRGB>::from(asparagus_candidates[i])
                    .setColor(0, 255, 0);  // 緑
            } else {
                FluentCloud<pcl::PointXYZRGB>::from(asparagus_candidates[i])
                    .setColor(255, 0, 0);  // 赤
            }
        }
        
        // 4. 結果の出力
        if (!asparagus_candidates.empty()) {
            // 全候補を結合
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_asparagus(new pcl::PointCloud<pcl::PointXYZRGB>);
            for (const auto& candidate : asparagus_candidates) {
                *all_asparagus += *candidate;
            }
            
            // パブリッシュ
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*all_asparagus, output_msg);
            output_msg.header = latest_pointcloud_->header;
            output_pub_->publish(output_msg);
            
            // テキスト結果
            std_msgs::msg::String text_result;
            text_result.data = result_msg.str();
            result_pub_->publish(text_result);
        }
        
        // 処理時間
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        RCLCPP_INFO(this->get_logger(), "処理時間: %ld ms", duration.count());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AsparagusDetectorTest>());
    rclcpp::shutdown();
    return 0;
}