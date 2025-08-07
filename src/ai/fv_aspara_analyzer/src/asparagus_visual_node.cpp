/**
 * @file asparagus_clean_visual_node.cpp
 * @brief アスパラガス検出用クリーンビジュアライゼーションノード
 * @details cvx::drawShadowTextを使った高品質な日本語描画とスムーズアニメーション
 * @author Claude + Takashi Otsuka
 * @date 2025
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cvx/cvx_text.hpp>
#include <map>
#include <memory>
#include <string>
#include <chrono>
#include <cmath>
#include <set>
#include <limits>

/**
 * @brief 選択カーソル構造体
 */
struct SelectionCursor {
    double x = 0.0, y = 0.0;
    double target_x = 0.0, target_y = 0.0;
    double size = 15.0;
    double rotation = 0.0;
    double alpha = 0.2; // 80%透明度
    
    void update(double dt) {
        // カーソル移動
        x += (target_x - x) * 0.15;
        y += (target_y - y) * 0.15;
        
        // ゆっくり回転
        rotation += dt * 30.0;
    }
    
    void draw(cv::Mat& image) {
        cv::Point center(static_cast<int>(x), static_cast<int>(y));
        
        // 薄い白い十字（80%透明度）
        int cross_length = static_cast<int>(size);
        std::vector<int> angles = {0, 90, 180, 270};
        
        cv::Mat overlay = image.clone();
        
        for (int angle : angles) {
            double total_angle = rotation + angle;
            int end_x = center.x + static_cast<int>(cross_length * std::cos(total_angle * M_PI / 180.0));
            int end_y = center.y + static_cast<int>(cross_length * std::sin(total_angle * M_PI / 180.0));
            
            cv::line(overlay, center, cv::Point(end_x, end_y), cv::Scalar(255, 255, 255), 2);
        }
        
        // アルファブレンド
        cv::addWeighted(image, 1.0 - alpha, overlay, alpha, 0, image);
    }
};

/**
 * @brief アニメーション付きバウンディングボックス
 */
struct AnimatedBoundingBox {
    // 位置・サイズ
    double target_x = 0.0, target_y = 0.0, target_w = 0.0, target_h = 0.0;
    double current_x = 0.0, current_y = 0.0, current_w = 0.0, current_h = 0.0;
    
    // 検出情報
    double confidence = 0.0;
    std::string class_name;
    std::string class_id;
    std::chrono::steady_clock::time_point last_seen;
    bool is_selected = false;
    
    // アニメーション
    double fade_alpha = 1.0;
    double lerp_speed = 0.15;
    
    void update(double dt) {
        // スムーズ移動
        current_x += (target_x - current_x) * lerp_speed;
        current_y += (target_y - current_y) * lerp_speed;
        current_w += (target_w - current_w) * lerp_speed;
        current_h += (target_h - current_h) * lerp_speed;
        
        // フェード処理
        auto now = std::chrono::steady_clock::now();
        auto time_since_seen = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_seen);
        
        if (time_since_seen.count() > 500) { // 0.5秒以上見えない
            fade_alpha = std::max(0.0, fade_alpha - 2.0 * dt);
        } else {
            fade_alpha = std::min(1.0, fade_alpha + 3.0 * dt);
        }
    }
    
    bool should_remove() const {
        return fade_alpha <= 0.01;
    }
};

/**
 * @class AsparagusVisualNode
 * @brief アスパラガス検出用ビジュアライゼーションノード
 */
class AsparagusVisualNode : public rclcpp::Node {
public:
    explicit AsparagusVisualNode(const std::string& camera_name = "d415")
        : Node("asparagus_visual_" + camera_name), camera_name_(camera_name)
    {
        // サブスクライバー
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/fv/" + camera_name + "/color/image_raw", 10,
            std::bind(&AsparagusVisualNode::image_callback, this, std::placeholders::_1)
        );
        
        detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/fv/" + camera_name + "/object_detection/detections", 10,
            std::bind(&AsparagusVisualNode::detection_callback, this, std::placeholders::_1)
        );
        
        // パブリッシャー
        animated_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/fv/" + camera_name + "/asparagus_visual/image", 10
        );
        
        // 60FPSアニメーションタイマー
        animation_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(16), // ~60FPS
            std::bind(&AsparagusVisualNode::animation_update, this)
        );
        
        last_update_time_ = std::chrono::steady_clock::now();
        
        RCLCPP_INFO(this->get_logger(), "🎯✨ %s用 ビジュアルシステム起動", camera_name.c_str());
        RCLCPP_INFO(this->get_logger(), "📺 出力: /fv/%s/asparagus_visual/image", camera_name.c_str());
        RCLCPP_INFO(this->get_logger(), "💫 cvx::drawShadowText + 60FPS");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        current_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    
    void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        auto current_time = std::chrono::steady_clock::now();
        
        // 全ての検出を処理（複数選択対応）
        std::set<std::string> used_detections;
        
        // 新しい検出データを準備
        std::vector<std::map<std::string, double>> new_detections;
        for (const auto& detection : msg->detections) {
            double center_x = detection.bbox.center.position.x;
            double center_y = detection.bbox.center.position.y;
            double width = detection.bbox.size_x;
            double height = detection.bbox.size_y;
            double confidence = detection.results.empty() ? 0.0 : detection.results[0].hypothesis.score;
            std::string class_id = detection.results.empty() ? "" : detection.results[0].hypothesis.class_id;
            
            new_detections.push_back({
                {"center_x", center_x},
                {"center_y", center_y},
                {"width", width},
                {"height", height},
                {"confidence", confidence}
            });
            
            // クラス名を日本語に変換
            std::string class_name = get_japanese_class_name(class_id);
        }
        
        // 既存ボックスの更新
        for (auto& [box_id, box] : animated_boxes_) {
            if (!box) continue;
            
            // 最も近い検出を探す
            double best_distance = std::numeric_limits<double>::max();
            size_t best_match_idx = SIZE_MAX;
            
            for (size_t i = 0; i < new_detections.size(); ++i) {
                if (used_detections.count(std::to_string(i))) continue;
                
                double distance = std::sqrt(
                    std::pow(new_detections[i]["center_x"] - box->current_x, 2) +
                    std::pow(new_detections[i]["center_y"] - box->current_y, 2)
                );
                
                if (distance < 80 && distance < best_distance) {
                    best_distance = distance;
                    best_match_idx = i;
                }
            }
            
            if (best_match_idx != SIZE_MAX) {
                used_detections.insert(std::to_string(best_match_idx));
                // 既存ボックス更新
                const auto& det = new_detections[best_match_idx];
                box->target_x = det.at("center_x");
                box->target_y = det.at("center_y");
                box->target_w = det.at("width");
                box->target_h = det.at("height");
                box->confidence = det.at("confidence");
                box->last_seen = current_time;
            }
        }
        
        // 新しい検出の追加
        for (size_t i = 0; i < new_detections.size(); ++i) {
            if (used_detections.count(std::to_string(i))) continue;
            
            const auto& det = new_detections[i];
            std::string box_id = "detection_" + std::to_string(animated_boxes_.size());
            auto box = std::make_unique<AnimatedBoundingBox>();
            
            box->target_x = box->current_x = det.at("center_x");
            box->target_y = box->current_y = det.at("center_y");
            box->target_w = box->current_w = det.at("width");
            box->target_h = box->current_h = det.at("height");
            box->confidence = det.at("confidence");
            box->class_name = get_japanese_class_name(""); // 仮
            box->last_seen = current_time;
            box->fade_alpha = 0.0;
            
            // 最初の検出または最高信頼度を選択
            if (!has_selection_) {
                box->is_selected = true;
                selection_cursor_.target_x = det.at("center_x");
                selection_cursor_.target_y = det.at("center_y");
                has_selection_ = true;
                RCLCPP_INFO(this->get_logger(), "選択: %s (%.1f%%)", 
                           box->class_name.c_str(), box->confidence * 100.0);
            }
            
            animated_boxes_[box_id] = std::move(box);
        }
        
        update_selection();
    }
    
    void animation_update() {
        auto current_time = std::chrono::steady_clock::now();
        auto dt = std::chrono::duration<double>(current_time - last_update_time_).count();
        last_update_time_ = current_time;
        
        // カーソル更新
        selection_cursor_.update(dt);
        
        // ボックス更新と削除
        std::vector<std::string> boxes_to_remove;
        for (auto& [box_id, box] : animated_boxes_) {
            if (!box) continue;
            box->update(dt);
            if (box->should_remove()) {
                boxes_to_remove.push_back(box_id);
            }
        }
        
        for (const auto& box_id : boxes_to_remove) {
            animated_boxes_.erase(box_id);
        }
        
        if (animated_boxes_.empty()) {
            has_selection_ = false;
        }
        
        process_and_publish();
    }
    
    void process_and_publish() {
        if (!current_image_) return;
        
        cv::Mat result = current_image_->image.clone();
        
        // 選択時に微妙に明るく
        if (has_selection_) {
            cv::convertScaleAbs(result, result, 1.05, 5);
        }
        
        // 選択ボックス描画
        for (const auto& [box_id, box] : animated_boxes_) {
            if (!box || box->fade_alpha <= 0.01) continue;
            draw_clean_box(result, *box);
        }
        
        // 選択カーソル描画
        if (has_selection_) {
            selection_cursor_.draw(result);
        }
        
        // 簡単なUI描画
        draw_simple_ui(result);
        
        // 配信
        auto animated_msg = cv_bridge::CvImage(current_image_->header, "bgr8", result).toImageMsg();
        animated_pub_->publish(*animated_msg);
    }
    
    void draw_clean_box(cv::Mat& image, const AnimatedBoundingBox& box) {
        int center_x = static_cast<int>(box.current_x);
        int center_y = static_cast<int>(box.current_y);
        int half_w = static_cast<int>(box.current_w / 2);
        int half_h = static_cast<int>(box.current_h / 2);
        
        int x1 = center_x - half_w;
        int y1 = center_y - half_h;
        int x2 = center_x + half_w;
        int y2 = center_y + half_h;
        
        if (box.is_selected) {
            // 選択時：緑のDropShadow付きボックス（不透明）
            cv::Mat shadow_overlay = image.clone();
            cv::rectangle(shadow_overlay, cv::Point(x1 + 1, y1 + 1), cv::Point(x2 + 1, y2 + 1), 
                         cv::Scalar(0, 0, 0), 2);
            cv::addWeighted(image, 0.9, shadow_overlay, 0.1, 0, image);
            
            // メインボックス（緑）
            cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 200, 0), 3);
        } else {
            // 非選択時：40%透明度の白
            cv::Mat overlay = image.clone();
            cv::rectangle(overlay, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 255, 255), 2);
            cv::addWeighted(image, 0.6, overlay, 0.4, 0, image);
        }
        
        // ラベル（cvx::drawShadowText使用）
        std::string label_text = box.class_name + " " + std::to_string(static_cast<int>(box.confidence * 100)) + "%";
        if (box.is_selected) {
            label_text = "選択中: " + label_text;
        }
        
        // ラベル背景
        int text_size_w = static_cast<int>(label_text.length() * 12); // 概算
        
        if (box.is_selected) {
            // ラベルDropShadow（薄く）
            cv::Mat shadow_overlay = image.clone();
            cv::rectangle(shadow_overlay, cv::Point(x1 + 1, y1 - 29), 
                         cv::Point(x1 + text_size_w + 11, y1 + 1), cv::Scalar(0, 0, 0), -1);
            cv::addWeighted(image, 0.95, shadow_overlay, 0.05, 0, image);
        }
        
        // 背景
        cv::Scalar bg_color = box.is_selected ? cv::Scalar(0, 150, 0) : cv::Scalar(80, 80, 80);
        cv::rectangle(image, cv::Point(x1, y1 - 30), cv::Point(x1 + text_size_w + 10, y1), bg_color, -1);
        
        // テキスト描画（cvx::drawShadowText）
        cvx::drawShadowText(image, label_text, cv::Point(x1 + 5, y1 - 8), 
                           cvx::WHITE, cvx::BLACK, 0.6, 2, 0);
    }
    
    void draw_simple_ui(cv::Mat& image) {
        // ステータス
        std::string status_text;
        if (has_selection_) {
            auto selected_boxes = get_selected_boxes();
            if (!selected_boxes.empty()) {
                const auto& selected_box = selected_boxes[0];
                status_text = camera_name_ + ": " + selected_box->class_name + " " + 
                             std::to_string(static_cast<int>(selected_box->confidence * 100)) + "%";
            } else {
                status_text = camera_name_ + ": 選択中...";
            }
        } else {
            status_text = camera_name_ + ": スキャン中...";
        }
        
        // ステータス背景
        int text_width = static_cast<int>(status_text.length() * 10);
        cv::rectangle(image, cv::Point(10, 10), cv::Point(20 + text_width, 35), cv::Scalar(50, 50, 50), -1);
        
        // ステータステキスト（cvx::drawText）
        cvx::drawText(image, status_text, cv::Point(15, 28), cvx::WHITE, 0.6, 2, 0);
    }
    
    std::string get_japanese_class_name(const std::string& class_id) {
        if (class_id == "0") return "アスパラガス";
        if (class_id == "1") return "穂先";
        return "オブジェクト";
    }
    
    void update_selection() {
        // 選択カーソルを最も信頼度の高い選択ボックスに追従
        auto selected_boxes = get_selected_boxes();
        if (!selected_boxes.empty()) {
            const auto& best_selected = *std::max_element(selected_boxes.begin(), selected_boxes.end(),
                [](const AnimatedBoundingBox* a, const AnimatedBoundingBox* b) {
                    return a->confidence < b->confidence;
                });
            selection_cursor_.target_x = best_selected->current_x;
            selection_cursor_.target_y = best_selected->current_y;
        }
    }
    
    std::vector<AnimatedBoundingBox*> get_selected_boxes() {
        std::vector<AnimatedBoundingBox*> selected;
        for (const auto& [box_id, box] : animated_boxes_) {
            if (box && box->is_selected) {
                selected.push_back(box.get());
            }
        }
        return selected;
    }
    
    // メンバ変数
    std::string camera_name_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr animated_pub_;
    rclcpp::TimerBase::SharedPtr animation_timer_;
    
    cv_bridge::CvImagePtr current_image_;
    std::map<std::string, std::unique_ptr<AnimatedBoundingBox>> animated_boxes_;
    SelectionCursor selection_cursor_;
    bool has_selection_ = false;
    
    std::chrono::steady_clock::time_point last_update_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    std::string camera_name = "d415";
    if (argc > 1) {
        camera_name = argv[1];
    }
    
    auto node = std::make_shared<AsparagusVisualNode>(camera_name);
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}