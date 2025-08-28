/**
 * @file fluent_animation.cpp
 * @brief FluentAnimation - 美しく流れるようなアニメーションライブラリ実装
 */

#include <fluent_vision/fluent_animation.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>

namespace fluent_vision {

// ======== SpotlightEffect 静的メンバー初期化 ========
cv::Point2f SpotlightEffect::current_center_ = cv::Point2f(0, 0);
float SpotlightEffect::current_radius_ = 0.0f;
float SpotlightEffect::current_intensity_ = 1.0f;

// ======== FluentVision拡張 - アニメーション統合 ========
// FluentVision& FluentVision::animate(FluentAnimation animation) {
//     // アニメーションをマネージャーに追加
//     auto& manager = AnimationManager::getInstance();
//     static int animation_counter = 0;
//     std::string name = "auto_animation_" + std::to_string(animation_counter++);
//     manager.add(name, animation);
    
//     return *this;
// }

FluentVision& FluentVision::spotlight(cv::Point2f center, float radius, float duration) {
    auto spotlight_anim = FluentAnimation::spotlightOn(center, radius)
        .duration(duration)
        .ease(EaseType::EaseOut)
        .onUpdate([this, center](float value) {
            // TODO: 実装が必要
            // if (current_mat_.empty()) return;
            // cv::Mat result = SpotlightEffect::applyToImage(current_mat_, center, 
            //                                               value * 150.0f, 1.2f);
            // current_mat_ = result;
        })
        .play();
    
    // animate(spotlight_anim);
    return *this;
}

FluentVision& FluentVision::fadeBackground(float darkness, float duration) {
    auto fade_anim = FluentAnimation::fade()
        .from(1.0f).to(darkness)
        .duration(duration)
        .ease(EaseType::EaseInOut)
        .onUpdate([this](float value) {
            // TODO: 実装が必要
            // if (current_mat_.empty()) return;
            // current_mat_ = current_mat_ * value;
        })
        .play();
    
    // animate(fade_anim);
    return *this;
}

FluentVision& FluentVision::pulseGlow(float min_intensity, float max_intensity, float duration) {
    auto pulse_anim = FluentAnimation::pulseEffect(min_intensity, max_intensity)
        .duration(duration)
        .onUpdate([this](float value) {
            // TODO: 実装が必要
            // if (current_mat_.empty()) return;
            // cv::Mat brightened;
            // current_mat_.convertTo(brightened, -1, value, 0);
            // current_mat_ = brightened;
        })
        .play();
    
    // animate(pulse_anim);
    return *this;
}

// ======== アニメーション統合ユーティリティ ========
FluentVision& FluentVision::when_detected(std::function<bool()> condition, 
                                          std::function<void(FluentVision&)> animation_block) {
    if (condition()) {
        animation_block(*this);
    }
    return *this;
}

// ======== 高度なアニメーション効果 ========
namespace AnimationEffects {

cv::Mat createRadialGradientMask(cv::Size size, cv::Point2f center, float inner_radius, float outer_radius) {
    cv::Mat mask = cv::Mat::zeros(size, CV_32F);
    
    for (int y = 0; y < size.height; y++) {
        for (int x = 0; x < size.width; x++) {
            float distance = cv::norm(cv::Point2f(x, y) - center);
            
            if (distance <= inner_radius) {
                mask.at<float>(y, x) = 1.0f;
            } else if (distance <= outer_radius) {
                float ratio = (outer_radius - distance) / (outer_radius - inner_radius);
                mask.at<float>(y, x) = ratio;
            }
        }
    }
    
    return mask;
}

cv::Mat applyDynamicSpotlight(const cv::Mat& input, 
                             const std::vector<cv::Point2f>& centers,
                             const std::vector<float>& radii,
                             const std::vector<float>& intensities,
                             float background_darkness = 0.3f) {
    
    if (centers.empty()) {
        // スポットライトなし - 全体を暗く
        cv::Mat result;
        input.convertTo(result, -1, background_darkness, 0);
        return result;
    }
    
    cv::Mat result;
    input.convertTo(result, CV_32FC3, background_darkness, 0);
    
    for (size_t i = 0; i < centers.size(); i++) {
        cv::Mat mask = createRadialGradientMask(input.size(), centers[i], 
                                               radii[i] * 0.7f, radii[i]);
        
        // 3チャンネルマスク
        std::vector<cv::Mat> mask_channels(3, mask);
        cv::Mat mask_3ch;
        cv::merge(mask_channels, mask_3ch);
        
        // スポットライト効果
        cv::Mat brightened;
        input.convertTo(brightened, CV_32FC3, intensities[i], 0);
        
        // ブレンド
        result = result.mul(1.0f - mask_3ch) + brightened.mul(mask_3ch);
    }
    
    cv::Mat final_result;
    result.convertTo(final_result, CV_8UC3);
    return final_result;
}

cv::Mat createPulseGlow(const cv::Mat& input, float intensity, cv::Scalar glow_color) {
    cv::Mat result;
    input.convertTo(result, CV_32FC3);
    
    // グロー効果
    cv::Mat glow;
    cv::GaussianBlur(result, glow, cv::Size(21, 21), 10.0);
    
    cv::Scalar glow_scalar(glow_color[0] * intensity, 
                          glow_color[1] * intensity, 
                          glow_color[2] * intensity);
    
    result = result + glow_scalar;
    
    cv::Mat final_result;
    result.convertTo(final_result, CV_8UC3);
    return final_result;
}

} // namespace AnimationEffects

} // namespace fluent_vision