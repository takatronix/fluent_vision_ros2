#ifndef FLUENT_ANIMATION_HPP
#define FLUENT_ANIMATION_HPP

#include <fluent_vision/fluent_vision.hpp>
#include <opencv2/opencv.hpp>
#include <functional>
#include <memory>
#include <map>
#include <chrono>

namespace fluent_vision {

/**
 * @brief FluentAnimation - 美しく流れるようなアニメーションライブラリ
 * 
 * スムーズなフェード、パルス、スポットライト効果を簡単に実装
 * Method Chaining で美しいアニメーション記述を実現
 */

// ======== イージング関数 ========
enum class EaseType {
    Linear,
    EaseIn,
    EaseOut,
    EaseInOut,
    Bounce,
    Elastic,
    Back
};

class Easing {
public:
    static float apply(float t, EaseType type) {
        switch (type) {
            case EaseType::Linear:     return t;
            case EaseType::EaseIn:     return t * t;
            case EaseType::EaseOut:    return t * (2.0f - t);
            case EaseType::EaseInOut:  return t < 0.5f ? 2*t*t : -1+(4-2*t)*t;
            case EaseType::Bounce:     return bounce(t);
            case EaseType::Elastic:    return elastic(t);
            case EaseType::Back:       return back(t);
            default: return t;
        }
    }
    
private:
    static float bounce(float t) {
        if (t < 1/2.75f) return 7.5625f*t*t;
        else if (t < 2/2.75f) return 7.5625f*(t-=1.5f/2.75f)*t + 0.75f;
        else if (t < 2.5f/2.75f) return 7.5625f*(t-=2.25f/2.75f)*t + 0.9375f;
        else return 7.5625f*(t-=2.625f/2.75f)*t + 0.984375f;
    }
    
    static float elastic(float t) {
        return std::sin(13.0f * M_PI * 0.5f * t) * std::pow(2.0f, 10.0f * (t - 1.0f));
    }
    
    static float back(float t) {
        const float c1 = 1.70158f;
        const float c3 = c1 + 1.0f;
        return c3 * t * t * t - c1 * t * t;
    }
};

// ======== アニメーション状態管理 ========
struct AnimationState {
    float current_value = 0.0f;
    float target_value = 0.0f;
    float start_value = 0.0f;
    float duration = 1.0f;
    float elapsed = 0.0f;
    EaseType ease_type = EaseType::EaseInOut;
    bool is_active = false;
    bool auto_reverse = false;
    int repeat_count = 0;
    int current_repeat = 0;
    
    std::function<void(float)> on_update;
    std::function<void()> on_complete;
};

// ======== FluentAnimation メインクラス ========
class FluentAnimation {
public:
    FluentAnimation() = default;
    
    // ========== 基本アニメーション ==========
    static FluentAnimation fade() { return FluentAnimation().withName("fade"); }
    static FluentAnimation spotlight() { return FluentAnimation().withName("spotlight"); }
    static FluentAnimation pulse() { return FluentAnimation().withName("pulse"); }
    static FluentAnimation glow() { return FluentAnimation().withName("glow"); }
    static FluentAnimation slide() { return FluentAnimation().withName("slide"); }
    static FluentAnimation zoom() { return FluentAnimation().withName("zoom"); }
    static FluentAnimation rotate() { return FluentAnimation().withName("rotate"); }
    
    // ========== パラメータ設定 ==========
    FluentAnimation& from(float start_val) { 
        state_.start_value = start_val; 
        state_.current_value = start_val;
        return *this; 
    }
    
    FluentAnimation& to(float end_val) { 
        state_.target_value = end_val; 
        return *this; 
    }
    
    FluentAnimation& duration(float seconds) { 
        state_.duration = seconds; 
        return *this; 
    }
    
    FluentAnimation& ease(EaseType type) { 
        state_.ease_type = type; 
        return *this; 
    }
    
    FluentAnimation& repeat(int count) { 
        state_.repeat_count = count; 
        return *this; 
    }
    
    FluentAnimation& yoyo() { 
        state_.auto_reverse = true; 
        return *this; 
    }
    
    // ========== イベントハンドラ ==========
    FluentAnimation& onUpdate(std::function<void(float)> callback) {
        state_.on_update = callback;
        return *this;
    }
    
    FluentAnimation& onComplete(std::function<void()> callback) {
        state_.on_complete = callback;
        return *this;
    }
    
    // ========== 実行制御 ==========
    FluentAnimation& play() {
        state_.is_active = true;
        state_.elapsed = 0.0f;
        return *this;
    }
    
    FluentAnimation& pause() {
        state_.is_active = false;
        return *this;
    }
    
    FluentAnimation& reset() {
        state_.elapsed = 0.0f;
        state_.current_value = state_.start_value;
        state_.current_repeat = 0;
        return *this;
    }
    
    // ========== 高レベルアニメーション ==========
    
    // スポットライトアニメーション
    static FluentAnimation spotlightOn(cv::Point2f center, float radius) {
        return FluentAnimation()
            .withName("spotlight_on")
            .from(0.0f).to(1.0f)
            .duration(0.5f)
            .ease(EaseType::EaseOut)
            .onUpdate([center, radius](float value) {
                // スポットライト効果の実装
                // SpotlightEffect::apply(center, radius * value, value);
            });
    }
    
    // フェードインアニメーション
    static FluentAnimation fadeIn(float duration = 0.5f) {
        return FluentAnimation()
            .from(0.0f).to(1.0f)
            .duration(duration)
            .ease(EaseType::EaseOut);
    }
    
    // フェードアウトアニメーション
    static FluentAnimation fadeOut(float duration = 0.5f) {
        return FluentAnimation()
            .from(1.0f).to(0.0f)
            .duration(duration)
            .ease(EaseType::EaseIn);
    }
    
    // パルスアニメーション
    static FluentAnimation pulseEffect(float min_brightness = 0.8f, float max_brightness = 1.2f) {
        return FluentAnimation()
            .from(min_brightness).to(max_brightness)
            .duration(1.0f)
            .ease(EaseType::EaseInOut)
            .yoyo()
            .repeat(-1); // 無限ループ
    }
    
    // グロー効果
    static FluentAnimation glowEffect(cv::Scalar color, float intensity = 2.0f) {
        return FluentAnimation()
            .from(0.0f).to(intensity)
            .duration(0.3f)
            .ease(EaseType::EaseOut)
            .yoyo();
    }
    
    // ========== 更新処理 ==========
    void update(float delta_time) {
        if (!state_.is_active) return;
        
        state_.elapsed += delta_time;
        float t = std::min(state_.elapsed / state_.duration, 1.0f);
        
        // イージング適用
        float eased_t = Easing::apply(t, state_.ease_type);
        
        // 値の計算
        if (state_.auto_reverse && state_.current_repeat % 2 == 1) {
            state_.current_value = state_.target_value + (state_.start_value - state_.target_value) * eased_t;
        } else {
            state_.current_value = state_.start_value + (state_.target_value - state_.start_value) * eased_t;
        }
        
        // コールバック実行
        if (state_.on_update) {
            state_.on_update(state_.current_value);
        }
        
        // 完了チェック
        if (t >= 1.0f) {
            if (state_.repeat_count == -1 || state_.current_repeat < state_.repeat_count) {
                // リピート
                state_.elapsed = 0.0f;
                state_.current_repeat++;
            } else {
                // 完了
                state_.is_active = false;
                if (state_.on_complete) {
                    state_.on_complete();
                }
            }
        }
    }
    
    // ========== ゲッター ==========
    float getValue() const { return state_.current_value; }
    bool isActive() const { return state_.is_active; }
    float getProgress() const { return state_.elapsed / state_.duration; }
    
private:
    AnimationState state_;
    std::string name_;
    
    FluentAnimation& withName(const std::string& n) { 
        name_ = n; 
        return *this; 
    }
};

// ======== アニメーションマネージャー ========
class AnimationManager {
public:
    static AnimationManager& getInstance() {
        static AnimationManager instance;
        return instance;
    }
    
    void add(const std::string& name, FluentAnimation animation) {
        animations_[name] = animation;
    }
    
    void play(const std::string& name) {
        if (animations_.find(name) != animations_.end()) {
            animations_[name].play();
        }
    }
    
    void pause(const std::string& name) {
        if (animations_.find(name) != animations_.end()) {
            animations_[name].pause();
        }
    }
    
    void updateAll(float delta_time) {
        for (auto& [name, animation] : animations_) {
            animation.update(delta_time);
        }
    }
    
    void clear() {
        animations_.clear();
    }
    
private:
    std::map<std::string, FluentAnimation> animations_;
};

// ======== スポットライト効果実装 ========
class SpotlightEffect {
public:
    static void apply(cv::Point2f center, float radius, float intensity) {
        // 実装は後でFluentVisionクラスと連携
        current_center_ = center;
        current_radius_ = radius;
        current_intensity_ = intensity;
    }
    
    static cv::Mat applyToImage(const cv::Mat& input, cv::Point2f center, float radius, float intensity) {
        cv::Mat result = input.clone();
        cv::Mat mask = cv::Mat::zeros(input.size(), CV_32F);
        
        // 円形マスク作成
        cv::circle(mask, center, radius, cv::Scalar(1.0f), -1);
        
        // ガウシアンブラーでソフトエッジ
        cv::GaussianBlur(mask, mask, cv::Size(101, 101), 30.0);
        
        // 3チャンネルに拡張
        std::vector<cv::Mat> mask_channels(3, mask);
        cv::Mat mask_3ch;
        cv::merge(mask_channels, mask_3ch);
        
        // スポットライト効果適用
        cv::Mat darkened;
        input.convertTo(darkened, CV_32F, 0.3); // 背景を暗く
        
        cv::Mat brightened;
        input.convertTo(brightened, CV_32F, intensity); // スポットライト部分を明るく
        
        cv::Mat blended = darkened.mul(1.0 - mask_3ch) + brightened.mul(mask_3ch);
        
        blended.convertTo(result, CV_8U);
        return result;
    }
    
private:
    static cv::Point2f current_center_;
    static float current_radius_;
    static float current_intensity_;
};

} // namespace fluent_vision

// ========== 使用例 ==========
/*

// 基本的な使用例
auto fadeIn = FluentAnimation::fadeIn(0.5f)
    .onComplete([]() { 
        std::cout << "フェードイン完了!" << std::endl; 
    })
    .play();

// スポットライトアニメーション
auto spotlight = FluentAnimation::spotlightOn({320, 240}, 100)
    .duration(0.8f)
    .ease(EaseType::Bounce)
    .play();

// パルス効果
auto pulse = FluentAnimation::pulseEffect(0.8f, 1.2f)
    .duration(2.0f)
    .play();

// 複合アニメーション
FluentVision::stream("/camera/image")
    .when(asparagus_detected)
        .animate(FluentAnimation::spotlightOn(asparagus_center, 120)
                    .duration(0.5f)
                    .ease(EaseType::EaseOut))
        .animate(FluentAnimation::pulseEffect(0.9f, 1.1f)
                    .duration(1.5f))
        .fadeBackground(0.3f)
    .publish("/asparagus/spotlight");

// アニメーションマネージャー使用
auto& manager = AnimationManager::getInstance();
manager.add("spotlight", FluentAnimation::spotlightOn({320, 240}, 100));
manager.add("pulse", FluentAnimation::pulseEffect());

// メインループ
while (rclcpp::ok()) {
    manager.updateAll(0.016f); // 60FPS
    rclcpp::spin_some(node);
}

*/

#endif // FLUENT_ANIMATION_HPP