#pragma once
#include <map>
#include <vector>
#include <mutex>
#include <chrono>
#include <opencv2/opencv.hpp>

namespace fluent_ui {

// 共通オーバーレイ補間エンジン（ライブラリ側）
// 画面依存の構造体を避けるため、最小限の矩形・αのみを扱う
struct OverlayItem {
    int id { -1 };
    cv::Rect bbox;   // 出力用に丸めたbbox
    float alpha { 1.0f };
};

class OverlayEngine {
public:
    void setGains(double smooth_gain, double fade_in_gain, double fade_out_gain) {
        smooth_gain_ = smooth_gain; fade_in_gain_ = fade_in_gain; fade_out_gain_ = fade_out_gain;
    }
    // フェードアウト後の削除ポリシーを設定（αが一定未満かつ経過時間>age_msで削除）
    void setRemovalPolicy(float erase_alpha_min, double erase_age_ms) {
        erase_alpha_min_ = erase_alpha_min; erase_age_ms_ = erase_age_ms;
    }
    // 目標リストを丸ごと設定
    void setTargets(const std::vector<std::pair<int, cv::Rect>> &targets) {
        std::lock_guard<std::mutex> lk(mu_);
        for (auto &kv : states_) kv.second.seen = false;
        for (const auto &p : targets) {
            int id = p.first; cv::Rect r = p.second;
            cv::Rect2f tgt(static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.width), static_cast<float>(r.height));
            auto it = states_.find(id);
            if (it == states_.end()) {
                State st; st.cur = tgt; st.tgt = tgt; st.ca = 0.0f; st.ta = 1.0f; st.seen = true; st.last = std::chrono::steady_clock::now();
                states_.emplace(id, std::move(st));
            } else {
                it->second.tgt = tgt; it->second.ta = 1.0f; it->second.seen = true; it->second.last = std::chrono::steady_clock::now();
            }
        }
    }
    void tick(double dt) {
        std::lock_guard<std::mutex> lk(mu_);
        float tpos = static_cast<float>(std::clamp(smooth_gain_*dt, 0.0, 1.0));
        float tin  = static_cast<float>(std::clamp(fade_in_gain_*dt, 0.0, 1.0));
        // 選択中は追従を速めたいケースが多いため、外部でsmooth_gainを上げるのが基本だが、
        // エンジン側はfade_outをやや強めて残像を早めに消せるように下限0.0→上限1.0のまま維持。
        float tout = static_cast<float>(std::clamp(fade_out_gain_*dt, 0.0, 1.0));
        std::vector<int> erase;
        for (auto &kv : states_) {
            auto &s = kv.second;
            s.cur.x = s.cur.x + (s.tgt.x - s.cur.x) * tpos;
            s.cur.y = s.cur.y + (s.tgt.y - s.cur.y) * tpos;
            s.cur.width  = s.cur.width  + (s.tgt.width  - s.cur.width ) * tpos;
            s.cur.height = s.cur.height + (s.tgt.height - s.cur.height) * tpos;
            float ta = s.seen ? s.ta : 0.0f; float t = s.seen ? tin : tout;
            s.ca = s.ca + (ta - s.ca) * t; s.seen = false;
            auto age = std::chrono::steady_clock::now() - s.last;
            if (s.ca < erase_alpha_min_ && std::chrono::duration<double, std::milli>(age).count() > erase_age_ms_) erase.push_back(kv.first);
        }
        for (int id : erase) states_.erase(id);
    }
    std::vector<OverlayItem> snapshot() const {
        std::lock_guard<std::mutex> lk(mu_);
        std::vector<OverlayItem> out; out.reserve(states_.size());
        for (const auto &kv : states_) {
            const auto &s = kv.second;
            OverlayItem it; it.id = kv.first;
            it.bbox = cv::Rect(static_cast<int>(std::round(s.cur.x)), static_cast<int>(std::round(s.cur.y)),
                                static_cast<int>(std::round(std::max(0.0f, s.cur.width))), static_cast<int>(std::round(std::max(0.0f, s.cur.height))));
            it.alpha = s.ca; out.push_back(it);
        }
        return out;
    }
private:
    struct State { cv::Rect2f cur, tgt; float ca{0.0f}, ta{1.0f}; bool seen{false}; std::chrono::steady_clock::time_point last; };
    mutable std::mutex mu_;
    std::map<int, State> states_;
    double smooth_gain_ {12.0}, fade_in_gain_ {6.0}, fade_out_gain_ {3.0};
    // 削除ポリシー
    float erase_alpha_min_ {0.02f};
    double erase_age_ms_ {500.0};
};

} // namespace fluent_ui


