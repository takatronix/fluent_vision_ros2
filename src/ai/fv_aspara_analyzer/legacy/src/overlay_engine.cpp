#include "fv_aspara_analyzer/overlay_engine.hpp"

namespace fv_aspara_analyzer {

namespace {
static inline float lerp(float a, float b, float t) { return a + (b - a) * t; }
static inline cv::Rect2f lerpRect(const cv::Rect2f& a, const cv::Rect2f& b, float t) {
    return cv::Rect2f(
        lerp(a.x, b.x, t),
        lerp(a.y, b.y, t),
        lerp(a.width, b.width, t),
        lerp(a.height, b.height, t)
    );
}
}

void OverlayEngine::setGains(double smooth_gain, double fade_in_gain, double fade_out_gain)
{
    smooth_gain_ = smooth_gain;
    fade_in_gain_ = fade_in_gain;
    fade_out_gain_ = fade_out_gain;
}

void OverlayEngine::setTarget(const AsparaInfo& info)
{
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = states_.find(info.id);
    cv::Rect2f target(static_cast<float>(info.bounding_box_2d.x), static_cast<float>(info.bounding_box_2d.y),
                      static_cast<float>(info.bounding_box_2d.width), static_cast<float>(info.bounding_box_2d.height));
    if (it == states_.end()) {
        State st;
        st.current_bbox = target;
        st.target_bbox = target;
        st.current_alpha = 0.0f;  // 新規はフェードイン開始
        st.target_alpha = 1.0f;
        st.latest = info;
        st.seen_in_this_frame = true;
        st.last_seen = std::chrono::steady_clock::now();
        states_.emplace(info.id, std::move(st));
    } else {
        it->second.target_bbox = target;
        it->second.target_alpha = 1.0f;
        it->second.latest = info;
        it->second.seen_in_this_frame = true;
        it->second.last_seen = std::chrono::steady_clock::now();
    }
}

void OverlayEngine::setTargets(const std::vector<AsparaInfo>& list)
{
    std::lock_guard<std::mutex> lk(mutex_);
    // 一旦全て未観測にし、その後更新されたものだけ true にする
    for (auto &kv : states_) kv.second.seen_in_this_frame = false;
    for (const auto& a : list) {
        auto it = states_.find(a.id);
        cv::Rect2f tgt(static_cast<float>(a.bounding_box_2d.x), static_cast<float>(a.bounding_box_2d.y),
                       static_cast<float>(a.bounding_box_2d.width), static_cast<float>(a.bounding_box_2d.height));
        if (it == states_.end()) {
            State st; st.current_bbox = tgt; st.target_bbox = tgt; st.current_alpha = 0.0f; st.target_alpha = 1.0f; st.latest = a; st.seen_in_this_frame = true; st.last_seen = std::chrono::steady_clock::now();
            states_.emplace(a.id, std::move(st));
        } else {
            it->second.target_bbox = tgt;
            it->second.target_alpha = 1.0f;
            it->second.latest = a;
            it->second.seen_in_this_frame = true;
            it->second.last_seen = std::chrono::steady_clock::now();
        }
    }
}

void OverlayEngine::tick(double dt_seconds)
{
    std::lock_guard<std::mutex> lk(mutex_);
    float t_pos = static_cast<float>(std::clamp(smooth_gain_ * dt_seconds, 0.0, 1.0));
    float t_in  = static_cast<float>(std::clamp(fade_in_gain_ * dt_seconds, 0.0, 1.0));
    float t_out = static_cast<float>(std::clamp(fade_out_gain_ * dt_seconds, 0.0, 1.0));

    std::vector<int> to_erase;
    for (auto &kv : states_) {
        State &st = kv.second;
        // bbox 補間
        st.current_bbox = lerpRect(st.current_bbox, st.target_bbox, t_pos);
        // alpha 補間（見えたらin、見えない時はout）
        float tgt_a = st.seen_in_this_frame ? st.target_alpha : 0.0f;
        float t = st.seen_in_this_frame ? t_in : t_out;
        st.current_alpha = lerp(st.current_alpha, tgt_a, t);

        // 観測されなかったものは、フェードアウトが十分進めば削除
        if (!st.seen_in_this_frame) {
            auto age = std::chrono::steady_clock::now() - st.last_seen;
            double age_ms = std::chrono::duration<double, std::milli>(age).count();
            if (st.current_alpha < 0.02f && age_ms > 500.0) {
                to_erase.push_back(kv.first);
            }
        }
        // 次フレームに備えてフラグをクリア（setTarget(s) 側で立て直す）
        st.seen_in_this_frame = false;
    }
    for (int id : to_erase) states_.erase(id);
}

std::vector<AsparaInfo> OverlayEngine::snapshot() const
{
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<AsparaInfo> out; out.reserve(states_.size());
    for (const auto& kv : states_) {
        const State &st = kv.second;
        AsparaInfo a = st.latest;
        // current_bbox を出力へ反映
        a.bounding_box_2d = cv::Rect(
            static_cast<int>(std::round(st.current_bbox.x)),
            static_cast<int>(std::round(st.current_bbox.y)),
            static_cast<int>(std::round(std::max(0.0f, st.current_bbox.width))),
            static_cast<int>(std::round(std::max(0.0f, st.current_bbox.height)))
        );
        // αを meta 用に埋める（必要なら AsparaInfo にフィールド追加推奨）
        a.animation_alpha = st.current_alpha;
        out.push_back(std::move(a));
    }
    return out;
}

void OverlayEngine::clear()
{
    std::lock_guard<std::mutex> lk(mutex_);
    states_.clear();
}

} // namespace fv_aspara_analyzer


