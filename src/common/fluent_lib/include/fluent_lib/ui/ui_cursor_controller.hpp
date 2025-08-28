#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <utility>
#include <functional>
#include <chrono>

namespace fluent_ui {

// 画像上の十字カーソルを管理する軽量コントローラ（ヘッダオンリー）
class UiCursorController {
public:
    UiCursorController()
        : shown_(false), keep_ms_(5000), smooth_ms_(150),
          auto_on_(true), auto_off_if_miss_(true),
          last_hit_time_(std::chrono::steady_clock::now()),
          color_(0,255,0), size_px_(20), thickness_px_(2) {}

    // 設定
    void Configure(int keep_ms, int smooth_ms, const cv::Scalar &color,
                   int size_px, int thickness_px, bool auto_on, bool auto_off_if_miss) {
        keep_ms_ = keep_ms; smooth_ms_ = smooth_ms; color_ = color;
        size_px_ = size_px; thickness_px_ = thickness_px;
        auto_on_ = auto_on; auto_off_if_miss_ = auto_off_if_miss;
    }

    // 表示/非表示
    void ShowCursor() { shown_ = true; touch(); }
    void HideCursor() { shown_ = false; }
    bool IsShown() const { return shown_; }

    // 目標位置を設定（スムーズに current_ を追従）
    void SetPosition(int x, int y) { target_ = cv::Point(x,y); touch(); }

    // 現在位置（スムージング後）
    cv::Point GetPosition() const { return current_; }
    cv::Scalar Color() const { return color_; }
    int SizePx() const { return size_px_; }
    int ThicknessPx() const { return thickness_px_; }

    // 時間更新（dt秒）: 位置平滑＋タイムアウト判定
    void Tick(double dt_seconds) {
        // 平滑（一次遅れ）
        if (smooth_ms_ <= 0) {
            current_ = target_;
        } else {
            double alpha = std::min(1.0, dt_seconds / (static_cast<double>(smooth_ms_) / 1000.0));
            current_.x = static_cast<int>(std::round(current_.x + (target_.x - current_.x) * alpha));
            current_.y = static_cast<int>(std::round(current_.y + (target_.y - current_.y) * alpha));
        }
        // タイムアウト（直下ヒットが無くなってから keep_ms 経過で自動OFF）
        if (shown_ && auto_off_if_miss_) {
            auto now = std::chrono::steady_clock::now();
            double ms = std::chrono::duration<double, std::milli>(now - last_hit_time_).count();
            if (ms > static_cast<double>(keep_ms_)) {
                shown_ = false;
            }
        }
    }

    // 汎用自動ロジック（並び順は呼び出し側で用意）
    // - Offのとき: itemsが空でなければ items[0] を採用して中心へ移動しOn
    // - On のとき: カーソル直下のIDを返す（無ければ -1）。ヒットがあれば keep_ms リセット
    template <class T>
    int UpdateAutoLogic(const std::vector<T> &items,
                        const std::function<int(const T&)> &getId,
                        const std::function<cv::Rect(const T&)> &getRect) {
        if (!shown_) {
            if (auto_on_ && !items.empty()) {
                const T &top = items.front();
                auto r = getRect(top);
                cv::Point c(r.x + r.width/2, r.y + r.height/2);
                SetPosition(c.x, c.y);
                ShowCursor();
            }
            return -1;
        }
        for (const auto &it : items) {
            if (getRect(it).contains(current_)) { touch(); return getId(it); }
        }
        return -1;
    }

    // 便宜オーバーロード（pair<int,Rect>）
    int UpdateAutoLogic(const std::vector<std::pair<int, cv::Rect>> &rects) {
        return UpdateAutoLogic<std::pair<int, cv::Rect>>(
            rects,
            [](const auto &x){ return x.first; },
            [](const auto &x){ return x.second; }
        );
    }

private:
    void touch() { last_hit_time_ = std::chrono::steady_clock::now(); }

    bool shown_;
    int keep_ms_;
    int smooth_ms_;
    bool auto_on_;
    bool auto_off_if_miss_;
    std::chrono::steady_clock::time_point last_hit_time_;
    cv::Point target_ { -1, -1 };
    cv::Point current_ { -1, -1 };
    cv::Scalar color_;
    int size_px_;
    int thickness_px_;
};

} // namespace fluent_ui


