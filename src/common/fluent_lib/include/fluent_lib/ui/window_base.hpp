#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <algorithm>
#include "fluent_lib/ui/hud_base.hpp"

namespace fluent_ui {

/**
 * WindowBase: 半透明HUDウィンドウの基底クラス（ヘッダオンリー）
 * - 位置/サイズ: 現在値と目標値を持ち、tick(dt)でLERP補間
 * - 透過: 現在αと目標αを持ち、fade-in/outをゲインで制御
 * - 描画: render(base)は onDraw(local_canvas) を呼び、
 *         baseの該当矩形へ addWeighted で合成
 * - 再利用: 派生クラスは onDraw() でウィンドウ内部の描画のみ実装
 */
class WindowBase : public HUDWidget {
public:
    WindowBase()
        : smooth_gain_(8.0), fade_in_gain_(10.0), fade_out_gain_(8.0),
          z_index_(0), visible_(true)
    {
        cur_ = cv::Rect2f(0,0,0,0); tgt_ = cur_;
        alpha_cur_ = 0.0f; alpha_tgt_ = 0.0f;
        // 既定は不可視（呼び出し側で show() するとフェードイン）
    }

    virtual ~WindowBase() = default;

    // 基本設定
    void setGains(double smooth, double fade_in, double fade_out) {
        smooth_gain_ = smooth; fade_in_gain_ = fade_in; fade_out_gain_ = fade_out;
    }
    void setZIndex(int z) { z_index_ = z; }
    int zIndex() const { return z_index_; }

    // 目標矩形/現在矩形
    void setTargetRect(const cv::Rect &r) {
        cv::Rect2f rr(static_cast<float>(r.x), static_cast<float>(r.y),
                      static_cast<float>(r.width), static_cast<float>(r.height));
        tgt_ = rr;
    }
    void setImmediateRect(const cv::Rect &r) {
        cv::Rect2f rr(static_cast<float>(r.x), static_cast<float>(r.y),
                      static_cast<float>(r.width), static_cast<float>(r.height));
        cur_ = tgt_ = rr;
    }
    cv::Rect currentRect() const {
        return cv::Rect(
            static_cast<int>(std::round(cur_.x)),
            static_cast<int>(std::round(cur_.y)),
            static_cast<int>(std::round(std::max(0.f, cur_.width))),
            static_cast<int>(std::round(std::max(0.f, cur_.height)))
        );
    }

    // 表示/非表示（フェード制御）
    void show() { visible_ = true; alpha_tgt_ = 1.0f; }
    void hide() { visible_ = false; alpha_tgt_ = 0.0f; }
    void setAlpha(float a) { alpha_cur_ = alpha_tgt_ = std::clamp(a, 0.0f, 1.0f); }
    float alpha() const { return alpha_cur_; }
    bool isVisible() const { return alpha_cur_ > 0.001f; }

    // 毎フレーム更新（dt秒）
    void tick(double dt_seconds) {
        float tpos = static_cast<float>(std::clamp(smooth_gain_ * dt_seconds, 0.0, 1.0));
        cur_.x = cur_.x + (tgt_.x - cur_.x) * tpos;
        cur_.y = cur_.y + (tgt_.y - cur_.y) * tpos;
        cur_.width  = cur_.width  + (tgt_.width  - cur_.width ) * tpos;
        cur_.height = cur_.height + (tgt_.height - cur_.height) * tpos;
        float ta = visible_ ? 1.0f : 0.0f;
        float tg = static_cast<float>(std::clamp((visible_ ? fade_in_gain_ : fade_out_gain_) * dt_seconds, 0.0, 1.0));
        alpha_tgt_ = ta;
        alpha_cur_ = alpha_cur_ + (alpha_tgt_ - alpha_cur_) * tg;
    }

    // 合成描画（baseに半透明で重ねる）
    void render(cv::Mat &base) override {
        if (base.empty()) return;
        if (alpha_cur_ <= 0.001f) return;
        cv::Rect r = currentRect();
        r &= cv::Rect(0, 0, base.cols, base.rows);
        if (r.width <= 1 || r.height <= 1) return;
        // ローカルキャンバスを作成
        cv::Mat layer(r.height, r.width, CV_8UC3, cv::Scalar(0,0,0));
        // 背景（半透明黒を事前に描く → addWeightedで更にα合成される）
        if (draw_background_) {
            cv::rectangle(layer, cv::Rect(0,0,layer.cols, layer.rows), bg_color_, cv::FILLED);
            if (border_thickness_ > 0) {
                cv::rectangle(layer, cv::Rect(0,0,layer.cols, layer.rows), border_color_, border_thickness_, cv::LINE_AA);
            }
        }
        // コンテンツ描画
        onDraw(layer);
        // 合成
        cv::Mat roi = base(r);
        cv::addWeighted(layer, std::clamp(static_cast<double>(alpha_cur_), 0.0, 1.0),
                        roi, 1.0 - std::clamp(static_cast<double>(alpha_cur_), 0.0, 1.0), 0.0, roi);
    }

    // スタイル
    void setBackground(const cv::Scalar &bgr, bool enable=true) { bg_color_ = bgr; draw_background_ = enable; }
    void setBorder(const cv::Scalar &bgr, int thickness=1) { border_color_ = bgr; border_thickness_ = std::max(0, thickness); }

protected:
    // 派生クラスが内部描画を実装
    virtual void onDraw(cv::Mat &canvas) {
        // 既定は何も描かない（背景+枠のみ）
        (void)canvas;
    }

    // 補助: テキスト描画（影付き）
    static void drawShadowText(cv::Mat &img, const std::string &text, cv::Point org,
                               const cv::Scalar &fg, const cv::Scalar &shadow,
                               double scale=0.6, int thickness=2) {
        cv::putText(img, text, org + cv::Point(1,1), cv::FONT_HERSHEY_SIMPLEX, scale, shadow, thickness, cv::LINE_AA);
        cv::putText(img, text, org,               cv::FONT_HERSHEY_SIMPLEX, scale, fg,     thickness, cv::LINE_AA);
    }

private:
    cv::Rect2f cur_{};
    cv::Rect2f tgt_{};
    float alpha_cur_ {0.0f};
    float alpha_tgt_ {0.0f};
    double smooth_gain_;
    double fade_in_gain_;
    double fade_out_gain_;
    int z_index_;
    bool visible_;

    // style
    bool draw_background_ {true};
    cv::Scalar bg_color_ {0,0,0};
    cv::Scalar border_color_ {64,64,64};
    int border_thickness_ {1};
};

/**
 * AnchoredWindow: BBoxに対して左/右へ追従配置する簡易ウィンドウ
 */
class AnchoredWindow : public WindowBase {
public:
    enum class Side { Left, Right };
    void setAnchor(const cv::Rect &bbox, Side side, int width, int height, int gap_px=8, int y_offset=0) {
        cv::Rect r;
        if (side == Side::Left) {
            r.x = bbox.x - gap_px - width;
        } else {
            r.x = bbox.x + bbox.width + gap_px;
        }
        r.y = bbox.y + y_offset;
        r.width = width; r.height = height;
        setTargetRect(r);
    }
};

/**
 * InfoListWindow: テキスト行を縦に並べるシンプルな情報ウィンドウ
 */
class InfoListWindow : public WindowBase {
public:
    void setLines(std::vector<std::string> lines) { lines_ = std::move(lines); }
    void setTextStyle(double scale, int thickness, cv::Scalar color_fg, cv::Scalar color_shadow) {
        font_scale_ = scale; font_thickness_ = thickness; fg_ = color_fg; shadow_ = color_shadow;
    }
protected:
    void onDraw(cv::Mat &canvas) override {
        int pad = 8; int lh = static_cast<int>(std::round(20 * font_scale_));
        int y = pad + lh; // 1行目のベースライン
        for (const auto &s : lines_) {
            drawShadowText(canvas, s, cv::Point(pad, y), fg_, shadow_, font_scale_, font_thickness_);
            y += lh;
        }
    }
private:
    std::vector<std::string> lines_;
    double font_scale_ {0.7};
    int font_thickness_ {2};
    cv::Scalar fg_ {255,255,255};
    cv::Scalar shadow_ {0,0,0};
};

} // namespace fluent_ui


