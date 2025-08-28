#pragma once
#include <opencv2/opencv.hpp>

namespace fluent_ui {

// オフスクリーンHUD用の簡易キャンバス
// - layer: 8UC3 の作業用レイヤ
// - 各要素は layer に描画し、必要な矩形領域だけ base に α 合成する
class OffscreenCanvas {
public:
    // レイヤ作成（サイズ変更時は破棄→再作成）
    void create(const cv::Size &size) {
        if (size.width <= 0 || size.height <= 0) return;
        if (layer_.size() != size) layer_ = cv::Mat(size, CV_8UC3, cv::Scalar(0,0,0));
        else layer_.setTo(cv::Scalar(0,0,0));
    }
    // 全消去
    void clear() { if (!layer_.empty()) layer_.setTo(cv::Scalar(0,0,0)); }
    // レイヤ参照（直接描画用）
    cv::Mat &mat() { return layer_; }
    // 矩形塗りつぶし（layerに直接描画）
    void fillRect(const cv::Rect &r, const cv::Scalar &color) {
        if (layer_.empty()) return;
        cv::Rect rr = r & cv::Rect(0,0,layer_.cols, layer_.rows);
        if (rr.area() <= 0) return;
        cv::rectangle(layer_, rr, color, cv::FILLED);
    }
    // 指定領域のみ α 合成（layer→base）
    void blendRegion(cv::Mat &base, const cv::Rect &r, double alpha) {
        if (layer_.empty() || base.empty()) return;
        cv::Rect rr = r & cv::Rect(0,0,layer_.cols, layer_.rows);
        rr &= cv::Rect(0,0,base.cols, base.rows);
        if (rr.area() <= 0) return;
        cv::Mat roi_base = base(rr);
        cv::Mat roi_layer = layer_(rr);
        cv::addWeighted(roi_layer, std::clamp(alpha, 0.0, 1.0), roi_base, 1.0 - std::clamp(alpha, 0.0, 1.0), 0.0, roi_base);
    }
private:
    cv::Mat layer_;
};

} // namespace fluent_ui


