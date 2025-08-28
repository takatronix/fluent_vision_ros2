#pragma once
#include "hud_base.hpp"

namespace fluent_ui {

class StatusBox : public HUDWidget {
public:
    explicit StatusBox(const std::string& text = "", double alpha = 0.8)
        : text_(text), alpha_(alpha) {}
    void setText(const std::string& t) { text_ = t; }
    void setAlpha(double a) { alpha_ = a; }
    void render(cv::Mat& image) override {
        if (image.empty()) return;
        if (text_.empty()) return;
        cv::Scalar bg(0, 0, 0);
        cv::Scalar fg(255, 255, 255);
        int base = 0;
        double scale = 1.0;
        int thickness = 2;
        cv::Size sz = cv::getTextSize(text_, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &base);
        int pad = 8;
        cv::Rect rect((image.cols - sz.width) / 2 - pad, 10, sz.width + pad * 2, sz.height + pad * 2);
        rect &= cv::Rect(0, 0, image.cols, image.rows);
        if (rect.area() > 0) {
            cv::Mat roi = image(rect);
            cv::Mat overlay = roi.clone();
            cv::rectangle(overlay, cv::Rect(0,0,roi.cols,roi.rows), bg, cv::FILLED);
            cv::addWeighted(overlay, alpha_, roi, 1.0 - alpha_, 0.0, roi);
            cv::putText(image, text_, cv::Point(rect.x + pad, rect.y + rect.height - pad - base),
                        cv::FONT_HERSHEY_SIMPLEX, scale, fg, thickness, cv::LINE_AA);
        }
    }
private:
    std::string text_;
    double alpha_;
};

class InfoBoxNearBBox : public HUDWidget {
public:
    InfoBoxNearBBox() = default;
    void setLines(const std::vector<std::string>& lines) { lines_ = lines; }
    void setAnchor(const cv::Rect& bbox, const cv::Point& offset) { bbox_ = bbox; offset_ = offset; }
    void render(cv::Mat& image) override {
        if (image.empty() || lines_.empty()) return;
        cv::Scalar bg(0, 0, 0);
        cv::Scalar fg(255, 255, 255);
        int pad = 6; double scale = 0.7; int thickness = 2; int base = 0;
        int width = 0; int height = pad * 2;
        for (const auto& s : lines_) {
            cv::Size sz = cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &base);
            width = std::max(width, sz.width);
            height += sz.height + 4;
        }
        cv::Point org(bbox_.x + bbox_.width + offset_.x, bbox_.y + offset_.y);
        cv::Rect rect(org.x, org.y, width + pad * 2, height);
        rect &= cv::Rect(0, 0, image.cols, image.rows);
        if (rect.area() > 0) {
            cv::Mat roi = image(rect);
            cv::Mat overlay = roi.clone();
            cv::rectangle(overlay, cv::Rect(0,0,roi.cols,roi.rows), bg, cv::FILLED);
            cv::addWeighted(overlay, 0.85, roi, 0.15, 0.0, roi);
            int y = rect.y + pad + (base/2);
            for (const auto& s : lines_) {
                y += cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &base).height + 4;
                cv::putText(image, s, cv::Point(rect.x + pad, y), cv::FONT_HERSHEY_SIMPLEX, scale, fg, thickness, cv::LINE_AA);
            }
        }
    }
private:
    std::vector<std::string> lines_;
    cv::Rect bbox_;
    cv::Point offset_ {8, 8};
};

} // namespace fluent_ui


