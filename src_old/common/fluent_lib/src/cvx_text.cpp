#include "cvx/cvx_text.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/freetype.hpp>

namespace cvx {
void drawText(cv::Mat& img, const std::string& text, cv::Point org,
              cv::Scalar color, double font_scale, int thickness, int baseline_offset) {
    // 日本語・多言語対応: 1文字でも非ASCIIがあればputJapaneseTextを使う
    bool has_non_ascii = false;
    for (unsigned char c : text) {
        if (c >= 0x80) { has_non_ascii = true; break; }
    }
    if (has_non_ascii) {
        int fontHeight = static_cast<int>(24 * font_scale); // デフォルト換算
        putJapaneseText(img, text, org, fontHeight > 0 ? fontHeight : 24, color);
    } else {
        int baseline = 0;
        int font_face = cv::FONT_HERSHEY_SIMPLEX;
        cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
        cv::Point text_org = org + cv::Point(0, baseline_offset);
        cv::putText(img, text, text_org, font_face, font_scale, color, thickness, cv::LINE_AA);
    }
}

void putJapaneseText(cv::Mat& img, const std::string& text, cv::Point org, int fontHeight,
                   cv::Scalar color, const std::string& fontPath) {
    static cv::Ptr<cv::freetype::FreeType2> ft2 = cv::freetype::createFreeType2();
    static std::string loadedFont;
    if (loadedFont != fontPath) {
        ft2->loadFontData(fontPath, 0);
        loadedFont = fontPath;
    }
    ft2->putText(img, text, org, fontHeight, color, -1, cv::LINE_AA, true);
}
void drawTextPx(cv::Mat& img, const std::string& text, cv::Point org, int px_height,
                cv::Scalar color, int thickness, int baseline_offset) {
    double test_scale = 1.0;
    int baseline = 0;
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    cv::Size sz = cv::getTextSize(text, font_face, test_scale, thickness, &baseline);
    double scale = (sz.height > 0) ? static_cast<double>(px_height) / sz.height : 1.0;
    drawText(img, text, org, color, scale, thickness, baseline_offset);
}
void drawShadowText(cv::Mat& img, const std::string& text, cv::Point org,
                    cv::Scalar color, cv::Scalar shadow_color,
                    double font_scale, int thickness, int baseline_offset) {
    drawText(img, text, org + cv::Point(1,1), shadow_color, font_scale, thickness, baseline_offset);
    drawText(img, text, org, color, font_scale, thickness, baseline_offset);
}
void drawOutlineText(cv::Mat& img, const std::string& text, cv::Point org,
                    cv::Scalar color, cv::Scalar outline_color,
                    double font_scale, int thickness, int baseline_offset) {
    drawText(img, text, org + cv::Point(1,1), outline_color, font_scale, thickness, baseline_offset);
    drawText(img, text, org, color, font_scale, thickness, baseline_offset);
}
} // namespace cvx
