#pragma once
#include <opencv2/opencv.hpp>
#include <string>

namespace cvx {
    // カラー定数（OpenCVはBGR順）
    const cv::Scalar RED    (0,   0, 255);
    const cv::Scalar GREEN  (0, 255,   0);
    const cv::Scalar BLUE   (255,0,   0);
    const cv::Scalar WHITE  (255,255,255);
    const cv::Scalar BLACK  (0,   0,   0);
    const cv::Scalar YELLOW (0, 255, 255);
    const cv::Scalar CYAN   (255,255,  0);
    const cv::Scalar MAGENTA(255, 0, 255);

    // RGB指定のユーティリティ
    inline cv::Scalar rgb(int r, int g, int b) { return cv::Scalar(b, g, r); }

    // 文字列描画API（デフォルト引数で短く書ける）
    void drawText(cv::Mat& img, const std::string& text, cv::Point org,
                  cv::Scalar color = cv::Scalar(255,255,255),
                  double font_scale = 0.5,
                  int thickness = 1,
                  int baseline_offset = 0);
    void drawShadowText(cv::Mat& img, const std::string& text, cv::Point org,
                        cv::Scalar color = cv::Scalar(255,255,255),
                        cv::Scalar shadow_color = cv::Scalar(0,0,0),
                        double font_scale = 0.5,
                        int thickness = 1,
                        int baseline_offset = 0);
    void drawOutlineText(cv::Mat& img, const std::string& text, cv::Point org,
                        cv::Scalar color = cv::Scalar(255,255,255),
                        cv::Scalar outline_color = cv::Scalar(0,0,0),
                        double font_scale = 0.5,
                        int thickness = 1,
                        int baseline_offset = 0);
    // ピクセル高さ指定の文字描画API
    void drawTextPx(cv::Mat& img, const std::string& text, cv::Point org, int px_height,
                    cv::Scalar color = WHITE, int thickness = 2, int baseline_offset = 0);
    // 日本語（UTF-8）描画API（OpenCV freetype利用）
    void putJapaneseText(cv::Mat& img, const std::string& text, cv::Point org, int fontHeight,
                        cv::Scalar color = WHITE, const std::string& fontPath = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc");
}
