#include "fluent_text.hpp"
#if FLUENT_HAS_FREETYPE
#include <opencv2/freetype.hpp>
#endif

namespace fluent {
namespace text {

#if FLUENT_HAS_FREETYPE
// FreeType2インスタンスを一度だけ生成（シングルトン）
cv::Ptr<cv::freetype::FreeType2>& getFT2() {
    static cv::Ptr<cv::freetype::FreeType2> ft2 = cv::freetype::createFreeType2();
    return ft2;
}

// 最後にロードしたフォントパスを記憶
std::string& getLastFontPath() {
    static std::string last_font_path;
    return last_font_path;
}

// 日本語対応の内部描画関数
void putJapaneseText(cv::Mat& img, const std::string& text, cv::Point org, int fontHeight,
                   cv::Scalar color, const std::string& fontPath) {
    auto& ft2 = getFT2();
    auto& last_font = getLastFontPath();
    if (last_font != fontPath) {
        try {
            ft2->loadFontData(fontPath, 0);
            last_font = fontPath;
        } catch (const cv::Exception& e) {
            // フォント読み込み失敗時はputTextにフォールバック
            cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
            return;
        }
    }
    ft2->putText(img, text, org, fontHeight, color, -1, cv::LINE_AA, true);
}
#endif


void draw(cv::Mat& img, const std::string& text, cv::Point org,
          cv::Scalar color, double font_scale, int thickness, int baseline_offset) {
#if FLUENT_HAS_FREETYPE
    int fontHeight = static_cast<int>(24 * font_scale);
    cv::Point text_org = org + cv::Point(0, baseline_offset);
    putJapaneseText(img, text, text_org, fontHeight > 0 ? fontHeight : 24, color, "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc");
#else
    cv::putText(img, text, org + cv::Point(0, baseline_offset),
                cv::FONT_HERSHEY_SIMPLEX, font_scale, color,
                thickness, cv::LINE_AA);
#endif
}

void drawShadow(cv::Mat& img, const std::string& text, cv::Point org,
                cv::Scalar color, cv::Scalar shadow_color,
                double font_scale, int thickness, int baseline_offset) {
    cv::Point shadow_org = org + cv::Point(1,1);
    draw(img, text, shadow_org, shadow_color, font_scale, thickness, baseline_offset);
    draw(img, text, org, color, font_scale, thickness, baseline_offset);
}

} // namespace text
} // namespace fluent
