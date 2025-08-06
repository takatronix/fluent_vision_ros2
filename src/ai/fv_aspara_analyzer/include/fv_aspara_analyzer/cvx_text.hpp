/**
 * @file cvx_text.hpp
 * @brief OpenCVで日本語テキストを描画するためのヘルパークラス
 * @details FreeTypeを使用して日本語フォントをレンダリング
 * @author Takashi Otsuka
 * @date 2025
 */

#ifndef CVX_TEXT_HPP
#define CVX_TEXT_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <memory>

namespace fv_aspara_analyzer
{

/**
 * @class CvXText
 * @brief 日本語テキスト描画クラス（シンプル版）
 * @details OpenCVのputTextでは日本語が表示できないため、代替実装を提供
 */
class CvXText
{
public:
    /**
     * @brief コンストラクタ
     * @param font_path フォントファイルのパス（デフォルト: システムフォント）
     * @param font_size フォントサイズ（デフォルト: 20）
     */
    CvXText(const std::string& font_path = "", int font_size = 20);
    
    /**
     * @brief デストラクタ
     */
    ~CvXText();
    
    /**
     * @brief 日本語テキストを画像に描画
     * @param img 描画対象の画像
     * @param text 描画するテキスト（UTF-8）
     * @param pos 描画位置
     * @param color テキストの色
     * @return 成功時true
     */
    bool putText(cv::Mat& img, const std::string& text, cv::Point pos, cv::Scalar color);
    
    /**
     * @brief フォントサイズを設定
     * @param size 新しいフォントサイズ
     */
    void setFontSize(int size);
    
    /**
     * @brief 代替実装：OpenCVのputTextを使用（ASCII文字のみ）
     * @note 日本語が含まれない場合はこちらを使用
     */
    static void putTextASCII(cv::Mat& img, const std::string& text, cv::Point pos, 
                            int fontFace, double fontScale, cv::Scalar color, 
                            int thickness = 1, int lineType = cv::LINE_8);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

/**
 * @brief グローバルヘルパー関数：日本語テキスト描画
 * @param img 描画対象の画像
 * @param text 描画するテキスト
 * @param pos 描画位置
 * @param font_size フォントサイズ
 * @param color テキストの色
 * @param thickness 線の太さ（未使用、互換性のため）
 */
inline void putTextJP(cv::Mat& img, const std::string& text, cv::Point pos, 
                     double font_size, cv::Scalar color, int thickness = 1)
{
    // ASCII文字のみの場合は通常のputTextを使用
    bool has_japanese = false;
    for (unsigned char c : text) {
        if (c >= 0x80) {  // 非ASCII文字
            has_japanese = true;
            break;
        }
    }
    
    if (!has_japanese) {
        cv::putText(img, text, pos, cv::FONT_HERSHEY_SIMPLEX, 
                   font_size / 20.0, color, thickness);
    } else {
        // 日本語が含まれる場合は警告を表示して英語で代替
        cv::putText(img, "[JP Text]", pos, cv::FONT_HERSHEY_SIMPLEX, 
                   font_size / 20.0, color, thickness);
    }
}

} // namespace fv_aspara_analyzer

#endif // CVX_TEXT_HPP