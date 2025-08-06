/**
 * @file cvx_text.cpp
 * @brief 日本語テキスト描画クラスの実装
 * @author Takashi Otsuka
 * @date 2025
 */

#include "fv_aspara_analyzer/cvx_text.hpp"
#include <iostream>

namespace fv_aspara_analyzer
{

class CvXText::Impl
{
public:
    int font_size;
    std::string font_path;
    
    Impl(const std::string& path, int size) 
        : font_path(path), font_size(size) 
    {
        // フォントパスが指定されていない場合はデフォルトを使用
        if (font_path.empty()) {
            // Ubuntuのデフォルト日本語フォント
            font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc";
        }
    }
};

CvXText::CvXText(const std::string& font_path, int font_size)
    : pImpl(std::make_unique<Impl>(font_path, font_size))
{
}

CvXText::~CvXText() = default;

bool CvXText::putText(cv::Mat& img, const std::string& text, cv::Point pos, cv::Scalar color)
{
    // シンプルな実装：日本語が含まれる場合は英語で代替表示
    putTextJP(img, text, pos, static_cast<double>(pImpl->font_size), color);
    return true;
}

void CvXText::setFontSize(int size)
{
    pImpl->font_size = size;
}

void CvXText::putTextASCII(cv::Mat& img, const std::string& text, cv::Point pos, 
                           int fontFace, double fontScale, cv::Scalar color, 
                           int thickness, int lineType)
{
    cv::putText(img, text, pos, fontFace, fontScale, color, thickness, lineType);
}

} // namespace fv_aspara_analyzer