#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>

// Fluent統合ライブラリ - 美しいチェーン可能API
#include "image.hpp"
#include "cloud.hpp" 
#include "utils/fps.hpp"

// 便利なヘルパー関数
namespace Colors {
    const cv::Scalar RED(0, 0, 255);
    const cv::Scalar GREEN(0, 255, 0);
    const cv::Scalar BLUE(255, 0, 0);
    const cv::Scalar WHITE(255, 255, 255);
    const cv::Scalar BLACK(0, 0, 0);
    const cv::Scalar YELLOW(0, 255, 255);
}

// RGB変換
inline cv::Scalar rgb(int r, int g, int b) { 
    return cv::Scalar(b, g, r); 
}

// 簡単デバッグ
template<typename T>
void debug(const T& value, const std::string& label = "") {
    std::cout << label << ": " << value << std::endl;
}