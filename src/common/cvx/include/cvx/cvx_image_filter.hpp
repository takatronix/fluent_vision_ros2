#pragma once
#include <opencv2/opencv.hpp>

namespace cvx {
    /// @brief 平均値フィルタ（ぼかし）
    cv::Mat blur(const cv::Mat& src, int ksize=3);
    /// @brief ガウシアンフィルタ
    cv::Mat gaussianBlur(const cv::Mat& src, int ksize=3, double sigma=1.0);
    /// @brief メディアンフィルタ
    cv::Mat medianBlur(const cv::Mat& src, int ksize=3);
    /// @brief バイラテラルフィルタ
    cv::Mat bilateralFilter(const cv::Mat& src, int d=9, double sigmaColor=75.0, double sigmaSpace=75.0);
    /// @brief シャープ化（ラプラシアン）
    cv::Mat laplacian(const cv::Mat& src, int ksize=3);
    /// @brief Sobelエッジ検出
    cv::Mat sobel(const cv::Mat& src, int dx, int dy, int ksize=3);
    /// @brief Scharrエッジ検出
    cv::Mat scharr(const cv::Mat& src, int dx, int dy);
    /// @brief Cannyエッジ検出
    cv::Mat canny(const cv::Mat& src, double th1, double th2);
    /// @brief 2値化
    cv::Mat threshold(const cv::Mat& src, double thresh, double maxval, int type);
    /// @brief アダプティブ2値化
    cv::Mat adaptiveThreshold(const cv::Mat& src, double maxval, int method, int type, int blockSize, double C);
    /// @brief モルフォロジー（膨張）
    cv::Mat dilate(const cv::Mat& src, int ksize=3);
    /// @brief モルフォロジー（収縮）
    cv::Mat erode(const cv::Mat& src, int ksize=3);
    /// @brief オープニング
    cv::Mat morphologyOpen(const cv::Mat& src, int ksize=3);
    /// @brief クロージング
    cv::Mat morphologyClose(const cv::Mat& src, int ksize=3);
    /// @brief 幾何変換（アフィン）
    cv::Mat warpAffine(const cv::Mat& src, const cv::Mat& M, const cv::Size& dsize);
    /// @brief 幾何変換（射影）
    cv::Mat warpPerspective(const cv::Mat& src, const cv::Mat& M, const cv::Size& dsize);
}
