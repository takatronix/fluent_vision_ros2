#pragma once
#include <opencv2/core.hpp>

namespace cvx {
// 線分描画
void drawLine(cv::Mat& img, cv::Point pt1, cv::Point pt2,
              cv::Scalar color, int thickness = 1, int line_type = -1);
// 矩形描画
void drawRectangle(cv::Mat& img, cv::Rect rect,
                   cv::Scalar color, int thickness = 1, int line_type = -1);
// 円描画
void drawCircle(cv::Mat& img, cv::Point center, int radius, cv::Scalar color, int thickness = 1, int line_type = -1);
// クロス（十字）描画
void drawCross(cv::Mat& img, cv::Point center, int size, cv::Scalar color, int thickness = 1, int line_type = -1);
// ポリゴン描画
void drawPolygon(cv::Mat& img, const std::vector<cv::Point>& pts, cv::Scalar color, int thickness = 1, int line_type = -1, bool isClosed = true);
}
