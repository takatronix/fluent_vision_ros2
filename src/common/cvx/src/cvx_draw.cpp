#include "cvx/cvx_draw.hpp"
#include <opencv2/imgproc.hpp>

namespace cvx {

void drawLine(cv::Mat& img, cv::Point pt1, cv::Point pt2,
              cv::Scalar color, int thickness, int line_type) {
    int lt = (line_type == -1) ? cv::LINE_8 : line_type;
    cv::line(img, pt1, pt2, color, thickness, lt);
}

void drawRectangle(cv::Mat& img, cv::Rect rect,
                   cv::Scalar color, int thickness, int line_type) {
    int lt = (line_type == -1) ? cv::LINE_8 : line_type;
    cv::rectangle(img, rect, color, thickness, lt);
}

void drawCircle(cv::Mat& img, cv::Point center, int radius, cv::Scalar color, int thickness, int line_type) {
    int lt = (line_type == -1) ? cv::LINE_8 : line_type;
    cv::circle(img, center, radius, color, thickness, lt);
}

void drawCross(cv::Mat& img, cv::Point center, int size, cv::Scalar color, int thickness, int line_type) {
    int lt = (line_type == -1) ? cv::LINE_8 : line_type;
    cv::line(img, cv::Point(center.x - size, center.y), cv::Point(center.x + size, center.y), color, thickness, lt);
    cv::line(img, cv::Point(center.x, center.y - size), cv::Point(center.x, center.y + size), color, thickness, lt);
}

void drawPolygon(cv::Mat& img, const std::vector<cv::Point>& pts, cv::Scalar color, int thickness, int line_type, bool isClosed) {
    if (pts.size() < 2) return;
    int lt = (line_type == -1) ? cv::LINE_8 : line_type;
    const cv::Point* p = pts.data();
    int n = static_cast<int>(pts.size());
    cv::polylines(img, &p, &n, 1, isClosed, color, thickness, lt);
}

} // namespace cvx


