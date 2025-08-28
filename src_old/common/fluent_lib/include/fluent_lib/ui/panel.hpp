#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <algorithm>

namespace fluent_ui {

// Project and draw a PointCloud2 into a rectangular panel mapped from an image ROI.
// - pc2: organized or unorganized; expects fields x,y,z and optional rgb(float32)
// - roi_img: ROI region in the source image where the cloud is considered valid
// - panel: destination rectangle on canvas where points will be drawn
// Returns number of points drawn via out_count
inline void draw_cloud_panel(cv::Mat &canvas,
                             const sensor_msgs::msg::PointCloud2 &pc2,
                             const sensor_msgs::msg::CameraInfo &K,
                             const cv::Rect &roi_img,
                             const cv::Rect &panel,
                             int &out_count)
{
    out_count = 0;
    if (pc2.data.empty() || panel.width <= 2 || panel.height <= 2) return;
    double fx = K.k[0], fy = K.k[4], cx = K.k[2], cy = K.k[5];
    bool has_rgb = false;
    for (const auto &f : pc2.fields) if (f.name == "rgb") { has_rgb = true; break; }
    const uint8_t *data_ptr = pc2.data.data();
    const size_t step = pc2.point_step;
    const size_t n = static_cast<size_t>(pc2.width) * static_cast<size_t>(pc2.height);

    auto putDot = [&](int px, int py, const cv::Vec3b &c){
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int xx = px + dx, yy = py + dy;
                if (xx > panel.x && xx < panel.x + panel.width - 1 &&
                    yy > panel.y && yy < panel.y + panel.height - 1) {
                    canvas.at<cv::Vec3b>(yy, xx) = c;
                }
            }
        }
    };

    for (size_t i = 0; i < n; ++i) {
        const uint8_t *pt = data_ptr + i * step;
        float x, y, z; std::memcpy(&x, pt + 0, 4); std::memcpy(&y, pt + 4, 4); std::memcpy(&z, pt + 8, 4);
        if (!std::isfinite(z) || z <= 0.0f) continue;
        double u = fx * (static_cast<double>(x) / static_cast<double>(z)) + cx;
        double v = fy * (static_cast<double>(y) / static_cast<double>(z)) + cy;
        if (u < roi_img.x || u >= roi_img.x + roi_img.width ||
            v < roi_img.y || v >= roi_img.y + roi_img.height) continue;
        int px = panel.x + 1 + static_cast<int>((u - roi_img.x) / roi_img.width * (panel.width - 2));
        int py = panel.y + 1 + static_cast<int>((v - roi_img.y) / roi_img.height * (panel.height - 2));
        cv::Vec3b c(0, 255, 0);
        if (has_rgb && step >= 16) {
            float rgbf; std::memcpy(&rgbf, pt + 12, 4);
            uint32_t rgb; std::memcpy(&rgb, &rgbf, 4);
            uint8_t r = (rgb >> 16) & 0xff, g = (rgb >> 8) & 0xff, b = rgb & 0xff;
            c = cv::Vec3b(b, g, r);
        }
        putDot(px, py, c);
        ++out_count;
    }
}

// Paste a grayscale histogram strip below a bbox
inline void paste_histogram_strip(cv::Mat &canvas,
                                  const cv::Mat &strip_gray,
                                  const cv::Rect &bbox,
                                  int strip_height_px = 8,
                                  int offset_y_px = 4)
{
    if (strip_gray.empty() || bbox.width <= 1 || bbox.height <= 1) return;
    const int strip_h = std::max(2, std::min(64, strip_height_px));
    cv::Mat strip_resized; cv::resize(strip_gray, strip_resized, cv::Size(bbox.width, strip_h), 0, 0, cv::INTER_NEAREST);
    cv::Mat strip_u8;
    if (strip_resized.type() != CV_8UC1) { strip_resized.convertTo(strip_u8, CV_8UC1); } else { strip_u8 = strip_resized; }
    cv::Mat strip_bgr; cv::cvtColor(strip_u8, strip_bgr, cv::COLOR_GRAY2BGR);
    int sx = std::clamp(bbox.x, 0, std::max(0, canvas.cols - 1));
    int sy = std::clamp(bbox.y + bbox.height + offset_y_px, 0, std::max(0, canvas.rows - 1));
    int sw = std::min(strip_bgr.cols, canvas.cols - sx);
    int sh = std::min(strip_bgr.rows, canvas.rows - sy);
    if (sw > 0 && sh > 0) {
        strip_bgr(cv::Rect(0, 0, sw, sh)).copyTo(canvas(cv::Rect(sx, sy, sw, sh)));
    }
}

} // namespace fluent_ui


