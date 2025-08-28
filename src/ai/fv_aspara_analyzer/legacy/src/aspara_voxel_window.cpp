#include "fv_aspara_analyzer/aspara_voxel_window.hpp"
#include <cv_bridge/cv_bridge.h>

namespace fv_aspara_analyzer {

AsparaVoxelWindow::AsparaVoxelWindow(rclcpp::Node* node)
    : node_(node), overlay_enabled_(true), overlay_alpha_(0.5), straighten_(false), zoom_(1.0) {
    loadParams();
}

void AsparaVoxelWindow::loadParams() {
    position_ = node_->get_parameter("voxel_window.position").as_string();
    overlay_enabled_ = node_->get_parameter("voxel_window.overlay_enabled").as_bool();
    overlay_alpha_ = std::clamp(node_->get_parameter("voxel_window.overlay_alpha").as_double(), 0.0, 1.0);
    straighten_ = node_->get_parameter("voxel_window.straighten").as_bool();
    zoom_ = std::clamp(node_->get_parameter("voxel_window.zoom").as_double(), 0.1, 10.0);
}

void AsparaVoxelWindow::drawCloudToPanel(
    cv::Mat& output_image,
    const sensor_msgs::msg::PointCloud2& pc2,
    const cv::Rect& roi_rect,
    const cv::Rect& pnl_rect,
    const sensor_msgs::msg::CameraInfo::SharedPtr& caminfo) {
    if (!caminfo || pc2.data.empty()) return;
    double fx = caminfo->k[0];
    double fy = caminfo->k[4];
    double cx = caminfo->k[2];
    double cy = caminfo->k[5];
    bool has_rgb = false; for (const auto& f : pc2.fields) if (f.name == "rgb") { has_rgb = true; break; }
    const uint8_t* data_ptr = pc2.data.data();
    size_t step = pc2.point_step;
    size_t n = pc2.width * pc2.height;
    auto putDot = [&](int px, int py, const cv::Vec3b& c){
        if (std::abs(zoom_ - 1.0) > 1e-6) {
            double cxp = pnl_rect.x + pnl_rect.width * 0.5;
            double cyp = pnl_rect.y + pnl_rect.height * 0.5;
            px = static_cast<int>(std::round(cxp + (px - cxp) * zoom_));
            py = static_cast<int>(std::round(cyp + (py - cyp) * zoom_));
        }
        for (int dy=-1; dy<=1; ++dy) {
            for (int dx=-1; dx<=1; ++dx) {
                int xx = px+dx, yy = py+dy;
                if (xx > pnl_rect.x && xx < pnl_rect.x + pnl_rect.width-1 &&
                    yy > pnl_rect.y && yy < pnl_rect.y + pnl_rect.height-1) {
                    output_image.at<cv::Vec3b>(yy, xx) = c;
                }
            }
        }
    };
    for (size_t i = 0; i < n; ++i) {
        const uint8_t* pt = data_ptr + i * step;
        float x, y, z; std::memcpy(&x, pt + 0, 4); std::memcpy(&y, pt + 4, 4); std::memcpy(&z, pt + 8, 4);
        if (!std::isfinite(z) || z <= 0.0f) continue;
        double u = fx * (static_cast<double>(x) / static_cast<double>(z)) + cx;
        double v = fy * (static_cast<double>(y) / static_cast<double>(z)) + cy;
        if (u < roi_rect.x || u >= roi_rect.x + roi_rect.width || v < roi_rect.y || v >= roi_rect.y + roi_rect.height) continue;
        int px = pnl_rect.x + 1 + static_cast<int>((u - roi_rect.x) / roi_rect.width * (pnl_rect.width - 2));
        int py = pnl_rect.y + 1 + static_cast<int>((v - roi_rect.y) / roi_rect.height * (pnl_rect.height - 2));
        cv::Vec3b c(0,255,0);
        if (has_rgb && step >= 16) {
            float rgbf; std::memcpy(&rgbf, pt + 12, 4);
            uint32_t rgb; std::memcpy(&rgb, &rgbf, 4);
            uint8_t r = (rgb >> 16) & 0xff, g = (rgb >> 8) & 0xff, b = rgb & 0xff;
            c = cv::Vec3b(b,g,r);
        }
        putDot(px, py, c);
    }
}

void AsparaVoxelWindow::drawCloudToPanelStraight(
    cv::Mat& output_image,
    const sensor_msgs::msg::PointCloud2& pc2,
    const cv::Rect& pnl_rect) {
    if (pc2.data.empty()) return;
    const uint8_t* data_ptr = pc2.data.data();
    size_t step = pc2.point_step;
    size_t n = pc2.width * pc2.height;
    std::vector<cv::Vec3f> pts; pts.reserve(n);
    std::vector<cv::Vec3b> cols; cols.reserve(n);
    bool has_rgb = false; for (const auto& f : pc2.fields) if (f.name == "rgb") { has_rgb = true; break; }
    for (size_t i = 0; i < n; ++i) {
        const uint8_t* pt = data_ptr + i * step;
        float x, y, z; std::memcpy(&x, pt + 0, 4); std::memcpy(&y, pt + 4, 4); std::memcpy(&z, pt + 8, 4);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z <= 0.0f) continue;
        pts.emplace_back(x, y, z);
        if (has_rgb && step >= 16) {
            float rgbf; std::memcpy(&rgbf, pt + 12, 4);
            uint32_t rgb; std::memcpy(&rgb, &rgbf, 4);
            cols.emplace_back(static_cast<uint8_t>(rgb & 0xff), static_cast<uint8_t>((rgb >> 8) & 0xff), static_cast<uint8_t>((rgb >> 16) & 0xff));
        } else {
            cols.emplace_back(0,255,0);
        }
    }
    if (pts.size() < 3) return;
    cv::Mat data(static_cast<int>(pts.size()), 3, CV_32F);
    for (int i = 0; i < data.rows; ++i) {
        data.at<float>(i,0) = pts[i][0];
        data.at<float>(i,1) = pts[i][1];
        data.at<float>(i,2) = pts[i][2];
    }
    cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
    cv::Mat eigvecs = pca.eigenvectors.clone(); // rows: components, cols: dims
    cv::Vec3f pc1(eigvecs.at<float>(0,0), eigvecs.at<float>(0,1), eigvecs.at<float>(0,2));
    // 連続性: 前フレームと逆向きなら反転
    if (has_prev_axis_) {
        float dp = pc1.dot(prev_pc1_);
        if (dp < 0.0f) pc1 *= -1.0f;
    }
    prev_pc1_ = pc1; has_prev_axis_ = true;
    // 射影
    cv::Mat proj; pca.project(data, proj);
    // 符号はカメラYとの相関で安定化（PC1が増えるとyが減る=上に向くように）
    double sum_pc1 = 0.0, sum_y = 0.0, sum_pc1y = 0.0, sum_pc1_2 = 0.0, sum_y_2 = 0.0;
    int m = proj.rows;
    for (int i = 0; i < m; ++i) {
        double pc1v = proj.at<float>(i,0);
        double yv = pts[i][1];
        sum_pc1 += pc1v; sum_y += yv; sum_pc1y += pc1v * yv; sum_pc1_2 += pc1v * pc1v; sum_y_2 += yv * yv;
    }
    double denom = std::sqrt(std::max(1e-12, m*sum_pc1_2 - sum_pc1*sum_pc1)) * std::sqrt(std::max(1e-12, m*sum_y_2 - sum_y*sum_y));
    double corr = (m*sum_pc1y - sum_pc1*sum_y) / (denom + 1e-12);
    float sgn = (corr > 0.0) ? -1.0f : 1.0f;
    // ロバスト範囲: 中央±IQRの拡張
    std::vector<float> xs, ys; xs.reserve(proj.rows); ys.reserve(proj.rows);
    for (int i = 0; i < proj.rows; ++i) {
        xs.push_back(proj.at<float>(i,1));
        ys.push_back(static_cast<float>(sgn * proj.at<float>(i,0)));
    }
    auto quantile = [](std::vector<float> v, double q){
        if (v.empty()) return 0.0; std::nth_element(v.begin(), v.begin()+static_cast<size_t>(q*(v.size()-1)), v.end());
        return static_cast<double>(v[static_cast<size_t>(q*(v.size()-1))]);
    };
    double x_q1 = quantile(xs, 0.25), x_q3 = quantile(xs, 0.75);
    double y_q1 = quantile(ys, 0.25), y_q3 = quantile(ys, 0.75);
    double minx = x_q1 - 1.5*(x_q3 - x_q1), maxx = x_q3 + 1.5*(x_q3 - x_q1);
    double miny = y_q1 - 1.5*(y_q3 - y_q1), maxy = y_q3 + 1.5*(y_q3 - y_q1);
    // フォールバック
    if (!(maxx > minx)) { minx = *std::min_element(xs.begin(), xs.end()); maxx = *std::max_element(xs.begin(), xs.end()); }
    if (!(maxy > miny)) { miny = *std::min_element(ys.begin(), ys.end()); maxy = *std::max_element(ys.begin(), ys.end()); }
    for (int i = 0; i < proj.rows; ++i) {
        double X = proj.at<float>(i,1);
        double Y = sgn * proj.at<float>(i,0);
        if (X < minx) minx = X; if (X > maxx) maxx = X;
        if (Y < miny) miny = Y; if (Y > maxy) maxy = Y;
    }
    if (!(maxx > minx && maxy > miny)) return;
    auto putDot = [&](int px, int py, const cv::Vec3b& c){
        if (std::abs(zoom_ - 1.0) > 1e-6) {
            double cxp = pnl_rect.x + pnl_rect.width * 0.5;
            double cyp = pnl_rect.y + pnl_rect.height * 0.5;
            px = static_cast<int>(std::round(cxp + (px - cxp) * zoom_));
            py = static_cast<int>(std::round(cyp + (py - cyp) * zoom_));
        }
        for (int dy=-1; dy<=1; ++dy) {
            for (int dx=-1; dx<=1; ++dx) {
                int xx = px+dx, yy = py+dy;
                if (xx > pnl_rect.x && xx < pnl_rect.x + pnl_rect.width-1 &&
                    yy > pnl_rect.y && yy < pnl_rect.y + pnl_rect.height-1) {
                    output_image.at<cv::Vec3b>(yy, xx) = c;
                }
            }
        }
    };
    for (int i = 0; i < proj.rows; ++i) {
        double X = proj.at<float>(i,1);
        double Y = sgn * proj.at<float>(i,0);
        int px = pnl_rect.x + 1 + static_cast<int>(std::round((X - minx) / (maxx - minx + 1e-9) * (pnl_rect.width - 2)));
        int py = pnl_rect.y + 1 + static_cast<int>(std::round((maxy - Y) / (maxy - miny + 1e-9) * (pnl_rect.height - 2)));
        putDot(px, py, cols[i]);
    }
}

void AsparaVoxelWindow::drawPanels(
    cv::Mat& output_image,
    const cv::Rect& roi,
    const sensor_msgs::msg::PointCloud2* raw_cloud,
    const sensor_msgs::msg::PointCloud2* filtered_cloud,
    const sensor_msgs::msg::Image::SharedPtr& depth_img,
    const sensor_msgs::msg::Image::SharedPtr& color_img,
    const sensor_msgs::msg::CameraInfo::SharedPtr& caminfo) {
    (void)raw_cloud; (void)depth_img; (void)color_img; // reserved for future use (background previews)
    loadParams();

    const int panel_w = std::min(160, roi.width);
    const int panel_h = roi.height;
    int panel_x_l = std::max(5, roi.x - panel_w - 8);
    int panel_y = std::max(5, roi.y);
    cv::Rect panel_l(panel_x_l, panel_y, panel_w, std::min(panel_h, output_image.rows - panel_y - 5));
    int panel_x_r = std::min(output_image.cols - panel_w - 5, roi.x + roi.width + 8);
    cv::Rect panel_r(panel_x_r, panel_y, panel_w, std::min(panel_h, output_image.rows - panel_y - 5));

    bool draw_left = (position_ == "left");
    cv::Rect target_panel = draw_left ? panel_l : panel_r;

    // overlay background
    if (overlay_enabled_) {
        cv::Mat roi_img = output_image(target_panel);
        cv::Mat overlay_panel; roi_img.copyTo(overlay_panel);
        cv::rectangle(overlay_panel, cv::Rect(0,0,target_panel.width,target_panel.height), cv::Scalar(0,0,0), -1);
        cv::addWeighted(overlay_panel, overlay_alpha_, roi_img, 1.0 - overlay_alpha_, 0.0, roi_img);
    }

    // choose cloud
    const sensor_msgs::msg::PointCloud2* cloud = filtered_cloud ? filtered_cloud : raw_cloud;
    if (!cloud || cloud->data.empty()) return;

    int drawn = 0;
    if (straighten_) drawCloudToPanelStraight(output_image, *cloud, target_panel), (void)drawn;
    else             drawCloudToPanel(output_image, *cloud, roi, target_panel, caminfo);

    // タイトル表示は行わない（ユーザー要望）
}

} // namespace fv_aspara_analyzer


