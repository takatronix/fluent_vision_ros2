#include "fv_aspara_analyzer/aspara_visualizer.hpp"
#include "fluent_lib/ui/offscreen_canvas.hpp"
#include <cv_bridge/cv_bridge.h>

namespace fv_aspara_analyzer {

AsparaVisualizer::AsparaVisualizer(rclcpp::Node* node, OverlayEngine* overlay_engine)
    : node_(node), overlay_(overlay_engine)
{
    // 最小: 依存パブリッシャ/TFはノード側のものを利用するためここでは初期化しない
}

cv::Mat AsparaVisualizer::renderCurrentImage(
    const cv::Mat& color_image,
    const std::vector<AsparaInfo>& aspara_list,
    int selected_aspara_id,
    const std::map<std::string, float>& /*fps_info*/,
    bool /*enable_pointcloud_processing*/)
{
    if (color_image.empty()) {
        // 入力が空でも、安定した出力を返す（待機画面）
        int w = 640, h = 480;
        cv::Mat standby(h, w, CV_8UC3, cv::Scalar(0,0,0));
        std::string text = "待機中...";
        int base = 0; double scale = 1.0; int th = 2;
        cv::Size ts = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, th, &base);
        cv::putText(standby, text, cv::Point((w - ts.width) / 2, (h + ts.height) / 2), cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(255,255,255), th, cv::LINE_AA);
        return standby;
    }
    cv::Mat out = color_image.clone();

    // OverlayEngineから最新スナップショット（無ければ引数）
    std::vector<AsparaInfo> to_draw = aspara_list;
    double sg = 12.0, fg_in = 6.0, fg_out = 3.0;
    try {
        sg = node_->declare_parameter<double>("overlay.smooth_gain", 12.0);
        fg_in = node_->declare_parameter<double>("overlay.fade_in_gain", 6.0);
        fg_out = node_->declare_parameter<double>("overlay.fade_out_gain", 3.0);
    } catch (...) {}
    if (overlay_) {
        // 全個体のターゲットを毎フレーム供給（補間tickはノード側タイマーで実施）
        overlay_->setTargets(aspara_list);
        overlay_->setGains(sg, fg_in, fg_out);
        auto snap = overlay_->snapshot();
        if (!snap.empty()) to_draw = std::move(snap);
    }

    // YAMLパラメータ（既定値）
    // 選択/非選択の基準α（アニメーションαと乗算）、線幅、色、ラベル背景α
    double base_sel_alpha = 0.8, base_unsel_alpha = 0.4;
    int sel_th = 2, unsel_th = 1;
    cv::Scalar sel_color(0,255,0), unsel_color(128,128,128);
    double label_bg_alpha = 0.4;
    try {
        base_sel_alpha = node_->declare_parameter<double>("overlay.selected.alpha", 0.8);
        base_unsel_alpha = node_->declare_parameter<double>("overlay.unselected.alpha", 0.4);
        sel_th = node_->declare_parameter<int>("overlay.selected.thickness_px", 2);
        unsel_th = node_->declare_parameter<int>("overlay.unselected.thickness_px", 1);
        label_bg_alpha = node_->declare_parameter<double>("label.bg_alpha", 0.4);
    } catch (...) {}

    // 収穫可否の表示（選択中のみ）
    bool harvest_enabled = node_->declare_parameter<bool>("harvest.enabled", true);
    bool harvest_selected_only = node_->declare_parameter<bool>("harvest.selected_only", true);
    double harvest_thr_cm = node_->declare_parameter<double>("harvest.threshold_cm", 23.0);
    cv::Scalar harvest_green(0,255,0), harvest_orange(0,165,255), title_text(255,255,255);
    double title_bg_a = node_->declare_parameter<double>("harvest.title_bg_alpha", 0.8);

    // HUDレイヤ（半透明領域の合成に使用）
    fluent_ui::OffscreenCanvas hud;
    hud.create(out.size());

    // 描画
    for (const auto& a : to_draw) {
        bool is_selected = (a.id == selected_aspara_id);
        const cv::Rect& b = a.bounding_box_2d;
        if (b.width <= 0 || b.height <= 0) continue;

        // デフォルト色/太さ/α
        cv::Scalar color = is_selected ? sel_color : unsel_color;
        int th = is_selected ? sel_th : unsel_th;

        // 収穫ハイライト色の優先（選択中のみ）
        bool show_title = false;
        std::string title;
        if (harvest_enabled && (!harvest_selected_only || is_selected) && a.length_valid) {
            // m→cm 変換、整数丸め
            int len_cm_round = static_cast<int>(std::round(a.length * 100.0f));
            if (len_cm_round > 0) {
                bool harvestable = (len_cm_round >= static_cast<int>(std::round(harvest_thr_cm)));
                color = harvestable ? harvest_green : harvest_orange;
                if (is_selected) { // 非選択は枠のみ
                    show_title = true;
                    title = std::string("アスパラ") + std::to_string(len_cm_round) + "cm # " + std::to_string(a.id);
                }
            }
        }

        // αに応じて枠の濃さを調整（基準α×アニメーションα）
        double eff_alpha = (is_selected ? base_sel_alpha : base_unsel_alpha) * std::clamp(static_cast<double>(a.animation_alpha), 0.0, 1.0);
        cv::Scalar cblend = color * eff_alpha + cv::Scalar(0,0,0) * (1.0 - eff_alpha);
        cv::rectangle(out, b, cblend, th, cv::LINE_AA);

        // ID/信頼度ラベル（全個体）
        char buf[128];
        std::snprintf(buf, sizeof(buf), "ID:%d 信頼度:%.1f%%", a.id, std::clamp(a.confidence * 100.0f, 0.0f, 100.0f));
        std::string label(buf);
        int base = 0; double scale = 0.7; int t = 2; cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, scale, t, &base);
        cv::Rect lrect(b.x, b.y - ts.height - 6, ts.width + 10, ts.height + 6);
        if (lrect.y < 0) lrect.y = b.y + 2;
        lrect &= cv::Rect(0, 0, out.cols, out.rows);
        if (lrect.area() > 0) {
            // 背景はオフスクリーンに描いて矩形領域のみ合成
            hud.fillRect(lrect, cv::Scalar(0,0,0));
            hud.blendRegion(out, lrect, label_bg_alpha);
            cv::putText(out, label, cv::Point(lrect.x + 5, lrect.y + lrect.height - 4), cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(255,255,255), t, cv::LINE_AA);
        }

        // タイトル（収穫可能/不可）
        if (show_title) {
            int base2 = 0; double scale2 = 0.8; int t2 = 2; cv::Size ts2 = cv::getTextSize(title, cv::FONT_HERSHEY_SIMPLEX, scale2, t2, &base2);
            cv::Rect trect(b.x, b.y - ts2.height - 10, ts2.width + 12, ts2.height + 10);
            if (trect.y < 0) trect.y = b.y + b.height + 4;
            trect &= cv::Rect(0, 0, out.cols, out.rows);
            if (trect.area() > 0) {
                hud.fillRect(trect, color);
                hud.blendRegion(out, trect, title_bg_a);
                cv::putText(out, title, cv::Point(trect.x + 6, trect.y + trect.height - 6), cv::FONT_HERSHEY_SIMPLEX, scale2, title_text, t2, cv::LINE_AA);
            }
        }
    }

    return out;
}

cv::Mat AsparaVisualizer::renderAnnotatedImage(
    const cv::Mat& image,
    const AsparaInfo& /*aspara_info*/,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& /*filtered_cloud*/,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& /*pca_line_cloud*/,
    const sensor_msgs::msg::CameraInfo& /*camera_info*/,
    float /*length*/,
    float /*straightness*/,
    bool /*is_harvestable*/)
{
    return image.clone();
}

void AsparaVisualizer::updateSmoothAnimation(
    std::vector<AsparaInfo>& /*aspara_list*/,
    float /*delta_time*/)
{
    // 最小実装: 何もしない（OverlayEngineが将来担当）
}

void AsparaVisualizer::publishImage(
    const cv::Mat& image,
    const std_msgs::msg::Header& header,
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher)
{
    if (!publisher) return;
    if (image.empty()) return;
    auto msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image).toImageMsg();
    publisher->publish(*msg);
}

cv::Point2f AsparaVisualizer::project3DTo2D(
    const pcl::PointXYZRGB& p,
    const sensor_msgs::msg::CameraInfo& cam)
{
    // pinhole: u = fx * x/z + cx, v = fy * y/z + cy
    if (p.z <= 0.0f) return cv::Point2f(-1,-1);
    double fx = cam.k[0], fy = cam.k[4], cx = cam.k[2], cy = cam.k[5];
    float u = static_cast<float>(fx * (static_cast<double>(p.x) / static_cast<double>(p.z)) + cx);
    float v = static_cast<float>(fy * (static_cast<double>(p.y) / static_cast<double>(p.z)) + cy);
    return cv::Point2f(u, v);
}

// 3Dマーカー/点群/TFの最小実装は省略（必要時に実装）
visualization_msgs::msg::MarkerArray AsparaVisualizer::createAsparaCylinderMarkers(
    const std::vector<AsparaInfo>& /*aspara_list*/,
    int /*selected_aspara_id*/,
    const std::string& /*frame_id*/)
{
    return visualization_msgs::msg::MarkerArray();
}

void AsparaVisualizer::publishMarkerArray(const visualization_msgs::msg::MarkerArray& /*markers*/)
{
    // 最小実装: 何もしない
}

void AsparaVisualizer::publishFilteredPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& /*cloud*/,
    const std::string& /*frame_id*/,
    int /*aspara_id*/)
{
}

void AsparaVisualizer::publishRootTF(
    const geometry_msgs::msg::Point& /*root_position*/,
    const std::string& /*frame_id*/,
    int /*aspara_id*/)
{
}

void AsparaVisualizer::drawAsparaDetailInfo(
    cv::Mat& /*image*/,
    const AsparaInfo& /*aspara*/,
    bool /*is_selected*/)
{
}

} // namespace fv_aspara_analyzer


