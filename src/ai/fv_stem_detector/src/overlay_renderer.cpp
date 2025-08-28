#include "fv_stem_detector/overlay_renderer.hpp"
#include "fluent_lib/fluent.hpp"

namespace fv_stem_detector {

OverlayRenderer::OverlayRenderer(const OverlayStyle& style)
  : style_(style) {}

void OverlayRenderer::drawRegions(cv::Mat& img,
                                  const std::vector<cv::Rect>& fixed_regions,
                                  const std::vector<int>& fixed_ids,
                                  const std::vector<cv::Rect>& yolo_regions,
                                  const std::vector<int>& yolo_ids) const {
  for (size_t i=0;i<fixed_regions.size();++i) {
    const auto& r = fixed_regions[i];
    cv::rectangle(img, r, style_.fixed_rect_color, style_.rect_thickness);
    if (style_.show_region_labels) {
      int x = std::min(img.cols-10, r.x + r.width + 6);
      int y = std::max(0, r.y + 24 - 6);
      fluent::text::draw(img, std::string("固定-") + std::to_string(fixed_ids[i]), cv::Point(x + 4, y), style_.label_text_color, 0.5, 1, 0);
    }
  }
  for (size_t i=0;i<yolo_regions.size();++i) {
    const auto& r = yolo_regions[i];
    cv::rectangle(img, r, style_.yolo_rect_color, style_.rect_thickness);
    if (style_.show_region_labels) {
      int x = std::min(img.cols-10, r.x + r.width + 6);
      int y = std::max(0, r.y + 24 - 6);
      fluent::text::draw(img, std::string("YOLO-") + std::to_string(yolo_ids[i]), cv::Point(x + 4, y), style_.label_text_color, 0.5, 1, 0);
    }
  }
}

void OverlayRenderer::drawDetection(cv::Mat& img,
                                    const cv::Rect& roi,
                                    const cv::Point& root_px,
                                    const cv::Point& tip_px,
                                    const cv::Point& approach_px,
                                    int aspara_id,
                                    int yolo_conf_pct,
                                    int pca_conf_pct,
                                    double approach_cm,
                                    double length_cm,
                                    double roi_ms,
                                    bool use_yolo_color) const {
  cv::rectangle(img, roi, use_yolo_color ? style_.yolo_rect_color : style_.fixed_rect_color, style_.rect_thickness);
  cv::circle(img, root_px, style_.cut_point_radius, style_.cut_point_color, style_.cut_point_thickness);
  cv::circle(img, tip_px,  style_.cut_point_radius, cv::Scalar(0,255,255), style_.cut_point_thickness);
  cv::line(img, root_px, tip_px, cv::Scalar(255,255,0), 1, cv::LINE_AA);
  cv::circle(img, approach_px, style_.approach_point_radius, style_.approach_point_color, style_.approach_point_thickness);

  // 行ごとにfluent_textで描画
  int x = std::clamp(roi.x + 6, 0, img.cols - 10);
  int y = std::clamp(roi.y + 24, 0, img.rows - 10);
  auto put = [&](const std::string &s){
    fluent::text::draw(img, s, cv::Point(x, y), style_.label_text_color, 0.5, 1, 0);
    y += 14;
  };
  char buf[128];
  snprintf(buf, sizeof(buf), "アスパラ#%d %d%%", aspara_id, yolo_conf_pct); put(buf);
  snprintf(buf, sizeof(buf), "2D信頼度: %d%%", yolo_conf_pct); put(buf);
  snprintf(buf, sizeof(buf), "3D信頼度: %d%%", pca_conf_pct); put(buf);
  snprintf(buf, sizeof(buf), "アプローチ距離: %.1f cm", approach_cm); put(buf);
  snprintf(buf, sizeof(buf), "長さ推定(直線): %.1f cm", length_cm); put(buf);
  snprintf(buf, sizeof(buf), "処理: %.1f ms", roi_ms); put(buf);
}

} // namespace fv_stem_detector


