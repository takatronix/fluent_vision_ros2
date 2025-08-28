#pragma once

#include <opencv2/opencv.hpp>
#include <string>

namespace fv_stem_detector {

struct OverlayStyle {
  cv::Scalar fixed_rect_color {255, 0, 0};
  cv::Scalar yolo_rect_color  {0, 255, 0};
  int rect_thickness {2};

  cv::Scalar cut_point_color {0, 0, 255};
  int cut_point_radius {6};
  int cut_point_thickness {2};

  cv::Scalar label_text_color {255,255,255};
  bool show_region_labels {true};

  cv::Scalar approach_point_color {255,0,255};
  int approach_point_radius {5};
  int approach_point_thickness {2};
};

class OverlayRenderer {
public:
  explicit OverlayRenderer(const OverlayStyle& style);

  void drawRegions(cv::Mat& img,
                   const std::vector<cv::Rect>& fixed_regions,
                   const std::vector<int>& fixed_ids,
                   const std::vector<cv::Rect>& yolo_regions,
                   const std::vector<int>& yolo_ids) const;

  void drawDetection(cv::Mat& img,
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
                     bool use_yolo_color) const;

private:
  OverlayStyle style_;
};

} // namespace fv_stem_detector


