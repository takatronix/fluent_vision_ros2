#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace fv_stem_detector {

struct StemRegion {
  int id {0};
  int x {0};
  int y {0};
  int w {0};
  int h {0};
  float yolo_confidence {std::numeric_limits<float>::quiet_NaN()};
  int class_id {-1};
};

struct StemDetectionResult {
  int region_id {0};
  int x {0};
  int y {0};
  bool detected {false};
  double duration_ms {0.0};
};

class StemDetector {
public:
  StemDetector() = default;

  std::vector<StemDetectionResult> detectBottomPoints(
      const cv::Mat& bgrImage,
      const cv::Mat& depthImage,
      const std::vector<StemRegion>& regions) const;
};

}  // namespace fv_stem_detector


