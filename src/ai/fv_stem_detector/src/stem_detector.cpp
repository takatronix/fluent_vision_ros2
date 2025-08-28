#include "fv_stem_detector/stem_detector.hpp"

#include <opencv2/imgproc.hpp>
#include <chrono>
#include <limits>
#include <cmath>

namespace fv_stem_detector {

static inline bool isValidDepth16U(uint16_t d) {
  return d > 0;  // 0 is invalid in most RealSense depth images
}

std::vector<StemDetectionResult> StemDetector::detectBottomPoints(
    const cv::Mat& bgrImage,
    const cv::Mat& depthImage,
    const std::vector<StemRegion>& regions) const {
  std::vector<StemDetectionResult> results;
  results.reserve(regions.size());

  for (const StemRegion& region : regions) {
    auto t0 = std::chrono::high_resolution_clock::now();
    StemDetectionResult out;
    out.region_id = region.id;

    const int rx = std::max(0, region.x);
    const int ry = std::max(0, region.y);
    const int rw = std::max(0, region.w);
    const int rh = std::max(0, region.h);
    if (rw <= 0 || rh <= 0) {
      results.push_back(out);
      continue;
    }

    const int x2 = std::min(bgrImage.cols, rx + rw);
    const int y2 = std::min(bgrImage.rows, ry + rh);
    if (rx >= x2 || ry >= y2) {
      results.push_back(out);
      continue;
    }

    cv::Rect roi(rx, ry, x2 - rx, y2 - ry);

    cv::Mat gray;
    cv::cvtColor(bgrImage(roi), gray, cv::COLOR_BGR2GRAY);
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0.0);
    cv::Mat edges;
    cv::Canny(blurred, edges, 50, 120);

    // Morphological closing to strengthen vertical structures
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 9));
    cv::Mat morphed;
    cv::morphologyEx(edges, morphed, cv::MORPH_CLOSE, kernel);

    // Connected components; collect candidates that look like stems
    cv::Mat labels, stats, centroids;
    int n = cv::connectedComponentsWithStats(morphed, labels, stats, centroids, 8, CV_32S);

    const double imgCx = static_cast<double>(bgrImage.cols) * 0.5;
    const double imgCy = static_cast<double>(bgrImage.rows) * 0.5;

    struct Candidate { int bx; int by; double dist; uint16_t depth; };
    bool hasCandidate = false;
    Candidate best {0, 0, std::numeric_limits<double>::infinity(), std::numeric_limits<uint16_t>::max()};

    for (int label = 1; label < n; ++label) {
      int area = stats.at<int>(label, cv::CC_STAT_AREA);
      int w = stats.at<int>(label, cv::CC_STAT_WIDTH);
      int h = stats.at<int>(label, cv::CC_STAT_HEIGHT);
      if (h < 20 || w < 2) continue;  // too small
      double aspect = static_cast<double>(h) / std::max(1, w);
      if (aspect < 2.0) continue;      // prefer tall narrow

      // bottom-most pixel for this component
      int left = stats.at<int>(label, cv::CC_STAT_LEFT);
      int top = stats.at<int>(label, cv::CC_STAT_TOP);
      int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
      int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
      int bottomY = -1;
      int bottomX = -1;
      for (int y = top + height - 1; y >= top; --y) {
        bool found = false;
        for (int x = left; x < left + width; ++x) {
          if (labels.at<int>(y, x) == label) {
            bottomY = y;
            bottomX = x;
            found = true;
            break;
          }
        }
        if (found) break;
      }
      if (bottomX < 0 || bottomY < 0) continue;

      int absX = roi.x + bottomX;
      int absY = roi.y + bottomY;
      double dist = std::hypot(static_cast<double>(absX) - imgCx, static_cast<double>(absY) - imgCy);

      uint16_t depthVal = std::numeric_limits<uint16_t>::max();
      if (!depthImage.empty() && depthImage.type() == CV_16UC1) {
        uint16_t d0 = depthImage.at<uint16_t>(absY, absX);
        if (isValidDepth16U(d0)) {
          depthVal = d0;
        } else {
          // search a small window upwards for valid depth
          bool ok = false;
          int bx = absX, by = absY;
          for (int dy = 0; dy < 6 && !ok; ++dy) {
            int yy = std::max(0, by - dy);
            for (int dx = -2; dx <= 2; ++dx) {
              int xx = std::clamp(bx + dx, 0, depthImage.cols - 1);
              uint16_t dd = depthImage.at<uint16_t>(yy, xx);
              if (isValidDepth16U(dd)) {
                depthVal = dd;
                absX = xx; absY = yy; // adjust to valid point
                bottomX = absX - roi.x; bottomY = absY - roi.y;
                ok = true;
                break;
              }
            }
          }
          if (!ok) continue; // skip this candidate if no valid depth found
        }
      }

      Candidate cand {absX, absY, dist, depthVal};
      if (!hasCandidate || cand.dist < best.dist || (std::abs(cand.dist - best.dist) < 1e-3 && cand.depth < best.depth)) {
        best = cand;
        hasCandidate = true;
      }
    }

    if (!hasCandidate) {
      results.push_back(out);
      continue;
    }

    out.x = best.bx;
    out.y = best.by;
    out.detected = true;
    auto t1 = std::chrono::high_resolution_clock::now();
    out.duration_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    results.push_back(out);
  }

  return results;
}

}  // namespace fv_stem_detector


