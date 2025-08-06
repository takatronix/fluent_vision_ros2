#include "fv_object_detector/object_tracker.hpp"
#include <algorithm>
#include <limits>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

namespace fv_object_detector
{

ObjectTracker::ObjectTracker() : next_object_id_(0), selected_object_id_(-1) {}

void ObjectTracker::assignObjectIds(std::vector<DetectionData> &detections) {
  // 前回の検出数を保存
  size_t previous_detection_count = current_detections_.size();

  // 各オブジェクトの初期化
  for (auto &det : detections) {
    det.object_id = -1;      // 未割り当て状態
  }

  // 既に割り当て済みのIDを追跡するセット
  std::set<int> assigned_ids;

  // 前フレームのオブジェクトと現在フレームのオブジェクトを対応付ける
  if (!current_detections_.empty()) {
    // 前フレームの各オブジェクトに対して、最も近い現在フレームのオブジェクトを探す
    std::vector<bool> current_matched(detections.size(), false);

    for (const auto &prev_det : current_detections_) {
      cv::Point2f prev_center(prev_det.bbox.x + prev_det.bbox.width / 2,
                              prev_det.bbox.y + prev_det.bbox.height / 2);

      float min_distance = 50.0f; // 距離の閾値（これより大きい場合はマッチしない）
      int best_match_idx = -1;

      // 最も近い現在フレームのオブジェクトを探す
      for (size_t i = 0; i < detections.size(); ++i) {
        if (current_matched[i])
          continue; // マッチしたオブジェクトはスキップ

        cv::Point2f curr_center(
            detections[i].bbox.x + detections[i].bbox.width / 2,
            detections[i].bbox.y + detections[i].bbox.height / 2);

        float distance = cv::norm(curr_center - prev_center);

        if (distance < min_distance) {
          min_distance = distance;
          best_match_idx = static_cast<int>(i);
        }
      }

      // マッチした場合、IDを引き継ぐ
      if (best_match_idx >= 0) {
        detections[best_match_idx].object_id = prev_det.object_id;
        current_matched[best_match_idx] = true;
        assigned_ids.insert(prev_det.object_id);
      }
    }
  }

  // 未割り当てのオブジェクトに新しいIDを割り当てる
  for (auto &det : detections) {
    if (det.object_id < 0) {
      det.object_id = next_object_id_++;
    }
  }

  // 現在の検出結果を更新
  current_detections_ = detections;
}

const std::vector<DetectionData> &
ObjectTracker::getCurrentDetections() const {
  return current_detections_;
}

int ObjectTracker::getDetectionCountByClass(int class_id) const {
  return std::count_if(current_detections_.begin(), current_detections_.end(),
                       [class_id](const DetectionData &det) {
                         return det.class_id == class_id;
                       });
}

int ObjectTracker::getTotalDetectionCount() const {
  return current_detections_.size();
}

void ObjectTracker::reset() {
  next_object_id_ = 0;
  current_detections_.clear();
  selected_object_id_ = -1;
}

} // namespace fv_object_detector 