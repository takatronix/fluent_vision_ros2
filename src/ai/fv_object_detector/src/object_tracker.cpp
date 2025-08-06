/**
 * @file object_tracker.cpp
 * @brief オブジェクトトラッカーの実装ファイル
 * @details フレーム間でのオブジェクトID追跡機能を提供
 * @author Takashi Otsuka
 * @date 2024
 * @version 1.0
 */

#include "fv_object_detector/object_tracker.hpp"
#include <algorithm>
#include <limits>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

namespace fv_object_detector
{

/**
 * @brief コンストラクタ
 * @details オブジェクトトラッカーの初期化
 */
ObjectTracker::ObjectTracker() : next_object_id_(0), selected_object_id_(-1) {}

/**
 * @brief オブジェクトIDの割り当て
 * @param detections 検出結果の配列
 * @details 前フレームのオブジェクトと現在フレームのオブジェクトを対応付け、一意のIDを割り当て
 * 
 * 処理内容：
 * - 前フレームのオブジェクトとの距離計算
 * - 最も近いオブジェクトとのマッチング
 * - 新しいオブジェクトへのID割り当て
 * - 現在の検出結果の更新
 */
void ObjectTracker::assignObjectIds(std::vector<DetectionData> &detections) {
  // 前回の検出数を保存
  size_t previous_detection_count = current_detections_.size();

  // ===== 各オブジェクトの初期化 =====
  for (auto &det : detections) {
    det.object_id = -1;      // 未割り当て状態
  }

  // ===== 既に割り当て済みのIDを追跡するセット =====
  std::set<int> assigned_ids;

  // ===== 前フレームのオブジェクトと現在フレームのオブジェクトを対応付ける =====
  if (!current_detections_.empty()) {
    // 前フレームの各オブジェクトに対して、最も近い現在フレームのオブジェクトを探す
    std::vector<bool> current_matched(detections.size(), false);

    for (const auto &prev_det : current_detections_) {
      // 前フレームのオブジェクト中心座標を計算
      cv::Point2f prev_center(prev_det.bbox.x + prev_det.bbox.width / 2,
                              prev_det.bbox.y + prev_det.bbox.height / 2);

      float min_distance = 50.0f; // 距離の閾値（これより大きい場合はマッチしない）
      int best_match_idx = -1;

      // ===== 最も近い現在フレームのオブジェクトを探す =====
      for (size_t i = 0; i < detections.size(); ++i) {
        if (current_matched[i])
          continue; // マッチしたオブジェクトはスキップ

        // 現在フレームのオブジェクト中心座標を計算
        cv::Point2f curr_center(
            detections[i].bbox.x + detections[i].bbox.width / 2,
            detections[i].bbox.y + detections[i].bbox.height / 2);

        // ユークリッド距離を計算
        float distance = cv::norm(curr_center - prev_center);

        if (distance < min_distance) {
          min_distance = distance;
          best_match_idx = static_cast<int>(i);
        }
      }

      // ===== マッチした場合、IDを引き継ぐ =====
      if (best_match_idx >= 0) {
        detections[best_match_idx].object_id = prev_det.object_id;
        current_matched[best_match_idx] = true;
        assigned_ids.insert(prev_det.object_id);
      }
    }
  }

  // ===== 未割り当てのオブジェクトに新しいIDを割り当てる =====
  for (auto &det : detections) {
    if (det.object_id < 0) {
      det.object_id = next_object_id_++;
    }
  }

  // ===== 現在の検出結果を更新 =====
  current_detections_ = detections;
}

/**
 * @brief 現在の検出結果を取得
 * @return const std::vector<DetectionData>& 現在の検出結果の配列
 * @details 最新の検出結果とオブジェクトIDを取得
 */
const std::vector<DetectionData> &
ObjectTracker::getCurrentDetections() const {
  return current_detections_;
}

/**
 * @brief クラスIDごとの検出数を取得
 * @param class_id クラスID
 * @return int 指定したクラスIDの検出数
 * @details 現在の検出結果から特定クラスのオブジェクト数をカウント
 */
int ObjectTracker::getDetectionCountByClass(int class_id) const {
  return std::count_if(current_detections_.begin(), current_detections_.end(),
                       [class_id](const DetectionData &det) {
                         return det.class_id == class_id;
                       });
}

/**
 * @brief 全検出数を取得
 * @return int 全検出数
 * @details 現在の検出結果の総数を取得
 */
int ObjectTracker::getTotalDetectionCount() const {
  return current_detections_.size();
}

/**
 * @brief トラッカーをリセット
 * @details オブジェクトIDカウンタと検出結果を初期化
 * 
 * リセット内容：
 * - 次に割り当てるオブジェクトIDを0にリセット
 * - 現在の検出結果をクリア
 * - 選択中のオブジェクトIDを-1にリセット
 */
void ObjectTracker::reset() {
  next_object_id_ = 0;
  current_detections_.clear();
  selected_object_id_ = -1;
}

} // namespace fv_object_detector 