#pragma once

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <limits>

namespace fv_aspara_analyzer {

/**
 * @brief v2用 アスパラ情報（最小構成）
 * - ID, BBox, 長さ[m]（未算出はNaN）
 * - 選択フラグはUI側（ノード）で制御するが、保持も可能
 * - 判定関数はしきい値を引数で受け取り、UIロジックから呼び出す
 */
struct AsparaInfo {
    int id { -1 };
    cv::Rect bbox { 0, 0, 0, 0 };
    double confidence_0_1 { std::numeric_limits<double>::quiet_NaN() };
    double length_m { std::numeric_limits<double>::quiet_NaN() };
    bool is_selected { false };
    rclcpp::Time last_update;  // 最終更新（検出反映時刻）

    inline bool hasLength() const {
        return std::isfinite(length_m) && length_m > 0.0;
    }
    inline bool IsValidAsAsparagus(double min_impossible_m, double max_impossible_m) const {
        if (!hasLength()) return true; // 未算出はとりあえず許容（UIで従来色表示）
        return (length_m >= min_impossible_m && length_m <= max_impossible_m);
    }
    inline bool IsHarvestable(double harvest_min_m, double harvest_max_m) const {
        if (!hasLength()) return false;
        return (length_m >= harvest_min_m && length_m <= harvest_max_m);
    }
};

} // namespace fv_aspara_analyzer


