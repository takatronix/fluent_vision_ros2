#include "fv_aspara_analyzer/aspara_analysis_logic.hpp"
#include <limits>
#include <algorithm>

namespace fv_aspara_analyzer {

SkeletonWorldResult computeSkeletonBySliceCentroid(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
    int num_points)
{
    SkeletonWorldResult out;
    out.points.clear();
    if (!cloud || cloud->empty() || num_points < 2) {
        return out;
    }
    float y_min = std::numeric_limits<float>::max();
    float y_max = std::numeric_limits<float>::lowest();
    for (const auto &p : cloud->points) {
        if (!std::isfinite(p.y) || !std::isfinite(p.z) || p.z <= 0.0f) continue;
        y_min = std::min(y_min, p.y);
        y_max = std::max(y_max, p.y);
    }
    float y_range = y_max - y_min;
    if (!(y_range > 0.0f)) {
        return out;
    }
    // 根本は y が最大の点付近
    float best_y = std::numeric_limits<float>::lowest();
    pcl::PointXYZRGB best;
    for (const auto &p : cloud->points) {
        if (!std::isfinite(p.y) || !std::isfinite(p.z) || p.z <= 0.0f) continue;
        if (p.y > best_y) { best_y = p.y; best = p; }
    }
    out.root_world = Eigen::Vector3f(best.x, best.y, best.z);

    for (int i = 0; i < num_points; ++i) {
        float ratio = static_cast<float>(i) / static_cast<float>(num_points - 1);
        float y_center = y_max - ratio * y_range; // 根本(y_max)→先端(y_min)
        float slice_h = y_range / static_cast<float>(num_points);
        float y_lower = y_center - slice_h * 0.5f;
        float y_upper = y_center + slice_h * 0.5f;

        // スライス内重心
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        int count = 0;
        for (const auto &p : cloud->points) {
            if (!std::isfinite(p.y) || !std::isfinite(p.z) || p.z <= 0.0f) continue;
            if (p.y >= y_lower && p.y <= y_upper) {
                centroid += Eigen::Vector3f(p.x, p.y, p.z);
                ++count;
            }
        }
        if (count > 0) {
            centroid /= static_cast<float>(count);
            out.points.emplace_back(centroid);
        } else {
            // 点がないスライスはスキップ（後段で補間も可）
        }
    }
    // 欠損スライスがあっても最低2点あればOK
    if (out.points.size() < 2) out.points.clear();
    return out;
}

} // namespace fv_aspara_analyzer


