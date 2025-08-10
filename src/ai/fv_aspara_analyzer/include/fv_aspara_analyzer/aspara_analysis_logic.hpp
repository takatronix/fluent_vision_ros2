// Copyright (c) 2025 Fluent Vision
#ifndef FV_ASPARA_ANALYZER__ASPARA_ANALYSIS_LOGIC_HPP_
#define FV_ASPARA_ANALYZER__ASPARA_ANALYSIS_LOGIC_HPP_

#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fv_aspara_analyzer {

struct SkeletonWorldResult {
  std::vector<Eigen::Vector3f> points;  // 根本→先端の順
  Eigen::Vector3f root_world;           // 根本推定
};

// 曲がり対応のスライス重心法で骨格を抽出
// 入力: 前景点群（対象アスパラのみを含む想定）
// 出力: 根本から先端へ並んだ世界座標の点列
SkeletonWorldResult computeSkeletonBySliceCentroid(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
    int num_points);

} // namespace fv_aspara_analyzer

#endif  // FV_ASPARA_ANALYZER__ASPARA_ANALYSIS_LOGIC_HPP_


