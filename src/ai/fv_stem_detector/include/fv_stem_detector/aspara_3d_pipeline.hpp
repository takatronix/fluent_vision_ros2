#pragma once

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Core>
#include <memory>
#include <vector>
#include <string>

namespace fv_stem_detector {

struct Intrinsics {
  double fx {0.0}, fy {0.0}, cx {0.0}, cy {0.0};
};

struct RegionInput {
  cv::Rect roi;
};

struct RegionOutput {
  bool ok {false};
  std::string fail_reason; // 失敗理由（ok=false時に設定）
  double approach_z_m {0.0};
  Eigen::Vector3f pca_center;
  Eigen::Vector3f pca_axis;
  float pca_conf {0.0f};
  Eigen::Vector3f end_root;
  Eigen::Vector3f end_tip;
  float length_m {0.0f};
  cv::Point approach_px;
  cv::Point root_px;
  cv::Point tip_px;
  int point_count {0};
  double roi_ms {0.0};
  // optional debug clouds per stage (may be null if not collected)
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_raw_zband;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_post_voxel;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_post_sor;
};

struct PipelineParams {
  // sampling
  int pc_sample_stride_px {2};
  // band
  double depth_band_minus_m {0.05};
  double depth_band_plus_m {0.05};
  // pass-through Y (ground cut)
  double ground_cut_auto_max_p90_offset_m {0.005};
  // filters
  double voxel_leaf_m {0.004};
  int sor_mean_k {50};
  double sor_stddev_mul {1.2};
  // gates
  double pca_min_confidence {0.60};
  double min_length_m {0.10};
  double max_length_m {0.40};
  // approach
  int approach_window_px {5};
  int approach_min_valid {5};
  // approach search (fixed-mode向けにROI内を広く探索するオプション)
  bool wide_approach_search {false};
  int approach_grid_step_px {8};
  double approach_exclude_bottom_ratio {0.0}; // ROI下部の除外比率（0.0=除外なし, 0.25=下25%除外）
  // scale
  double depth_scale_m {0.001};
  // debug collection
  bool collect_debug_clouds {false};
};

class Aspara3DPipeline {
public:
  RegionOutput processRegion(
    const cv::Mat& color_bgr,
    const cv::Mat& depth_16u_or_32f,
    const Intrinsics& K,
    const PipelineParams& params,
    const RegionInput& in) const;
};

} // namespace fv_stem_detector


