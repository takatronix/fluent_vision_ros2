#include "fv_stem_detector/aspara_3d_pipeline.hpp"
#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <chrono>
#include <limits>
#include <algorithm>
#include <cmath>

namespace fv_stem_detector {

static cv::Point projectToPixel(const Eigen::Vector3f& P, const Intrinsics& K) {
  double u = (static_cast<double>(P.x()) * K.fx) / static_cast<double>(P.z()) + K.cx;
  double v = (static_cast<double>(P.y()) * K.fy) / static_cast<double>(P.z()) + K.cy;
  return cv::Point(static_cast<int>(std::round(u)), static_cast<int>(std::round(v)));
}

RegionOutput Aspara3DPipeline::processRegion(
  const cv::Mat& color_bgr,
  const cv::Mat& depth,
  const Intrinsics& K,
  const PipelineParams& p,
  const RegionInput& in) const
{
  RegionOutput out;
  auto t0 = std::chrono::high_resolution_clock::now();
  if (in.roi.width <= 0 || in.roi.height <= 0 || color_bgr.empty() || depth.empty()) {
    out.fail_reason = "invalid_input_or_roi";
    auto t1 = std::chrono::high_resolution_clock::now();
    out.roi_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    return out;
  }

  cv::Rect roi = in.roi & cv::Rect(0,0, depth.cols, depth.rows);
  if (roi.width <= 0 || roi.height <= 0) {
    out.fail_reason = "roi_clipped_empty";
    auto t1 = std::chrono::high_resolution_clock::now();
    out.roi_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    return out;
  }

  // timer started at function entry

  // approach depth (median). デフォルトはROI中心だが、wide_approach_search時はグリッド探索
  auto medianDepthAt = [&](int cx, int cy)->double{
    std::vector<float> vals; vals.reserve((p.approach_window_px*2+1)*(p.approach_window_px*2+1));
    for (int dv = -p.approach_window_px/2; dv <= p.approach_window_px/2; ++dv) {
      for (int du = -p.approach_window_px/2; du <= p.approach_window_px/2; ++du) {
        int u = std::clamp(cx + du, 0, depth.cols-1);
        int v = std::clamp(cy + dv, 0, depth.rows-1);
        float z = 0.0f;
        if (depth.type() == CV_16UC1) {
          uint16_t raw = depth.at<uint16_t>(v,u); if (raw==0) continue; z = static_cast<float>(raw) * static_cast<float>(p.depth_scale_m);
        } else if (depth.type() == CV_32FC1) {
          float m = depth.at<float>(v,u); if (!(std::isfinite(m)) || m<=0.0f) continue; z = m;
        }
        if (z>0.0f) vals.push_back(z);
      }
    }
    if (static_cast<int>(vals.size()) < p.approach_min_valid) return std::numeric_limits<double>::quiet_NaN();
    size_t m = vals.size()/2; std::nth_element(vals.begin(), vals.begin()+m, vals.end());
    return static_cast<double>(vals[m]);
  };

  double approach_z = std::numeric_limits<double>::quiet_NaN();
  cv::Point approach_px = cv::Point(roi.x + roi.width/2, roi.y + roi.height/2);
  if (p.wide_approach_search) {
    // ROI内をグリッドでスキャンし、最も手前(最小Z)の中央値を選択
    double best_z = std::numeric_limits<double>::infinity();
    cv::Point best_pt = approach_px;
    int step = std::max(1, p.approach_grid_step_px);
    int y_limit = roi.y + roi.height;
    if (p.approach_exclude_bottom_ratio > 0.0) {
      double ratio = std::clamp(p.approach_exclude_bottom_ratio, 0.0, 0.9);
      y_limit = roi.y + static_cast<int>(std::round(roi.height * (1.0 - ratio)));
      y_limit = std::clamp(y_limit, roi.y, roi.y + roi.height);
    }
    for (int v = roi.y + step/2; v < y_limit; v += step) {
      for (int u = roi.x + step/2; u < roi.x + roi.width; u += step) {
        double z = medianDepthAt(u, v);
        if (std::isfinite(z) && z > 0.0 && z < best_z) {
          best_z = z; best_pt = cv::Point(u, v);
        }
      }
    }
    if (std::isfinite(best_z)) {
      approach_z = best_z; approach_px = best_pt;
    } else {
      // フォールバック: 中心
      approach_z = medianDepthAt(approach_px.x, approach_px.y);
    }
  } else {
    approach_z = medianDepthAt(approach_px.x, approach_px.y);
  }
  if (!std::isfinite(approach_z)) {
    out.fail_reason = "approach_depth_median_invalid";
    auto t1 = std::chrono::high_resolution_clock::now();
    out.roi_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    return out;
  }
  out.approach_z_m = approach_z;

  // sample cloud within band
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  int stride = std::max(1, p.pc_sample_stride_px);
  for (int v = roi.y; v < roi.y + roi.height; v += stride) {
    for (int u = roi.x; u < roi.x + roi.width; u += stride) {
      float Z = 0.0f;
      if (depth.type() == CV_16UC1) {
        uint16_t raw = depth.at<uint16_t>(v,u); if (raw==0) continue; Z = static_cast<float>(raw) * static_cast<float>(p.depth_scale_m);
      } else if (depth.type() == CV_32FC1) {
        float m = depth.at<float>(v,u); if (!(std::isfinite(m)) || m<=0.0f) continue; Z = m;
      }
      if (Z < approach_z - p.depth_band_minus_m || Z > approach_z + p.depth_band_plus_m) continue;
      double X = (static_cast<double>(u) - K.cx) * Z / K.fx;
      double Y = (static_cast<double>(v) - K.cy) * Z / K.fy;
      pcl::PointXYZRGB pt; pt.x = static_cast<float>(X); pt.y = static_cast<float>(Y); pt.z = static_cast<float>(Z);
      const cv::Vec3b& c = color_bgr.at<cv::Vec3b>(std::clamp(v,0,color_bgr.rows-1), std::clamp(u,0,color_bgr.cols-1));
      pt.r = c[2]; pt.g = c[1]; pt.b = c[0];
      cloud->points.push_back(pt);
    }
  }
  cloud->width = cloud->points.size(); cloud->height = 1; cloud->is_dense = false; out.point_count = static_cast<int>(cloud->size());
  if (p.collect_debug_clouds) out.cloud_raw_zband = std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(cloud);
  if (cloud->points.size() < 10) {
    out.fail_reason = "zband_points_too_few";
    auto t1 = std::chrono::high_resolution_clock::now();
    out.roi_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    return out;
  }

  // ground cut (y-pass-through auto)
  {
    std::vector<float> ys; ys.reserve(cloud->points.size());
    for (auto &pt : cloud->points) ys.push_back(pt.y);
    size_t k = static_cast<size_t>(std::floor(0.9 * (ys.size()-1)));
    std::nth_element(ys.begin(), ys.begin()+k, ys.end());
    float y90 = ys[k]; float y_max = y90 + static_cast<float>(p.ground_cut_auto_max_p90_offset_m);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outc(new pcl::PointCloud<pcl::PointXYZRGB>);
    outc->points.reserve(cloud->points.size());
    for (auto &pt : cloud->points) if (pt.y <= y_max) outc->points.push_back(pt);
    outc->width = outc->points.size(); outc->height = 1; outc->is_dense = false; cloud = outc;
  }

  // voxel + SOR
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr v(new pcl::PointCloud<pcl::PointXYZRGB>);
  {
    pcl::VoxelGrid<pcl::PointXYZRGB> vg; vg.setInputCloud(cloud);
    float ls = static_cast<float>(p.voxel_leaf_m);
    vg.setLeafSize(ls, ls, ls); vg.filter(*v);
  }
  if (p.collect_debug_clouds) out.cloud_post_voxel = v;
  if (v->size() >= 10) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr o(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; sor.setInputCloud(v);
    sor.setMeanK(p.sor_mean_k); sor.setStddevMulThresh(static_cast<float>(p.sor_stddev_mul)); sor.filter(*o);
    v = o;
  }
  if (p.collect_debug_clouds) out.cloud_post_sor = v;
  if (v->size() < 10) {
    out.fail_reason = "post_filter_points_too_few";
    auto t1 = std::chrono::high_resolution_clock::now();
    out.roi_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    return out;
  }

  // PCA axis
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  for (auto &pt : v->points) center += Eigen::Vector3f(pt.x, pt.y, pt.z);
  center /= static_cast<float>(v->points.size());
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (auto &pt : v->points) { Eigen::Vector3f d(pt.x, pt.y, pt.z); d -= center; cov += d * d.transpose(); }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov); if (es.info()!=Eigen::Success) {
    out.fail_reason = "pca_failed";
    auto t1 = std::chrono::high_resolution_clock::now();
    out.roi_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    return out;
  }
  Eigen::Vector3f evals = es.eigenvalues(); Eigen::Matrix3f evecs = es.eigenvectors(); int idx_max; evals.maxCoeff(&idx_max);
  float conf = (evals.sum()>0.0f) ? (evals(idx_max)/evals.sum()) : 0.0f; if (conf < static_cast<float>(p.pca_min_confidence)) {
    out.fail_reason = "pca_confidence_low";
    auto t1 = std::chrono::high_resolution_clock::now();
    out.roi_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    return out;
  }
  Eigen::Vector3f axis = evecs.col(idx_max).normalized();
  // PCAの固有ベクトルは符号が不定のため、+Y（画像下方向）を正にそろえる
  if (axis.y() < 0.0f) {
    axis = -axis;
  }
  out.pca_center = center; out.pca_axis = axis; out.pca_conf = conf;

  // project min/max along axis
  float smin = std::numeric_limits<float>::infinity();
  float smax = -std::numeric_limits<float>::infinity();
  for (auto &pt : v->points) {
    Eigen::Vector3f d(pt.x, pt.y, pt.z); d -= center; float s = d.dot(axis);
    if (s < smin) { smin = s; out.end_root = center + s * axis; }
    if (s > smax) { smax = s; out.end_tip  = center + s * axis; }
  }
  float length = (out.end_tip - out.end_root).norm();
  if (length < static_cast<float>(p.min_length_m) || length > static_cast<float>(p.max_length_m)) {
    out.fail_reason = "length_gate_rejected";
    auto t1 = std::chrono::high_resolution_clock::now();
    out.roi_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
    return out;
  }
  out.length_m = length;

  // 根本/穂先の定義を安定化: カメラ光学座標で Y が大きい（下側）が根本
  if (out.end_root.y() < out.end_tip.y()) {
    std::swap(out.end_root, out.end_tip);
  }

  // pixel positions
  out.approach_px = approach_px;
  out.root_px = projectToPixel(out.end_root, K);
  out.tip_px  = projectToPixel(out.end_tip,  K);

  auto t1 = std::chrono::high_resolution_clock::now();
  out.roi_ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;
  out.ok = true; return out;
}

} // namespace fv_stem_detector


