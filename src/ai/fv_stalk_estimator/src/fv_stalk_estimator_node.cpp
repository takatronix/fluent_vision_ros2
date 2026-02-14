// fv_stalk_estimator_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <unordered_map>
#include <mutex>

#include <visualization_msgs/msg/marker_array.hpp>

#include "fv_msgs/msg/detection_array.hpp"
#include "fv_msgs/msg/detection_cloud_indices.hpp"
#include "fv_msgs/msg/stalk_metrics.hpp"
#include "fv_msgs/msg/stalk_metrics_array.hpp"

#include "fluent_lib/fluent_cloud/pipeline.hpp" // compute_pca_metrics

namespace fv_stalk_estimator {

using Cloud = pcl::PointCloud<pcl::PointXYZRGB>;
using CloudPtr = Cloud::Ptr;

struct Intrinsics {
  double fx{0}, fy{0}, cx{0}, cy{0};
  std::string frame_id;
  bool valid() const { return fx > 0.0 && fy > 0.0; }
};

class StalkEstimatorNode : public rclcpp::Node {
public:
  explicit StalkEstimatorNode(const rclcpp::NodeOptions &options)
  : rclcpp::Node("fv_stalk_estimator_node", options)
  {
    this->declare_parameter<std::string>("cloud_topic", "~/cloud");
    this->declare_parameter<std::string>("counts_topic", "~/counts");
    this->declare_parameter<std::string>("detections_topic", "~/detections");
    this->declare_parameter<std::string>("camera_info_topic", "~/camera_info");
    this->declare_parameter<std::string>("output_topic", "~/metrics");
    this->declare_parameter<bool>("publish_markers", true);
    this->declare_parameter<std::string>("marker_topic", "~/markers");
    this->declare_parameter<double>("marker_scale_m", 0.015);
    this->declare_parameter<double>("pca_trim_low", 0.05);
    this->declare_parameter<double>("pca_trim_high", 0.95);
    this->declare_parameter<bool>("pca_filter_root.enable", false);
    this->declare_parameter<double>("pca_filter_root.length_m", 0.02);
    this->declare_parameter<bool>("invert_vertical", false);
    // 根本選択ロジック: 2D投影でより下側(vが大)の端点を根本とする（安定性重視）
    this->declare_parameter<bool>("root_select.nearest_bottom", false);
    this->declare_parameter<double>("root_select.z_margin_m", 0.01);
    // 時間方向スムージング（微振れ抑制）
    this->declare_parameter<bool>("smooth.enable", true);
    this->declare_parameter<double>("smooth.alpha", 0.6);        // prevの寄与（0.0-1.0）
    this->declare_parameter<double>("smooth.max_jump_m", 0.08);  // 大ジャンプ時はリセット
    // 地面RANSAC（任意）: 前処理デバッグトピック（ground_remove前）から平面推定
    this->declare_parameter<bool>("ground.enable", true);
    this->declare_parameter<std::string>("ground.cloud_topic", "");
    this->declare_parameter<double>("ground.distance_threshold", 0.01);
    this->declare_parameter<int>("ground.max_iterations", 200);
    this->declare_parameter<double>("ground.max_tilt_deg", 35.0); // 上向き法線との許容傾き
    // 骨格点列生成
    this->declare_parameter<bool>("skeleton.enable", true);
    this->declare_parameter<int>("skeleton.num_points", 10);  // 骨格点の数

    readParameters();

    // ポイントクラウドは巨大データなのでBEST_EFFORTにしてフレームドロップ許容（queue=1で最新のみ処理）
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_, rclcpp::QoS(1).best_effort(),
      std::bind(&StalkEstimatorNode::onCloud, this, std::placeholders::_1));
    counts_sub_ = this->create_subscription<fv_msgs::msg::DetectionCloudIndices>(counts_topic_, rclcpp::QoS(1).best_effort(),
      std::bind(&StalkEstimatorNode::onCounts, this, std::placeholders::_1));
    det_sub_ = this->create_subscription<fv_msgs::msg::DetectionArray>(detections_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&StalkEstimatorNode::onDetections, this, std::placeholders::_1));
    {
      rclcpp::QoS qos_info(5);
      qos_info.best_effort();
      info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic_, qos_info,
        std::bind(&StalkEstimatorNode::onCameraInfo, this, std::placeholders::_1));
    }

    pub_ = this->create_publisher<fv_msgs::msg::StalkMetricsArray>(output_topic_, rclcpp::QoS(10));
    if (publish_markers_) {
      marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, rclcpp::QoS(10));
    }
    if (ground_enable_ && !ground_cloud_topic_.empty()) {
      // ground_remove 前のデバッグ点群（例: /fv/d405/pointcloud/debug/step2_sor）
      rclcpp::QoS qos(3); qos.best_effort();
      ground_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        ground_cloud_topic_, qos,
        std::bind(&StalkEstimatorNode::onGroundCloud, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Ground RANSAC subscribed: %s", ground_cloud_topic_.c_str());
    }

    RCLCPP_INFO(get_logger(), "fv_stalk_estimator_node ready: cloud=%s counts=%s det=%s out=%s",
                cloud_topic_.c_str(), counts_topic_.c_str(), detections_topic_.c_str(), output_topic_.c_str());
  }

private:
  void readParameters() {
    cloud_topic_ = this->get_parameter("cloud_topic").as_string();
    counts_topic_ = this->get_parameter("counts_topic").as_string();
    detections_topic_ = this->get_parameter("detections_topic").as_string();
    camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    publish_markers_ = this->get_parameter("publish_markers").as_bool();
    marker_topic_ = this->get_parameter("marker_topic").as_string();
    marker_scale_m_ = this->get_parameter("marker_scale_m").as_double();
    pca_trim_low_ = this->get_parameter("pca_trim_low").as_double();
    pca_trim_high_ = this->get_parameter("pca_trim_high").as_double();
    pca_filter_root_enable_ = this->get_parameter("pca_filter_root.enable").as_bool();
    pca_filter_root_length_m_ = this->get_parameter("pca_filter_root.length_m").as_double();
    invert_vertical_ = this->get_parameter("invert_vertical").as_bool();
    root_nearest_bottom_ = this->get_parameter("root_select.nearest_bottom").as_bool();
    root_z_margin_m_ = this->get_parameter("root_select.z_margin_m").as_double();
    smooth_enable_ = this->get_parameter("smooth.enable").as_bool();
    smooth_alpha_ = this->get_parameter("smooth.alpha").as_double();
    smooth_max_jump_m_ = this->get_parameter("smooth.max_jump_m").as_double();
    ground_enable_ = this->get_parameter("ground.enable").as_bool();
    ground_cloud_topic_ = this->get_parameter("ground.cloud_topic").as_string();
    ground_dist_thresh_ = this->get_parameter("ground.distance_threshold").as_double();
    ground_max_iter_ = this->get_parameter("ground.max_iterations").as_int();
    ground_max_tilt_deg_ = this->get_parameter("ground.max_tilt_deg").as_double();
    skeleton_enable_ = this->get_parameter("skeleton.enable").as_bool();
    skeleton_num_points_ = std::max(2, static_cast<int>(this->get_parameter("skeleton.num_points").as_int()));
    if (pca_trim_low_ < 0.0) pca_trim_low_ = 0.0;
    if (pca_trim_high_ > 1.0) pca_trim_high_ = 1.0;
    if (pca_trim_high_ <= pca_trim_low_) pca_trim_high_ = std::min(0.95, pca_trim_low_ + 0.8);
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    intr_.fx = msg->k[0]; intr_.fy = msg->k[4]; intr_.cx = msg->k[2]; intr_.cy = msg->k[5];
    intr_.frame_id = msg->header.frame_id;
  }

  void onDetections(const fv_msgs::msg::DetectionArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    last_det_stamp_ = msg->header.stamp;
    det_map_.clear();
    for (const auto &d : msg->detections) {
      det_map_[d.id] = d;
    }
  }

  void onGroundCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!ground_enable_) return;
    CloudPtr cloud(new Cloud);
    pcl::fromROSMsg(*msg, *cloud);
    if (!cloud || cloud->points.size() < 100) return;
    pcl::SACSegmentation<pcl::PointXYZRGB> seg; seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE); seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(std::max(50, ground_max_iter_));
    seg.setDistanceThreshold(static_cast<float>(std::max(0.002, ground_dist_thresh_)));
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coeff);
    if (!coeff || coeff->values.size() < 4 || inliers->indices.size() < 50) return;
    Eigen::Vector3f n(coeff->values[0], coeff->values[1], coeff->values[2]);
    float d = coeff->values[3];
    if (n.norm() < 1e-6f) return;
    n.normalize();
    // 法線を上向き(+Z)に揃える
    Eigen::Vector3f up(0,0,1);
    if (n.dot(up) < 0.0f) { n = -n; d = -d; }
    double tilt_deg = std::acos(std::max(-1.0f, std::min(1.0f, n.dot(up)))) * 180.0 / M_PI;
    if (tilt_deg > ground_max_tilt_deg_) {
      return; // 傾き過大
    }
    {
      std::lock_guard<std::mutex> lk(mutex_);
      ground_n_ = n; ground_d_ = d; ground_valid_ = true; ground_frame_ = msg->header.frame_id; ground_stamp_ = msg->header.stamp;
    }
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    last_cloud_msg_ = msg;
  }

  void onCounts(const fv_msgs::msg::DetectionCloudIndices::SharedPtr msg) {
    // 入力が揃っているか確認
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      cloud_msg = last_cloud_msg_;
    }
    if (!cloud_msg) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "Waiting for aggregated cloud");
      return;
    }

    CloudPtr cloud(new Cloud);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if (cloud->empty()) {
      return;
    }

    // オフセットで分割
    size_t offset = 0;
    fv_msgs::msg::StalkMetricsArray out;
    out.header = msg->header;
    // metricsのframe_idは可能ならカメラのframe_idに統一（TF不整合の回避）
    if (!intr_.frame_id.empty()) {
      out.header.frame_id = intr_.frame_id;
    }

    for (size_t i = 0; i < msg->ids.size() && i < msg->counts.size(); ++i) {
      const int32_t id = msg->ids[i];
      const uint32_t count = msg->counts[i];
      if (count == 0) {
        continue;
      }
      uint32_t actual_count = count;
      if (offset + count > cloud->points.size()) {
        // タイミングのズレでcountsとcloudが不一致の場合、利用可能な範囲を使う
        if (offset >= cloud->points.size()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Count offset exceeds cloud size: offset=%zu total=%zu (skipping)", offset, cloud->points.size());
          break;
        }
        actual_count = static_cast<uint32_t>(cloud->points.size() - offset);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Count exceeds cloud size: offset=%zu count=%u total=%zu, using actual=%u", offset, count, cloud->points.size(), actual_count);
      }

      CloudPtr sub(new Cloud);
      sub->points.reserve(actual_count);
      sub->points.insert(sub->points.end(), cloud->points.begin() + static_cast<long>(offset), cloud->points.begin() + static_cast<long>(offset + actual_count));
      sub->width = sub->points.size(); sub->height = 1; sub->is_dense = false;
      offset += actual_count;

      // 根本フィルタ（曲がった根本を除外してPCA計算）
      // 最適化版: サンプリングで高速に軸方向推定
      CloudPtr sub_for_pca = sub;
      if (pca_filter_root_enable_ && pca_filter_root_length_m_ > 0.0 && sub->points.size() > 30) {
        // ステップ1: サンプリングで軸方向を高速推定（全体の1/10をサンプル）
        CloudPtr sampled(new Cloud);
        size_t sample_step = std::max<size_t>(1, sub->points.size() / 30);  // 約30点サンプル
        for (size_t i = 0; i < sub->points.size(); i += sample_step) {
          const auto &p = sub->points[i];
          if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && p.z > 0.0f) {
            sampled->points.push_back(p);
          }
        }
        if (sampled->points.size() >= 5) {
          sampled->width = sampled->points.size(); sampled->height = 1; sampled->is_dense = false;
          auto sampled_metrics = fluent_cloud::compute_pca_metrics(sampled);

          // ステップ2: サンプルPCAの軸で全点を投影
          std::vector<std::pair<float, size_t>> proj_and_idx;
          proj_and_idx.reserve(sub->points.size());
          for (size_t i = 0; i < sub->points.size(); ++i) {
            const auto &p = sub->points[i];
            if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && p.z > 0.0f) {
              Eigen::Vector3f v(p.x, p.y, p.z);
              float t = sampled_metrics.axis.dot(v - sampled_metrics.center);
              proj_and_idx.emplace_back(t, i);
            }
          }

          // ステップ3: 投影値でソートし、端から2cmフィルタ
          if (!proj_and_idx.empty()) {
            std::sort(proj_and_idx.begin(), proj_and_idx.end());
            float t_min = proj_and_idx.front().first;
            float t_max = proj_and_idx.back().first;
            // 画像下側（Y大）が根本なので、projection最大側から除外
            // ただし軸の向きが不定なので、両端から少し除外（安全策）
            float filter_from_min = t_min + static_cast<float>(pca_filter_root_length_m_ * 0.5f);
            float filter_from_max = t_max - static_cast<float>(pca_filter_root_length_m_ * 0.5f);

            sub_for_pca.reset(new Cloud);
            sub_for_pca->points.reserve(sub->points.size());
            for (const auto &[t, idx] : proj_and_idx) {
              if (t >= filter_from_min && t <= filter_from_max) {  // 両端除外
                sub_for_pca->points.push_back(sub->points[idx]);
              }
            }
            sub_for_pca->width = sub_for_pca->points.size();
            sub_for_pca->height = 1;
            sub_for_pca->is_dense = false;
          }
        }
      }

      auto metrics = fluent_cloud::compute_pca_metrics(sub_for_pca);
      // トリム再計算（pca_trim_low_/high_が既定と異なる場合）
      if (pca_trim_low_ != 0.05 || pca_trim_high_ != 0.95) {
        std::vector<float> ts; ts.reserve(sub->points.size());
        auto isFinite = [](const pcl::PointXYZRGB &p){ return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && p.z > 0.0f; };
        for (const auto &p : sub->points) if (isFinite(p)) {
          Eigen::Vector3f v(p.x,p.y,p.z); ts.push_back(metrics.axis.dot(v - metrics.center));
        }
        if (!ts.empty()) {
          size_t n = ts.size();
          size_t il = static_cast<size_t>(std::floor(n * std::max(0.0, std::min(1.0, pca_trim_low_))));
          size_t ih = static_cast<size_t>(std::floor(n * std::max(0.0, std::min(1.0, pca_trim_high_))));
          if (il >= n) il = n-1; if (ih >= n) ih = n-1;
          std::nth_element(ts.begin(), ts.begin()+il, ts.end());
          float tmin = ts[il];
          std::nth_element(ts.begin(), ts.begin()+ih, ts.end());
          float tmax = ts[ih];
          if (tmax < tmin) std::swap(tmax, tmin);
          metrics.tmin = tmin; metrics.tmax = tmax;
          metrics.length_m = static_cast<double>(tmax - tmin);
        }
      }
      // エンドポイント（カメラ座標）
      // まず骨格点を計算してから、その両端を使って根本/先端を決定
      Eigen::Vector3f pmin = metrics.center + metrics.axis * metrics.tmin;
      Eigen::Vector3f pmax = metrics.center + metrics.axis * metrics.tmax;

      // 仮の骨格点を計算（root/tip決定のため）
      std::vector<geometry_msgs::msg::Point> temp_skeleton;
      if (skeleton_enable_) {
        temp_skeleton = computeSkeletonPoints(
            sub, metrics.center, metrics.axis, metrics.tmin, metrics.tmax,
            pmin, pmax, skeleton_num_points_);
      }

      Eigen::Vector3f p_root, p_tip;
      bool used_skeleton = false;

      // 骨格点を使った根本/先端判定（最優先）
      if (!temp_skeleton.empty() && temp_skeleton.size() >= 2) {
        // 骨格点の最初と最後
        const auto &sk_first = temp_skeleton.front();
        const auto &sk_last = temp_skeleton.back();

        Eigen::Vector3f p_first(sk_first.x, sk_first.y, sk_first.z);
        Eigen::Vector3f p_last(sk_last.x, sk_last.y, sk_last.z);

        // 画像Y座標で判定: Y大=下側=根本、Y小=上側=先端
        double v_first = projectV(p_first);
        double v_last = projectV(p_last);

        if (v_first > v_last) {
          // 最初が下側（根本）、最後が上側（先端）
          p_root = p_first;
          p_tip = p_last;
        } else {
          // 最初が上側（先端）、最後が下側（根本）
          p_root = p_last;
          p_tip = p_first;
        }
        used_skeleton = true;
      }

      // 骨格点判定が使えない場合のフォールバック
      if (!used_skeleton) {
        // 地面RANSACが有効なら、平面からの高さで根本を決定（低い方=根本）
        bool used_plane = false;
        Eigen::Vector3f n_local; float d_local;
        {
          std::lock_guard<std::mutex> lk(mutex_);
          if (ground_valid_) { n_local = ground_n_; d_local = ground_d_; used_plane = true; }
        }
        if (used_plane) {
          auto signed_height = [&](const Eigen::Vector3f &p){ return n_local.dot(p) + d_local; };
          double ha = signed_height(pmin);
          double hb = signed_height(pmax);
          // より低い(小さい)方を根本に
          if (hb <= ha) { p_root = pmax; p_tip = pmin; } else { p_root = pmin; p_tip = pmax; }
          // 平面に近すぎて差が小さい場合は2D投影にフォールバック（1cm未満）
          if (std::abs(hb - ha) < 0.01) used_plane = false;
        }
        if (!used_plane) {
          // 2D投影の下側(=vが大)を根本とみなす
          double vmin = projectV(pmin);
          double vmax = projectV(pmax);
          bool use_y_fallback = !intr_.valid();
          double a = use_y_fallback ? static_cast<double>(pmin.y()) : vmin;
          double b = use_y_fallback ? static_cast<double>(pmax.y()) : vmax;
          if (b >= a) { p_root = pmax; p_tip = pmin; } else { p_root = pmin; p_tip = pmax; }
        }
      }
      if (invert_vertical_) std::swap(p_root, p_tip);

      // 時系列スムージング（IDごと）
      if (smooth_enable_) {
        auto it = last_rt_.find(id);
        if (it == last_rt_.end()) {
          last_rt_[id] = {p_root, p_tip};
        } else {
          auto prev = it->second;
          auto dist = [](const Eigen::Vector3f &a, const Eigen::Vector3f &b){ return (a-b).norm(); };
          bool reset = (dist(prev.first, p_root) > static_cast<float>(smooth_max_jump_m_)) ||
                       (dist(prev.second, p_tip) > static_cast<float>(smooth_max_jump_m_));
          if (reset) {
            it->second = {p_root, p_tip};
          } else {
            float ap = static_cast<float>(std::clamp(smooth_alpha_, 0.0, 1.0));
            Eigen::Vector3f nr = ap * prev.first  + (1.0f - ap) * p_root;
            Eigen::Vector3f nt = ap * prev.second + (1.0f - ap) * p_tip;
            it->second = {nr, nt};
            p_root = nr; p_tip = nt;
          }
        }
      }

      fv_msgs::msg::StalkMetrics m;
      m.header = out.header;
      m.id = id;
      m.root_camera = toPointMsg(p_root);
      m.tip_camera = toPointMsg(p_tip);
      m.axis_camera = toVecMsg(metrics.axis);
      m.length_m = static_cast<float>(metrics.length_m);
      m.curvature = static_cast<float>(metrics.curvature_ratio);
      m.thickness_m = static_cast<float>(metrics.diameter_m);
      m.root_radius_m = 0.0f;
      m.grade = m.GRADE_UNKNOWN;
      m.root_world = m.root_camera; // TODO: TFがあれば変換
      m.tip_world = m.tip_camera;
      m.updated_at = msg->header.stamp;
      // 検出信頼度
      {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = det_map_.find(id);
        if (it != det_map_.end()) {
          m.confidence = std::max(0.f, std::min(1.f, it->second.conf_fused));
        } else {
          m.confidence = 0.0f;
        }
      }
      // 骨格点列を設定（既に計算済み）
      if (skeleton_enable_) {
        // root/tip判定で使用したtemp_skeletonを再利用
        // ただしroot/tipが確定したので、正しい向きで再計算
        m.skeleton_points = computeSkeletonPoints(
            sub, metrics.center, metrics.axis,
            metrics.tmin, metrics.tmax,
            p_root, p_tip, skeleton_num_points_);
      }
      out.stalks.push_back(std::move(m));
    }

    if (!out.stalks.empty()) {
      pub_->publish(out);
      if (publish_markers_ && marker_pub_) {
        publishMarkers(out);
      }
    }
  }

  geometry_msgs::msg::Point toPointMsg(const Eigen::Vector3f &p) const {
    geometry_msgs::msg::Point q; q.x = p.x(); q.y = p.y(); q.z = p.z(); return q;
  }
  geometry_msgs::msg::Vector3 toVecMsg(const Eigen::Vector3f &v) const {
    geometry_msgs::msg::Vector3 q; q.x = v.x(); q.y = v.y(); q.z = v.z(); return q;
  }

  double projectV(const Eigen::Vector3f &p) const {
    // v = cy + fy * y / z （z>0前提）
    double z = static_cast<double>(p.z());
    if (z <= 0.0 || !intr_.valid()) return 0.0;
    return intr_.cy + intr_.fy * static_cast<double>(p.y()) / z;
  }

  // 骨格点列を計算: 点群をPCA軸に沿った区間に分割し、各区間の重心を骨格点とする
  std::vector<geometry_msgs::msg::Point> computeSkeletonPoints(
      const CloudPtr &cloud,
      const Eigen::Vector3f &center,
      const Eigen::Vector3f &axis,
      float tmin, float tmax,
      const Eigen::Vector3f &p_root,
      const Eigen::Vector3f &p_tip,
      int num_points) {
    std::vector<geometry_msgs::msg::Point> skeleton;
    if (!cloud || cloud->empty() || num_points < 2) return skeleton;

    // root→tipの方向を決定（軸の向きを合わせる）
    Eigen::Vector3f dir = (p_tip - p_root).normalized();
    bool flip = (axis.dot(dir) < 0);
    Eigen::Vector3f oriented_axis = flip ? -axis : axis;
    float oriented_tmin = flip ? -tmax : tmin;
    float oriented_tmax = flip ? -tmin : tmax;

    // 点群を軸に沿ったt値でソートしてビン分け
    struct PointWithT {
      Eigen::Vector3f pos;
      float t;
    };
    std::vector<PointWithT> pts;
    pts.reserve(cloud->points.size());
    for (const auto &p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.z <= 0.0f) continue;
      Eigen::Vector3f v(p.x, p.y, p.z);
      float t = oriented_axis.dot(v - center);
      if (t >= oriented_tmin && t <= oriented_tmax) {
        pts.push_back({v, t});
      }
    }
    if (pts.empty()) return skeleton;

    // num_points個の区間に分割し、各区間の重心を計算
    float range = oriented_tmax - oriented_tmin;
    if (range < 1e-6f) return skeleton;

    skeleton.reserve(static_cast<size_t>(num_points));
    for (int i = 0; i < num_points; ++i) {
      float bin_start = oriented_tmin + range * static_cast<float>(i) / static_cast<float>(num_points);
      float bin_end = oriented_tmin + range * static_cast<float>(i + 1) / static_cast<float>(num_points);

      Eigen::Vector3f sum(0, 0, 0);
      int count = 0;
      for (const auto &pt : pts) {
        if (pt.t >= bin_start && pt.t < bin_end) {
          sum += pt.pos;
          count++;
        }
      }
      // 最後のビンは境界を含める
      if (i == num_points - 1) {
        for (const auto &pt : pts) {
          if (pt.t >= bin_end - 1e-6f && pt.t <= oriented_tmax + 1e-6f) {
            sum += pt.pos;
            count++;
          }
        }
      }

      geometry_msgs::msg::Point sp;
      if (count > 0) {
        Eigen::Vector3f centroid = sum / static_cast<float>(count);
        sp.x = centroid.x();
        sp.y = centroid.y();
        sp.z = centroid.z();
      } else {
        // 点がない区間は線形補間で埋める
        float t_mid = (bin_start + bin_end) / 2.0f;
        float alpha = (t_mid - oriented_tmin) / range;
        Eigen::Vector3f interp = p_root + alpha * (p_tip - p_root);
        sp.x = interp.x();
        sp.y = interp.y();
        sp.z = interp.z();
      }
      skeleton.push_back(sp);
    }

    return skeleton;
  }

  void publishMarkers(const fv_msgs::msg::StalkMetricsArray &arr) {
    visualization_msgs::msg::MarkerArray ma;
    // Clear previous markers
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(clear);

    std::string frame = intr_.frame_id.empty() ? arr.header.frame_id : intr_.frame_id;
    rclcpp::Time now = this->now();

    for (const auto &m : arr.stalks) {
      if (m.root_camera.z <= 0.0f) continue;

      // 1. Root marker (red sphere)
      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = frame;
      mk.header.stamp = now;
      mk.ns = "stalk_root";
      mk.id = static_cast<int>(m.id);
      mk.type = visualization_msgs::msg::Marker::SPHERE;
      mk.action = visualization_msgs::msg::Marker::ADD;
      mk.pose.position.x = m.root_camera.x;
      mk.pose.position.y = m.root_camera.y;
      mk.pose.position.z = m.root_camera.z;
      mk.pose.orientation.w = 1.0;
      mk.scale.x = marker_scale_m_;
      mk.scale.y = marker_scale_m_;
      mk.scale.z = marker_scale_m_;
      mk.color.r = 1.0f; mk.color.g = 0.0f; mk.color.b = 0.0f; mk.color.a = 1.0f;
      mk.lifetime = rclcpp::Duration(0, 0);
      ma.markers.push_back(std::move(mk));

      // 1b. Tip marker (blue sphere)
      visualization_msgs::msg::Marker tip_mk;
      tip_mk.header.frame_id = frame;
      tip_mk.header.stamp = now;
      tip_mk.ns = "stalk_tip";
      tip_mk.id = static_cast<int>(m.id);
      tip_mk.type = visualization_msgs::msg::Marker::SPHERE;
      tip_mk.action = visualization_msgs::msg::Marker::ADD;
      tip_mk.pose.position.x = m.tip_camera.x;
      tip_mk.pose.position.y = m.tip_camera.y;
      tip_mk.pose.position.z = m.tip_camera.z;
      tip_mk.pose.orientation.w = 1.0;
      tip_mk.scale.x = marker_scale_m_;
      tip_mk.scale.y = marker_scale_m_;
      tip_mk.scale.z = marker_scale_m_;
      tip_mk.color.r = 0.0f; tip_mk.color.g = 0.5f; tip_mk.color.b = 1.0f; tip_mk.color.a = 1.0f;
      tip_mk.lifetime = rclcpp::Duration(0, 0);
      ma.markers.push_back(std::move(tip_mk));

      // 2. Skeleton segments (white cylinders between points)
      if (m.skeleton_points.size() >= 2) {
        for (size_t i = 0; i < m.skeleton_points.size() - 1; ++i) {
          const auto &p1 = m.skeleton_points[i];
          const auto &p2 = m.skeleton_points[i + 1];

          visualization_msgs::msg::Marker cyl;
          cyl.header.frame_id = frame;
          cyl.header.stamp = now;
          cyl.ns = "stalk_skeleton";
          cyl.id = static_cast<int>(m.id) * 1000 + static_cast<int>(i);
          cyl.type = visualization_msgs::msg::Marker::CYLINDER;
          cyl.action = visualization_msgs::msg::Marker::ADD;

          // Position at midpoint
          cyl.pose.position.x = (p1.x + p2.x) / 2.0;
          cyl.pose.position.y = (p1.y + p2.y) / 2.0;
          cyl.pose.position.z = (p1.z + p2.z) / 2.0;

          // Orientation to align cylinder with segment
          double dx = p2.x - p1.x;
          double dy = p2.y - p1.y;
          double dz = p2.z - p1.z;
          double len = std::sqrt(dx*dx + dy*dy + dz*dz);

          // Default cylinder is along Z axis, need to rotate to point direction
          Eigen::Vector3d dir(dx/len, dy/len, dz/len);
          Eigen::Vector3d z_axis(0, 0, 1);
          Eigen::Vector3d rot_axis = z_axis.cross(dir);
          double rot_angle = std::acos(z_axis.dot(dir));

          if (rot_axis.norm() > 1e-6) {
            rot_axis.normalize();
            Eigen::AngleAxisd aa(rot_angle, rot_axis);
            Eigen::Quaterniond q(aa);
            cyl.pose.orientation.x = q.x();
            cyl.pose.orientation.y = q.y();
            cyl.pose.orientation.z = q.z();
            cyl.pose.orientation.w = q.w();
          } else {
            cyl.pose.orientation.w = 1.0;
          }

          cyl.scale.x = marker_scale_m_;  // diameter
          cyl.scale.y = marker_scale_m_;  // diameter
          cyl.scale.z = len;  // height (length of segment)
          cyl.color.r = 1.0f; cyl.color.g = 1.0f; cyl.color.b = 1.0f; cyl.color.a = 0.8f;
          cyl.lifetime = rclcpp::Duration(0, 0);
          ma.markers.push_back(std::move(cyl));
        }

        // 3. Boundary markers (white discs at skeleton points)
        for (size_t i = 0; i < m.skeleton_points.size(); ++i) {
          const auto &p = m.skeleton_points[i];

          visualization_msgs::msg::Marker disc;
          disc.header.frame_id = frame;
          disc.header.stamp = now;
          disc.ns = "stalk_boundary";
          disc.id = static_cast<int>(m.id) * 10000 + static_cast<int>(i);
          disc.type = visualization_msgs::msg::Marker::CYLINDER;
          disc.action = visualization_msgs::msg::Marker::ADD;

          disc.pose.position.x = p.x;
          disc.pose.position.y = p.y;
          disc.pose.position.z = p.z;

          // Orient disc perpendicular to skeleton direction
          if (i == 0 && m.skeleton_points.size() > 1) {
            // First point: use direction to next point
            const auto &p_next = m.skeleton_points[i + 1];
            double dx = p_next.x - p.x;
            double dy = p_next.y - p.y;
            double dz = p_next.z - p.z;
            double len = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (len > 1e-6) {
              Eigen::Vector3d dir(dx/len, dy/len, dz/len);
              Eigen::Vector3d z_axis(0, 0, 1);
              Eigen::Vector3d rot_axis = z_axis.cross(dir);
              double rot_angle = std::acos(std::max(-1.0, std::min(1.0, z_axis.dot(dir))));

              if (rot_axis.norm() > 1e-6) {
                rot_axis.normalize();
                Eigen::AngleAxisd aa(rot_angle, rot_axis);
                Eigen::Quaterniond q(aa);
                disc.pose.orientation.x = q.x();
                disc.pose.orientation.y = q.y();
                disc.pose.orientation.z = q.z();
                disc.pose.orientation.w = q.w();
              } else {
                disc.pose.orientation.w = 1.0;
              }
            } else {
              disc.pose.orientation.w = 1.0;
            }
          } else if (i == m.skeleton_points.size() - 1 && m.skeleton_points.size() > 1) {
            // Last point: use direction from previous point
            const auto &p_prev = m.skeleton_points[i - 1];
            double dx = p.x - p_prev.x;
            double dy = p.y - p_prev.y;
            double dz = p.z - p_prev.z;
            double len = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (len > 1e-6) {
              Eigen::Vector3d dir(dx/len, dy/len, dz/len);
              Eigen::Vector3d z_axis(0, 0, 1);
              Eigen::Vector3d rot_axis = z_axis.cross(dir);
              double rot_angle = std::acos(std::max(-1.0, std::min(1.0, z_axis.dot(dir))));

              if (rot_axis.norm() > 1e-6) {
                rot_axis.normalize();
                Eigen::AngleAxisd aa(rot_angle, rot_axis);
                Eigen::Quaterniond q(aa);
                disc.pose.orientation.x = q.x();
                disc.pose.orientation.y = q.y();
                disc.pose.orientation.z = q.z();
                disc.pose.orientation.w = q.w();
              } else {
                disc.pose.orientation.w = 1.0;
              }
            } else {
              disc.pose.orientation.w = 1.0;
            }
          } else if (m.skeleton_points.size() > 2) {
            // Middle points: average direction
            const auto &p_prev = m.skeleton_points[i - 1];
            const auto &p_next = m.skeleton_points[i + 1];
            double dx = p_next.x - p_prev.x;
            double dy = p_next.y - p_prev.y;
            double dz = p_next.z - p_prev.z;
            double len = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (len > 1e-6) {
              Eigen::Vector3d dir(dx/len, dy/len, dz/len);
              Eigen::Vector3d z_axis(0, 0, 1);
              Eigen::Vector3d rot_axis = z_axis.cross(dir);
              double rot_angle = std::acos(std::max(-1.0, std::min(1.0, z_axis.dot(dir))));

              if (rot_axis.norm() > 1e-6) {
                rot_axis.normalize();
                Eigen::AngleAxisd aa(rot_angle, rot_axis);
                Eigen::Quaterniond q(aa);
                disc.pose.orientation.x = q.x();
                disc.pose.orientation.y = q.y();
                disc.pose.orientation.z = q.z();
                disc.pose.orientation.w = q.w();
              } else {
                disc.pose.orientation.w = 1.0;
              }
            } else {
              disc.pose.orientation.w = 1.0;
            }
          } else {
            disc.pose.orientation.w = 1.0;
          }

          // Thin disc (boundary marker)
          disc.scale.x = marker_scale_m_ * 2.0;  // diameter (2x wider than skeleton)
          disc.scale.y = marker_scale_m_ * 2.0;  // diameter
          disc.scale.z = marker_scale_m_ * 0.1;  // very thin (1/10 of thickness)
          disc.color.r = 1.0f; disc.color.g = 1.0f; disc.color.b = 1.0f; disc.color.a = 0.9f;
          disc.lifetime = rclcpp::Duration(0, 0);
          ma.markers.push_back(std::move(disc));
        }
      }
    }
    if (!ma.markers.empty() && marker_pub_) marker_pub_->publish(ma);
  }

  // params
  std::string cloud_topic_;
  std::string counts_topic_;
  std::string detections_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;
  bool publish_markers_{true};
  std::string marker_topic_;
  double marker_scale_m_{0.015};
  double pca_trim_low_{0.05};
  double pca_trim_high_{0.95};
  bool pca_filter_root_enable_{false};
  double pca_filter_root_length_m_{0.02};
  bool invert_vertical_{false};
  bool root_nearest_bottom_{true};
  double root_z_margin_m_{0.01};
  bool smooth_enable_{true};
  double smooth_alpha_{0.6};
  double smooth_max_jump_m_{0.08};
  bool skeleton_enable_{true};
  int skeleton_num_points_{10};

  // subs/pubs
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<fv_msgs::msg::DetectionCloudIndices>::SharedPtr counts_sub_;
  rclcpp::Subscription<fv_msgs::msg::DetectionArray>::SharedPtr det_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<fv_msgs::msg::StalkMetricsArray>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_sub_;

  // state
  std::mutex mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_msg_;
  std::unordered_map<int32_t, fv_msgs::msg::Detection2D> det_map_;
  builtin_interfaces::msg::Time last_det_stamp_;
  Intrinsics intr_;
  std::unordered_map<int32_t, std::pair<Eigen::Vector3f, Eigen::Vector3f>> last_rt_;
  // Ground plane (n.x * X + n.y * Y + n.z * Z + d = 0), n is upward
  Eigen::Vector3f ground_n_{0,0,1};
  float ground_d_{0.0f};
  bool ground_valid_{false};
  std::string ground_frame_;
  builtin_interfaces::msg::Time ground_stamp_;
  bool ground_enable_{true};
  std::string ground_cloud_topic_;
  double ground_dist_thresh_{0.01};
  int ground_max_iter_{200};
  double ground_max_tilt_deg_{35.0};
};

} // namespace fv_stalk_estimator

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fv_stalk_estimator::StalkEstimatorNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
