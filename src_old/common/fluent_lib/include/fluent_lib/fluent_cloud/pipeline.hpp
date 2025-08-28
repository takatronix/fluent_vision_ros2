#pragma once

// Minimal sample:
//   auto den = fluent_cloud::filters::VoxelGrid<pcl::PointXYZRGB>().setLeafSize(0.005).filter(cloud);
//   den = fluent_cloud::filters::StatisticalOutlierRemoval<pcl::PointXYZRGB>().setMeanK(50).setStddevMulThresh(1.0).filter(den);
//   auto m = fluent_cloud::compute_pca_metrics(den);

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>
#include <algorithm>
#include <cmath>

#include "fluent_lib/fluent_cloud/filters.hpp"

namespace fluent_cloud {

inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_sor(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
    double voxel_leaf_size,
    int mean_k,
    double stddev_mul)
{
    auto voxel = filters::VoxelGrid<pcl::PointXYZRGB>().setLeafSize(voxel_leaf_size).filter(input);
    auto out = filters::StatisticalOutlierRemoval<pcl::PointXYZRGB>()
        .setMeanK(mean_k)
        .setStddevMulThresh(stddev_mul)
        .filter(voxel);
    return out;
}

struct PCAMetrics {
    double length_m{0.0};
    double diameter_m{0.0};
    double curvature_ratio{0.0};
    Eigen::Vector3f axis{0,0,1};
    Eigen::Vector3f center{0,0,0};
    float tmin{0.0f};
    float tmax{0.0f};
};

inline PCAMetrics compute_pca_metrics(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    PCAMetrics m;
    if (!cloud || cloud->points.empty()) return m;

    auto isFinite = [](const pcl::PointXYZRGB &p){ return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && p.z > 0.0f; };

    Eigen::Vector4f mean4; pcl::compute3DCentroid(*cloud, mean4);
    Eigen::Matrix3f cov; pcl::computeCovarianceMatrixNormalized(*cloud, Eigen::Vector4f(mean4[0],mean4[1],mean4[2],1.0f), cov);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
    m.center = Eigen::Vector3f(mean4[0], mean4[1], mean4[2]);
    m.axis = es.eigenvectors().col(2).normalized();

    std::vector<float> ts; ts.reserve(cloud->points.size());
    for (const auto &p : cloud->points) if (isFinite(p)) {
        Eigen::Vector3f v(p.x,p.y,p.z); ts.push_back(m.axis.dot(v - m.center));
    }
    if (ts.empty()) return m;
    size_t n = ts.size(); size_t i05 = static_cast<size_t>(std::floor(n * 0.05)); size_t i95 = static_cast<size_t>(std::floor(n * 0.95));
    std::nth_element(ts.begin(), ts.begin()+i05, ts.end()); m.tmin = ts[i05];
    std::nth_element(ts.begin(), ts.begin()+i95, ts.end()); m.tmax = ts[i95];
    if (m.tmax < m.tmin) std::swap(m.tmax, m.tmin);

    m.length_m = static_cast<double>(m.tmax - m.tmin);

    // diameter: median of perpendicular distances in 20%-80% band, doubled
    std::vector<float> dists; dists.reserve(cloud->points.size());
    for (const auto &p : cloud->points) if (isFinite(p)) {
        Eigen::Vector3f v(p.x,p.y,p.z);
        float t = m.axis.dot(v - m.center);
        if (m.tmax <= m.tmin) continue;
        float s = (t - m.tmin) / (m.tmax - m.tmin);
        if (s < 0.2f || s > 0.8f) continue;
        Eigen::Vector3f perp = (v - m.center) - m.axis * t;
        dists.push_back(perp.norm());
    }
    if (!dists.empty()) {
        size_t mid = dists.size()/2; std::nth_element(dists.begin(), dists.begin()+mid, dists.end());
        m.diameter_m = 2.0 * static_cast<double>(dists[mid]);
    }

    // curvature ratio: RMS perpendicular / length
    double sum2 = 0.0; size_t cnt = 0;
    for (const auto &p : cloud->points) if (isFinite(p)) {
        Eigen::Vector3f v(p.x,p.y,p.z);
        float t = m.axis.dot(v - m.center);
        Eigen::Vector3f perp = (v - m.center) - m.axis * t;
        double d = static_cast<double>(perp.norm()); sum2 += d*d; ++cnt;
    }
    if (cnt > 0 && m.length_m > 1e-6) {
        double rms = std::sqrt(sum2 / static_cast<double>(cnt));
        m.curvature_ratio = rms / m.length_m;
    }

    return m;
}

} // namespace fluent_cloud


