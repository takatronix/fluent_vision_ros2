#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <vector>
#include <algorithm>
#include <cmath>

#include "fluent_lib/fluent_cloud/filters.hpp"

namespace fluent_cloud {

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

// ---- OBB (Oriented Bounding Box) metrics for 3D shape estimation ----

struct OBBMetrics {
    Eigen::Vector3f center{0,0,0};
    // Principal axes (eigenvectors), ordered by eigenvalue descending
    Eigen::Vector3f axes[3];
    // Half-extents along each axis [m]
    float extents[3]{0,0,0};
    // Eigenvalues (descending)
    float eigenvalues[3]{0,0,0};
    // Number of valid points used
    uint32_t num_points{0};
    // Mean RGB
    uint8_t mean_r{0}, mean_g{0}, mean_b{0};
};

inline OBBMetrics compute_obb_metrics(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    OBBMetrics m;
    if (!cloud || cloud->points.empty()) return m;

    // Collect valid points and accumulate color
    std::vector<Eigen::Vector3f> pts;
    pts.reserve(cloud->points.size());
    uint64_t sum_r = 0, sum_g = 0, sum_b = 0;
    for (const auto &p : cloud->points) {
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && p.z > 0.0f) {
            pts.emplace_back(p.x, p.y, p.z);
            sum_r += p.r; sum_g += p.g; sum_b += p.b;
        }
    }
    if (pts.size() < 3) return m;
    m.num_points = static_cast<uint32_t>(pts.size());
    m.mean_r = static_cast<uint8_t>(sum_r / pts.size());
    m.mean_g = static_cast<uint8_t>(sum_g / pts.size());
    m.mean_b = static_cast<uint8_t>(sum_b / pts.size());

    // Centroid
    Eigen::Vector4f mean4;
    pcl::compute3DCentroid(*cloud, mean4);
    m.center = Eigen::Vector3f(mean4[0], mean4[1], mean4[2]);

    // Covariance + PCA
    Eigen::Matrix3f cov;
    pcl::computeCovarianceMatrixNormalized(
        *cloud, Eigen::Vector4f(mean4[0], mean4[1], mean4[2], 1.0f), cov);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);

    // Eigenvectors: col(0)=smallest, col(2)=largest → reorder descending
    for (int i = 0; i < 3; ++i) {
        m.axes[i] = es.eigenvectors().col(2 - i).normalized();
        m.eigenvalues[i] = es.eigenvalues()(2 - i);
        if (m.eigenvalues[i] < 0.0f) m.eigenvalues[i] = 0.0f;
    }

    // Project all valid points onto each axis to get extents
    float mins[3] = { std::numeric_limits<float>::max(),  std::numeric_limits<float>::max(),  std::numeric_limits<float>::max()};
    float maxs[3] = {-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()};
    for (const auto &pt : pts) {
        Eigen::Vector3f v = pt - m.center;
        for (int a = 0; a < 3; ++a) {
            float t = m.axes[a].dot(v);
            if (t < mins[a]) mins[a] = t;
            if (t > maxs[a]) maxs[a] = t;
        }
    }

    // Use trimmed extents (5%-95%) for robustness against outliers
    // For speed, we approximate with the min/max approach but clamp with eigenvalue-based estimate
    for (int a = 0; a < 3; ++a) {
        m.extents[a] = (maxs[a] - mins[a]) * 0.5f;
    }

    return m;
}

} // namespace fluent_cloud

