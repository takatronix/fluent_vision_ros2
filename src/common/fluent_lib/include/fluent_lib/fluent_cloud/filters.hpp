#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace fluent_cloud::filters {

template <typename PointT>
class VoxelGrid {
public:
    VoxelGrid &setLeafSize(double s) { leaf_size_ = s; return *this; }
    typename pcl::PointCloud<PointT>::Ptr filter(const typename pcl::PointCloud<PointT>::Ptr &in) const {
        if (!in) return typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> f; f.setInputCloud(in); float ls = static_cast<float>(leaf_size_);
        f.setLeafSize(ls, ls, ls); f.filter(*out); return out;
    }
private:
    double leaf_size_{0.005};
};

template <typename PointT>
class StatisticalOutlierRemoval {
public:
    StatisticalOutlierRemoval &setMeanK(int k) { mean_k_ = k; return *this; }
    StatisticalOutlierRemoval &setStddevMulThresh(double s) { stddev_mul_ = s; return *this; }
    typename pcl::PointCloud<PointT>::Ptr filter(const typename pcl::PointCloud<PointT>::Ptr &in) const {
        if (!in) return typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::StatisticalOutlierRemoval<PointT> f; f.setInputCloud(in);
        f.setMeanK(mean_k_); f.setStddevMulThresh(stddev_mul_); f.filter(*out); return out;
    }
private:
    int mean_k_{50};
    double stddev_mul_{1.0};
};

} // namespace fluent_cloud::filters



