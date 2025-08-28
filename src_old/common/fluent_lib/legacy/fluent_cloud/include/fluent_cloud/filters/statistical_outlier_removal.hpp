#ifndef FLUENT_CLOUD_FILTERS_STATISTICAL_OUTLIER_REMOVAL_HPP
#define FLUENT_CLOUD_FILTERS_STATISTICAL_OUTLIER_REMOVAL_HPP

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fluent_cloud {
namespace filters {

template<typename PointT>
class StatisticalOutlierRemoval {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    
    StatisticalOutlierRemoval() : mean_k_(50), std_dev_mul_(1.0) {}
    
    StatisticalOutlierRemoval& setMeanK(int k) {
        mean_k_ = k;
        return *this;
    }
    
    StatisticalOutlierRemoval& setStddevMulThresh(float std_dev) {
        std_dev_mul_ = std_dev;
        return *this;
    }
    
    PointCloudPtr filter(const PointCloudPtr& input) {
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(input);
        sor.setMeanK(mean_k_);
        sor.setStddevMulThresh(std_dev_mul_);
        
        PointCloudPtr output(new PointCloud);
        sor.filter(*output);
        return output;
    }
    
    PointCloudPtr filterInliers(const PointCloudPtr& input) {
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(input);
        sor.setMeanK(mean_k_);
        sor.setStddevMulThresh(std_dev_mul_);
        sor.setNegative(false);
        
        PointCloudPtr output(new PointCloud);
        sor.filter(*output);
        return output;
    }
    
    PointCloudPtr filterOutliers(const PointCloudPtr& input) {
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(input);
        sor.setMeanK(mean_k_);
        sor.setStddevMulThresh(std_dev_mul_);
        sor.setNegative(true);
        
        PointCloudPtr output(new PointCloud);
        sor.filter(*output);
        return output;
    }
    
private:
    int mean_k_;
    float std_dev_mul_;
};

} // namespace filters
} // namespace fluent_cloud

#endif // FLUENT_CLOUD_FILTERS_STATISTICAL_OUTLIER_REMOVAL_HPP