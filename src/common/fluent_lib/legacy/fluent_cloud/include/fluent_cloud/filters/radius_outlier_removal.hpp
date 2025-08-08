#ifndef FLUENT_CLOUD_FILTERS_RADIUS_OUTLIER_REMOVAL_HPP
#define FLUENT_CLOUD_FILTERS_RADIUS_OUTLIER_REMOVAL_HPP

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fluent_cloud {
namespace filters {

template<typename PointT>
class RadiusOutlierRemoval {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    
    RadiusOutlierRemoval() : radius_(0.05), min_neighbors_(2) {}
    
    RadiusOutlierRemoval& setRadiusSearch(float radius) {
        radius_ = radius;
        return *this;
    }
    
    RadiusOutlierRemoval& setMinNeighborsInRadius(int min_neighbors) {
        min_neighbors_ = min_neighbors;
        return *this;
    }
    
    PointCloudPtr filter(const PointCloudPtr& input) {
        pcl::RadiusOutlierRemoval<PointT> ror;
        ror.setInputCloud(input);
        ror.setRadiusSearch(radius_);
        ror.setMinNeighborsInRadius(min_neighbors_);
        
        PointCloudPtr output(new PointCloud);
        ror.filter(*output);
        return output;
    }
    
private:
    float radius_;
    int min_neighbors_;
};

} // namespace filters
} // namespace fluent_cloud

#endif // FLUENT_CLOUD_FILTERS_RADIUS_OUTLIER_REMOVAL_HPP