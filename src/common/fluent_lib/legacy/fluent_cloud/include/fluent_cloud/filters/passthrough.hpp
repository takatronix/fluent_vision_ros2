#ifndef FLUENT_CLOUD_FILTERS_PASSTHROUGH_HPP
#define FLUENT_CLOUD_FILTERS_PASSTHROUGH_HPP

#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fluent_cloud {
namespace filters {

template<typename PointT>
class PassThrough {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    
    PassThrough() : field_name_("z"), min_(-FLT_MAX), max_(FLT_MAX) {}
    
    PassThrough& setFilterFieldName(const std::string& field_name) {
        field_name_ = field_name;
        return *this;
    }
    
    PassThrough& setFilterLimits(float min, float max) {
        min_ = min;
        max_ = max;
        return *this;
    }
    
    PassThrough& setFilterLimitsNegative(bool negative) {
        negative_ = negative;
        return *this;
    }
    
    PointCloudPtr filter(const PointCloudPtr& input) {
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(input);
        pass.setFilterFieldName(field_name_);
        pass.setFilterLimits(min_, max_);
        pass.setFilterLimitsNegative(negative_);
        
        PointCloudPtr output(new PointCloud);
        pass.filter(*output);
        return output;
    }
    
    // 便利メソッド
    PointCloudPtr filterX(const PointCloudPtr& input, float min, float max) {
        return setFilterFieldName("x").setFilterLimits(min, max).filter(input);
    }
    
    PointCloudPtr filterY(const PointCloudPtr& input, float min, float max) {
        return setFilterFieldName("y").setFilterLimits(min, max).filter(input);
    }
    
    PointCloudPtr filterZ(const PointCloudPtr& input, float min, float max) {
        return setFilterFieldName("z").setFilterLimits(min, max).filter(input);
    }
    
private:
    std::string field_name_;
    float min_, max_;
    bool negative_ = false;
};

} // namespace filters
} // namespace fluent_cloud

#endif // FLUENT_CLOUD_FILTERS_PASSTHROUGH_HPP