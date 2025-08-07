#ifndef FLUENT_CLOUD_FILTERS_VOXEL_GRID_HPP
#define FLUENT_CLOUD_FILTERS_VOXEL_GRID_HPP

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fluent_cloud {
namespace filters {

template<typename PointT>
class VoxelGrid {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    
    VoxelGrid() : leaf_size_(0.01f) {}
    
    VoxelGrid& setLeafSize(float size) {
        leaf_size_ = size;
        return *this;
    }
    
    VoxelGrid& setLeafSize(float x, float y, float z) {
        leaf_x_ = x;
        leaf_y_ = y;
        leaf_z_ = z;
        use_uniform_ = false;
        return *this;
    }
    
    PointCloudPtr filter(const PointCloudPtr& input) {
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(input);
        
        if (use_uniform_) {
            vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        } else {
            vg.setLeafSize(leaf_x_, leaf_y_, leaf_z_);
        }
        
        PointCloudPtr output(new PointCloud);
        vg.filter(*output);
        return output;
    }
    
private:
    float leaf_size_;
    float leaf_x_, leaf_y_, leaf_z_;
    bool use_uniform_ = true;
};

} // namespace filters
} // namespace fluent_cloud

#endif // FLUENT_CLOUD_FILTERS_VOXEL_GRID_HPP