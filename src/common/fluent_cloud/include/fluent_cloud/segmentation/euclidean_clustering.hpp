#ifndef FLUENT_CLOUD_SEGMENTATION_EUCLIDEAN_CLUSTERING_HPP
#define FLUENT_CLOUD_SEGMENTATION_EUCLIDEAN_CLUSTERING_HPP

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fluent_cloud {
namespace segmentation {

template<typename PointT>
class EuclideanClustering {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    
    struct ClusterResult {
        std::vector<PointCloudPtr> clusters;
        std::vector<pcl::PointIndices> indices;
        std::vector<Eigen::Vector4f> centroids;
        std::vector<Eigen::Vector4f> min_points;
        std::vector<Eigen::Vector4f> max_points;
    };
    
    EuclideanClustering()
        : cluster_tolerance_(0.02),
          min_cluster_size_(100),
          max_cluster_size_(25000) {}
    
    EuclideanClustering& setClusterTolerance(float tolerance) {
        cluster_tolerance_ = tolerance;
        return *this;
    }
    
    EuclideanClustering& setMinClusterSize(int size) {
        min_cluster_size_ = size;
        return *this;
    }
    
    EuclideanClustering& setMaxClusterSize(int size) {
        max_cluster_size_ = size;
        return *this;
    }
    
    ClusterResult cluster(const PointCloudPtr& input) {
        ClusterResult result;
        
        // KdTree作成
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(input);
        
        // クラスタリング実行
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(input);
        
        ec.extract(result.indices);
        
        // 各クラスタを個別のPointCloudに変換
        for (const auto& cluster_indices : result.indices) {
            PointCloudPtr cluster(new PointCloud);
            
            for (const auto& idx : cluster_indices.indices) {
                cluster->points.push_back(input->points[idx]);
            }
            
            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true;
            cluster->header = input->header;
            
            // 統計情報を計算
            Eigen::Vector4f centroid;
            Eigen::Vector4f min_pt, max_pt;
            pcl::compute3DCentroid(*cluster, centroid);
            pcl::getMinMax3D(*cluster, min_pt, max_pt);
            
            result.clusters.push_back(cluster);
            result.centroids.push_back(centroid);
            result.min_points.push_back(min_pt);
            result.max_points.push_back(max_pt);
        }
        
        return result;
    }
    
    // バウンディングボックスサイズでフィルタリング
    ClusterResult clusterWithSizeFilter(const PointCloudPtr& input,
                                       float min_width = 0.0, float max_width = FLT_MAX,
                                       float min_height = 0.0, float max_height = FLT_MAX,
                                       float min_depth = 0.0, float max_depth = FLT_MAX) {
        ClusterResult all_clusters = cluster(input);
        ClusterResult filtered;
        
        for (size_t i = 0; i < all_clusters.clusters.size(); ++i) {
            Eigen::Vector4f size = all_clusters.max_points[i] - all_clusters.min_points[i];
            
            if (size[0] >= min_width && size[0] <= max_width &&
                size[1] >= min_height && size[1] <= max_height &&
                size[2] >= min_depth && size[2] <= max_depth) {
                
                filtered.clusters.push_back(all_clusters.clusters[i]);
                filtered.indices.push_back(all_clusters.indices[i]);
                filtered.centroids.push_back(all_clusters.centroids[i]);
                filtered.min_points.push_back(all_clusters.min_points[i]);
                filtered.max_points.push_back(all_clusters.max_points[i]);
            }
        }
        
        return filtered;
    }
    
private:
    float cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
};

} // namespace segmentation
} // namespace fluent_cloud

#endif // FLUENT_CLOUD_SEGMENTATION_EUCLIDEAN_CLUSTERING_HPP