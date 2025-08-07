#ifndef FLUENT_CLOUD_TRANSFORMS_TF_TRANSFORM_HPP
#define FLUENT_CLOUD_TRANSFORMS_TF_TRANSFORM_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace fluent_cloud {
namespace transforms {

template<typename PointT>
class TFTransform {
public:
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    
    TFTransform(rclcpp::Node::SharedPtr node) 
        : node_(node),
          tf_buffer_(std::make_unique<tf2_ros::Buffer>(node->get_clock())),
          tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_)) {}
    
    // PCLのヘッダーフレームIDを使った変換
    PointCloudPtr transform(const PointCloudPtr& input, const std::string& target_frame) {
        if (input->header.frame_id.empty()) {
            RCLCPP_WARN(node_->get_logger(), "Point cloud has no frame_id set!");
            return input;
        }
        
        return transform(input, input->header.frame_id, target_frame);
    }
    
    // 明示的なフレーム指定変換
    PointCloudPtr transform(const PointCloudPtr& input, 
                           const std::string& source_frame,
                           const std::string& target_frame,
                           const rclcpp::Time& time = rclcpp::Time(0)) {
        try {
            // TF2で変換を取得
            geometry_msgs::msg::TransformStamped transform_stamped;
            if (time == rclcpp::Time(0)) {
                // 最新のTFを使用
                transform_stamped = tf_buffer_->lookupTransform(
                    target_frame, source_frame, tf2::TimePointZero);
            } else {
                // 指定時刻のTFを使用
                transform_stamped = tf_buffer_->lookupTransform(
                    target_frame, source_frame, time);
            }
            
            // Eigenに変換
            Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform_stamped.transform);
            
            // PCLで変換実行
            PointCloudPtr output(new PointCloud);
            pcl::transformPointCloud(*input, *output, transform_eigen.matrix().cast<float>());
            
            // ヘッダー情報を更新
            output->header.frame_id = target_frame;
            output->header.stamp = input->header.stamp;
            
            return output;
            
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(node_->get_logger(), "TF transform failed: %s", ex.what());
            return input;  // 変換失敗時は元のデータを返す
        }
    }
    
    // ROS2メッセージを直接変換
    sensor_msgs::msg::PointCloud2 transformMsg(const sensor_msgs::msg::PointCloud2& input,
                                               const std::string& target_frame) {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer_->lookupTransform(target_frame, input.header.frame_id, 
                                           input.header.stamp);
            
            sensor_msgs::msg::PointCloud2 output;
            tf2::doTransform(input, output, transform_stamped);
            return output;
            
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(node_->get_logger(), "TF transform failed: %s", ex.what());
            return input;
        }
    }
    
    // 変換が利用可能かチェック
    bool canTransform(const std::string& source_frame,
                      const std::string& target_frame,
                      const rclcpp::Time& time = rclcpp::Time(0)) {
        try {
            if (time == rclcpp::Time(0)) {
                return tf_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero);
            } else {
                return tf_buffer_->canTransform(target_frame, source_frame, time);
            }
        } catch (...) {
            return false;
        }
    }
    
    // 変換を待つ
    bool waitForTransform(const std::string& source_frame,
                         const std::string& target_frame,
                         const rclcpp::Duration& timeout) {
        try {
            tf_buffer_->lookupTransform(target_frame, source_frame, 
                                       tf2::TimePointZero, timeout);
            return true;
        } catch (tf2::TransformException& ex) {
            return false;
        }
    }
    
private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace transforms
} // namespace fluent_cloud

#endif // FLUENT_CLOUD_TRANSFORMS_TF_TRANSFORM_HPP