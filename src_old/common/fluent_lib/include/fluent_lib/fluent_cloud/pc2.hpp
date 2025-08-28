#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace fluent_cloud::pc2
{

inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr from_msg(const sensor_msgs::msg::PointCloud2 &msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(msg, *cloud);
    return cloud;
}

inline sensor_msgs::msg::PointCloud2 to_msg(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                                            const std::string &frame_id,
                                            const rclcpp::Time &stamp)
{
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(cloud, out);
    out.header.frame_id = frame_id;
    out.header.stamp = stamp;
    return out;
}

inline sensor_msgs::msg::PointCloud2 to_msg(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                            const std::string &frame_id,
                                            const rclcpp::Time &stamp)
{
    return to_msg(*cloud, frame_id, stamp);
}

} // namespace fluent_cloud::pc2


