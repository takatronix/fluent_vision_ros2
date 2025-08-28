#ifndef FLUENT_VISION_SLAM_HPP
#define FLUENT_VISION_SLAM_HPP

#include <fluent_vision/fluent_vision.hpp>
#include <fluent_cloud/fluent_cloud.hpp>
#include <rtabmap_msgs/msg/map_data.hpp>
#include <rtabmap_msgs/srv/get_map.hpp>

namespace fluent_vision {

/**
 * @brief FluentSLAM - RTABMAPとの美しい連携
 * 
 * Visual SLAMを流れるように記述
 */
class FluentSLAM {
public:
    // ========== RTABMAPとの連携 ==========
    static FluentSLAM fromRTABMAP(const std::string& rtabmap_namespace = "rtabmap");
    
    // マップ操作
    FluentSLAM& getMap();
    FluentSLAM& getLocalMap(float radius = 10.0);
    FluentSLAM& optimizeMap();
    FluentSLAM& saveMap(const std::string& filename);
    
    // 特徴点処理
    FluentSLAM& extractFeatures(const std::string& type = "ORB");
    FluentSLAM& matchFeatures(FluentVision& current_frame);
    FluentSLAM& addKeyframe(FluentVision& frame);
    
    // ループクロージャ
    FluentSLAM& detectLoopClosure();
    FluentSLAM& refineWithLoopClosure();
    
    // 3Dマップ処理
    FluentSLAM& buildOctomap(float resolution = 0.05);
    FluentSLAM& buildMesh();
    FluentSLAM& colorizeMap(const std::vector<FluentVision>& keyframes);
    
    // 可視化
    FluentSLAM& visualizeTrajectory();
    FluentSLAM& visualizeMap();
    FluentSLAM& publishTF();
    
    // 出力
    fluent_cloud::FluentCloudXYZRGB toPointCloud() const;
    nav_msgs::msg::OccupancyGrid toOccupancyGrid() const;
    
private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<rtabmap_msgs::srv::GetMap>::SharedPtr map_client_;
    rtabmap_msgs::msg::MapData::SharedPtr map_data_;
};

// ========== 使用例 ==========
/*
// RTABMAPと連携したSLAM処理
auto slam = FluentSLAM::fromRTABMAP()
    .getMap()
    .optimizeMap()
    .buildOctomap(0.1)
    .visualizeMap();

// リアルタイム処理
FluentVision::stream("/camera/image")
    .extractFeatures("SURF")
    .matchWith(slam)
    .when(loop_detected)
        .refineMap()
    .publishPose("/robot/pose");

// マップの後処理
auto refined_map = slam
    .toPointCloud()
    .removeOutliers()
    .smoothMLS()
    .colorByHeight()
    .save("refined_map.pcd");
*/

} // namespace fluent_vision

#endif // FLUENT_VISION_SLAM_HPP