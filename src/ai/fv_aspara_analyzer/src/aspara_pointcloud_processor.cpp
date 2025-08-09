#include "fv_aspara_analyzer/aspara_pointcloud_processor.hpp"
#include "fv_aspara_analyzer/fv_aspara_analyzer_node.hpp"
#include <fluent.hpp>
#include <cmath>
#include <algorithm>

namespace fv_aspara_analyzer {

AsparaPointcloudProcessor::AsparaPointcloudProcessor(FvAsparaAnalyzerNode* node_ptr)
    : node_(node_ptr)
{
    // パブリッシャー初期化
    // YAMLの output_filtered_pointcloud_topic を使用（未設定時は従来名）
    std::string filtered_topic = "output_filtered_pointcloud";
    try {
        if (node_->has_parameter("output_filtered_pointcloud_topic")) {
            filtered_topic = node_->get_parameter("output_filtered_pointcloud_topic").as_string();
        }
    } catch (...) {}
    filtered_pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        filtered_topic, 10);
    annotated_image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
        "output_annotated_image", 10);
    // マーカートピックはパラメータ化
    std::string marker_topic = node_->declare_parameter<std::string>(
        "output_marker_topic", "aspara_markers");
    markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        marker_topic, 10);
    // 表示寿命をパラメータ化
    marker_lifetime_sec_ = node_->declare_parameter<double>("marker_lifetime_sec", 0.3);
    
    // TFブロードキャスター初期化
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    
    // パラメータはFvAsparaAnalyzerNodeのメンバから直接参照
    pointcloud_distance_min_ = node_->pointcloud_distance_min_;
    pointcloud_distance_max_ = node_->pointcloud_distance_max_;
    noise_reduction_neighbors_ = node_->noise_reduction_neighbors_;
    noise_reduction_std_dev_ = node_->noise_reduction_std_dev_;
    voxel_leaf_size_ = node_->voxel_leaf_size_;
    
    RCLCPP_INFO(node_->get_logger(), "AsparaPointcloudProcessor initialized: distance=%.2f-%.2f, voxel=%.4f", 
                pointcloud_distance_min_, pointcloud_distance_max_, voxel_leaf_size_);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr AsparaPointcloudProcessor::extractPointCloudFromDepth(
    const cv::Rect& bbox,
    const sensor_msgs::msg::Image::SharedPtr& depth_image,
    const sensor_msgs::msg::Image::SharedPtr& color_image,
    const sensor_msgs::msg::CameraInfo::SharedPtr& camera_info)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    if (!depth_image || !camera_info || !color_image) {
        return cloud;
    }
    
    // 深度画像とカラー画像をOpenCV形式に変換
    cv::Mat depth_mat;
    cv::Mat color_mat;
    
    try {
        // 深度画像の変換
        cv_bridge::CvImagePtr depth_ptr;
        if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            depth_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
            depth_mat = depth_ptr->image;
        } else if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            depth_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
            depth_ptr->image.convertTo(depth_mat, CV_16UC1, 1000.0); // メートルをミリメートルに変換
        } else {
            RCLCPP_WARN(node_->get_logger(), "Unsupported depth encoding: %s", depth_image->encoding.c_str());
            return cloud;
        }
        
        // カラー画像の変換
        cv_bridge::CvImagePtr color_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::BGR8);
        color_mat = color_ptr->image;
        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "CV bridge exception in extractPointCloudFromDepth: %s", e.what());
        return cloud;
    }
    
    // 画像サイズの一致確認
    if (depth_mat.rows != color_mat.rows || depth_mat.cols != color_mat.cols) {
        RCLCPP_WARN(node_->get_logger(), 
                    "Depth and color image size mismatch: depth(%dx%d) vs color(%dx%d)",
                    depth_mat.cols, depth_mat.rows, color_mat.cols, color_mat.rows);
        return cloud;
    }
    
    // カメラパラメータの取得と有効性チェック
    double fx = camera_info->k[0];
    double fy = camera_info->k[4];
    double cx = camera_info->k[2];
    double cy = camera_info->k[5];
    
    // 0除算の回避
    if (fx <= 0.0 || fy <= 0.0) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid camera parameters: fx=%.3f, fy=%.3f", fx, fy);
        return cloud;
    }
    
    // バウンディングボックスの有効性確認と範囲クリップ
    if (bbox.width <= 0 || bbox.height <= 0) {
        RCLCPP_WARN(node_->get_logger(), "Invalid bounding box size: %dx%d", bbox.width, bbox.height);
        return cloud;
    }
    
    int x_start = std::max(0, bbox.x);
    int y_start = std::max(0, bbox.y);
    int x_end = std::min(depth_mat.cols, bbox.x + bbox.width);
    int y_end = std::min(depth_mat.rows, bbox.y + bbox.height);
    
    // クリップ後のサイズ確認
    if (x_start >= x_end || y_start >= y_end) {
        RCLCPP_WARN(node_->get_logger(), 
                    "Bounding box outside image bounds: bbox(%d,%d,%d,%d), image(%dx%d)",
                    bbox.x, bbox.y, bbox.width, bbox.height, depth_mat.cols, depth_mat.rows);
        return cloud;
    }
    
    // バウンディングボックス内のピクセルのみを処理
    for (int v = y_start; v < y_end; v++) {
        for (int u = x_start; u < x_end; u++) {
            // 深度値を取得（単位：mm）
            uint16_t depth_mm = depth_mat.at<uint16_t>(v, u);
            
            // 無効な深度値をスキップ
            if (depth_mm == 0) continue;
            
            // メートルに変換
            float depth_m = depth_mm / 1000.0f;
            
            // 距離フィルタリング
            if (depth_m < pointcloud_distance_min_ || depth_m > pointcloud_distance_max_) {
                continue;
            }
            
            // 3D座標を計算（カメラ座標系）
            pcl::PointXYZRGB point;
            point.z = depth_m;
            point.x = (u - cx) * depth_m / fx;
            point.y = (v - cy) * depth_m / fy;
            
            // カラー情報を追加
            cv::Vec3b color = color_mat.at<cv::Vec3b>(v, u);
            point.b = color[0];
            point.g = color[1];
            point.r = color[2];
            
            cloud->points.push_back(point);
        }
    }
    
    // 点群のメタデータを設定
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    
    RCLCPP_DEBUG(node_->get_logger(), "Extracted %zu points from depth image bbox", cloud->points.size());
    
    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr AsparaPointcloudProcessor::applyNoiseReduction(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud)
{
    // 点群サイズチェック
    if (input_cloud->points.size() < 10) {
        RCLCPP_WARN(node_->get_logger(), "Too few points for noise reduction");
        return input_cloud;
    }

    // ステップ1: ボクセルグリッドフィルタ（ダウンサンプリング）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_filter.filter(*voxel_filtered);

    if (voxel_filtered->points.size() < 10) {
        RCLCPP_WARN(node_->get_logger(), "Too few points after voxel filtering");
        return voxel_filtered;
    }

    // ステップ2: 統計的外れ値除去
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr denoised_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
    statistical_filter.setInputCloud(voxel_filtered);
    statistical_filter.setMeanK(noise_reduction_neighbors_);
    statistical_filter.setStddevMulThresh(noise_reduction_std_dev_);
    statistical_filter.filter(*denoised_cloud);

    RCLCPP_DEBUG(node_->get_logger(), 
                 "Noise reduction: %zu -> %zu -> %zu points",
                 input_cloud->points.size(),
                 voxel_filtered->points.size(),
                 denoised_cloud->points.size());

    return denoised_cloud;
}

geometry_msgs::msg::Point AsparaPointcloudProcessor::estimateRootPosition(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aspara_cloud)
{
    geometry_msgs::msg::Point root_position;
    
    if (aspara_cloud->points.empty()) {
        return root_position;
    }

    // Z座標の最小値を根元位置として使用
    float min_z = std::numeric_limits<float>::max();
    pcl::PointXYZRGB root_point;
    
    for (const auto& point : aspara_cloud->points) {
        if (point.z < min_z) {
            min_z = point.z;
            root_point = point;
        }
    }

    root_position.x = root_point.x;
    root_position.y = root_point.y;
    root_position.z = root_point.z;

    return root_position;
}

float AsparaPointcloudProcessor::calculateStraightness(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aspara_cloud)
{
    if (aspara_cloud->points.size() < 3) {
        return 0.0f;
    }

    // PCAによる真っ直ぐ度計算
    pcl::PCA<pcl::PointXYZRGB> pca;
    pca.setInputCloud(aspara_cloud);
    
    Eigen::Vector3f eigenvalues = pca.getEigenValues();
    
    // 真っ直ぐ度 = 最大固有値 / 全固有値の和
    if (eigenvalues[1] == 0.0f) {
        return 1.0f;  // 完全な直線
    }
    
    float straightness = eigenvalues[0] / (eigenvalues[0] + eigenvalues[1] + eigenvalues[2]);
    return std::min(1.0f, straightness);
}

float AsparaPointcloudProcessor::calculateLength(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aspara_cloud)
{
    if (aspara_cloud->points.size() < 2) {
        return 0.0f;
    }

    // Z軸方向の最大・最小値を取得
    float min_val = std::numeric_limits<float>::max();
    float max_val = std::numeric_limits<float>::lowest();

    for (const auto& point : aspara_cloud->points) {
        float z = point.z;  // Z軸を主軸として仮定
        if (z < min_val) min_val = z;
        if (z > max_val) max_val = z;
    }

    return std::abs(max_val - min_val);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr AsparaPointcloudProcessor::generatePCALine(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aspara_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pca_line(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (aspara_cloud->points.size() < 3) {
        return pca_line;
    }

    // PCA計算
    pcl::PCA<pcl::PointXYZRGB> pca;
    pca.setInputCloud(aspara_cloud);
    
    // 主成分軸とデータの範囲を取得
    Eigen::Vector4f centroid = pca.getMean();
    Eigen::Vector3f principal_axis = pca.getEigenVectors().col(0); // 第1主成分
    
    // データの投影範囲を計算
    float min_proj = std::numeric_limits<float>::max();
    float max_proj = std::numeric_limits<float>::lowest();
    
    for (const auto& point : aspara_cloud->points) {
        Eigen::Vector3f p(point.x, point.y, point.z);
        Eigen::Vector3f centered = p - centroid.head<3>();
        float projection = centered.dot(principal_axis);
        
        if (projection < min_proj) min_proj = projection;
        if (projection > max_proj) max_proj = projection;
    }
    
    // PCAライン上に等間隔でポイントを生成
    int num_points = 50; // ライン解像度
    for (int i = 0; i < num_points; ++i) {
        float t = min_proj + (max_proj - min_proj) * i / (num_points - 1);
        Eigen::Vector3f line_point = centroid.head<3>() + t * principal_axis;
        
        pcl::PointXYZ point;
        point.x = line_point.x();
        point.y = line_point.y();
        point.z = line_point.z();
        
        pca_line->points.push_back(point);
    }
    
    // 点群のメタデータを設定
    pca_line->width = pca_line->points.size();
    pca_line->height = 1;
    pca_line->is_dense = false;
    
    return pca_line;
}

cv::Point2f AsparaPointcloudProcessor::project3DTo2D(
    const pcl::PointXYZRGB& point,
    const sensor_msgs::msg::CameraInfo& camera_info)
{
    // ガード
    if (point.z <= 0.0f || !std::isfinite(point.z)) {
        RCLCPP_DEBUG(node_->get_logger(), 
            "[PROJECT] Invalid Z: z=%.3f", point.z);
        return cv::Point2f(-1.f, -1.f);
    }

    // 内部パラメータ
    double fx = camera_info.k[0];
    double fy = camera_info.k[4];
    double cx = camera_info.k[2];
    double cy = camera_info.k[5];
    
    // カメラ内部パラメータの解像度チェック
    // camera_infoの解像度と実際の画像解像度が異なる場合はスケーリング
    // ※点群がregistered_pointsの場合、camera_infoの解像度と一致する必要がある
    RCLCPP_DEBUG(node_->get_logger(),
        "[PROJECT] Camera params: fx=%.2f fy=%.2f cx=%.2f cy=%.2f (raw)",
        fx, fy, cx, cy);

    // 正規化座標 (OpenCVと同じ定義)
    double x = static_cast<double>(point.x) / static_cast<double>(point.z);
    double y = static_cast<double>(point.y) / static_cast<double>(point.z);

    // 歪み（plumb_bob想定）
    double u = 0.0, v = 0.0;
    if (!camera_info.d.empty() && camera_info.distortion_model == "plumb_bob") {
        const double k1 = camera_info.d.size() > 0 ? camera_info.d[0] : 0.0;
        const double k2 = camera_info.d.size() > 1 ? camera_info.d[1] : 0.0;
        const double p1 = camera_info.d.size() > 2 ? camera_info.d[2] : 0.0;
        const double p2 = camera_info.d.size() > 3 ? camera_info.d[3] : 0.0;
        const double k3 = camera_info.d.size() > 4 ? camera_info.d[4] : 0.0;

        const double r2 = x*x + y*y;
        const double r4 = r2*r2;
        const double r6 = r4*r2;
        const double radial = 1.0 + k1*r2 + k2*r4 + k3*r6;
        const double x_tangential = 2.0*p1*x*y + p2*(r2 + 2.0*x*x);
        const double y_tangential = p1*(r2 + 2.0*y*y) + 2.0*p2*x*y;
        const double x_distorted = x*radial + x_tangential;
        const double y_distorted = y*radial + y_tangential;
        u = fx * x_distorted + cx;
        v = fy * y_distorted + cy;
    } else {
        // 歪みなし
        u = fx * x + cx;
        v = fy * y + cy;
    }

    // 追加デバッグ: 非有限値ガード
    if (!std::isfinite(u) || !std::isfinite(v)) {
        return cv::Point2f(-1.f, -1.f);
    }
    return cv::Point2f(static_cast<float>(u), static_cast<float>(v));
}

void AsparaPointcloudProcessor::publishFilteredPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const std::string& frame_id,
    int /*aspara_id*/)
{
    if (!cloud || cloud->points.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Cannot publish empty or null point cloud");
        return;
    }
    
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    output_msg.header.stamp = node_->now();
    output_msg.header.frame_id = frame_id;
    
    filtered_pointcloud_pub_->publish(output_msg);
}

// Overload with explicit timestamp (used by analyzer thread)
void AsparaPointcloudProcessor::publishFilteredPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const std::string& frame_id,
    int /*aspara_id*/,
    const rclcpp::Time& stamp)
{
    if (!cloud || cloud->points.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Cannot publish empty or null point cloud");
        return;
    }
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    output_msg.header.stamp = stamp;
    output_msg.header.frame_id = frame_id;
    filtered_pointcloud_pub_->publish(output_msg);
}

void AsparaPointcloudProcessor::publishRootTF(
    const geometry_msgs::msg::Point& root_position,
    const std::string& frame_id,
    int aspara_id)
{
    // 廃止: TFの氾濫を避けるためMarkerに置換
    (void)root_position; (void)frame_id; (void)aspara_id;
}

void AsparaPointcloudProcessor::publishAsparaMarker(
    const geometry_msgs::msg::Point& root_position,
    const std::string& frame_id,
    int aspara_id,
    float length_m,
    bool is_harvestable,
    const rclcpp::Time& stamp)
{
    if (!markers_pub_) return;
    visualization_msgs::msg::MarkerArray array_msg;
    visualization_msgs::msg::Marker cyl;
    cyl.header.stamp = stamp;
    cyl.header.frame_id = frame_id;
    // camera名をprefix化（fv/d415/... → d415）
    std::string camera_prefix = frame_id;
    auto p = camera_prefix.find("/d415/");
    if (p != std::string::npos) camera_prefix = "d415";
    p = frame_id.find("/d405/");
    if (p != std::string::npos) camera_prefix = "d405";

    cyl.ns = camera_prefix + "_aspara";
    cyl.id = aspara_id;
    cyl.type = visualization_msgs::msg::Marker::CYLINDER;
    cyl.action = visualization_msgs::msg::Marker::ADD;
    cyl.pose.position = root_position; // 根元に立てる
    cyl.pose.orientation.w = 1.0;      // Z軸向き（簡易）
    cyl.scale.x = 0.02;                // 直径 2cm（調整可）
    cyl.scale.y = 0.02;
    cyl.scale.z = std::max(0.01f, length_m); // 高さを長さに合わせる
    if (is_harvestable) {
        cyl.color.r = 0.0; cyl.color.g = 1.0; cyl.color.b = 0.0; cyl.color.a = 0.9;
    } else {
        cyl.color.r = 1.0; cyl.color.g = 0.0; cyl.color.b = 0.0; cyl.color.a = 0.9;
    }
    cyl.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
    array_msg.markers.push_back(cyl);
    // 距離ラベル
    visualization_msgs::msg::Marker txt;
    txt.header = cyl.header;
    txt.ns = cyl.ns + "_dist";
    txt.id = aspara_id; // ペア
    txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::msg::Marker::ADD;
    txt.pose = cyl.pose;
    txt.pose.position.z += cyl.scale.z + 0.03; // 少し上
    txt.scale.z = 0.06; // 6cm文字
    txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 1.0; txt.color.a = 0.95;
    // 距離[m]（原点からのユークリッド距離）
    double dist = std::sqrt(
        cyl.pose.position.x * cyl.pose.position.x +
        cyl.pose.position.y * cyl.pose.position.y +
        cyl.pose.position.z * cyl.pose.position.z);
    txt.text = (camera_prefix + ": " + std::to_string(static_cast<float>(dist))).substr(0, 16) + " m";
    txt.lifetime = cyl.lifetime;
    array_msg.markers.push_back(txt);
    markers_pub_->publish(array_msg);
}

// Overload: specify axis direction (unit vector) for cylinder orientation
void AsparaPointcloudProcessor::publishAsparaMarker(
    const geometry_msgs::msg::Point& root_position,
    const std::string& frame_id,
    int aspara_id,
    float length_m,
    bool is_harvestable,
    const rclcpp::Time& stamp,
    const geometry_msgs::msg::Vector3& axis_dir)
{
    if (!markers_pub_) return;

    // Normalize axis_dir (fallback to Z if invalid)
    double ax = axis_dir.x, ay = axis_dir.y, az = axis_dir.z;
    double norm = std::sqrt(ax*ax + ay*ay + az*az);
    if (norm < 1e-6) { ax = 0; ay = 0; az = 1; norm = 1; }
    ax /= norm; ay /= norm; az /= norm;

    // Compute quaternion rotating Z(0,0,1) to axis (ax,ay,az)
    // Reference: shortest-arc quaternion between two vectors
    double zx = 0.0, zy = 0.0, zz = 1.0;
    double cx = zy*az - zz*ay;
    double cy = zz*ax - zx*az;
    double cz = zx*ay - zy*ax;
    double dot = zx*ax + zy*ay + zz*az; // = az
    geometry_msgs::msg::Quaternion q;
    if (dot < -0.999999) {
        // Opposite direction: rotate 180 deg around X axis
        q.x = 1.0; q.y = 0.0; q.z = 0.0; q.w = 0.0;
    } else {
        double s = std::sqrt((1.0 + dot) * 2.0);
        double invs = 1.0 / s;
        q.x = cx * invs;
        q.y = cy * invs;
        q.z = cz * invs;
        q.w = s * 0.5;
    }

    // Position at the center of the cylinder along axis: root + axis * (length/2)
    geometry_msgs::msg::Point center;
    center.x = root_position.x + static_cast<double>(length_m) * 0.5 * ax;
    center.y = root_position.y + static_cast<double>(length_m) * 0.5 * ay;
    center.z = root_position.z + static_cast<double>(length_m) * 0.5 * az;

    visualization_msgs::msg::MarkerArray array_msg;
    visualization_msgs::msg::Marker cyl;
    cyl.header.stamp = stamp;
    cyl.header.frame_id = frame_id;

    std::string camera_prefix = frame_id;
    auto p = camera_prefix.find("/d415/");
    if (p != std::string::npos) camera_prefix = "d415";
    p = frame_id.find("/d405/");
    if (p != std::string::npos) camera_prefix = "d405";

    cyl.ns = camera_prefix + "_aspara";
    cyl.id = aspara_id;
    cyl.type = visualization_msgs::msg::Marker::CYLINDER;
    cyl.action = visualization_msgs::msg::Marker::ADD;
    cyl.pose.position = center;
    cyl.pose.orientation = q;
    cyl.scale.x = 0.02;
    cyl.scale.y = 0.02;
    cyl.scale.z = std::max(0.01f, length_m);
    if (is_harvestable) {
        cyl.color.r = 0.0; cyl.color.g = 1.0; cyl.color.b = 0.0; cyl.color.a = 0.9;
    } else {
        cyl.color.r = 1.0; cyl.color.g = 0.0; cyl.color.b = 0.0; cyl.color.a = 0.9;
    }
    cyl.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
    array_msg.markers.push_back(cyl);

    // Text label above tip
    visualization_msgs::msg::Marker txt;
    txt.header = cyl.header;
    txt.ns = cyl.ns + "_dist";
    txt.id = aspara_id;
    txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::msg::Marker::ADD;
    txt.pose.position.x = center.x + ax * (static_cast<double>(length_m) * 0.5 + 0.03);
    txt.pose.position.y = center.y + ay * (static_cast<double>(length_m) * 0.5 + 0.03);
    txt.pose.position.z = center.z + az * (static_cast<double>(length_m) * 0.5 + 0.03);
    txt.scale.z = 0.06;
    txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 1.0; txt.color.a = 0.95;
    double dist = std::sqrt(center.x*center.x + center.y*center.y + center.z*center.z);
    txt.text = (camera_prefix + ": " + std::to_string(static_cast<float>(dist))).substr(0, 16) + " m";
    txt.lifetime = cyl.lifetime;
    array_msg.markers.push_back(txt);

    markers_pub_->publish(array_msg);
}

void AsparaPointcloudProcessor::publishAnnotatedImage(
    const cv::Mat& image,
    const AsparaInfo& aspara_info,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pca_line_cloud,
    float length,
    float straightness,
    bool is_harvestable)
{
    RCLCPP_DEBUG(node_->get_logger(), "Publishing annotated image for aspara %d", aspara_info.id);
    cv::Mat annotated_image = image.clone();
    
    // バウンディングボックスを描画
    cv::Scalar bbox_color = is_harvestable ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255); // 収穫可能：緑、不可：赤
    cv::rectangle(annotated_image, aspara_info.bounding_box_2d, bbox_color, 3);
    
    // 基本的なテキスト描画のみ
    std::string info_text = cv::format("ID:%d %.2f", aspara_info.id, aspara_info.confidence);
    cv::putText(annotated_image, info_text, cv::Point(aspara_info.bounding_box_2d.x, aspara_info.bounding_box_2d.y - 10), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, bbox_color, 2);
    
    std::string status_text = is_harvestable ? "収穫可能" : "未成熟";
    std::string status_info = cv::format("L:%.1fcm S:%.2f %s", length * 100, straightness, status_text.c_str());
    cv::putText(annotated_image, status_info, cv::Point(aspara_info.bounding_box_2d.x, aspara_info.bounding_box_2d.y + aspara_info.bounding_box_2d.height + 25), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, bbox_color, 2);
    
    // 注釈付き画像をパブリッシュ
    try {
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            sensor_msgs::image_encodings::BGR8,
            annotated_image).toImageMsg();
        
        annotated_image_pub_->publish(*msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "CV bridge exception in publishAnnotatedImage: %s", e.what());
    }
}

} // namespace fv_aspara_analyzer