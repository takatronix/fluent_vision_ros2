#include "fv_aspara_analyzer/aspara_pointcloud_processor.hpp"
#include "fv_aspara_analyzer/fv_aspara_analyzer_node.hpp"
#include <fluent.hpp>

namespace fv_aspara_analyzer {

AsparaPointcloudProcessor::AsparaPointcloudProcessor(FvAsparaAnalyzerNode* node_ptr)
    : node_(node_ptr)
{
    // パブリッシャー初期化
    filtered_pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "output_filtered_pointcloud", 10);
    annotated_image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
        "output_annotated_image", 10);
    
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
    // カメラ内部パラメータ
    double fx = camera_info.k[0];  // K[0,0] - 焦点距離X
    double fy = camera_info.k[4];  // K[1,1] - 焦点距離Y
    double cx = camera_info.k[2];  // K[0,2] - 光学中心X
    double cy = camera_info.k[5];  // K[1,2] - 光学中心Y

    // 3D点を2Dに投影
    double u = (point.x * fx / point.z) + cx;
    double v = (point.y * fy / point.z) + cy;

    // レンズ歪み補正（必要に応じて）
    if (!camera_info.d.empty()) {
        double k1 = camera_info.d[0];  // 半径方向歪み係数1
        double k2 = camera_info.d[1];  // 半径方向歪み係数2
        double p1 = camera_info.d[2];  // 接線方向歪み係数1
        double p2 = camera_info.d[3];  // 接線方向歪み係数2
        double k3 = (camera_info.d.size() > 4) ? camera_info.d[4] : 0.0;  // 半径方向歪み係数3

        // 正規化座標
        double x_norm = (u - cx) / fx;
        double y_norm = (v - cy) / fy;
        
        double r2 = x_norm * x_norm + y_norm * y_norm;  // 半径の2乗
        double r4 = r2 * r2;                            // 半径の4乗
        double r6 = r4 * r2;                            // 半径の6乗

        // 半径方向歪み
        double radial_distortion = 1 + k1 * r2 + k2 * r4 + k3 * r6;

        // 接線方向歪み
        double dx = 2 * p1 * x_norm * y_norm + p2 * (r2 + 2 * x_norm * x_norm);
        double dy = p1 * (r2 + 2 * y_norm * y_norm) + 2 * p2 * x_norm * y_norm;

        // 歪み補正を適用
        x_norm = x_norm * radial_distortion + dx;
        y_norm = y_norm * radial_distortion + dy;

        // ピクセル座標に戻す
        u = x_norm * fx + cx;
        v = y_norm * fy + cy;
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

void AsparaPointcloudProcessor::publishRootTF(
    const geometry_msgs::msg::Point& root_position,
    const std::string& frame_id,
    int aspara_id)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = node_->now();
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = "aspara_" + std::to_string(aspara_id) + "_root";
    
    transform_stamped.transform.translation.x = root_position.x;
    transform_stamped.transform.translation.y = root_position.y;
    transform_stamped.transform.translation.z = root_position.z;
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;
    
    tf_broadcaster_->sendTransform(transform_stamped);
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