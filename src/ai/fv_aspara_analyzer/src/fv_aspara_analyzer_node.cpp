/**
 * @file fv_aspara_analyzer_node.cpp
 * @brief アスパラガス解析ノードのメイン実装ファイル
 * @details 3D点群データと2D検出結果を統合してアスパラガスの品質評価を行う
 * @author Takashi Otsuka
 * @date 2024
 * @version 1.0
 */

#include "fv_aspara_analyzer/fv_aspara_analyzer_node.hpp"

namespace fv_aspara_analyzer
{

/**
 * @brief コンストラクタ
 * @details ノードの初期化、パラメータ読み込み、トピックの設定を行う
 * 
 * 初期化内容：
 * - ROS2パラメータの宣言と取得
 * - サブスクライバー・パブリッシャーの作成
 * - TF2（座標変換）の初期化
 * - ログ出力の設定
 */
FvAsparaAnalyzerNode::FvAsparaAnalyzerNode() : Node("fv_aspara_analyzer")
{
    RCLCPP_WARN(this->get_logger(), "===== FV Aspara Analyzer Node Constructor START =====");
    
    // ===== パラメータ宣言 =====
    this->declare_parameter<double>("min_confidence", 0.5);                    // 最小信頼度閾値
    this->declare_parameter<double>("pointcloud_distance_min", 0.1);           // 点群処理最小距離（10cm）
    this->declare_parameter<double>("pointcloud_distance_max", 2.0);           // 点群処理最大距離（2m）
    this->declare_parameter<double>("aspara_filter_distance", 0.05);           // アスパラガスフィルタ距離（5cm）
    this->declare_parameter<int>("noise_reduction_neighbors", 50);             // 統計フィルタの近傍点数
    this->declare_parameter<double>("noise_reduction_std_dev", 1.0);           // 統計フィルタの標準偏差
    this->declare_parameter<double>("voxel_leaf_size", 0.005);                 // ボクセルサイズ（5mm）
    this->declare_parameter<double>("harvest_min_length", 0.23);               // 収穫最小長さ（23cm）
    this->declare_parameter<double>("harvest_max_length", 0.50);               // 収穫最大長さ（50cm）
    this->declare_parameter<double>("straightness_threshold", 0.7);            // 真っ直ぐ度閾値

    // ===== パラメータ取得 =====
    min_confidence_ = this->get_parameter("min_confidence").as_double();
    pointcloud_distance_min_ = this->get_parameter("pointcloud_distance_min").as_double();
    pointcloud_distance_max_ = this->get_parameter("pointcloud_distance_max").as_double();
    aspara_filter_distance_ = this->get_parameter("aspara_filter_distance").as_double();
    noise_reduction_neighbors_ = this->get_parameter("noise_reduction_neighbors").as_int();
    noise_reduction_std_dev_ = this->get_parameter("noise_reduction_std_dev").as_double();
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
    harvest_min_length_ = this->get_parameter("harvest_min_length").as_double();
    harvest_max_length_ = this->get_parameter("harvest_max_length").as_double();
    straightness_threshold_ = this->get_parameter("straightness_threshold").as_double();

    // ===== トピック名パラメータ宣言 =====
    // デフォルト値は空の文字列にして、設定ファイルから読み込む
    this->declare_parameter<std::string>("detection_topic", "");
    this->declare_parameter<std::string>("pointcloud_topic", "");
    this->declare_parameter<std::string>("camera_info_topic", "");
    this->declare_parameter<std::string>("mask_topic", "");
    this->declare_parameter<std::string>("camera_topic", "");
    this->declare_parameter<std::string>("output_filtered_pointcloud_topic", "");
    this->declare_parameter<std::string>("output_annotated_image_topic", "");

    // ===== トピック名取得 =====
    std::string detection_topic = this->get_parameter("detection_topic").as_string();
    std::string pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
    std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
    std::string mask_topic = this->get_parameter("mask_topic").as_string();
    std::string camera_topic = this->get_parameter("camera_topic").as_string();
    std::string output_filtered_pointcloud_topic = this->get_parameter("output_filtered_pointcloud_topic").as_string();
    std::string output_annotated_image_topic = this->get_parameter("output_annotated_image_topic").as_string();

    // ===== トピック設定ログ出力 =====
    RCLCPP_WARN(this->get_logger(), "Topic configuration:");
    RCLCPP_WARN(this->get_logger(), "  Detection: %s", detection_topic.c_str());
    RCLCPP_WARN(this->get_logger(), "  Pointcloud: %s", pointcloud_topic.c_str());
    RCLCPP_WARN(this->get_logger(), "  Camera info: %s", camera_info_topic.c_str());
    RCLCPP_WARN(this->get_logger(), "  Camera: %s", camera_topic.c_str());
    RCLCPP_WARN(this->get_logger(), "  Output annotated: %s", output_annotated_image_topic.c_str());

    // ===== サブスクライバー初期化 =====
    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        detection_topic, 10, 
        std::bind(&FvAsparaAnalyzerNode::detectionCallback, this, std::placeholders::_1));

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic, 10,
        std::bind(&FvAsparaAnalyzerNode::pointcloudCallback, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, 10,
        std::bind(&FvAsparaAnalyzerNode::cameraInfoCallback, this, std::placeholders::_1));

    mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        mask_topic, 10,
        std::bind(&FvAsparaAnalyzerNode::maskCallback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic, 10,
        std::bind(&FvAsparaAnalyzerNode::imageCallback, this, std::placeholders::_1));

    // ===== パブリッシャー初期化 =====
    filtered_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_filtered_pointcloud_topic, 10);
    
    annotated_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        output_annotated_image_topic, 10);

    // ===== TF2（座標変換）初期化 =====
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // ===== 初期化完了ログ =====
    RCLCPP_WARN(this->get_logger(), "All subscribers created successfully");
    RCLCPP_WARN(this->get_logger(), "All publishers created successfully");
    RCLCPP_WARN(this->get_logger(), "===== FV Aspara Analyzer Node Constructor END =====");
    RCLCPP_INFO(this->get_logger(), "FV Aspara Analyzer Node initialized");
}

/**
 * @brief デストラクタ
 * @details リソースの適切な解放を行う
 */
FvAsparaAnalyzerNode::~FvAsparaAnalyzerNode() {}

/**
 * @brief 2D検出結果のコールバック関数
 * @param msg 検出結果の配列メッセージ
 * @details YOLO等の物体検出結果を受信し、アスパラガス情報を更新
 * 
 * 処理内容：
 * - 信頼度フィルタリング
 * - バウンディングボックス抽出
 * - アスパラガス情報の作成
 * - 最高信頼度のアスパラガスを選択して処理
 */
void FvAsparaAnalyzerNode::detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    RCLCPP_WARN(this->get_logger(), "Detection callback received %zu detections", msg->detections.size());
    
    // 点群とカメラ情報の可用性チェック
    if (!latest_pointcloud_ || !latest_camera_info_) {
        RCLCPP_WARN(this->get_logger(), "Point cloud or camera info not available yet. PC:%s, CI:%s", 
                   latest_pointcloud_ ? "OK" : "NULL", latest_camera_info_ ? "OK" : "NULL");
        return;
    }

    // 前回のアスパラガスリストをクリア
    aspara_list_.clear();

    // 各検出結果を処理
    for (const auto& detection : msg->detections) {
        if (detection.results.empty()) continue;
        
        // 信頼度フィルタリング
        float confidence = detection.results[0].hypothesis.score;
        if (confidence < min_confidence_) continue;

        // バウンディングボックス抽出
        cv::Rect bbox(
            static_cast<int>(detection.bbox.center.position.x - detection.bbox.size_x / 2),
            static_cast<int>(detection.bbox.center.position.y - detection.bbox.size_y / 2),
            static_cast<int>(detection.bbox.size_x),
            static_cast<int>(detection.bbox.size_y)
        );

        // アスパラガス情報を作成
        AsparaInfo aspara_info;
        aspara_info.id = static_cast<int>(aspara_list_.size());
        aspara_info.confidence = confidence;
        aspara_info.bounding_box_2d = bbox;
        aspara_info.last_update_time = this->now();

        aspara_list_.push_back(aspara_info);
    }

    // 最高信頼度のアスパラガスを処理
    if (!aspara_list_.empty()) {
        // 信頼度でソート
        std::sort(aspara_list_.begin(), aspara_list_.end(),
                  [](const AsparaInfo& a, const AsparaInfo& b) {
                      return a.confidence > b.confidence;
                  });
        
        processAsparagus(aspara_list_[0]);
    }
}

/**
 * @brief 3D点群データのコールバック関数
 * @param msg 点群データメッセージ
 * @details RealSense等からの3D点群を受信し、解析用データとして保存
 */
void FvAsparaAnalyzerNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_WARN_ONCE(this->get_logger(), "Pointcloud callback received first pointcloud");
    latest_pointcloud_ = msg;
}

/**
 * @brief カメラ情報のコールバック関数
 * @param msg カメラキャリブレーション情報
 * @details カメラの内部パラメータを受信し、3D-2D変換に使用
 */
void FvAsparaAnalyzerNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    RCLCPP_WARN_ONCE(this->get_logger(), "Camera info callback received first camera info");
    latest_camera_info_ = msg;
}

/**
 * @brief マスク画像のコールバック関数
 * @param msg セグメンテーションマスク画像
 * @details アスパラガス領域のマスクを受信し、精度向上に使用
 */
void FvAsparaAnalyzerNode::maskCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        latest_mask_ = cv_ptr->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
    }
}

/**
 * @brief カラー画像のコールバック関数
 * @param msg カラー画像メッセージ
 * @details 可視化用のカラー画像を受信
 */
void FvAsparaAnalyzerNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_WARN_ONCE(this->get_logger(), "Image callback received first image");
    latest_color_image_ = msg;
}

/**
 * @brief 3D点を2D画像座標に投影
 * @param point 3D点
 * @param camera_info カメラ情報
 * @return 2D画像座標
 * @details カメラの内部パラメータを使用して3D-2D変換
 * 
 * 変換処理：
 * - カメラ内部パラメータ（fx, fy, cx, cy）を使用
 * - レンズ歪み補正を適用（k1, k2, p1, p2, k3）
 * - 正規化座標での歪み補正
 */
cv::Point2f FvAsparaAnalyzerNode::project3DTo2D(
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

/**
 * @brief バウンディングボックスで点群をフィルタリング
 * @param input_cloud 入力点群
 * @param bbox 2Dバウンディングボックス
 * @param camera_info カメラ情報
 * @return フィルタリング済み点群
 * @details 2D検出領域に対応する3D点群を抽出
 * 
 * フィルタリング処理：
 * - 無効点の除去（NaN、無限大値）
 * - 距離フィルタリング（設定範囲内）
 * - 3D-2D投影によるバウンディングボックス内の点抽出
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr FvAsparaAnalyzerNode::filterPointCloudByBoundingBox(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
    const cv::Rect& bbox,
    const sensor_msgs::msg::CameraInfo& camera_info)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (const auto& point : input_cloud->points) {
        // 無効点をスキップ
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }

        // 距離フィルタリング
        float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (distance < pointcloud_distance_min_ || distance > pointcloud_distance_max_) {
            continue;
        }

        // 2Dに投影
        cv::Point2f projected = project3DTo2D(point, camera_info);

        // バウンディングボックス内かチェック
        if (bbox.contains(cv::Point(static_cast<int>(projected.x), static_cast<int>(projected.y)))) {
            filtered_cloud->points.push_back(point);
        }
    }

    // 点群のメタデータを設定
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = false;

    return filtered_cloud;
}

/**
 * @brief 点群のノイズ除去処理
 * @param input_cloud 入力点群
 * @return ノイズ除去済み点群
 * @details 統計的外れ値除去とボクセルグリッドフィルタを適用
 * 
 * ノイズ除去処理：
 * - ボクセルグリッドフィルタ（ダウンサンプリング）
 * - 統計的外れ値除去（ノイズ除去）
 * - 点群密度に応じた適応的処理
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr FvAsparaAnalyzerNode::applyNoiseReduction(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud)
{
    // 点群サイズチェック
    if (input_cloud->points.size() < 10) {
        RCLCPP_WARN(this->get_logger(), "Too few points for noise reduction");
        return input_cloud;
    }

    // ステップ1: ボクセルグリッドフィルタ（ダウンサンプリング）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_filter.filter(*voxel_filtered);

    if (voxel_filtered->points.size() < 10) {
        RCLCPP_WARN(this->get_logger(), "Too few points after voxel filtering");
        return voxel_filtered;
    }

    // ステップ2: 統計的外れ値除去
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr denoised_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
    statistical_filter.setInputCloud(voxel_filtered);
    statistical_filter.setMeanK(noise_reduction_neighbors_);
    statistical_filter.setStddevMulThresh(noise_reduction_std_dev_);
    statistical_filter.filter(*denoised_cloud);

    // デバッグログ
    RCLCPP_DEBUG(this->get_logger(), 
                 "Noise reduction: %zu -> %zu -> %zu points",
                 input_cloud->points.size(),
                 voxel_filtered->points.size(),
                 denoised_cloud->points.size());

    return denoised_cloud;
}

/**
 * @brief アスパラガスの根元位置を推定
 * @param aspara_cloud アスパラガス点群
 * @return 根元の3D座標
 * @details 点群の下端部から根元位置を推定
 * 
 * 推定方法：
 * - Z座標の最小値を根元位置として使用
 * - 地面との交点計算は将来的に実装予定
 */
geometry_msgs::msg::Point FvAsparaAnalyzerNode::estimateRootPosition(
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

/**
 * @brief アスパラガスの真っ直ぐ度を計算
 * @param aspara_cloud アスパラガス点群
 * @return 真っ直ぐ度（0.0-1.0）
 * @details PCAの第1主成分の分散から直線性を評価
 * 
 * 計算方法：
 * - 主成分分析（PCA）を実行
 * - 第1主成分の固有値の比率を計算
 * - 高値ほど直線的（真っ直ぐ）
 */
float FvAsparaAnalyzerNode::calculateStraightness(
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

/**
 * @brief アスパラガスの長さを計算
 * @param aspara_cloud アスパラガス点群
 * @return 長さ（メートル）
 * @details 点群の主軸方向の最大距離を計算
 * 
 * 計算方法：
 * - Z軸方向の最大・最小値を取得
 * - その差を長さとして使用
 * - 将来的には曲線長の計算を実装予定
 */
float FvAsparaAnalyzerNode::calculateLength(
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

/**
 * @brief PCA直線を生成
 * @param aspara_cloud アスパラガス点群
 * @return PCA直線の点群
 * @details 主成分分析による直線近似を生成
 * 
 * 生成処理：
 * - PCAで主成分軸を計算
 * - データの投影範囲を取得
 * - 直線上に等間隔でポイントを生成
 * - 可視化用のデバッグデータとして使用
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr FvAsparaAnalyzerNode::generatePCALine(
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

/**
 * @brief アスパラガス情報の処理
 * @param aspara_info アスパラガス情報
 * @details 品質評価と収穫判定の実行
 * 
 * 処理フロー：
 * - 点群フィルタリング
 * - ノイズ除去
 * - 品質評価（長さ、真っ直ぐ度）
 * - 収穫適性判定
 * - 結果の可視化とパブリッシュ
 */
void FvAsparaAnalyzerNode::processAsparagus(const AsparaInfo& aspara_info)
{
    RCLCPP_WARN(this->get_logger(), "Processing asparagus %d with confidence %.3f", aspara_info.id, aspara_info.confidence);
    
    // 全体処理時間の計測開始
    Stopwatch total_stopwatch;
    
    // mutableなコピーを作成して処理時間を記録できるようにする
    AsparaInfo mutable_aspara_info = aspara_info;
    
    if (!latest_color_image_) {
        RCLCPP_WARN(this->get_logger(), "Color image not available yet");
        return;
    }

    // ROS点群をPCL形式に変換
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*latest_pointcloud_, *pcl_cloud);

    // バウンディングボックスで点群フィルタリング
    Stopwatch filter_stopwatch;
    auto filtered_cloud = filterPointCloudByBoundingBox(pcl_cloud, aspara_info.bounding_box_2d, *latest_camera_info_);
    mutable_aspara_info.processing_times.filter_bbox_ms = filter_stopwatch.elapsed_ms();
    
    if (filtered_cloud->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "No points found in bounding box");
        return;
    }

    // ノイズ除去
    Stopwatch noise_stopwatch;
    auto denoised_cloud = applyNoiseReduction(filtered_cloud);
    mutable_aspara_info.processing_times.noise_reduction_ms = noise_stopwatch.elapsed_ms();

    if (denoised_cloud->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "No points left after noise reduction");
        return;
    }

    // アスパラガス特性を計算
    Stopwatch measurement_stopwatch;
    auto root_position = estimateRootPosition(denoised_cloud);
    float straightness = calculateStraightness(denoised_cloud);
    float length = calculateLength(denoised_cloud);
    mutable_aspara_info.processing_times.measurement_ms = measurement_stopwatch.elapsed_ms();
    
    // 可視化用PCAラインを生成
    Stopwatch pca_stopwatch;
    auto pca_line = generatePCALine(denoised_cloud);
    mutable_aspara_info.processing_times.pca_calculation_ms = pca_stopwatch.elapsed_ms();

    // 収穫適性を判定
    bool is_harvestable = (length >= harvest_min_length_ && 
                          length <= harvest_max_length_ && 
                          straightness >= straightness_threshold_);

    // カラー画像をcv::Matに変換
    cv::Mat color_image;
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(latest_color_image_, sensor_msgs::image_encodings::BGR8);
        color_image = cv_ptr->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        return;
    }

    // 可視化処理時間の計測
    Stopwatch vis_stopwatch;
    
    // 結果をパブリッシュ
    publishFilteredPointCloud(denoised_cloud, latest_pointcloud_->header.frame_id, aspara_info.id);
    publishRootTF(root_position, latest_pointcloud_->header.frame_id, aspara_info.id);
    
    // 全体処理時間を記録
    mutable_aspara_info.processing_times.total_ms = total_stopwatch.elapsed_ms();
    mutable_aspara_info.processing_times.visualization_ms = vis_stopwatch.elapsed_ms();
    
    publishAnnotatedImage(color_image, mutable_aspara_info, denoised_cloud, pca_line, length, straightness, is_harvestable);

    // 結果ログ出力（処理時間含む）
    RCLCPP_INFO(this->get_logger(), 
                "Aspara ID:%d, Confidence:%.2f, Length:%.3fm, Straightness:%.2f, Harvestable:%s, Total:%.1fms (Filter:%.1fms, Noise:%.1fms, PCA:%.1fms)",
                aspara_info.id, aspara_info.confidence, length, straightness, 
                is_harvestable ? "YES" : "NO",
                mutable_aspara_info.processing_times.total_ms,
                mutable_aspara_info.processing_times.filter_bbox_ms,
                mutable_aspara_info.processing_times.noise_reduction_ms,
                mutable_aspara_info.processing_times.pca_calculation_ms);
}

/**
 * @brief フィルタリング済み点群をパブリッシュ
 * @param cloud 点群データ
 * @param frame_id 座標系ID
 * @param aspara_id アスパラガスID
 * @details デバッグ用の可視化データを出力
 */
void FvAsparaAnalyzerNode::publishFilteredPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const std::string& frame_id,
    int aspara_id)
{
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    output_msg.header.stamp = this->now();
    output_msg.header.frame_id = frame_id;
    
    filtered_pointcloud_pub_->publish(output_msg);
}

/**
 * @brief 根元位置のTF座標をパブリッシュ
 * @param root_position 根元位置
 * @param frame_id 座標系ID
 * @param aspara_id アスパラガスID
 * @details ロボット制御用の座標変換情報を出力
 */
void FvAsparaAnalyzerNode::publishRootTF(
    const geometry_msgs::msg::Point& root_position,
    const std::string& frame_id,
    int aspara_id)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
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

/**
 * @brief 注釈付き画像をパブリッシュ
 * @param image 元画像
 * @param aspara_info アスパラガス情報
 * @param filtered_cloud フィルタリング済み点群
 * @param pca_line_cloud PCA直線点群
 * @param length 長さ
 * @param straightness 真っ直ぐ度
 * @param is_harvestable 収穫可能フラグ
 * @details デバッグ用の可視化画像を生成・出力
 * 
 * 可視化内容：
 * - バウンディングボックス（収穫可能：緑、不可：赤）
 * - フィルタリング済み点群（シアン色）
 * - PCA直線（ピンク色）
 * - 情報テキスト（ID、信頼度、長さ、真っ直ぐ度、収穫適性）
 */
void FvAsparaAnalyzerNode::publishAnnotatedImage(
    const cv::Mat& image,
    const AsparaInfo& aspara_info,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pca_line_cloud,
    float length,
    float straightness,
    bool is_harvestable)
{
    RCLCPP_WARN(this->get_logger(), "Publishing annotated image for aspara %d", aspara_info.id);
    cv::Mat annotated_image = image.clone();
    
    // バウンディングボックスを描画
    cv::Scalar bbox_color = is_harvestable ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255); // 収穫可能：緑、不可：赤
    cv::rectangle(annotated_image, aspara_info.bounding_box_2d, bbox_color, 3);
    
    // フィルタリング済み点群を画像上に描画
    if (!filtered_cloud->points.empty() && latest_camera_info_) {
        for (const auto& point : filtered_cloud->points) {
            cv::Point2f projected = project3DTo2D(point, *latest_camera_info_);
            if (projected.x >= 0 && projected.x < image.cols && projected.y >= 0 && projected.y < image.rows) {
                cv::circle(annotated_image, cv::Point(projected.x, projected.y), 2, cv::Scalar(255, 255, 0), -1); // シアン色の点
            }
        }
    }
    
    // PCA直線をピンク色で描画
    if (!pca_line_cloud->points.empty() && latest_camera_info_ && pca_line_cloud->points.size() > 1) {
        std::vector<cv::Point> line_points_2d;
        
        // 全PCA直線点を2Dに投影
        for (const auto& point : pca_line_cloud->points) {
            // pcl::PointXYZをpcl::PointXYZRGBに変換（投影関数用）
            pcl::PointXYZRGB rgb_point;
            rgb_point.x = point.x;
            rgb_point.y = point.y; 
            rgb_point.z = point.z;
            
            cv::Point2f projected = project3DTo2D(rgb_point, *latest_camera_info_);
            if (projected.x >= 0 && projected.x < image.cols && projected.y >= 0 && projected.y < image.rows) {
                line_points_2d.push_back(cv::Point(projected.x, projected.y));
            }
        }
        
        // 接続されたピンク色の線分を描画
        cv::Scalar pink_color(255, 0, 255); // ピンク色（BGR形式）
        for (size_t i = 1; i < line_points_2d.size(); ++i) {
            cv::line(annotated_image, line_points_2d[i-1], line_points_2d[i], pink_color, 3);
        }
    }
    
    // テキスト情報を準備（日本語）
    std::string status_text = is_harvestable ? "収穫可能" : "未成熟";
    cv::Scalar text_color = is_harvestable ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    
    // 情報テキストを描画
    int y_offset = aspara_info.bounding_box_2d.y - 10;
    if (y_offset < 30) y_offset = aspara_info.bounding_box_2d.y + aspara_info.bounding_box_2d.height + 30;
    
    putTextJP(annotated_image, 
              cv::format("ID: %d | 信頼度: %.2f", aspara_info.id, aspara_info.confidence),
              cv::Point(aspara_info.bounding_box_2d.x, y_offset), 
              14.0, text_color, 2);
    
    putTextJP(annotated_image, 
              cv::format("長さ: %.1fcm | 真っ直ぐ度: %.2f", length * 100, straightness),
              cv::Point(aspara_info.bounding_box_2d.x, y_offset + 25), 
              14.0, text_color, 2);
                
    putTextJP(annotated_image, 
              status_text,
              cv::Point(aspara_info.bounding_box_2d.x, y_offset + 50), 
              16.0, text_color, 2);
    
    // 点群統計情報を追加
    putTextJP(annotated_image, 
              cv::format("点群数: %zu", filtered_cloud->points.size()),
              cv::Point(aspara_info.bounding_box_2d.x, y_offset + 75), 
              12.0, cv::Scalar(255, 255, 255), 2);
    
    // 処理時間情報を画面上部に表示（半透明の背景）
    cv::Mat overlay = annotated_image.clone();
    cv::Scalar time_bg_color(0, 0, 0); // 黒背景
    cv::Scalar time_text_color(255, 255, 255); // 白文字
    
    // 背景の矩形を描画（半透明にするためにオーバーレイを使用）
    cv::rectangle(overlay, 
                 cv::Point(5, 5), 
                 cv::Point(420, 125), 
                 time_bg_color, 
                 cv::FILLED);
    
    // 半透明のブレンド（alpha=0.7）
    cv::addWeighted(overlay, 0.7, annotated_image, 0.3, 0, annotated_image);
    
    // 処理時間情報を表示（日本語版）
    int time_y = 25;
    putTextJP(annotated_image, 
              "=== 処理時間計測 ===",
              cv::Point(10, time_y), 
              12.0, cv::Scalar(0, 255, 255), 2); // 黄色で強調
    
    time_y += 25;
    // 全体時間を大きく表示（日本語）
    putTextJP(annotated_image, 
              cv::format("合計: %.1f ms (%.1f FPS)", 
                        aspara_info.processing_times.total_ms,
                        1000.0 / aspara_info.processing_times.total_ms),
              cv::Point(10, time_y), 
              14.0, cv::Scalar(0, 255, 0), 2); // 緑色で全体時間
    
    time_y += 22;
    // 各処理の内訳を表示（日本語）
    putTextJP(annotated_image, 
              cv::format("領域抽出: %.1fms, ノイズ除去: %.1fms", 
                        aspara_info.processing_times.filter_bbox_ms,
                        aspara_info.processing_times.noise_reduction_ms),
              cv::Point(10, time_y), 
              9.0, time_text_color, 1);
    
    time_y += 18;
    putTextJP(annotated_image, 
              cv::format("PCA解析: %.1fms, 測定: %.1fms", 
                        aspara_info.processing_times.pca_calculation_ms,
                        aspara_info.processing_times.measurement_ms),
              cv::Point(10, time_y), 
              9.0, time_text_color, 1);
    
    time_y += 18;
    // 点群のポイント数を赤色で警告的に表示（多い場合）
    cv::Scalar points_color = filtered_cloud->points.size() > 500 ? cv::Scalar(0, 0, 255) : time_text_color;
    putTextJP(annotated_image, 
              cv::format("点群数: %zu", filtered_cloud->points.size()),
              cv::Point(10, time_y), 
              9.0, points_color, 1);
    
    // 注釈付き画像をパブリッシュ
    try {
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
            latest_color_image_->header,
            sensor_msgs::image_encodings::BGR8,
            annotated_image).toImageMsg();
        
        annotated_image_pub_->publish(*msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge exception in publishAnnotatedImage: %s", e.what());
    }
}

} // namespace fv_aspara_analyzer

/**
 * @brief メイン関数
 * @param argc コマンドライン引数の数
 * @param argv コマンドライン引数の配列
 * @return 終了コード
 * @details ROS2ノードの初期化と実行
 * 
 * 実行内容：
 * - ROS2の初期化
 * - アスパラガス解析ノードの作成
 * - ノードの実行（スピン）
 * - 適切な終了処理
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto logger = rclcpp::get_logger("fv_aspara_analyzer_main");
    RCLCPP_WARN(logger, "===== Starting FV Aspara Analyzer Node Main =====");
    
    try {
        auto node = std::make_shared<fv_aspara_analyzer::FvAsparaAnalyzerNode>();
        RCLCPP_WARN(logger, "Node created, starting spin...");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Exception in main: %s", e.what());
    }
    
    RCLCPP_WARN(logger, "===== Shutting down FV Aspara Analyzer Node =====");
    rclcpp::shutdown();
    return 0;
}