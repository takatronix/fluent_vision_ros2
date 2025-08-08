#ifndef FLUENT_CLOUD_VISUALIZATION_POINT_CLOUD_VIEWER_HPP
#define FLUENT_CLOUD_VISUALIZATION_POINT_CLOUD_VIEWER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace fluent_cloud {
namespace visualization {

/**
 * @brief Foxglove風の点群ビューワー機能
 * 既存のノードに組み込んで使用可能
 */
class PointCloudViewer {
public:
    struct ViewerConfig {
        // 表示設定
        int image_width = 1280;
        int image_height = 720;
        float point_size = 3.0f;
        float fov = 60.0f;  // 視野角（度）
        
        // カメラ設定
        float camera_distance = 2.0f;
        float camera_pitch = -30.0f;  // 俯角
        float camera_yaw = 0.0f;
        
        // 色設定
        enum ColorMode {
            COLOR_BY_HEIGHT,    // 高さで色分け
            COLOR_BY_DISTANCE,  // 距離で色分け
            COLOR_BY_INTENSITY, // 輝度で色分け
            COLOR_RGB,          // RGB色
            COLOR_SELECTED      // 選択状態で色分け
        } color_mode = COLOR_BY_HEIGHT;
        
        // グリッド設定
        bool show_grid = true;
        float grid_size = 0.1f;  // 10cm
        int grid_count = 20;
        
        // 情報表示
        bool show_info = true;
        bool show_axes = true;
        bool show_selection = true;
    };
    
    PointCloudViewer(rclcpp::Node* node, const std::string& name_prefix = "viewer") 
        : node_(node), name_prefix_(name_prefix) {
        
        // パブリッシャー作成
        image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
            name_prefix + "/image", 10);
            
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            name_prefix + "/markers", 10);
            
        // マウスクリックサブスクライバー
        click_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
            name_prefix + "/clicked_point", 10,
            [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                handleClick(msg->point.x, msg->point.y);
            });
    }
    
    /**
     * @brief 点群を2D画像として描画（Foxglove風）
     */
    void renderPointCloud(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
        const ViewerConfig& config = ViewerConfig(),
        int selected_index = -1) {
        
        // キャンバス作成
        cv::Mat image(config.image_height, config.image_width, CV_8UC3, cv::Scalar(20, 20, 20));
        
        // カメラ行列計算
        Eigen::Matrix4f view_matrix = calculateViewMatrix(config);
        Eigen::Matrix4f proj_matrix = calculateProjectionMatrix(config);
        
        // グリッド描画
        if (config.show_grid) {
            drawGrid(image, view_matrix, proj_matrix, config);
        }
        
        // 座標軸描画
        if (config.show_axes) {
            drawAxes(image, view_matrix, proj_matrix);
        }
        
        // 点群描画
        std::vector<ProjectedPoint> projected_points;
        projectPointCloud(cloud, view_matrix, proj_matrix, projected_points);
        
        // Zバッファソート（奥から手前）
        std::sort(projected_points.begin(), projected_points.end(),
                  [](const ProjectedPoint& a, const ProjectedPoint& b) {
                      return a.depth > b.depth;
                  });
        
        // 点描画
        for (const auto& pp : projected_points) {
            cv::Scalar color = getPointColor(pp, config);
            
            // 選択アスパラガスは強調
            if (selected_index >= 0 && pp.cluster_id == selected_index) {
                cv::circle(image, cv::Point(pp.screen_x, pp.screen_y), 
                          config.point_size * 1.5, cv::Scalar(0, 255, 0), -1);
            } else {
                cv::circle(image, cv::Point(pp.screen_x, pp.screen_y), 
                          config.point_size, color, -1);
            }
        }
        
        // 情報表示
        if (config.show_info) {
            drawInfo(image, cloud->size(), config);
        }
        
        // 画像パブリッシュ
        publishImage(image);
        
        // 3Dマーカーも更新
        if (config.show_selection) {
            publishSelectionMarkers(cloud, selected_index);
        }
    }
    
    /**
     * @brief 複数の点群を異なる色で表示
     */
    void renderMultiplePointClouds(
        const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clouds,
        const std::vector<cv::Scalar>& colors,
        const ViewerConfig& config = ViewerConfig()) {
        
        cv::Mat image(config.image_height, config.image_width, CV_8UC3, cv::Scalar(20, 20, 20));
        
        Eigen::Matrix4f view_matrix = calculateViewMatrix(config);
        Eigen::Matrix4f proj_matrix = calculateProjectionMatrix(config);
        
        if (config.show_grid) {
            drawGrid(image, view_matrix, proj_matrix, config);
        }
        
        // 全点群を投影してZソート
        std::vector<ProjectedPoint> all_points;
        
        for (size_t i = 0; i < clouds.size(); ++i) {
            std::vector<ProjectedPoint> projected;
            projectPointCloud(clouds[i], view_matrix, proj_matrix, projected);
            
            // クラスタIDを設定
            for (auto& pp : projected) {
                pp.cluster_id = i;
                pp.custom_color = colors[i];
            }
            
            all_points.insert(all_points.end(), projected.begin(), projected.end());
        }
        
        // Zソート
        std::sort(all_points.begin(), all_points.end(),
                  [](const ProjectedPoint& a, const ProjectedPoint& b) {
                      return a.depth > b.depth;
                  });
        
        // 描画
        for (const auto& pp : all_points) {
            cv::circle(image, cv::Point(pp.screen_x, pp.screen_y), 
                      config.point_size, pp.custom_color, -1);
        }
        
        publishImage(image);
    }
    
    /**
     * @brief インタラクティブビュー（マウス操作対応）
     */
    class InteractiveViewer {
    public:
        InteractiveViewer(PointCloudViewer& viewer) : viewer_(viewer) {}
        
        void startInteractive(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
            cloud_ = cloud;
            config_ = ViewerConfig();
            
            // タイマーで定期更新
            timer_ = viewer_.node_->create_wall_timer(
                std::chrono::milliseconds(33),  // 30fps
                [this]() { update(); });
        }
        
        void onMouseDrag(float dx, float dy) {
            config_.camera_yaw += dx * 0.5f;
            config_.camera_pitch = std::max(-89.0f, std::min(89.0f, config_.camera_pitch + dy * 0.5f));
        }
        
        void onMouseWheel(float delta) {
            config_.camera_distance = std::max(0.1f, config_.camera_distance - delta * 0.1f);
        }
        
        void onKeyPress(char key) {
            switch (key) {
                case 'h': config_.color_mode = ViewerConfig::COLOR_BY_HEIGHT; break;
                case 'd': config_.color_mode = ViewerConfig::COLOR_BY_DISTANCE; break;
                case 'r': config_.color_mode = ViewerConfig::COLOR_RGB; break;
                case 'g': config_.show_grid = !config_.show_grid; break;
                case 'i': config_.show_info = !config_.show_info; break;
            }
        }
        
    private:
        PointCloudViewer& viewer_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
        ViewerConfig config_;
        rclcpp::TimerBase::SharedPtr timer_;
        
        void update() {
            if (cloud_) {
                viewer_.renderPointCloud(cloud_, config_);
            }
        }
    };
    
    InteractiveViewer createInteractiveViewer() {
        return InteractiveViewer(*this);
    }
    
private:
    rclcpp::Node* node_;
    std::string name_prefix_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
    
    struct ProjectedPoint {
        float screen_x, screen_y;
        float depth;
        float world_z;
        cv::Scalar color;
        cv::Scalar custom_color;
        int cluster_id = -1;
    };
    
    Eigen::Matrix4f calculateViewMatrix(const ViewerConfig& config) {
        // カメラ位置計算
        float pitch_rad = config.camera_pitch * M_PI / 180.0f;
        float yaw_rad = config.camera_yaw * M_PI / 180.0f;
        
        Eigen::Vector3f eye(
            config.camera_distance * cos(pitch_rad) * sin(yaw_rad),
            config.camera_distance * cos(pitch_rad) * cos(yaw_rad),
            config.camera_distance * sin(pitch_rad)
        );
        
        Eigen::Vector3f center(0, 0, 0);
        Eigen::Vector3f up(0, 0, 1);
        
        // ビュー行列（lookAt）
        Eigen::Vector3f f = (center - eye).normalized();
        Eigen::Vector3f s = f.cross(up).normalized();
        Eigen::Vector3f u = s.cross(f);
        
        Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
        view(0, 0) = s.x(); view(0, 1) = s.y(); view(0, 2) = s.z();
        view(1, 0) = u.x(); view(1, 1) = u.y(); view(1, 2) = u.z();
        view(2, 0) = -f.x(); view(2, 1) = -f.y(); view(2, 2) = -f.z();
        view(0, 3) = -s.dot(eye);
        view(1, 3) = -u.dot(eye);
        view(2, 3) = f.dot(eye);
        
        return view;
    }
    
    Eigen::Matrix4f calculateProjectionMatrix(const ViewerConfig& config) {
        float aspect = static_cast<float>(config.image_width) / config.image_height;
        float fov_rad = config.fov * M_PI / 180.0f;
        float near_plane = 0.1f;
        float far_plane = 100.0f;
        
        float f = 1.0f / tan(fov_rad * 0.5f);
        
        Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
        proj(0, 0) = f / aspect;
        proj(1, 1) = f;
        proj(2, 2) = (far_plane + near_plane) / (near_plane - far_plane);
        proj(2, 3) = (2.0f * far_plane * near_plane) / (near_plane - far_plane);
        proj(3, 2) = -1.0f;
        
        return proj;
    }
    
    void projectPointCloud(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
        const Eigen::Matrix4f& view,
        const Eigen::Matrix4f& proj,
        std::vector<ProjectedPoint>& projected) {
        
        projected.clear();
        projected.reserve(cloud->size());
        
        for (const auto& point : cloud->points) {
            Eigen::Vector4f p(point.x, point.y, point.z, 1.0f);
            
            // ビュー変換
            Eigen::Vector4f view_pos = view * p;
            
            // プロジェクション変換
            Eigen::Vector4f proj_pos = proj * view_pos;
            
            if (proj_pos.w() > 0) {
                ProjectedPoint pp;
                pp.screen_x = (proj_pos.x() / proj_pos.w() + 1.0f) * 0.5f * config_.image_width;
                pp.screen_y = (1.0f - proj_pos.y() / proj_pos.w()) * 0.5f * config_.image_height;
                pp.depth = -view_pos.z();  // カメラ空間でのZ
                pp.world_z = point.z;
                pp.color = cv::Scalar(point.b, point.g, point.r);
                
                if (pp.screen_x >= 0 && pp.screen_x < config_.image_width &&
                    pp.screen_y >= 0 && pp.screen_y < config_.image_height) {
                    projected.push_back(pp);
                }
            }
        }
    }
    
    cv::Scalar getPointColor(const ProjectedPoint& pp, const ViewerConfig& config) {
        switch (config.color_mode) {
            case ViewerConfig::COLOR_BY_HEIGHT: {
                // 高さで色分け（青→緑→赤）
                float normalized_z = (pp.world_z + 0.5f) / 2.0f;  // -0.5～1.5mを0～1に
                normalized_z = std::max(0.0f, std::min(1.0f, normalized_z));
                
                if (normalized_z < 0.5f) {
                    return cv::Scalar(255 * (1 - 2 * normalized_z), 0, 255 * 2 * normalized_z);
                } else {
                    return cv::Scalar(0, 255 * 2 * (normalized_z - 0.5f), 255 * (1 - normalized_z));
                }
            }
            
            case ViewerConfig::COLOR_BY_DISTANCE: {
                // 距離で色分け
                float normalized_d = pp.depth / 3.0f;  // 0-3mを0-1に
                normalized_d = std::max(0.0f, std::min(1.0f, normalized_d));
                return cv::Scalar(255 * normalized_d, 255 * (1 - normalized_d), 0);
            }
            
            case ViewerConfig::COLOR_RGB:
            default:
                return pp.custom_color.val[0] > 0 ? pp.custom_color : pp.color;
        }
    }
    
    void drawGrid(cv::Mat& image, const Eigen::Matrix4f& view, 
                  const Eigen::Matrix4f& proj, const ViewerConfig& config) {
        
        cv::Scalar grid_color(50, 50, 50);
        
        for (int i = -config.grid_count; i <= config.grid_count; ++i) {
            // X方向の線
            drawLine3D(image, 
                      Eigen::Vector3f(i * config.grid_size, -config.grid_count * config.grid_size, 0),
                      Eigen::Vector3f(i * config.grid_size, config.grid_count * config.grid_size, 0),
                      view, proj, grid_color);
            
            // Y方向の線
            drawLine3D(image,
                      Eigen::Vector3f(-config.grid_count * config.grid_size, i * config.grid_size, 0),
                      Eigen::Vector3f(config.grid_count * config.grid_size, i * config.grid_size, 0),
                      view, proj, grid_color);
        }
    }
    
    void drawAxes(cv::Mat& image, const Eigen::Matrix4f& view, const Eigen::Matrix4f& proj) {
        // X軸（赤）
        drawLine3D(image, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0.5, 0, 0),
                  view, proj, cv::Scalar(0, 0, 255), 2);
        
        // Y軸（緑）
        drawLine3D(image, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0.5, 0),
                  view, proj, cv::Scalar(0, 255, 0), 2);
        
        // Z軸（青）
        drawLine3D(image, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0.5),
                  view, proj, cv::Scalar(255, 0, 0), 2);
    }
    
    void drawLine3D(cv::Mat& image, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2,
                    const Eigen::Matrix4f& view, const Eigen::Matrix4f& proj,
                    const cv::Scalar& color, int thickness = 1) {
        
        cv::Point2f screen_p1 = projectPoint(p1, view, proj);
        cv::Point2f screen_p2 = projectPoint(p2, view, proj);
        
        cv::line(image, screen_p1, screen_p2, color, thickness);
    }
    
    cv::Point2f projectPoint(const Eigen::Vector3f& p, 
                            const Eigen::Matrix4f& view,
                            const Eigen::Matrix4f& proj) {
        Eigen::Vector4f p4(p.x(), p.y(), p.z(), 1.0f);
        Eigen::Vector4f view_pos = view * p4;
        Eigen::Vector4f proj_pos = proj * view_pos;
        
        if (proj_pos.w() > 0) {
            float x = (proj_pos.x() / proj_pos.w() + 1.0f) * 0.5f * config_.image_width;
            float y = (1.0f - proj_pos.y() / proj_pos.w()) * 0.5f * config_.image_height;
            return cv::Point2f(x, y);
        }
        
        return cv::Point2f(-1, -1);
    }
    
    void drawInfo(cv::Mat& image, size_t point_count, const ViewerConfig& config) {
        std::vector<std::string> info_lines;
        info_lines.push_back("Points: " + std::to_string(point_count));
        info_lines.push_back("View: " + std::to_string(int(config.camera_yaw)) + 
                           "," + std::to_string(int(config.camera_pitch)));
        info_lines.push_back("Color: " + getColorModeName(config.color_mode));
        
        int y = 30;
        for (const auto& line : info_lines) {
            cv::putText(image, line, cv::Point(10, y), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            y += 20;
        }
    }
    
    std::string getColorModeName(ViewerConfig::ColorMode mode) {
        switch (mode) {
            case ViewerConfig::COLOR_BY_HEIGHT: return "Height";
            case ViewerConfig::COLOR_BY_DISTANCE: return "Distance";
            case ViewerConfig::COLOR_BY_INTENSITY: return "Intensity";
            case ViewerConfig::COLOR_RGB: return "RGB";
            case ViewerConfig::COLOR_SELECTED: return "Selected";
            default: return "Unknown";
        }
    }
    
    void publishImage(const cv::Mat& image) {
        cv_bridge::CvImage cv_image;
        cv_image.header.stamp = node_->now();
        cv_image.header.frame_id = "viewer";
        cv_image.encoding = "bgr8";
        cv_image.image = image;
        
        image_pub_->publish(cv_image.toImageMsg());
    }
    
    void publishSelectionMarkers(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                int selected_index) {
        // TODO: 選択マーカーの3D表示
    }
    
    void handleClick(float x, float y) {
        RCLCPP_INFO(node_->get_logger(), "Clicked at: %.1f, %.1f", x, y);
        // TODO: クリック処理
    }
    
    ViewerConfig config_;
};

} // namespace visualization
} // namespace fluent_cloud

#endif // FLUENT_CLOUD_VISUALIZATION_POINT_CLOUD_VIEWER_HPP