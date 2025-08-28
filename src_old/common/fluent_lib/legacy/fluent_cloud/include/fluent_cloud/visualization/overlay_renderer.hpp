#ifndef FLUENT_CLOUD_VISUALIZATION_OVERLAY_RENDERER_HPP
#define FLUENT_CLOUD_VISUALIZATION_OVERLAY_RENDERER_HPP

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <Eigen/Core>

namespace fluent_cloud {
namespace visualization {

/**
 * @brief 点群を2D画像にオーバーレイレンダリングするユーティリティ
 */
class OverlayRenderer {
public:
    struct RenderConfig {
        // 表示設定
        float point_size = 2.0f;
        float line_thickness = 1.0f;
        float alpha = 0.7f;           // 透明度
        bool use_depth_color = true;  // 深度による色分け
        bool anti_aliasing = true;    // アンチエイリアス
        
        // 深度色設定
        float near_distance = 0.3f;   // 近距離閾値
        float far_distance = 2.0f;    // 遠距離閾値
        
        // パフォーマンス設定
        int point_skip = 1;           // 表示用間引き
        bool use_gpu = false;         // GPU使用（将来実装）
    };
    
    /**
     * @brief 点群を画像にオーバーレイ
     */
    static void overlayPointCloud(
        cv::Mat& image,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
        const sensor_msgs::msg::CameraInfo& camera_info,
        const RenderConfig& config = RenderConfig()) {
        
        if (!cloud || cloud->empty()) return;
        
        // カメラパラメータ取得
        float fx = camera_info.k[0];
        float fy = camera_info.k[4];
        float cx = camera_info.k[2];
        float cy = camera_info.k[5];
        
        // 点群を2Dに投影してソート（奥から手前）
        std::vector<ProjectedPoint> projected_points;
        projectPoints(cloud, fx, fy, cx, cy, projected_points);
        
        // Z値でソート（遠い順）
        std::sort(projected_points.begin(), projected_points.end(),
                  [](const ProjectedPoint& a, const ProjectedPoint& b) {
                      return a.z > b.z;
                  });
        
        // レンダリング
        for (const auto& pp : projected_points) {
            if (pp.x >= 0 && pp.x < image.cols && 
                pp.y >= 0 && pp.y < image.rows) {
                
                cv::Scalar color;
                if (config.use_depth_color) {
                    color = getDepthColor(pp.z, config);
                } else {
                    color = cv::Scalar(pp.b, pp.g, pp.r);
                }
                
                if (config.anti_aliasing) {
                    drawSmoothCircle(image, cv::Point2f(pp.x, pp.y), 
                                   config.point_size, color, config.alpha);
                } else {
                    drawCircle(image, cv::Point(pp.x, pp.y), 
                             config.point_size, color, config.alpha);
                }
            }
        }
    }
    
    /**
     * @brief 複数の点群を異なる色でオーバーレイ
     */
    static void overlayMultiplePointClouds(
        cv::Mat& image,
        const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clouds,
        const std::vector<cv::Scalar>& colors,
        const sensor_msgs::msg::CameraInfo& camera_info,
        const RenderConfig& config = RenderConfig()) {
        
        if (clouds.empty()) return;
        
        float fx = camera_info.k[0];
        float fy = camera_info.k[4];
        float cx = camera_info.k[2];
        float cy = camera_info.k[5];
        
        // 全点群を投影
        std::vector<ProjectedPoint> all_projected;
        
        for (size_t i = 0; i < clouds.size(); ++i) {
            if (!clouds[i] || clouds[i]->empty()) continue;
            
            std::vector<ProjectedPoint> projected;
            projectPoints(clouds[i], fx, fy, cx, cy, projected);
            
            // 色を設定
            cv::Scalar cloud_color = (i < colors.size()) ? 
                colors[i] : cv::Scalar(255, 255, 255);
            
            for (auto& pp : projected) {
                pp.cloud_id = i;
                pp.custom_color = cloud_color;
            }
            
            all_projected.insert(all_projected.end(), 
                               projected.begin(), projected.end());
        }
        
        // Zソート
        std::sort(all_projected.begin(), all_projected.end(),
                  [](const ProjectedPoint& a, const ProjectedPoint& b) {
                      return a.z > b.z;
                  });
        
        // レンダリング
        for (const auto& pp : all_projected) {
            if (pp.x >= 0 && pp.x < image.cols && 
                pp.y >= 0 && pp.y < image.rows) {
                
                drawSmoothCircle(image, cv::Point2f(pp.x, pp.y), 
                               config.point_size, pp.custom_color, config.alpha);
            }
        }
    }
    
    /**
     * @brief 3Dバウンディングボックスをオーバーレイ
     */
    static void overlay3DBoundingBox(
        cv::Mat& image,
        const Eigen::Vector3f& center,
        const Eigen::Vector3f& size,
        const sensor_msgs::msg::CameraInfo& camera_info,
        const cv::Scalar& color = cv::Scalar(0, 255, 0),
        float line_thickness = 2.0f) {
        
        float fx = camera_info.k[0];
        float fy = camera_info.k[4];
        float cx = camera_info.k[2];
        float cy = camera_info.k[5];
        
        // 8つの頂点を計算
        std::vector<Eigen::Vector3f> vertices;
        for (int i = 0; i < 8; ++i) {
            Eigen::Vector3f v;
            v.x() = center.x() + ((i & 1) ? size.x()/2 : -size.x()/2);
            v.y() = center.y() + ((i & 2) ? size.y()/2 : -size.y()/2);
            v.z() = center.z() + ((i & 4) ? size.z()/2 : -size.z()/2);
            vertices.push_back(v);
        }
        
        // 2Dに投影
        std::vector<cv::Point> points_2d;
        for (const auto& v : vertices) {
            if (v.z() > 0) {
                int x = fx * v.x() / v.z() + cx;
                int y = fy * v.y() / v.z() + cy;
                points_2d.push_back(cv::Point(x, y));
            }
        }
        
        if (points_2d.size() != 8) return;
        
        // エッジを描画
        const int edges[12][2] = {
            {0,1}, {1,3}, {3,2}, {2,0},  // 前面
            {4,5}, {5,7}, {7,6}, {6,4},  // 背面
            {0,4}, {1,5}, {2,6}, {3,7}   // 側面
        };
        
        for (const auto& edge : edges) {
            cv::line(image, points_2d[edge[0]], points_2d[edge[1]], 
                    color, line_thickness, cv::LINE_AA);
        }
    }
    
    /**
     * @brief ヒートマップオーバーレイ（密度表示）
     */
    static void overlayHeatmap(
        cv::Mat& image,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
        const sensor_msgs::msg::CameraInfo& camera_info,
        float kernel_size = 10.0f,
        float alpha = 0.5f) {
        
        if (!cloud || cloud->empty()) return;
        
        float fx = camera_info.k[0];
        float fy = camera_info.k[4];
        float cx = camera_info.k[2];
        float cy = camera_info.k[5];
        
        // 密度マップ作成
        cv::Mat density_map = cv::Mat::zeros(image.rows, image.cols, CV_32FC1);
        
        // 各点を投影して密度を計算
        for (const auto& point : cloud->points) {
            if (point.z > 0) {
                int x = fx * point.x / point.z + cx;
                int y = fy * point.y / point.z + cy;
                
                if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                    // ガウシアンカーネルで密度を加算
                    int ksize = kernel_size;
                    for (int dy = -ksize; dy <= ksize; ++dy) {
                        for (int dx = -ksize; dx <= ksize; ++dx) {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx >= 0 && nx < image.cols && 
                                ny >= 0 && ny < image.rows) {
                                float dist = sqrt(dx*dx + dy*dy);
                                float weight = exp(-dist*dist / (2*ksize*ksize/9));
                                density_map.at<float>(ny, nx) += weight;
                            }
                        }
                    }
                }
            }
        }
        
        // 正規化
        cv::normalize(density_map, density_map, 0, 1, cv::NORM_MINMAX);
        
        // カラーマップ適用
        cv::Mat heatmap;
        cv::applyColorMap(density_map * 255, heatmap, cv::COLORMAP_JET);
        
        // アルファブレンド
        for (int y = 0; y < image.rows; ++y) {
            for (int x = 0; x < image.cols; ++x) {
                float d = density_map.at<float>(y, x);
                if (d > 0.01) {  // 閾値
                    cv::Vec3b& pixel = image.at<cv::Vec3b>(y, x);
                    cv::Vec3b heat = heatmap.at<cv::Vec3b>(y, x);
                    float a = alpha * d;  // 密度に応じた透明度
                    pixel[0] = pixel[0] * (1 - a) + heat[0] * a;
                    pixel[1] = pixel[1] * (1 - a) + heat[1] * a;
                    pixel[2] = pixel[2] * (1 - a) + heat[2] * a;
                }
            }
        }
    }
    
private:
    struct ProjectedPoint {
        float x, y, z;
        uint8_t r, g, b;
        int cloud_id = -1;
        cv::Scalar custom_color;
    };
    
    static void projectPoints(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
        float fx, float fy, float cx, float cy,
        std::vector<ProjectedPoint>& projected) {
        
        projected.clear();
        projected.reserve(cloud->size());
        
        for (const auto& point : cloud->points) {
            if (point.z > 0.1 && point.z < 10.0) {  // 有効範囲
                ProjectedPoint pp;
                pp.x = fx * point.x / point.z + cx;
                pp.y = fy * point.y / point.z + cy;
                pp.z = point.z;
                pp.r = point.r;
                pp.g = point.g;
                pp.b = point.b;
                projected.push_back(pp);
            }
        }
    }
    
    static cv::Scalar getDepthColor(float depth, const RenderConfig& config) {
        if (depth < config.near_distance) {
            // 近距離：青
            return cv::Scalar(255, 0, 0);
        } else if (depth < config.far_distance) {
            // 中距離：緑→黄→赤のグラデーション
            float t = (depth - config.near_distance) / 
                     (config.far_distance - config.near_distance);
            
            if (t < 0.5) {
                // 緑→黄
                return cv::Scalar(0, 255, 255 * (t * 2));
            } else {
                // 黄→赤
                return cv::Scalar(0, 255 * (2 - t * 2), 255);
            }
        } else {
            // 遠距離：赤
            return cv::Scalar(0, 0, 255);
        }
    }
    
    static void drawCircle(cv::Mat& img, const cv::Point& center,
                          float radius, const cv::Scalar& color, float alpha) {
        cv::circle(img, center, radius, color, -1, cv::LINE_AA);
    }
    
    static void drawSmoothCircle(cv::Mat& img, const cv::Point2f& center,
                                float radius, const cv::Scalar& color, float alpha) {
        // サブピクセル精度での円描画
        int x_min = std::max(0, static_cast<int>(center.x - radius - 1));
        int x_max = std::min(img.cols - 1, static_cast<int>(center.x + radius + 1));
        int y_min = std::max(0, static_cast<int>(center.y - radius - 1));
        int y_max = std::min(img.rows - 1, static_cast<int>(center.y + radius + 1));
        
        float radius_sq = radius * radius;
        
        for (int y = y_min; y <= y_max; ++y) {
            for (int x = x_min; x <= x_max; ++x) {
                float dx = x - center.x;
                float dy = y - center.y;
                float dist_sq = dx * dx + dy * dy;
                
                if (dist_sq <= radius_sq) {
                    // エッジのアンチエイリアス
                    float edge_dist = sqrt(dist_sq) - radius + 1;
                    float edge_alpha = 1.0f;
                    if (edge_dist > 0) {
                        edge_alpha = std::max(0.0f, 1.0f - edge_dist);
                    }
                    
                    float final_alpha = alpha * edge_alpha;
                    cv::Vec3b& pixel = img.at<cv::Vec3b>(y, x);
                    pixel[0] = pixel[0] * (1 - final_alpha) + color[0] * final_alpha;
                    pixel[1] = pixel[1] * (1 - final_alpha) + color[1] * final_alpha;
                    pixel[2] = pixel[2] * (1 - final_alpha) + color[2] * final_alpha;
                }
            }
        }
    }
};

} // namespace visualization
} // namespace fluent_cloud

#endif // FLUENT_CLOUD_VISUALIZATION_OVERLAY_RENDERER_HPP