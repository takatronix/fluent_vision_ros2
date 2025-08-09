/**
 * @file fv_aspara_analyzer_node.hpp
 * @brief アスパラガス解析ノードのヘッダーファイル
 * @details 3D点群データと2D検出結果を統合してアスパラガスの品質評価を行う
 * @author Takashi Otsuka
 * @date 2024
 * @version 1.0
 */

#ifndef FV_ASPARA_ANALYZER_NODE_HPP
#define FV_ASPARA_ANALYZER_NODE_HPP

// ROS2関連のインクルード
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

// OpenCV関連のインクルード
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// PCL（Point Cloud Library）関連のインクルード
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>
#include <pcl/exceptions.h>

// TF2（座標変換）関連のインクルード
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 時間計測用
#include <chrono>

// 非同期処理用
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <queue>

// 日本語テキスト描画
// cvx_text.hppは削除済み（fluent統合ライブラリを使用）

// サービス関連
#include <std_srvs/srv/trigger.hpp>

// ===== Fluent Library =====
#include <fluent.hpp>
#include "aspara_selection.hpp"
#include "aspara_info.hpp"

namespace fv_aspara_analyzer
{

// AsparaInfo関連構造体は aspara_info.hpp に移動済み


/**
 * @class FvAsparaAnalyzerNode
 * @brief アスパラガス解析メインノードクラス
 * @details 3D点群と2D検出結果を統合し、アスパラガスの品質評価と収穫判定を行う
 * 
 * 主な機能：
 * - 3D点群のノイズ除去とフィルタリング
 * - PCA（主成分分析）による直線性評価
 * - 長さ測定と収穫適性判定
 * - 可視化とデバッグ情報の出力
 */
class FvAsparaAnalyzerNode : public rclcpp::Node
{
    friend class AnalyzerThread;
public:
    /**
     * @brief コンストラクタ
     */
    FvAsparaAnalyzerNode();
    
    /**
     * @brief デストラクタ
     */
    ~FvAsparaAnalyzerNode();

    // ===== コールバック関数 =====
    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void maskCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void mouseClickCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    
    // ===== アスパラガス関連付け =====
    void associateAsparagusParts(const vision_msgs::msg::Detection2DArray::SharedPtr& detections, AsparaInfo& aspara_info);
    bool isAssociatedSpike(const cv::Rect& body_bbox, const cv::Rect& spike_bbox);
    
    // ===== サービスコールバック =====
    void nextAsparaguService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void prevAsparaguService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    // ===== カメラ情報取得 =====
    void getCameraInfoOnce(const std::string& camera_info_topic);
    
    
    
    
    // ===== 選択管理関数群 =====
    
    // ===== カーソル機能 =====
    /**
     * @brief カーソル位置を設定
     * @param x カーソルのX座標
     * @param y カーソルのY座標
     */
    void setCursor(int x, int y);
    
    /**
     * @brief カーソル位置を取得
     * @param x カーソルのX座標（出力）
     * @param y カーソルのY座標（出力）
     */
    void getCursor(int& x, int& y) const;
    
    /**
     * @brief カーソル位置のアスパラを選択
     * @return 選択成功したらtrue
     */
    bool selectAsparaAtCursor();
    
    /**
     * @brief 次のアスパラへカーソル移動
     * @return 移動成功したらtrue
     */
    bool moveCursorToNext();
    
    /**
     * @brief 前のアスパラへカーソル移動
     * @return 移動成功したらtrue
     */
    bool moveCursorToPrev();
    

    
    /**
     * @brief 注釈付き画像をパブリッシュ
     * @param image 元画像
     * @param aspara_info アスパラガス情報（処理時間含む）
     * @param filtered_cloud フィルタリング済み点群
     * @param pca_line_cloud PCA直線点群
     * @param length 長さ
     * @param straightness 真っ直ぐ度
     * @param is_harvestable 収穫可能フラグ
     * @details デバッグ用の可視化画像を生成・出力
     * 
     * @note 問題点：描画処理が重い、フレームレート低下の原因
     * @note 改良点：描画処理の最適化、条件付き描画が必要
     */
    void publishCurrentImage();


    // ===== ROS2 サブスクライバー =====
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;      ///< 2D検出結果サブスクライバー
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;          ///< 3D点群サブスクライバー
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;          ///< カメラ情報サブスクライバー
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;                      ///< マスク画像サブスクライバー
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;                     ///< カラー画像サブスクライバー
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;                     ///< 深度画像サブスクライバー
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr mouse_click_sub_;             ///< マウスクリックサブスクライバー

    // ===== ROS2 パブリッシャー =====
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pointcloud_pub_;    ///< フィルタリング済み点群パブリッシャー
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr selected_pointcloud_pub_;    ///< 選択中のアスパラガス点群パブリッシャー
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detected_all_pointcloud_pub_; ///< 検出中すべてのアスパラ点群
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_pub_;              ///< 注釈付き画像パブリッシャー
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr annotated_image_compressed_pub_;  ///< 注釈付き圧縮画像パブリッシャー
    
    // ===== ROS2 サービス =====
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr next_asparagus_service_;              ///< 次のアスパラガス選択サービス
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prev_asparagus_service_;              ///< 前のアスパラガス選択サービス
    
    // CallbackGroupは削除済み（シングルスレッド構成）                                ///< サービス用コールバックグループ
    
    // ===== TF2（座標変換）関連 =====
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                                             ///< TFバッファ
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                                ///< TFリスナー
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;                          ///< TFブロードキャスター

    // ===== データストレージ =====
    std::vector<AsparaInfo> aspara_list_;                                                    ///< アスパラガス情報リスト
    std::mutex aspara_list_mutex_;                                                           ///< アスパラガスリスト用ミューテックス
    std::mutex image_data_mutex_;                                                            ///< 画像データ用ミューテックス（分離）
    AsparaSelection aspara_selection_;                                                       ///< アスパラ選択管理
    sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;                             ///< 最新の点群データ
    sensor_msgs::msg::Image::SharedPtr latest_depth_image_;                                  ///< 最新の深度画像（image_data_mutexで保護）
    sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;                             ///< 最新のカメラ情報
    sensor_msgs::msg::Image::SharedPtr latest_color_image_;                                  ///< 最新のカラー画像（image_data_mutexで保護）
    cv::Mat latest_mask_;                                                                    ///< 最新のマスク画像（image_data_mutexで保護）
    int selected_aspara_id_;                                                                 ///< 選択中のアスパラガスID（-1=未選択）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr selected_pointcloud_;                            ///< 選択中のアスパラガス点群
    std::map<int, AsparaInfo> tracked_asparagus_;                                            ///< 追跡中アスパラガス辞書
    vision_msgs::msg::Detection2DArray::SharedPtr latest_detections_;                        ///< 最新の検出結果（穂情報取得用）
    
    // ===== パラメータ群 =====
    double min_confidence_;                                                                   ///< 最小信頼度閾値
    double pointcloud_distance_min_;                                                          ///< 点群距離最小値
    double pointcloud_distance_max_;                                                          ///< 点群距離最大値
    double aspara_filter_distance_;                                                           ///< アスパラガスフィルタ距離
    
    // ===== ノイズ除去パラメータ =====
    int noise_reduction_neighbors_;                                                           ///< 統計的外れ値除去の近傍点数
    double noise_reduction_std_dev_;                                                          ///< 統計的外れ値除去の標準偏差
    double voxel_leaf_size_;                                                                  ///< ボクセルグリッドの葉サイズ
    
    // ===== 処理パラメータ =====
    double harvest_min_length_;                                                               ///< 収穫最小長さ
    double harvest_max_length_;                                                               ///< 収穫最大長さ
    double straightness_threshold_;                                                           ///< 真っ直ぐ度閾値
    bool enable_pointcloud_processing_;                                                       ///< ポイントクラウド処理有効化フラグ
    double depth_unit_m_16u_ {0.001};                                                         ///< 16UC1のメートル換算係数
    bool enable_detected_all_points_{false};                                                  ///< 全検出点群の公開有効化
    
    // ===== 選択管理パラメータ =====
    double object_tracking_overlap_threshold_;                                                ///< 重複度閾値
    double object_tracking_timeout_ms_;                                                      ///< 追跡タイムアウト（ms）
    
    // ===== 内部状態 =====
    int next_asparagus_id_;                                                                  ///< 次のアスパラガスID
    rclcpp::Time last_selection_time_;                                                       ///< 最後の選択時刻
    int skeleton_points_count_;                                                              ///< 骨格ポイント数（設定ファイルから）
    
    // ===== FPS測定用 =====
    std::unique_ptr<fluent::utils::FPSMeter> color_fps_meter_;                              ///< カラー画像FPS計測
    std::unique_ptr<fluent::utils::FPSMeter> depth_fps_meter_;                              ///< 深度画像FPS計測  
    std::unique_ptr<fluent::utils::FPSMeter> detection_fps_meter_;                          ///< 検出FPS計測
    std::unique_ptr<fluent::utils::FPSMeter> segmentation_fps_meter_;                       ///< セグメンテーションFPS計測
    
    // ===== 検出処理時間計測用 =====
    fluent::utils::Stopwatch detection_stopwatch_;                                          ///< 検出処理時間計測
    std::atomic<double> last_analysis_time_ms_{0.0};                                        ///< 最後の分析時間（ミリ秒）
    std::atomic<double> last_pointcloud_time_ms_{0.0};                                      ///< 最後の点群処理時間（ミリ秒）
    std::unique_ptr<fluent::utils::FPSMeter> pointcloud_fps_meter_;                         ///< 点群処理FPS計測
    
    // ===== 非同期点群処理用 =====
    std::unique_ptr<class AnalyzerThread> analyzer_thread_;                                  ///< 点群解析専用スレッド
    
    // ===== 高頻度出力タイマー =====
    rclcpp::TimerBase::SharedPtr animation_timer_;                                          ///< アニメーション用30FPSタイマー

    // ===== カーソル管理 =====
    cv::Point cursor_position_;                                                              ///< カーソル位置（-1,-1で非表示）
    cv::Point smooth_cursor_position_;                                                       ///< スムーズアニメーション用カーソル位置
    bool cursor_visible_;                                                                    ///< カーソル表示フラグ
    std::chrono::steady_clock::time_point cursor_animation_start_;                          ///< カーソルアニメーション開始時刻
    std::chrono::steady_clock::time_point last_detection_time_;                             ///< 最後にアスパラを検出した時刻
    int cursor_auto_hide_ms_;                                                               ///< カーソル自動消去時間（ミリ秒）

    // ===== ノード接続状態監視（ステータス用） =====
    std::chrono::steady_clock::time_point last_detection_msg_time_;  ///< 最後に検出メッセージを受信した時刻
    std::chrono::steady_clock::time_point last_depth_msg_time_;      ///< 最後に深度メッセージを受信した時刻
    std::chrono::steady_clock::time_point last_color_msg_time_;      ///< 最後にカラー画像メッセージを受信した時刻
    bool detection_node_seen_ {false};                                ///< 検出ノードから一度でも受信したか
    bool depth_node_seen_ {false};                                     ///< 深度ノードから一度でも受信したか
    bool camera_node_seen_ {false};                                    ///< カメラノードから一度でも受信したか
    std::atomic<bool> last_pointcloud_was_organized_{false};          ///< 直近の点群がorganized(height>1)か

    // デバッグ表示制御
    bool debug_overlay_ {true};                                        ///< 画面デバッグオーバーレイの有効/無効

};

} // namespace fv_aspara_analyzer

#endif // FV_ASPARA_ANALYZER_NODE_HPP