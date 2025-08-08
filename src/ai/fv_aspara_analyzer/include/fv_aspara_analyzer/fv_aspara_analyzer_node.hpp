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
public:
    /**
     * @brief コンストラクタ
     * @details ノードの初期化、パラメータ読み込み、トピックの設定を行う
     */
    FvAsparaAnalyzerNode();
    
    /**
     * @brief デストラクタ
     * @details リソースの適切な解放を行う
     */
    ~FvAsparaAnalyzerNode();

private:
    // ===== コールバック関数群 =====
    
    /**
     * @brief 2D検出結果のコールバック関数
     * @param msg 検出結果の配列メッセージ
     * @details YOLO等の物体検出結果を受信し、アスパラガス情報を更新
     */
    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    
    /**
     * @brief 3D点群データのコールバック関数
     * @param msg 点群データメッセージ
     * @details RealSense等からの3D点群を受信し、解析用データとして保存
     */
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    /**
     * @brief カメラ情報のコールバック関数
     * @param msg カメラキャリブレーション情報
     * @details カメラの内部パラメータを受信し、3D-2D変換に使用
     */
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    
    /**
     * @brief マスク画像のコールバック関数
     * @param msg セグメンテーションマスク画像
     * @details アスパラガス領域のマスクを受信し、精度向上に使用
     */
    void maskCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    /**
     * @brief カラー画像のコールバック関数
     * @param msg カラー画像メッセージ
     * @details 可視化用のカラー画像を受信
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    /**
     * @brief 深度画像のコールバック関数
     * @param msg 深度画像メッセージ
     */
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    /**
     * @brief 深度画像から効率的に点群を生成
     * @param bbox 2Dバウンディングボックス
     * @return フィルタリング済み点群
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractPointCloudFromDepth(
        const cv::Rect& bbox);

    // ===== コア処理関数群 =====
    
    /**
     * @brief バウンディングボックスで点群をフィルタリング
     * @param input_cloud 入力点群
     * @param bbox 2Dバウンディングボックス
     * @param camera_info カメラ情報
     * @return フィルタリング済み点群
     * @details 2D検出領域に対応する3D点群を抽出
     * 
     * @note 問題点：距離フィルタリングが不十分、エッジケースの処理が不明確
     * @note 改良点：深度範囲チェック、点群密度の正規化が必要
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterPointCloudByBoundingBox(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
        const cv::Rect& bbox,
        const sensor_msgs::msg::CameraInfo& camera_info);
    
    /**
     * @brief 点群のノイズ除去処理
     * @param input_cloud 入力点群
     * @return ノイズ除去済み点群
     * @details 統計的外れ値除去とボクセルグリッドフィルタを適用
     * 
     * @note 問題点：パラメータが固定値、点群密度に応じた動的調整なし
     * @note 改良点：点群密度に応じた適応的パラメータ調整が必要
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr applyNoiseReduction(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud);
    
    /**
     * @brief 3D点を2D画像座標に投影
     * @param point 3D点
     * @param camera_info カメラ情報
     * @return 2D画像座標
     * @details カメラの内部パラメータを使用して3D-2D変換
     * 
     * @note 問題点：レンズ歪み補正が不完全
     * @note 改良点：レンズ歪みパラメータの完全な適用が必要
     */
    cv::Point2f project3DTo2D(
        const pcl::PointXYZRGB& point,
        const sensor_msgs::msg::CameraInfo& camera_info);
    
    /**
     * @brief アスパラガスの根元位置を推定
     * @param aspara_cloud アスパラガス点群
     * @return 根元の3D座標
     * @details 点群の下端部から根元位置を推定
     * 
     * @note 問題点：地面との交点計算が不正確
     * @note 改良点：地面平面との交点計算、複数フレームでの追跡が必要
     */
    geometry_msgs::msg::Point estimateRootPosition(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aspara_cloud);
    
    /**
     * @brief アスパラガスの真っ直ぐ度を計算
     * @param aspara_cloud アスパラガス点群
     * @return 真っ直ぐ度（0.0-1.0）
     * @details PCAの第1主成分の分散から直線性を評価
     * 
     * @note 問題点：曲がったアスパラガスの評価が不正確
     * @note 改良点：曲線フィッティング、複数セグメント分割評価が必要
     */
    float calculateStraightness(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aspara_cloud);
    
    /**
     * @brief アスパラガスの長さを計算
     * @param aspara_cloud アスパラガス点群
     * @return 長さ（mm）
     * @details 点群の主軸方向の最大距離を計算
     * 
     * @note 問題点：曲がったアスパラガスの長さ測定が不正確
     * @note 改良点：曲線長の計算、複数セグメントでの積分が必要
     */
    float calculateLength(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aspara_cloud);
    
    /**
     * @brief PCA直線を生成
     * @param aspara_cloud アスパラガス点群
     * @return PCA直線の点群
     * @details 主成分分析による直線近似を生成
     * 
     * @note 問題点：曲がったアスパラガスでは不適切
     * @note 改良点：スプライン補間、複数セグメント分割が必要
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr generatePCALine(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& aspara_cloud);
    
    /**
     * @brief アスパラガス情報の処理
     * @param aspara_info アスパラガス情報
     * @details 品質評価と収穫判定の実行
     * 
     * @note 問題点：処理の並列化なし、リアルタイム性が不十分
     * @note 改良点：マルチスレッド処理、処理時間の最適化が必要
     */
    void processAsparagus(const AsparaInfo& aspara_info);
    
    // ===== 選択管理関数群 =====
    
    /**
     * @brief アスパラガス選択管理
     * @param new_detections 新しい検出結果
     * @details 非同期追跡システムによる選択状態管理
     */
    void updateAsparaguSelection(const std::vector<AsparaInfo>& new_detections);
    
    /**
     * @brief 検出結果からアスパラガス部位を関連付け
     * @param detections 全検出結果
     * @param aspara_info アスパラガス情報
     * @details 本体（クラスID 0）と穂（クラスID 1）を関連付け
     */
    void associateAsparagusParts(const vision_msgs::msg::Detection2DArray::SharedPtr& detections,
                                AsparaInfo& aspara_info);
    
    /**
     * @brief 最適なアスパラガスを選択
     * @param aspara_list アスパラガス候補リスト
     * @return 選択されたアスパラガスのID
     */
    int selectBestAsparagus(const std::vector<AsparaInfo>& aspara_list);
    
    /**
     * @brief 次のアスパラガスを選択（サービス）
     */
    void nextAsparaguService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    /**
     * @brief 前のアスパラガスを選択（サービス）
     */
    void prevAsparaguService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    /**
     * @brief 選択中アスパラガス情報を公開
     */
    void publishSelectedAsparaguInfo();
    
    /**
     * @brief アスパラガスの骨格ポイントを抽出
     * @param cloud アスパラガス点群
     * @param points_count 抽出するポイント数（5-10）
     * @return 骨格ポイント配列
     */
    std::vector<SkeletonPoint> extractSkeletonPoints(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, int points_count);
    
    /**
     * @brief グレードを文字列に変換
     * @param grade グレード列挙体
     * @return 日本語グレード文字列
     */
    std::string gradeToString(AsparaguGrade grade) const;
    
    /**
     * @brief バウンディングボックスの重複度を計算
     * @param bbox1 矩形1
     * @param bbox2 矩形2
     * @return 重複度（0.0-1.0）
     */
    float calculateOverlapRatio(const cv::Rect& bbox1, const cv::Rect& bbox2);
    
    /**
     * @brief アスパラガス本体と穂の空間的関連性を判定
     * @param body_bbox 本体のバウンディングボックス
     * @param spike_bbox 穂のバウンディングボックス
     * @return 関連性ありかどうか
     */
    bool isAssociatedSpike(const cv::Rect& body_bbox, const cv::Rect& spike_bbox);
    
    // ===== ユーティリティ関数群 =====
    
    /**
     * @brief フィルタリング済み点群をパブリッシュ
     * @param cloud 点群データ
     * @param frame_id 座標系ID
     * @param aspara_id アスパラガスID
     * @details デバッグ用の可視化データを出力
     */
    void publishFilteredPointCloud(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
        const std::string& frame_id,
        int aspara_id);
    
    /**
     * @brief 根元位置のTF座標をパブリッシュ
     * @param root_position 根元位置
     * @param frame_id 座標系ID
     * @param aspara_id アスパラガスID
     * @details ロボット制御用の座標変換情報を出力
     */
    void publishRootTF(
        const geometry_msgs::msg::Point& root_position,
        const std::string& frame_id,
        int aspara_id);
    
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
    void publishAnnotatedImage(
        const cv::Mat& image,
        const AsparaInfo& aspara_info,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& filtered_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& pca_line_cloud,
        float length,
        float straightness,
        bool is_harvestable);

    // ===== ROS2 サブスクライバー =====
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;      ///< 2D検出結果サブスクライバー
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;          ///< 3D点群サブスクライバー
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;          ///< カメラ情報サブスクライバー
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;                      ///< マスク画像サブスクライバー
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;                     ///< カラー画像サブスクライバー
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;                     ///< 深度画像サブスクライバー

    // ===== ROS2 パブリッシャー =====
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pointcloud_pub_;    ///< フィルタリング済み点群パブリッシャー
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr selected_pointcloud_pub_;    ///< 選択中のアスパラガス点群パブリッシャー
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_pub_;              ///< 注釈付き画像パブリッシャー
    
    // ===== ROS2 サービス =====
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr next_asparagus_service_;              ///< 次のアスパラガス選択サービス
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prev_asparagus_service_;              ///< 前のアスパラガス選択サービス
    
    // ===== CallbackGroup（マルチスレッド対応）=====
    rclcpp::CallbackGroup::SharedPtr image_callback_group_;                                  ///< 画像処理用コールバックグループ
    rclcpp::CallbackGroup::SharedPtr detection_callback_group_;                              ///< 検出処理用コールバックグループ
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;                                ///< サービス用コールバックグループ
    
    // ===== TF2（座標変換）関連 =====
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                                             ///< TFバッファ
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                                ///< TFリスナー
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;                          ///< TFブロードキャスター

    // ===== データストレージ =====
    std::vector<AsparaInfo> aspara_list_;                                                    ///< アスパラガス情報リスト
    AsparaSelection aspara_selection_;                                                       ///< アスパラ選択管理
    sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;                             ///< 最新の点群データ
    sensor_msgs::msg::Image::SharedPtr latest_depth_image_;                                  ///< 最新の深度画像
    sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;                             ///< 最新のカメラ情報
    sensor_msgs::msg::Image::SharedPtr latest_color_image_;                                  ///< 最新のカラー画像
    cv::Mat latest_mask_;                                                                    ///< 最新のマスク画像
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
    
    // ===== 検出処理時間計測用 =====
    fluent::utils::Stopwatch detection_stopwatch_;                                          ///< 検出処理時間計測
    
    // ===== 非同期点群処理用 =====
    std::thread pointcloud_worker_thread_;                                                   ///< 点群処理ワーカースレッド
    std::mutex pointcloud_mutex_;                                                            ///< 点群処理用ミューテックス
    std::condition_variable pointcloud_cv_;                                                  ///< 点群処理用条件変数
    std::atomic<bool> shutdown_flag_;                                                        ///< シャットダウンフラグ
    std::atomic<bool> processing_in_progress_;                                               ///< 処理中フラグ
    std::queue<AsparaInfo> processing_queue_;                                               ///< 処理待ちキュー
    
    // ===== 非同期処理メソッド =====
    /**
     * @brief ワーカースレッドのメインループ
     */
    void pointcloudWorkerLoop();
    
    /**
     * @brief アスパラガス処理をキューに追加
     * @param aspara_info 処理対象アスパラガス情報
     */
    void enqueueAsparaguProcessing(const AsparaInfo& aspara_info);
};

} // namespace fv_aspara_analyzer

#endif // FV_ASPARA_ANALYZER_NODE_HPP