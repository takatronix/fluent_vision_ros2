/**
 * @file aspara_info.hpp
 * @brief アスパラガス情報データ構造の定義
 * @details アスパラガス検出・分析に関する全てのデータ構造を含む
 * @author FluentVision Team
 * @date 2025
 * @version 1.0
 */

#ifndef FV_ASPARA_ANALYZER__ASPARA_INFO_HPP_
#define FV_ASPARA_ANALYZER__ASPARA_INFO_HPP_

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace fv_aspara_analyzer
{

/**
 * @brief アスパラガス品質グレード
 */
enum class AsparaguGrade
{
    UNKNOWN = 0,  ///< 未判定
    A_GRADE = 1,  ///< A級（最高品質）
    B_GRADE = 2,  ///< B級（標準品質）
    C_GRADE = 3,  ///< C級（低品質）
    OUT_OF_SPEC = 4  ///< 規格外
};

/**
 * @brief 処理時間記録構造体
 */
struct ProcessingTimes
{
    double total_ms = 0.0;              ///< 全体処理時間（ミリ秒）
    double filter_bbox_ms = 0.0;        ///< バウンディングボックスフィルタ時間
    double noise_reduction_ms = 0.0;    ///< ノイズ除去時間
    double measurement_ms = 0.0;        ///< 測定時間
    double pca_calculation_ms = 0.0;    ///< PCA計算時間
    double visualization_ms = 0.0;      ///< 可視化時間
};

/**
 * @brief アスパラガス部位情報構造体
 * @details アスパラガス本体（クラスID=0）と穂（クラスID=1）の統一管理
 */
struct AsparagusPart
{
    int class_id = -1;                         ///< クラスID（0=本体、1=穂）
    cv::Rect bounding_box_2d;                  ///< 2Dバウンディングボックス
    float confidence = 0.0f;                   ///< 検出信頼度
    bool is_valid = false;                     ///< 有効フラグ
};

/**
 * @brief 骨格ポイント構造体
 * @details アスパラガス骨格の5-10点抽出、長さ・曲がり度の詳細分析用
 */
struct SkeletonPoint
{
    cv::Point2f image_point;                   ///< 2D画像座標
    geometry_msgs::msg::Point world_point;     ///< 3D世界座標
    float distance_from_base = 0.0f;           ///< 根元からの距離（メートル）
    float radius_at_point = 0.0f;              ///< その点での半径（メートル）
};

/**
 * @brief アスパラガス統合情報構造体
 * @details アスパラガスの検出から分析まで全ての情報を統合管理
 * 
 * 設計方針：
 * - SelectedAsparaInfo構造体を統合（1クラス1ファイル原則）
 * - 検出情報、分析結果、ロボット制御情報を全て含む
 * - 選択状態は外部で管理（selected_aspara_id_による判定）
 */
struct AsparaInfo
{
    int id;                                    ///< アスパラガスの一意識別ID
    float confidence;                          ///< 検出信頼度（0.0-1.0）
    cv::Rect bounding_box_2d;                  ///< 2D画像上のバウンディングボックス
    
    // 穂情報（クラスID 0=本体、1=穂）
    AsparagusPart body_part;                   ///< アスパラガス本体情報（クラスID 0）
    std::vector<AsparagusPart> spike_parts;    ///< アスパラガス穂情報リスト（クラスID 1）
    
    // 3D分析結果
    sensor_msgs::msg::PointCloud2 filtered_pointcloud;  ///< フィルタリング済み3D点群
    sensor_msgs::msg::PointCloud2 asparagus_pointcloud; ///< ROI抽出後の生点群（フィルター前）
    geometry_msgs::msg::Point root_position_3d; ///< 根元の3D座標
    float length;                              ///< アスパラガスの長さ（mm）
    float straightness;                        ///< 真っ直ぐ度（0.0-1.0）
    bool is_harvestable;                       ///< 収穫可能フラグ
    
    // 品質情報（SelectedAsparaInfoから統合）
    float distance_from_camera = 0.0f;         ///< カメラからの距離（メートル）
    float curvature = 0.0f;                    ///< 曲がり度（メートル）
    float diameter = 0.0f;                     ///< 太さ（メートル）
    AsparaguGrade grade = AsparaguGrade::UNKNOWN; ///< 品質グレード
    
    // 切断情報
    bool is_cutting_target = false;            ///< 切断対象フラグ
    cv::Point2f cut_point_pixel;               ///< 切断ポイント画像座標（ピクセル）
    geometry_msgs::msg::Point cut_point_world; ///< 切断ポイント3D座標（メートル）
    
    // 骨格ポイント配列（5-10点）
    std::vector<SkeletonPoint> skeleton_points; ///< 骨格ポイント配列（頂点から根元まで）
    
    rclcpp::Time last_update_time;             ///< 最後の更新時刻
    ProcessingTimes processing_times;          ///< 処理時間記録
    
    // アニメーション用フィールド
    cv::Rect smooth_bbox;                      ///< スムージングされたバウンディングボックス
    float animation_alpha = 1.0f;              ///< アニメーション透明度
    bool is_new = true;                        ///< 新規検出フラグ
    int frame_count = 0;                       ///< フレームカウント
    
    // 追跡用フィールド
    int detection_count = 0;                   ///< 検出回数
    float overlap_ratio = 0.0f;                ///< 前フレームとの重複度
    rclcpp::Time first_detected_time;          ///< 初回検出時刻
};

}  // namespace fv_aspara_analyzer

#endif  // FV_ASPARA_ANALYZER__ASPARA_INFO_HPP_