#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <fv_stem_detector/msg/stem_detection_array.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <map>
#include <unordered_set>
#include "fluent_lib/ui/overlay_engine.hpp"
#include "fluent_lib/ui/ui_cursor_controller.hpp"
#include "fluent_lib/ui/window_base.hpp"
#include "fluent_lib/ui/info_window.hpp"
#include "fv_aspara_analyzer/aspara_list.hpp"
#include "fv_aspara_analyzer/aspara_info.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "fv_aspara_analyzer/srv/get_selected_asparagus.hpp"

namespace fv_aspara_analyzer {

/**
 * @class FvAsparaAnalyzerNode
 * @brief v2最小ノード（高可読・短コード）
 *
 * 目的:
 * - カメラ画像をパススルー表示する
 * - 検出結果(Detection2DArray)のBBoxを `fluent_ui::OverlayEngine` に渡して補間し、枠とラベルを描画する
 * - 画像が未到着の間も「待機中…」画面を周期出力し、表示が停止しないようにする
 *
 * 拡張方針:
 * - 機能は小さな関数に分割し、容易に取り外し/差し替え可能にする
 * - コメントは日本語で、処理の意図・アルゴリズム・シーケンスを明確に記述する
 */
class FvAsparaAnalyzerNode : public rclcpp::Node {
public:
    /**
     * @brief コンストラクタ
     * - 必須パラメータ宣言
     * - I/O初期化（購読/配信）
     * - OverlayEngineのゲイン設定
     * - 画面更新タイマー開始
     */
    FvAsparaAnalyzerNode();
    ~FvAsparaAnalyzerNode();
    
private:
    // 動的パラメータ反映（自動保存対応）
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    rcl_interfaces::msg::SetParametersResult onParamSet(const std::vector<rclcpp::Parameter> &params);
    /**
     * @brief 検出コールバック
     * - Detection2DArray から本体クラス(0)の矩形を抽出
     * - OverlayEngine のターゲットに反映
     */
    void detectionCallback(const fv_stem_detector::msg::StemDetectionArray::SharedPtr msg);

    /**
     * @brief 画像コールバック
     * - 最新カラー画像を保持（パススルー描画に使用）
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief 現在のフレームを描画・配信
     * シーケンス:
     * 1) 入力がなければ待機画面を生成
     * 2) OverlayEngine の snapshot() を枠/ラベルとして描画
     * 3) `annotated_image_pub_` へ配信
     */
    void publishCurrentImage();

    // 深度からアプローチ距離[m]を推定（16UC1は depth_unit_m_16u、32FC1は[m]）
    bool computeApproachDistanceM(const cv::Point &px, double &out_m) const;

    // ROI点群出力は廃止

    /**
     * @brief Detection2DArray から (id, bbox) リストを生成
     * @param msg 検出メッセージ
     * @return (id, bbox) のペア配列
     * 備考: id は簡易実装（位置から一意性を擬似付与）。将来は AsparaSelection と置換
     */
    std::vector<std::pair<int, cv::Rect>> buildOverlayTargetsFromDetections(const fv_stem_detector::msg::StemDetectionArray::SharedPtr &msg) const;

    /**
     * @brief 「待機中…」の簡易画面を作成
     * @param size 生成する画像サイズ
     * @return BGR8 の待機画面
     */
    static cv::Mat drawWaitScreen(const cv::Size &size);

    /**
     * @brief IoU（矩形の重なり度合い）を計算
     */
    static double computeIoU(const cv::Rect &a, const cv::Rect &b);

    /**
     * @brief 新しい検出矩形に永続IDを付与
     * - 前フレーム矩形とIoU>thrでマッチ → 既存IDを継承
     * - マッチ不可 → 新規IDを採番
     */
    std::vector<std::pair<int, cv::Rect>> assignPersistentIds(const std::vector<cv::Rect> &rects);

    /**
     * @brief 選択IDを維持/再取得
     * - 既存選択が存在し、マッチできれば維持
     * - できなければ面積最大のIDを選択
     */
    void updateSelection(const std::vector<std::pair<int, cv::Rect>> &targets);

    // 内部更新の小関数群（可読性と再利用性のために分割）
    void updateActiveIds(const std::vector<std::pair<int, cv::Rect>> &with_ids);
    void updatePriorityAndCursor(const std::vector<std::pair<int, cv::Rect>> &with_ids);
    void mapConfidences(const fv_stem_detector::msg::StemDetectionArray::SharedPtr &msg,
                        const std::vector<std::pair<int, cv::Rect>> &with_ids);

    // StemDetectionArray から AsparaInfo の最小リストを構築（条件フィルタ込み）
    std::vector<AsparaInfo> buildAsparaInfosFromStem(const fv_stem_detector::msg::StemDetectionArray::SharedPtr &msg) const;

    // 入出力
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<fv_stem_detector::msg::StemDetectionArray>::SharedPtr detections_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr annotated_compressed_pub_;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr roi_points_pub_; // 廃止
    // 廃止: 選択アスパラ情報のトピック配信
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selected_info_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr cursor_click_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::TimerBase::SharedPtr animation_timer_;
    rclcpp::TimerBase::SharedPtr selected_info_timer_;
    // 追加: 選択中アスパラ取得サービス
    rclcpp::Service<fv_aspara_analyzer::srv::GetSelectedAsparagus>::SharedPtr get_selected_srv_;

    // 状態
    std::string camera_id_{};               // カメラ識別子（例: d405, d415）
    sensor_msgs::msg::Image::SharedPtr latest_color_;
    sensor_msgs::msg::Image::SharedPtr latest_depth_;
    sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;
    rclcpp::Time last_color_stamp_{};   // 最新カラーのstamp
    rclcpp::Time last_depth_stamp_{};
    fluent_ui::OverlayEngine overlay_;
    fluent_ui::UiCursorController cursor_;

    // ===== HUDウィンドウ =====
    fluent_ui::MirrorBBoxWindow approach_window_;
    fluent_ui::AutoSizeInfoWindow right_info_window_;
    cv::Point approach_px_{-1, -1};
    bool approach_valid_{false};
    int last_windows_id_{-1};
    cv::Rect last_windows_bbox_ {0,0,0,0};
    double last_approach_distance_m_ {-1.0};
    double depth_unit_m_16u_ {0.001};
    double approach_max_range_m_ {2.0};
    bool use_color_camera_info_flag_ {true};

    // ===== 収穫可否/長さ制約 =====
    double asparagus_length_impossible_min_m_ {0.10};   // これ未満はアスパラとして不適合
    double asparagus_length_impossible_max_m_ {0.50};   // これ超過はアスパラとして不適合
    double harvest_min_length_m_ {0.23};                 // 収穫可能の最小長さ
    double harvest_max_length_m_ {0.50};                 // 収穫可能の最大長さ

    // 簡易: ID→長さ[m]（将来、外部から更新/内部計測により埋まる想定）
    std::map<int, double> last_length_by_id_;

    // v2: 受信・フィルタ後のアスパラ最小情報リスト
    std::vector<AsparaInfo> aspara_infos_;

    // 判定関数（内部）
    inline bool isValidAsAsparagus(double length_m) const {
        return std::isfinite(length_m) && length_m >= asparagus_length_impossible_min_m_ && length_m <= asparagus_length_impossible_max_m_;
    }
    inline bool isHarvestable(double length_m) const {
        return std::isfinite(length_m) && length_m >= harvest_min_length_m_ && length_m <= harvest_max_length_m_;
    }

    // 深度バンド抽出（アプローチ点±）
    bool depth_band_enable_ {true};
    double depth_band_minus_m_ {0.05};
    double depth_band_plus_m_ {0.05};

    // ===== ID管理・選択 =====
    int next_id_ {1};
    int selected_id_ {-1};
    std::map<int, cv::Rect> last_id_to_rect_;
    std::map<int, double> last_confidence_by_id_; // ラベル用 信頼度（0..1）
    std::unordered_set<int> active_ids_;          // 今フレームで観測されたID集合
    AsparaList aspara_list_;                      // 優先順位と選択操作の管理

    // 直近の検出詳細（永続ID→StemDetection）
    std::map<int, fv_stem_detector::msg::StemDetection> last_det_by_id_;

    // 描画色（PCA/アプローチ/先端・根本・軸）
    cv::Scalar color_root_{0,0,255};
    cv::Scalar color_tip_{0,255,255};
    cv::Scalar color_axis_{255,255,0};
    cv::Scalar color_approach_{255,0,255};
    // 次/前 選択操作（サービス）
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr next_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prev_srv_;

    void onNext(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response>);
    void onPrev(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response>);
    void onGetSelected(const std::shared_ptr<fv_aspara_analyzer::srv::GetSelectedAsparagus::Request>,
                       std::shared_ptr<fv_aspara_analyzer::srv::GetSelectedAsparagus::Response>);

    // 最新の stem_detector 検出（描画用）
    std::vector<fv_stem_detector::msg::StemDetection> last_detections_;
};

} // namespace fv_aspara_analyzer


