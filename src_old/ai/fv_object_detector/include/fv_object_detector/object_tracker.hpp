#ifndef FV_OBJECT_DETECTOR_OBJECT_TRACKER_HPP_
#define FV_OBJECT_DETECTOR_OBJECT_TRACKER_HPP_

#include "fv_object_detector/detection_data.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>

namespace fv_object_detector
{

/**
 * @brief オブジェクト追跡クラス
 *
 * 検出されたオブジェクトに一意のIDを割り当て、フレーム間で追跡する機能を提供します。
 * シンプルで再利用しやすい設計です。
 */
class ObjectTracker {
public:
    /**
     * @brief コンストラクタ
     */
    ObjectTracker();

    /**
     * @brief 検出されたオブジェクトにIDを割り当てる
     *
     * 前フレームのオブジェクトと現在フレームのオブジェクトを対応付け、
     * 一致するオブジェクトには同じIDを割り当て、新しいオブジェクトには
     * 新しいIDを割り当てます。
     *
     * @param detections 現在フレームの検出結果
     */
    void assignObjectIds(std::vector<DetectionData>& detections);

    /**
     * @brief 現在の検出結果を取得する
     *
     * @return const std::vector<DetectionData>& 現在の検出結果
     */
    const std::vector<DetectionData>& getCurrentDetections() const;

    /**
     * @brief クラスIDごとの検出数を取得する
     *
     * @param class_id クラスID
     * @return int 指定したクラスIDの検出数
     */
    int getDetectionCountByClass(int class_id) const;

    /**
     * @brief 全検出数を取得する
     *
     * @return int 全検出数
     */
    int getTotalDetectionCount() const;

    /**
     * @brief トラッカーをリセットする
     */
    void reset();

private:
    int next_object_id_;                           ///< 次に割り当てるオブジェクトID
    int selected_object_id_;                       ///< 選択中のオブジェクトID
    std::vector<DetectionData> current_detections_; ///< 現在の検出結果
    std::map<int, cv::Point2f> previous_centers_;   ///< 前フレームのオブジェクト中心位置

    /**
     * @brief 2つのオブジェクトが同じオブジェクトかどうかを判定
     *
     * @param det1 検出結果1
     * @param det2 検出結果2
     * @param threshold 距離閾値
     * @return true 同じオブジェクトの場合
     */
    bool isSameObject(const DetectionData& det1, const DetectionData& det2, float threshold = 50.0f) const;

    /**
     * @brief 2点間の距離を計算
     *
     * @param p1 点1
     * @param p2 点2
     * @return float 距離
     */
    float calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2) const;
};

} // namespace fv_object_detector

#endif // FV_OBJECT_DETECTOR_OBJECT_TRACKER_HPP_ 