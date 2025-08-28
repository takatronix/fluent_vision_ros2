#ifndef FV_OBJECT_DETECTOR_DETECTION_DATA_HPP_
#define FV_OBJECT_DETECTOR_DETECTION_DATA_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

namespace fv_object_detector
{

/**
 * @brief 検出統計情報を格納する構造体
 */
struct DetectionStats {
    double inference_time_ms = 0.0;      ///< 推論時間（ミリ秒）
    double total_processing_time_ms = 0.0; ///< 総処理時間（ミリ秒）
    std::string device_used;             ///< 使用デバイス（CPU/GPU）
    int total_detections = 0;            ///< 総検出数
    int filtered_detections = 0;         ///< フィルタ後検出数
    double fps = 0.0;                    ///< 処理FPS
    double timestamp = 0.0;              ///< 統計作成時刻
};

/**
 * @brief 物体検出結果を格納する構造体
 * 
 * 物体検出の結果を一元管理する構造体です。
 * シンプルで再利用しやすい設計です。
 */
struct DetectionData {
    cv::Rect2f bbox;           ///< バウンディングボックス
    float confidence = 0.0f;   ///< 信頼度
    int class_id = 0;          ///< クラスID
    int object_id = -1;        ///< オブジェクトID（トラッキング用）
    std::string class_name;    ///< クラス名
    double timestamp = 0.0;    ///< 検出作成時刻（エポック秒）

    // デフォルトコンストラクタ
    DetectionData() = default;

    // コピーコンストラクタ
    DetectionData(const DetectionData& other)
        : bbox(other.bbox), confidence(other.confidence), class_id(other.class_id),
          object_id(other.object_id), class_name(other.class_name), timestamp(other.timestamp)
    {}

    // ムーブコンストラクタ
    DetectionData(DetectionData&& other) noexcept
        : bbox(std::move(other.bbox)), confidence(other.confidence), class_id(other.class_id),
          object_id(other.object_id), class_name(std::move(other.class_name)), timestamp(other.timestamp)
    {}

    // コピー演算子
    DetectionData& operator=(const DetectionData& other) {
        if (this != &other) {
            bbox = other.bbox;
            confidence = other.confidence;
            class_id = other.class_id;
            object_id = other.object_id;
            class_name = other.class_name;
            timestamp = other.timestamp;
        }
        return *this;
    }

    // ムーブ代入演算子
    DetectionData& operator=(DetectionData&& other) noexcept {
        if (this != &other) {
            bbox = std::move(other.bbox);
            confidence = other.confidence;
            class_id = other.class_id;
            object_id = other.object_id;
            class_name = std::move(other.class_name);
            timestamp = other.timestamp;
        }
        return *this;
    }

    /**
     * @brief バウンディングボックスの中心座標を取得
     * @return 中心座標
     */
    cv::Point2f getCenter() const {
        return cv::Point2f(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
    }

    /**
     * @brief バウンディングボックスの面積を取得
     * @return 面積
     */
    float getArea() const {
        return bbox.width * bbox.height;
    }
};

} // namespace fv_object_detector

#endif // FV_OBJECT_DETECTOR_DETECTION_DATA_HPP_ 