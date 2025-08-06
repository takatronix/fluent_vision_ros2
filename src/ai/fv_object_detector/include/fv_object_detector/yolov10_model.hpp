#ifndef FV_OBJECT_DETECTOR_YOLOV10_MODEL_HPP_
#define FV_OBJECT_DETECTOR_YOLOV10_MODEL_HPP_

#include "fv_object_detector/ai_model.hpp"
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <nlohmann/json.hpp>

namespace fv_object_detector
{

/**
 * @brief YOLOv10 物体検出モデル
 */
class YoloV10Model : public AIModel {
public:
    YoloV10Model();
    virtual ~YoloV10Model();

    /**
     * @brief モデル初期化
     * @param model_cfg モデル設定
     */
    void initialize(const nlohmann::json& model_cfg) override;

    /**
     * @brief 推論実行
     * @param image 入力画像
     * @return 推論結果
     */
    std::vector<DetectionData> infer(const cv::Mat& image) override;

private:
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;
    cv::Mat blob_;

    float min_confidence_ = 0.5f;
    float min_area_ = 100.0f;
    float nms_threshold_ = 0.45f;

    /**
     * @brief 推論結果を後処理
     * @param output 推論出力
     * @param original_size 元画像サイズ
     * @return 検出結果
     */
    std::vector<DetectionData> postProcess(const ov::Tensor& output, const cv::Size& original_size);

    /**
     * @brief Non-Maximum Suppressionを実行
     * @param detections 検出結果
     * @param threshold NMS閾値
     * @return フィルタリング後の検出結果
     */
    std::vector<DetectionData> applyNMS(const std::vector<DetectionData>& detections, float threshold);

    /**
     * @brief IoU（Intersection over Union）を計算
     * @param bbox1 バウンディングボックス1
     * @param bbox2 バウンディングボックス2
     * @return IoU値
     */
    float calculateIoU(const cv::Rect2f& bbox1, const cv::Rect2f& bbox2);
};

} // namespace fv_object_detector

#endif // FV_OBJECT_DETECTOR_YOLOV10_MODEL_HPP_ 