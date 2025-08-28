#ifndef YOLOV8_MODEL_HPP
#define YOLOV8_MODEL_HPP

#include "fv_object_detector/ai_model.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>

namespace fv_object_detector
{

class YoloV8Model : public AIModel
{
public:
    YoloV8Model();
    ~YoloV8Model();

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
    cv::dnn::Net net_;
    std::vector<cv::String> output_names_;
    
    /**
     * @brief 後処理
     * @param output モデル出力
     * @param original_size 元画像サイズ
     * @return 検出結果
     */
    std::vector<DetectionData> postProcess(const cv::Mat& output, const cv::Size& original_size);
    
    /**
     * @brief NMS（Non-Maximum Suppression）適用
     * @param detections 検出結果
     * @return NMS適用後の検出結果
     */
    std::vector<DetectionData> applyNMS(const std::vector<DetectionData>& detections);
    
    /**
     * @brief IoU計算
     * @param box1 バウンディングボックス1
     * @param box2 バウンディングボックス2
     * @return IoU値
     */
    float calculateIoU(const cv::Rect2f& box1, const cv::Rect2f& box2);
};

} // namespace fv_object_detector

#endif // YOLOV8_MODEL_HPP 