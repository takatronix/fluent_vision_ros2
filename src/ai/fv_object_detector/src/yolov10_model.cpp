#include "fv_object_detector/yolov10_model.hpp"
#include "fv_object_detector/ai_model.hpp"
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

namespace fv_object_detector
{

YoloV10Model::YoloV10Model() {}
YoloV10Model::~YoloV10Model() {}

/**
 * @brief モデル初期化
 * 
 * @param model_cfg モデル設定
 */
void YoloV10Model::initialize(const nlohmann::json& model_cfg) {
    try {
        RCLCPP_INFO(rclcpp::get_logger("YoloV10Model"), "モデルの読み込み開始: %s", model_path_.c_str());
        
        // 設定からパラメータを読み込み
        if (model_cfg.contains("confidence_threshold")) {
            min_confidence_ = model_cfg["confidence_threshold"].get<float>();
        }
        if (model_cfg.contains("nms_threshold")) {
            nms_threshold_ = model_cfg["nms_threshold"].get<float>();
        }
        if (model_cfg.contains("min_area")) {
            min_area_ = model_cfg["min_area"].get<float>();
        }
        
        ov::Core core;
        auto model = core.read_model(model_path_);
        compiled_model_ = core.compile_model(model, device_);
        infer_request_ = compiled_model_.create_infer_request();
        
        // モデル情報を表示
        printModelIOInfo(compiled_model_);
        
        RCLCPP_INFO(rclcpp::get_logger("YoloV10Model"), "モデル初期化完了: %s", model_path_.c_str());
        RCLCPP_INFO(rclcpp::get_logger("YoloV10Model"), "信頼度閾値: %f, NMS閾値: %f, 最小面積: %f", 
                    min_confidence_, nms_threshold_, min_area_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("YoloV10Model"), "モデル初期化エラー: %s", e.what());
        throw;
    }
}

/**
 * @brief 推論実行
 * @param image 入力画像
 * @return 推論結果
 */
std::vector<DetectionData> YoloV10Model::infer(const cv::Mat& image) {
    std::vector<DetectionData> detections;
    cv::Mat blob = AIModel::blobFromImage(image, input_width_, input_height_);

    auto input_port = compiled_model_.input();

    // blobのshapeを正しく取得しTensorを作成
    auto input_tensor = ov::Tensor(
        input_port.get_element_type(),
        {static_cast<size_t>(blob.size[0]), static_cast<size_t>(blob.size[1]), 
         static_cast<size_t>(blob.size[2]), static_cast<size_t>(blob.size[3])},
        blob.data
    );

    infer_request_.set_input_tensor(input_tensor);

    // 推論（時間計測開始）
    auto start_time = std::chrono::high_resolution_clock::now();
    try {
        infer_request_.infer();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("YoloV10Model"), "推論エラー: %s", e.what());
        return detections;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    setLastInferTime(duration.count() / 1000.0); // ミリ秒に変換

    // 後処理
    try {
        ov::Tensor output_tensor = infer_request_.get_output_tensor();
        detections = postProcess(output_tensor, image.size());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("YoloV10Model"), "後処理エラー: %s", e.what());
        return std::vector<DetectionData>();
    }
    
    // NMS適用
    detections = applyNMS(detections, nms_threshold_);
    
    RCLCPP_DEBUG(rclcpp::get_logger("YoloV10Model"), "検出数: %zu, 推論時間: %.2f ms", 
                 detections.size(), getLastInferTime());
    
    return detections;
}

/**
 * @brief 推論結果を後処理
 * @param output 推論出力
 * @param original_size 元画像サイズ
 * @return 検出結果
 */
std::vector<DetectionData> YoloV10Model::postProcess(const ov::Tensor& output, const cv::Size& original_size) {
    std::vector<DetectionData> detections;
    
    auto shape = output.get_shape();
    if (shape.size() != 3 || shape[2] < 6) {
        RCLCPP_ERROR(rclcpp::get_logger("YoloV10Model"), "出力テンソルshape異常: [%zu, %zu, %zu]", 
                     shape[0], shape[1], (shape.size() > 2 ? shape[2] : 0));
        return detections;
    }
    
    float* out_data = output.data<float>();
    size_t num_det = shape[1];
    size_t features = shape[2];
    
    for (size_t i = 0; i < num_det; ++i) {
        float* detection = out_data + i * features;
        float confidence = detection[4];

        if (std::isnan(confidence) || confidence < min_confidence_) continue;
        
        float x1 = detection[0];
        float y1 = detection[1];
        float x2 = detection[2];
        float y2 = detection[3];
        
        // 座標を画像範囲内に制限
        x1 = std::max(0.0f, std::min(x1, static_cast<float>(input_width_ - 1)));
        y1 = std::max(0.0f, std::min(y1, static_cast<float>(input_height_ - 1)));
        x2 = std::max(0.0f, std::min(x2, static_cast<float>(input_width_ - 1)));
        y2 = std::max(0.0f, std::min(y2, static_cast<float>(input_height_ - 1)));

        DetectionData det;
        det.bbox = cv::Rect2f(x1, y1, x2 - x1, y2 - y1);
        det.confidence = confidence;
        det.class_id = static_cast<int>(detection[5]);
        det.object_id = -1;
        det.class_name = getClassName(det.class_id);
        det.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count() / 1000.0;
        
        // 座標を元画像サイズにスケーリング
        det.bbox = scaleCoordinates(det.bbox, cv::Size(input_width_, input_height_), original_size);
        
        // 最小面積フィルタ
        if (det.bbox.width * det.bbox.height >= min_area_) {
            detections.push_back(det);
        }
    }
    
    return detections;
}

/**
 * @brief Non-Maximum Suppressionを実行
 * @param detections 検出結果
 * @param threshold NMS閾値
 * @return フィルタリング後の検出結果
 */
std::vector<DetectionData> YoloV10Model::applyNMS(const std::vector<DetectionData>& detections, float threshold) {
    if (detections.empty()) return detections;
    
    std::vector<DetectionData> result;
    std::vector<bool> suppressed(detections.size(), false);
    
    // 信頼度でソート
    std::vector<size_t> indices(detections.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), 
              [&detections](size_t a, size_t b) {
                  return detections[a].confidence > detections[b].confidence;
              });
    
    for (size_t i = 0; i < indices.size(); ++i) {
        if (suppressed[indices[i]]) continue;
        
        result.push_back(detections[indices[i]]);
        
        for (size_t j = i + 1; j < indices.size(); ++j) {
            if (suppressed[indices[j]]) continue;
            
            // 同じクラスの場合のみNMSを適用
            if (detections[indices[i]].class_id == detections[indices[j]].class_id) {
                float iou = calculateIoU(detections[indices[i]].bbox, detections[indices[j]].bbox);
                if (iou > threshold) {
                    suppressed[indices[j]] = true;
                }
            }
        }
    }
    
    return result;
}

/**
 * @brief IoU（Intersection over Union）を計算
 * @param bbox1 バウンディングボックス1
 * @param bbox2 バウンディングボックス2
 * @return IoU値
 */
float YoloV10Model::calculateIoU(const cv::Rect2f& bbox1, const cv::Rect2f& bbox2) {
    float x1 = std::max(bbox1.x, bbox2.x);
    float y1 = std::max(bbox1.y, bbox2.y);
    float x2 = std::min(bbox1.x + bbox1.width, bbox2.x + bbox2.width);
    float y2 = std::min(bbox1.y + bbox1.height, bbox2.y + bbox2.height);
    
    if (x2 <= x1 || y2 <= y1) return 0.0f;
    
    float intersection = (x2 - x1) * (y2 - y1);
    float area1 = bbox1.width * bbox1.height;
    float area2 = bbox2.width * bbox2.height;
    float union_area = area1 + area2 - intersection;
    
    return intersection / union_area;
}

} // namespace fv_object_detector 