#include "fv_object_detector/ai_model.hpp"
#include "fv_object_detector/yolov10_model.hpp"
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

namespace fv_object_detector
{

AIModel::AIModel() : last_infer_time_ms_(0.0)
{
}

/**
 * @brief バウンディングボックス座標を別解像度へスケーリング
 * @param bbox 元のバウンディングボックス
 * @param from_size 元画像サイズ
 * @param to_size 変換後画像サイズ
 * @return スケーリング後のバウンディングボックス
 */
cv::Rect2f AIModel::scaleCoordinates(const cv::Rect2f& bbox, const cv::Size& from_size, const cv::Size& to_size) {
    float x_scale = static_cast<float>(to_size.width) / from_size.width;
    float y_scale = static_cast<float>(to_size.height) / from_size.height;
    return cv::Rect2f(
        bbox.x * x_scale,
        bbox.y * y_scale,
        bbox.width * x_scale,
        bbox.height * y_scale
    );
}

/**
 * @brief 画像をblob形式に変換
 * @param image 入力画像
 * @param width 目標幅
 * @param height 目標高さ
 * @return blob形式の画像
 */
cv::Mat AIModel::blobFromImage(const cv::Mat& image, int width, int height) {
    cv::Mat blob = cv::dnn::blobFromImage(
        image,              // 入力画像
        1.0f / 255.0f,      // スケーリングファクター
        cv::Size(width, height), // モデル入力サイズ
        cv::Scalar(),       // 平均値（デフォルト0）
        true,               // RGB<->BGR変換
        false,              // チャンネルごとに分割しない
        CV_32F              // 32ビット浮動小数点型
    );
    return blob;
}

/**
 * @brief 設定ファイルパスからAIモデルインスタンスを生成するファクトリ関数
 * @param config_path モデル設定ファイルのパス（JSON）
 * @return std::unique_ptr<AIModel> 生成されたAIModel派生クラスのインスタンス
 */
std::unique_ptr<AIModel> AIModel::create(const std::string& config_path) {
    // 設定ファイルを読み込む
    std::ifstream ifs(config_path);
    if (!ifs) {
        throw std::runtime_error("[AIModel::create] 設定ファイルが開けません: " + config_path);
    }
    nlohmann::json config_json;
    ifs >> config_json;
    return createFromConfig(config_json);
}

/**
 * @brief 設定JSONからAIモデルインスタンスを生成するファクトリ関数
 * @param config_json モデル設定JSON
 * @return std::unique_ptr<AIModel> 生成されたAIModel派生クラスのインスタンス
 */
std::unique_ptr<AIModel> AIModel::createFromConfig(const nlohmann::json& config_json) {
    const auto& model_cfg = config_json["model"];
    std::string type = model_cfg["type"];
    
    if (type == "yolov10") {
        auto model = std::make_unique<YoloV10Model>();
        model->setCommonConfig(model_cfg, config_json);
        model->initialize(model_cfg);
        return model;
    }
    
    throw std::runtime_error("Unknown model type: " + type);
}

/**
 * @brief 共通設定を設定
 * @param model_cfg モデル設定
 * @param config_json 設定JSON
 */
void AIModel::setCommonConfig(const nlohmann::json& model_cfg, const nlohmann::json& config_json) {
    RCLCPP_INFO(rclcpp::get_logger("AIModel"), "[AIModel] setCommonConfig");
    name_ = model_cfg["name"].get<std::string>();
    device_ = model_cfg["device"].get<std::string>();
    input_width_ = model_cfg["input_width"].get<int>();
    input_height_ = model_cfg["input_height"].get<int>();
    model_path_ = model_cfg["path"].get<std::string>();
    config_ = config_json;

    RCLCPP_INFO(rclcpp::get_logger("AIModel"), "[AIModel] モデル名: %s", name_.c_str());
    RCLCPP_INFO(rclcpp::get_logger("AIModel"), "[AIModel] デバイス: %s", device_.c_str());
    RCLCPP_INFO(rclcpp::get_logger("AIModel"), "[AIModel] 入力サイズ: %dx%d", input_width_, input_height_);
    RCLCPP_INFO(rclcpp::get_logger("AIModel"), "[AIModel] モデルパス: %s", model_path_.c_str());
    
    // クラス名リストを読み込む
    if (config_json.contains("classes") && config_json["classes"].is_array()) {
        class_names_.clear();
        for (const auto& c : config_json["classes"]) {
            class_names_.push_back(c.get<std::string>());
        }
        
        std::string class_list = "[AIModel] クラス名リスト: ";
        for (size_t i = 0; i < class_names_.size(); ++i) {
            class_list += "[" + std::to_string(i) + "]" + class_names_[i] + ", ";
        }
        RCLCPP_INFO(rclcpp::get_logger("AIModel"), "%s", class_list.c_str());
    }
    RCLCPP_INFO(rclcpp::get_logger("AIModel"), "[AIModel] クラス名リスト: %zu", class_names_.size());
}

/**
 * @brief 設定ファイルを読み込む
 * @param path 設定ファイルパス
 */
void AIModel::loadConfig(const std::string& path) {
    RCLCPP_INFO(rclcpp::get_logger("AIModel"), "[AIModel] loadConfig");
    std::ifstream ifs(path);
    if (!ifs) {
        RCLCPP_ERROR(rclcpp::get_logger("AIModel"), "設定ファイルが開けません: %s", path.c_str());
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("AIModel"), "[AIModel] 設定ファイル読み込み開始: %s", path.c_str());
    try {
        nlohmann::json config_json;
        ifs >> config_json;
        config_ = config_json;
        RCLCPP_INFO(rclcpp::get_logger("AIModel"), "[AIModel] 設定ファイル読み込み成功: %s", path.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("AIModel"), "設定ファイル読み込みエラー: %s", e.what());
    }
}

/**
 * @brief クラス名を取得
 * @param class_id クラスID
 * @return クラス名
 */
std::string AIModel::getClassName(int class_id) const {
    if (class_id >= 0 && class_id < (int)class_names_.size()) {
        return class_names_[class_id];
    }
    return std::to_string(class_id);
}

/**
 * @brief モデルの入出力情報を表示
 * @param compiled_model コンパイルされたモデル
 */
void AIModel::printModelIOInfo(const ov::CompiledModel& compiled_model) {
    // 入力の形状・型・名前を取得
    auto& inputs = compiled_model.inputs();
    for (const auto& input : inputs) {
        auto partial_shape = input.get_partial_shape();
        auto elem_type = input.get_element_type();
        auto names = input.get_names();
        std::string name = names.empty() ? "(no name)" : *names.begin();
        RCLCPP_DEBUG(rclcpp::get_logger("AIModel"), "[ModelInfo] Input name: %s, partial shape: %s, type: %s", 
                     name.c_str(), partial_shape.to_string().c_str(), elem_type.to_string().c_str());
    }
    
    // 出力の形状・型・名前を取得
    auto& outputs = compiled_model.outputs();
    for (const auto& output : outputs) {
        auto partial_shape = output.get_partial_shape();
        auto elem_type = output.get_element_type();
        auto names = output.get_names();
        std::string name = names.empty() ? "(no name)" : *names.begin();
        RCLCPP_DEBUG(rclcpp::get_logger("AIModel"), "[ModelInfo] Output name: %s, partial shape: %s, type: %s", 
                     name.c_str(), partial_shape.to_string().c_str(), elem_type.to_string().c_str());
    }
}

} // namespace fv_object_detector 