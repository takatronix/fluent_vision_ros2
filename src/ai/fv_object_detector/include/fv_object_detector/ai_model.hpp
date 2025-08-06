#ifndef FV_OBJECT_DETECTOR_AI_MODEL_HPP_
#define FV_OBJECT_DETECTOR_AI_MODEL_HPP_

#include "fv_object_detector/detection_data.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <memory>
#include <vector>
#include <nlohmann/json.hpp>
#include <openvino/openvino.hpp>

namespace fv_object_detector
{

/**
 * @brief AIモデルの基底クラス
 * モデル設定の読み込み・初期化・前処理・後処理のインターフェースを提供
 */
class AIModel {
public:
    AIModel();
    virtual ~AIModel() = default;

    /**
     * @brief 設定ファイルパスからAIモデルインスタンスを生成するファクトリ関数
     * @param config_path モデル設定ファイルのパス（JSON）
     * @return std::unique_ptr<AIModel> 生成されたAIModel派生クラスのインスタンス
     */
    static std::unique_ptr<AIModel> create(const std::string& config_path);
    
    /**
     * @brief 設定JSONからAIモデルインスタンスを生成するファクトリ関数
     * @param config_json モデル設定JSON
     * @return std::unique_ptr<AIModel> 生成されたAIModel派生クラスのインスタンス
     */
    static std::unique_ptr<AIModel> createFromConfig(const nlohmann::json& config_json);

    /**
     * @brief 設定ファイルを読み込む
     * @param path 設定ファイルパス
     */
    virtual void loadConfig(const std::string& path);

    /**
     * @brief モデルの初期化（派生クラスで実装）
     * @param model_cfg モデル設定
     */
    virtual void initialize(const nlohmann::json& model_cfg) = 0;

    /**
     * @brief 推論実行（派生クラスで実装）
     * @param image 入力画像
     * @return 推論結果
     */
    virtual std::vector<DetectionData> infer(const cv::Mat& image) = 0;

    /**
     * @brief バウンディングボックス座標を別解像度へスケーリング
     * @param bbox 元のバウンディングボックス
     * @param from_size 元画像サイズ
     * @param to_size 変換後画像サイズ
     * @return スケーリング後のバウンディングボックス
     */
    static cv::Rect2f scaleCoordinates(const cv::Rect2f& bbox, const cv::Size& from_size, const cv::Size& to_size);

    /**
     * @brief 画像をblob形式に変換
     * @param image 入力画像
     * @param width 目標幅
     * @param height 目標高さ
     * @return blob形式の画像
     */
    static cv::Mat blobFromImage(const cv::Mat& image, int width, int height);

    /**
     * @brief モデル名を取得
     * @return モデル名
     */
    std::string getModelName() const { return name_; }

    /**
     * @brief デバイス名を取得
     * @return デバイス名
     */
    std::string getDevice() const { return device_; }

    /**
     * @brief モデルタイプを取得
     * @return モデルタイプ
     */
    std::string getModelType() const { return config_.contains("type") ? config_["type"].get<std::string>() : ""; }

    /**
     * @brief クラス名を取得
     * @param class_id クラスID
     * @return クラス名
     */
    std::string getClassName(int class_id) const;

    /**
     * @brief 推論時間を設定
     * @param ms 推論時間（ミリ秒）
     */
    void setLastInferTime(double ms) { last_infer_time_ms_ = ms; }

    /**
     * @brief 推論時間を取得
     * @return 推論時間（ミリ秒）
     */
    double getLastInferTime() const { return last_infer_time_ms_; }

protected:
    nlohmann::json config_;                    ///< モデル設定ファイル内容（JSON形式）
    std::vector<std::string> class_names_;     ///< クラス名リスト
    std::string model_path_;                   ///< モデルファイルのパス
    std::string device_;                       ///< 推論実行デバイス名（例: CPU, GPU）
    std::string name_;                         ///< モデル名
    int input_width_ = 0;                      ///< モデル入力画像の幅
    int input_height_ = 0;                     ///< モデル入力画像の高さ
    double last_infer_time_ms_ = 0.0;          ///< 最後の推論時間（ミリ秒）

    /**
     * @brief 共通設定を設定
     * @param model_cfg モデル設定
     * @param config_json 設定JSON
     */
    void setCommonConfig(const nlohmann::json& model_cfg, const nlohmann::json& config_json);

    /**
     * @brief モデルの入出力情報を表示
     * @param compiled_model コンパイル済みモデル
     */
    static void printModelIOInfo(const ov::CompiledModel& compiled_model);

    /**
     * @brief sigmoid関数
     * @param x 入力値
     * @return sigmoid値
     */
    static float sigmoid(float x) { return 1.0f / (1.0f + exp(-x)); }
};

} // namespace fv_object_detector

#endif // FV_OBJECT_DETECTOR_AI_MODEL_HPP_ 