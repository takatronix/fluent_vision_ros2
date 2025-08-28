/**
 * @file ai_model.hpp
 * @brief Fluent Vision AIモデル基底クラス
 * @details 物体検出AIモデルの共通インターフェースとファクトリパターンを提供
 * @author Takashi Otsuka
 * @date 2024
 * @version 1.0
 */

#ifndef FV_OBJECT_DETECTOR_AI_MODEL_HPP_
#define FV_OBJECT_DETECTOR_AI_MODEL_HPP_

// ===== インクルードファイル =====

// Fluent Vision関連
#include "fv_object_detector/detection_data.hpp"

// 標準ライブラリ
#include <string>
#include <memory>
#include <vector>

// 外部ライブラリ
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <openvino/openvino.hpp>

namespace fv_object_detector
{

/**
 * @class AIModel
 * @brief AIモデルの基底クラス
 * @details 物体検出AIモデルの共通インターフェースを定義
 * 
 * 主な機能：
 * - モデル設定の読み込み・初期化
 * - 前処理・後処理の共通実装
 * - ファクトリパターンによるモデル生成
 * - 推論時間の計測・管理
 * 
 * 対応モデル：
 * - YOLOv8（ONNX形式）
 * - YOLOv10（ONNX形式）
 * - その他OpenVINO対応モデル
 */
class AIModel {
public:
    /**
     * @brief コンストラクタ
     * @details 基本メンバー変数の初期化
     */
    AIModel();
    
    /**
     * @brief デストラクタ
     * @details 仮想デストラクタ（継承クラスでの適切なリソース解放を保証）
     */
    virtual ~AIModel() = default;

    // ===== ファクトリ関数群 =====
    
    /**
     * @brief 設定ファイルパスからAIモデルインスタンスを生成するファクトリ関数
     * @param config_path モデル設定ファイルのパス（JSON形式）
     * @return std::unique_ptr<AIModel> 生成されたAIModel派生クラスのインスタンス
     * @details JSON設定ファイルを読み込み、モデルタイプに応じて適切な派生クラスを生成
     * 
     * @note 対応形式：YOLOv8, YOLOv10
     * @note 設定ファイルにはmodel_type, model_path, device等の情報が必要
     */
    static std::unique_ptr<AIModel> create(const std::string& config_path);
    
    /**
     * @brief 設定JSONからAIモデルインスタンスを生成するファクトリ関数
     * @param config_json モデル設定JSONオブジェクト
     * @return std::unique_ptr<AIModel> 生成されたAIModel派生クラスのインスタンス
     * @details 既に読み込まれたJSONオブジェクトから直接モデルを生成
     * 
     * @note メモリ効率が良い（ファイルI/O不要）
     * @note 動的設定変更時に有用
     */
    static std::unique_ptr<AIModel> createFromConfig(const nlohmann::json& config_json);

    // ===== 基本操作関数群 =====
    
    /**
     * @brief 設定ファイルを読み込む
     * @param path 設定ファイルパス
     * @details JSON形式の設定ファイルを読み込み、config_メンバーに保存
     * 
     * @note ファイル形式：UTF-8エンコードのJSON
     * @note エラーハンドリング：ファイル不存在時は例外をスロー
     */
    virtual void loadConfig(const std::string& path);

    /**
     * @brief モデルの初期化（派生クラスで実装）
     * @param model_cfg モデル設定JSONオブジェクト
     * @details OpenVINOモデルの読み込み、コンパイル、デバイス割り当てを実行
     * 
     * @note 純粋仮想関数：派生クラスで必ず実装が必要
     * @note 初期化失敗時は例外をスロー
     */
    virtual void initialize(const nlohmann::json& model_cfg) = 0;

    /**
     * @brief 推論実行（派生クラスで実装）
     * @param image 入力画像（BGR形式）
     * @return std::vector<DetectionData> 検出結果の配列
     * @details 画像の前処理→推論→後処理の一連の流れを実行
     * 
     * @note 純粋仮想関数：派生クラスで必ず実装が必要
     * @note 推論時間は自動的に計測・保存される
     * @note 検出結果は信頼度順にソートされる
     */
    virtual std::vector<DetectionData> infer(const cv::Mat& image) = 0;

    // ===== ユーティリティ関数群 =====
    
    /**
     * @brief バウンディングボックス座標を別解像度へスケーリング
     * @param bbox 元のバウンディングボックス（正規化座標）
     * @param from_size 元画像サイズ（width, height）
     * @param to_size 変換後画像サイズ（width, height）
     * @return cv::Rect2f スケーリング後のバウンディングボックス
     * @details 画像リサイズ時の座標変換を正確に実行
     * 
     * @note アスペクト比の変更にも対応
     * @note 座標は画像境界内にクリップされる
     */
    static cv::Rect2f scaleCoordinates(const cv::Rect2f& bbox, const cv::Size& from_size, const cv::Size& to_size);

    /**
     * @brief 画像をblob形式に変換
     * @param image 入力画像（BGR形式）
     * @param width 目標幅（ピクセル）
     * @param height 目標高さ（ピクセル）
     * @return cv::Mat blob形式の画像（NCHW形式）
     * @details OpenVINO推論用の前処理を実行
     * 
     * @note リサイズ、正規化、チャンネル変換を一括実行
     * @note メモリ効率の良い連続メモリレイアウト
     */
    static cv::Mat blobFromImage(const cv::Mat& image, int width, int height);

    // ===== ゲッター関数群 =====
    
    /**
     * @brief モデル名を取得
     * @return std::string モデル名
     * @details 設定ファイルで指定されたモデルの識別名
     */
    std::string getModelName() const { return name_; }

    /**
     * @brief デバイス名を取得
     * @return std::string デバイス名
     * @details 推論実行デバイス（CPU, GPU, MYRIAD等）
     */
    std::string getDevice() const { return device_; }

    /**
     * @brief モデルタイプを取得
     * @return std::string モデルタイプ
     * @details モデルの種類（yolov8, yolov10等）
     */
    std::string getModelType() const { return config_.contains("type") ? config_["type"].get<std::string>() : ""; }

    /**
     * @brief クラス名を取得
     * @param class_id クラスID（0ベース）
     * @return std::string クラス名
     * @details クラスIDに対応する人間が読めるクラス名
     * 
     * @note クラスIDが範囲外の場合は"unknown"を返す
     */
    std::string getClassName(int class_id) const;

    /**
     * @brief 推論時間を設定
     * @param ms 推論時間（ミリ秒）
     * @details パフォーマンス監視用の推論時間を記録
     */
    void setLastInferTime(double ms) { last_infer_time_ms_ = ms; }

    /**
     * @brief 推論時間を取得
     * @return double 推論時間（ミリ秒）
     * @details 最後の推論実行にかかった時間を取得
     */
    double getLastInferTime() const { return last_infer_time_ms_; }

protected:
    // ===== 保護メンバー変数群 =====
    
    nlohmann::json config_;                    ///< モデル設定ファイル内容（JSON形式）
    std::vector<std::string> class_names_;     ///< クラス名リスト（COCO等の標準クラス名）
    std::string model_path_;                   ///< モデルファイルのパス（.onnx, .xml等）
    std::string device_;                       ///< 推論実行デバイス名（CPU, GPU, MYRIAD等）
    std::string name_;                         ///< モデル名（設定ファイルで指定）
    int input_width_ = 0;                      ///< モデル入力画像の幅（ピクセル）
    int input_height_ = 0;                     ///< モデル入力画像の高さ（ピクセル）
    double last_infer_time_ms_ = 0.0;          ///< 最後の推論時間（ミリ秒）

    // ===== 保護メソッド群 =====
    
    /**
     * @brief 共通設定を設定
     * @param model_cfg モデル設定JSONオブジェクト
     * @param config_json 全体設定JSONオブジェクト
     * @details 派生クラスで共通の設定項目を初期化
     * 
     * @note モデルパス、デバイス、クラス名等の基本設定
     */
    void setCommonConfig(const nlohmann::json& model_cfg, const nlohmann::json& config_json);

    /**
     * @brief モデルの入出力情報を表示
     * @param compiled_model コンパイル済みOpenVINOモデル
     * @details デバッグ用のモデル情報をコンソールに出力
     * 
     * @note 入力テンソルの形状、出力テンソルの形状等を表示
     */
    static void printModelIOInfo(const ov::CompiledModel& compiled_model);

    /**
     * @brief sigmoid関数
     * @param x 入力値
     * @return float sigmoid値（0.0-1.0）
     * @details 活性化関数として使用される標準的なsigmoid関数
     * 
     * @note 数値安定性を考慮した実装
     */
    static float sigmoid(float x) { return 1.0f / (1.0f + exp(-x)); }
};

} // namespace fv_object_detector

#endif // FV_OBJECT_DETECTOR_AI_MODEL_HPP_ 