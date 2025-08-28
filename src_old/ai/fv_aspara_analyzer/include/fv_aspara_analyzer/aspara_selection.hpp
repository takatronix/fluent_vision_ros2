/**
 * @file aspara_selection.hpp
 * @brief アスパラガス選択管理クラス
 * @details 複数アスパラのID管理、追跡、選択状態管理を担当
 * @author Takashi Otsuka
 * @date 2025-08-08
 */

#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fv_aspara_analyzer
{

// 前方宣言
struct AsparaInfo;

/**
 * @class AsparaSelection
 * @brief アスパラガス選択・ID管理クラス
 */
class AsparaSelection
{
public:
    /**
     * @brief コンストラクタ
     */
    AsparaSelection();

    /**
     * @brief デストラクタ
     */
    ~AsparaSelection() = default;

    /**
     * @brief 新しい検出結果から既存アスパラを見つけるか新規作成
     * @param bbox 新しい検出のバウンディングボックス
     * @param confidence 信頼度
     * @return アスパラガス情報（既存または新規）
     */
    AsparaInfo findOrCreateAsparagus(const cv::Rect& bbox, float confidence);

    /**
     * @brief バウンディングボックスの重複度を計算
     * @param bbox1 バウンディングボックス1
     * @param bbox2 バウンディングボックス2
     * @return 重複度（0.0-1.0）
     */
    float calculateOverlap(const cv::Rect& bbox1, const cv::Rect& bbox2);

    /**
     * @brief 次のアスパラガスを選択
     * @param aspara_list 現在のアスパラリスト
     * @return 次のアスパラID
     */
    int selectNextAsparagus(const std::vector<AsparaInfo>& aspara_list);

    /**
     * @brief 前のアスパラガスを選択
     * @param aspara_list 現在のアスパラリスト
     * @return 前のアスパラID
     */
    int selectPrevAsparagus(const std::vector<AsparaInfo>& aspara_list);

    /**
     * @brief 選択中アスパラIDを取得
     * @return 選択中アスパラID（-1なら選択なし）
     */
    int getSelectedAsparaId() const { return selected_aspara_id_; }

    /**
     * @brief 選択中アスパラIDを設定
     * @param id 選択するアスパラID
     */
    void setSelectedAsparaId(int id) { selected_aspara_id_ = id; }

    /**
     * @brief アスパラリストを更新してIDを管理
     * @param new_detections 新しい検出結果リスト
     * @param existing_aspara_list 既存アスパラリスト
     * @return 更新されたアスパラリスト
     */
    std::vector<AsparaInfo> updateAsparaList(
        const std::vector<std::pair<cv::Rect, float>>& new_detections,
        const std::vector<AsparaInfo>& existing_aspara_list);

    /**
     * @brief 最適な候補アスパラを選択（距離優先、信頼度次点）
     * @param aspara_list アスパラリスト
     * @return 最適なアスパラID
     */
    int selectBestCandidate(const std::vector<AsparaInfo>& aspara_list);

private:
    int selected_aspara_id_;        ///< 選択中アスパラID
    int next_aspara_id_;           ///< 次に割り当てるID
    float overlap_threshold_;       ///< 重複度閾値（70%）
    
    /**
     * @brief 新しいIDを生成
     * @return 新しいアスパラID
     */
    int generateNewId();
};

} // namespace fv_aspara_analyzer