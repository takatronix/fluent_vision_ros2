/**
 * @file aspara_selection.cpp
 * @brief アスパラガス選択管理クラス実装
 * @details 複数アスパラのID管理、追跡、選択状態管理の実装
 * @author Takashi Otsuka
 * @date 2025-08-08
 */

#include "fv_aspara_analyzer/aspara_selection.hpp"
#include "fv_aspara_analyzer/fv_aspara_analyzer_node.hpp"
#include <algorithm>

namespace fv_aspara_analyzer
{

AsparaSelection::AsparaSelection()
    : selected_aspara_id_(-1)
    , next_aspara_id_(1)
    , overlap_threshold_(0.7f)  // 70%
{
}

AsparaInfo AsparaSelection::findOrCreateAsparagus(const cv::Rect& bbox, float confidence)
{
    // 既存のアスパラと比較（この関数は既存リストとの比較用に拡張が必要）
    AsparaInfo aspara_info;
    aspara_info.id = generateNewId();  // とりあえず新ID（後で改善）
    aspara_info.confidence = confidence;
    aspara_info.bounding_box_2d = bbox;
    aspara_info.last_update_time = rclcpp::Clock().now();
    aspara_info.is_new = true;
    aspara_info.detection_count = 1;
    aspara_info.overlap_ratio = 0.0f;
    aspara_info.first_detected_time = rclcpp::Clock().now();
    
    return aspara_info;
}

float AsparaSelection::calculateOverlap(const cv::Rect& bbox1, const cv::Rect& bbox2)
{
    // 交差領域を計算
    cv::Rect intersection = bbox1 & bbox2;
    if (intersection.empty()) {
        return 0.0f;
    }
    
    // 交差面積
    int intersection_area = intersection.area();
    
    // 全体面積（和集合）
    int union_area = bbox1.area() + bbox2.area() - intersection_area;
    
    if (union_area == 0) {
        return 0.0f;
    }
    
    // IoU（Intersection over Union）を計算
    return static_cast<float>(intersection_area) / static_cast<float>(union_area);
}

std::vector<AsparaInfo> AsparaSelection::updateAsparaList(
    const std::vector<std::pair<cv::Rect, float>>& new_detections,
    const std::vector<AsparaInfo>& existing_aspara_list)
{
    std::vector<AsparaInfo> updated_list;
    std::vector<bool> existing_matched(existing_aspara_list.size(), false);
    
    // 各新検出に対して既存アスパラとマッチング
    for (const auto& detection : new_detections) {
        const cv::Rect& new_bbox = detection.first;
        float new_confidence = detection.second;
        
        int best_match_id = -1;
        float best_overlap = 0.0f;
        int best_match_index = -1;
        
        // 既存アスパラと重複度チェック
        for (size_t i = 0; i < existing_aspara_list.size(); ++i) {
            if (existing_matched[i]) continue;  // 既にマッチ済み
            
            float overlap = calculateOverlap(new_bbox, existing_aspara_list[i].bounding_box_2d);
            
            if (overlap > overlap_threshold_ && overlap > best_overlap) {
                best_overlap = overlap;
                best_match_id = existing_aspara_list[i].id;
                best_match_index = i;
            }
        }
        
        AsparaInfo aspara_info;
        
        if (best_match_id != -1) {
            // 既存アスパラを更新
            aspara_info = existing_aspara_list[best_match_index];
            aspara_info.confidence = new_confidence;
            
            // fluent_libアニメーション機能を使用したスムーズアニメーション
            // 既存の位置から新しい位置への補間
            using namespace fluent;
            
            // アニメーション係数（0.3で30%の更新率）
            float smooth_factor = 0.3f;
            
            // X座標のスムージング
            aspara_info.smooth_bbox.x = static_cast<int>(
                aspara_info.smooth_bbox.x * (1.0f - smooth_factor) + 
                new_bbox.x * smooth_factor);
            
            // Y座標のスムージング    
            aspara_info.smooth_bbox.y = static_cast<int>(
                aspara_info.smooth_bbox.y * (1.0f - smooth_factor) + 
                new_bbox.y * smooth_factor);
                
            // 幅のスムージング
            aspara_info.smooth_bbox.width = static_cast<int>(
                aspara_info.smooth_bbox.width * (1.0f - smooth_factor) + 
                new_bbox.width * smooth_factor);
                
            // 高さのスムージング
            aspara_info.smooth_bbox.height = static_cast<int>(
                aspara_info.smooth_bbox.height * (1.0f - smooth_factor) + 
                new_bbox.height * smooth_factor);
            
            aspara_info.bounding_box_2d = new_bbox;
            aspara_info.last_update_time = rclcpp::Clock().now();
            aspara_info.detection_count++;
            aspara_info.overlap_ratio = best_overlap;
            aspara_info.is_new = false;
            aspara_info.frame_count++;
            
            existing_matched[best_match_index] = true;
        } else {
            // 新しいアスパラを作成
            aspara_info.id = generateNewId();
            aspara_info.confidence = new_confidence;
            aspara_info.bounding_box_2d = new_bbox;
            aspara_info.last_update_time = rclcpp::Clock().now();
            aspara_info.is_new = true;
            aspara_info.detection_count = 1;
            aspara_info.overlap_ratio = 0.0f;
            aspara_info.first_detected_time = rclcpp::Clock().now();
        }
        
        updated_list.push_back(aspara_info);
    }
    
    return updated_list;
}

int AsparaSelection::selectNextAsparagus(const std::vector<AsparaInfo>& aspara_list)
{
    if (aspara_list.empty()) {
        selected_aspara_id_ = -1;
        return -1;
    }
    
    // 現在選択中のアスパラのインデックスを探す
    int current_index = -1;
    for (size_t i = 0; i < aspara_list.size(); ++i) {
        if (aspara_list[i].id == selected_aspara_id_) {
            current_index = i;
            break;
        }
    }
    
    // 次のアスパラを選択（循環）
    int next_index = (current_index + 1) % aspara_list.size();
    selected_aspara_id_ = aspara_list[next_index].id;
    
    return selected_aspara_id_;
}

int AsparaSelection::selectPrevAsparagus(const std::vector<AsparaInfo>& aspara_list)
{
    if (aspara_list.empty()) {
        selected_aspara_id_ = -1;
        return -1;
    }
    
    // 現在選択中のアスパラのインデックスを探す
    int current_index = -1;
    for (size_t i = 0; i < aspara_list.size(); ++i) {
        if (aspara_list[i].id == selected_aspara_id_) {
            current_index = i;
            break;
        }
    }
    
    // 前のアスパラを選択（循環）
    int prev_index = (current_index - 1 + aspara_list.size()) % aspara_list.size();
    selected_aspara_id_ = aspara_list[prev_index].id;
    
    return selected_aspara_id_;
}

int AsparaSelection::selectBestCandidate(const std::vector<AsparaInfo>& aspara_list)
{
    if (aspara_list.empty()) {
        return -1;
    }
    
    // 距離優先、同じなら信頼度でソート
    auto sorted_list = aspara_list;
    std::sort(sorted_list.begin(), sorted_list.end(),
              [](const AsparaInfo& a, const AsparaInfo& b) {
                  // 矩形面積で距離を推定（大きいほど近い）
                  int area_a = a.bounding_box_2d.width * a.bounding_box_2d.height;
                  int area_b = b.bounding_box_2d.width * b.bounding_box_2d.height;
                  
                  // 距離優先（面積が大きい = 距離が近い）
                  if (area_a != area_b) {
                      return area_a > area_b;  // 面積大きい方を優先
                  }
                  
                  // 距離が同じなら信頼度で比較
                  return a.confidence > b.confidence;
              });
    
    return sorted_list[0].id;
}

int AsparaSelection::generateNewId()
{
    return next_aspara_id_++;
}

} // namespace fv_aspara_analyzer