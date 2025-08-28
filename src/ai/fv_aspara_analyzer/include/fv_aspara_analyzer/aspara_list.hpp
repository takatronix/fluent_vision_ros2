#pragma once

#include <vector>
#include <map>
#include <opencv2/opencv.hpp>

namespace fv_aspara_analyzer {

/**
 * @brief アスパラ情報の優先順位と選択操作を一元管理する軽量クラス
 * - 面積降順の優先順位リストを検出更新時に一度だけ構築
 * - SelectNext/Prev は保持済みの優先順位上で循環
 * - SelectByPos は優先順位順で最初にヒットしたIDを返す
 */
class AsparaList {
public:
    AsparaList() = default;

    // 優先順位更新（面積降順）。矩形マップは外部の最新を参照して使う
    void updatePriority(const std::map<int, cv::Rect> &idToRect) {
        priorityIds_.clear(); priorityIds_.reserve(idToRect.size());
        // 昇順の map を優先順位に積み直す（面積降順で並べ替え）
        std::vector<std::pair<int, cv::Rect>> items;
        items.reserve(idToRect.size());
        for (const auto &kv : idToRect) { items.emplace_back(kv.first, kv.second); }
        std::sort(items.begin(), items.end(), [](const auto &a, const auto &b){ return a.second.area() > b.second.area(); });
        for (const auto &p : items) { priorityIds_.push_back(p.first); }
    }

    // 次へ（選択IDが未設定であれば先頭）。戻り値: 新しいselected_id（変化なければ元の値）
    int selectNext(int currentSelectedId) const {
        if (priorityIds_.empty()) return -1;
        if (currentSelectedId < 0) return priorityIds_.front();
        auto it = std::find(priorityIds_.begin(), priorityIds_.end(), currentSelectedId);
        if (it == priorityIds_.end() || std::next(it) == priorityIds_.end()) return priorityIds_.front();
        return *std::next(it);
    }

    // 前へ（循環）
    int selectPrev(int currentSelectedId) const {
        if (priorityIds_.empty()) return -1;
        if (currentSelectedId < 0) return priorityIds_.front();
        auto it = std::find(priorityIds_.begin(), priorityIds_.end(), currentSelectedId);
        if (it == priorityIds_.begin() || it == priorityIds_.end()) return priorityIds_.back();
        return *std::prev(it);
    }

    // 位置で選択（優先順位順で最初にヒット）
    int selectByPosition(int x, int y, const std::map<int, cv::Rect> &idToRect) const {
        for (int id : priorityIds_) {
            auto it = idToRect.find(id);
            if (it == idToRect.end()) continue;
            if (it->second.contains(cv::Point(x, y))) return id;
        }
        return -1;
    }

    const std::vector<int>& priority() const { return priorityIds_; }

private:
    std::vector<int> priorityIds_;
};

} // namespace fv_aspara_analyzer


