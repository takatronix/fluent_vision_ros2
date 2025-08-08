#ifndef ASPARA_ANALYZER_THREAD_HPP
#define ASPARA_ANALYZER_THREAD_HPP

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>

#include "aspara_info.hpp"

namespace fv_aspara_analyzer {

/**
 * @brief アスパラガス解析専用スレッドクラス
 * @details 重い点群処理を非同期で実行し、メインスレッドのブロッキングを回避
 */
class AnalyzerThread {
public:
    /**
     * @brief コンストラクタ
     * @param node_ptr 親ノードのポインタ（解析処理用）
     */
    explicit AnalyzerThread(class FvAsparaAnalyzerNode* node_ptr);
    
    /**
     * @brief デストラクタ
     * @details スレッドの安全な終了処理を実行
     */
    ~AnalyzerThread();
    
    /**
     * @brief アスパラガス解析をキューに追加
     * @param aspara_info 解析対象のアスパラガス情報
     */
    void enqueueAnalysis(const AsparaInfo& aspara_info);
    
    /**
     * @brief 現在処理中かどうかを確認
     * @return 処理中の場合true
     */
    bool isProcessing() const { return processing_in_progress_; }

private:
    /**
     * @brief ワーカースレッドのメインループ
     * @details キューからタスクを取得して解析処理を実行
     */
    void workerLoop();
    
    /**
     * @brief アスパラガス処理（スレッド内実行）
     * @param aspara_info アスパラガス情報
     */
    void processAsparagus(AsparaInfo& aspara_info);
    
    /**
     * @brief 解析結果をメインデータに反映
     * @param aspara_info 解析済みアスパラガス情報
     * @param processing_time_ms 処理時間（ミリ秒）
     */
    void updateAnalysisResult(const AsparaInfo& aspara_info, long processing_time_ms);
    
    // スレッド管理
    std::thread worker_thread_;
    std::atomic<bool> shutdown_flag_;
    std::atomic<bool> processing_in_progress_;
    
    // 最新データとミューテックス
    AsparaInfo latest_aspara_info_;
    std::atomic<bool> has_new_data_;
    std::mutex data_mutex_;
    std::condition_variable data_cv_;
    
    // 親ノードへの参照
    FvAsparaAnalyzerNode* node_;
    
    // 点群プロセッサ
    std::unique_ptr<class AsparaPointcloudProcessor> pointcloud_processor_;
};

} // namespace fv_aspara_analyzer

#endif // ASPARA_ANALYZER_THREAD_HPP