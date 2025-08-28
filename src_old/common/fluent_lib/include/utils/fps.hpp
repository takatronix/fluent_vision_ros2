/**
 * @file fps.hpp
 * @brief FPS計測ユーティリティ
 */

#pragma once

#include <chrono>
#include <deque>
#include <rclcpp/rclcpp.hpp>

namespace fluent {
namespace utils {

/**
 * @brief FPS計測クラス
 */
class FPSMeter {
public:
    /**
     * @brief コンストラクタ
     * @param window_size 移動平均のウィンドウサイズ
     */
    explicit FPSMeter(size_t window_size = 30);

    /**
     * @brief フレーム更新
     */
    void tick();
    
    /**
     * @brief フレーム更新（ROS時刻）
     * @param timestamp ROS時刻
     */
    void tick(const rclcpp::Time& timestamp);

    /**
     * @brief 現在のFPS取得
     * @return FPS値
     */
    double getCurrentFPS() const;

    /**
     * @brief 平均FPS取得
     * @return 平均FPS値
     */
    double getAverageFPS() const;

    /**
     * @brief 統計リセット
     */
    void reset();

    /**
     * @brief フレーム数取得
     * @return 総フレーム数
     */
    size_t getFrameCount() const { return frame_count_; }

private:
    size_t window_size_;
    size_t frame_count_;
    std::deque<std::chrono::high_resolution_clock::time_point> timestamps_;
    std::chrono::high_resolution_clock::time_point last_time_;
};

/**
 * @brief 簡単なストップウォッチ
 */
class Stopwatch {
public:
    /**
     * @brief コンストラクタ（自動開始）
     */
    Stopwatch();

    /**
     * @brief 計測開始
     */
    void start();

    /**
     * @brief 計測停止
     */
    void stop();

    /**
     * @brief リセット
     */
    void reset();

    /**
     * @brief 経過時間取得（ミリ秒）
     * @return 経過時間
     */
    double elapsed_ms() const;

    /**
     * @brief 経過時間取得（秒）
     * @return 経過時間
     */
    double elapsed_sec() const;

    /**
     * @brief 実行中かどうか
     * @return 実行状態
     */
    bool isRunning() const { return running_; }

private:
    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point end_time_;
    bool running_;
};

} // namespace utils
} // namespace fluent