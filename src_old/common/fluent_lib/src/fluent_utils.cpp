#include "utils/fps.hpp"
#include <algorithm>

namespace fluent {
namespace utils {

// =============================================================================
// FPSMeter実装
// =============================================================================

FPSMeter::FPSMeter(size_t window_size) 
    : window_size_(window_size), frame_count_(0) {
    last_time_ = std::chrono::high_resolution_clock::now();
}

void FPSMeter::tick() {
    auto now = std::chrono::high_resolution_clock::now();
    timestamps_.push_back(now);
    
    // ウィンドウサイズを超えた古いタイムスタンプを削除
    while (timestamps_.size() > window_size_) {
        timestamps_.pop_front();
    }
    
    frame_count_++;
    last_time_ = now;
}

void FPSMeter::tick(const rclcpp::Time& timestamp) {
    // ROS時刻は現在は無視して、システム時刻を使用
    tick();
}

double FPSMeter::getCurrentFPS() const {
    if (timestamps_.size() < 2) {
        return 0.0;
    }
    
    auto duration = std::chrono::duration<double>(timestamps_.back() - timestamps_.front()).count();
    if (duration > 0.0) {
        return (timestamps_.size() - 1) / duration;
    }
    return 0.0;
}

double FPSMeter::getAverageFPS() const {
    return getCurrentFPS();
}

void FPSMeter::reset() {
    timestamps_.clear();
    frame_count_ = 0;
    last_time_ = std::chrono::high_resolution_clock::now();
}

// =============================================================================
// Stopwatch実装
// =============================================================================

Stopwatch::Stopwatch() : running_(false) {
    start_time_ = std::chrono::high_resolution_clock::now();
    end_time_ = start_time_;
}

void Stopwatch::start() {
    start_time_ = std::chrono::high_resolution_clock::now();
    running_ = true;
}

void Stopwatch::stop() {
    if (running_) {
        end_time_ = std::chrono::high_resolution_clock::now();
        running_ = false;
    }
}

void Stopwatch::reset() {
    start_time_ = std::chrono::high_resolution_clock::now();
    end_time_ = start_time_;
    running_ = true;
}

double Stopwatch::elapsed_ms() const {
    auto end_time = running_ ? std::chrono::high_resolution_clock::now() : end_time_;
    return std::chrono::duration<double, std::milli>(end_time - start_time_).count();
}

double Stopwatch::elapsed_sec() const {
    auto end_time = running_ ? std::chrono::high_resolution_clock::now() : end_time_;
    return std::chrono::duration<double>(end_time - start_time_).count();
}

} // namespace utils
} // namespace fluent