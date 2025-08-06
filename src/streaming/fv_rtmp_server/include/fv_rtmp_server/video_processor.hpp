#pragma once

extern "C" {
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/pixfmt.h>
}

#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

#include <memory>
#include <string>

namespace fv_rtmp_server
{

class VideoProcessor
{
public:
    VideoProcessor();
    ~VideoProcessor();

    // Configuration
    void setTargetSize(int width, int height);
    void setFrameId(const std::string& frame_id) { frame_id_ = frame_id; }
    void setQualityMode(const std::string& mode) { quality_mode_ = mode; }

    // Frame processing
    bool processFrame(const uint8_t* input_data, int input_width, int input_height,
                     int64_t timestamp, sensor_msgs::msg::Image& output_msg);

    // Utility
    bool isInitialized() const { return is_initialized_; }
    void reset();

private:
    // Initialization
    bool initializeScaler(int input_width, int input_height);
    void cleanup();

    // Processing helpers
    SwsContext* createScaler(int src_width, int src_height, int dst_width, int dst_height);
    void fillImageMessage(sensor_msgs::msg::Image& msg, const uint8_t* data,
                         int width, int height, int64_t timestamp);

    // Configuration
    int target_width_;
    int target_height_;
    std::string frame_id_;
    std::string quality_mode_;

    // Scaling context
    SwsContext* sws_context_;
    bool is_initialized_;

    // Input/output format info
    int input_width_;
    int input_height_;
    AVPixelFormat input_format_;
    
    // Buffer management
    uint8_t* rgb_buffer_;
    size_t rgb_buffer_size_;
    
    // Performance tracking
    uint64_t frames_processed_;
    std::chrono::steady_clock::time_point last_process_time_;
};

} // namespace fv_rtmp_server 