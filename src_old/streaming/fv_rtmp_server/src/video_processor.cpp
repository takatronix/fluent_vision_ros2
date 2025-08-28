#include "fv_rtmp_server/video_processor.hpp"

extern "C" {
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

#include <cstring>
#include <chrono>

namespace fv_rtmp_server
{

VideoProcessor::VideoProcessor()
    : target_width_(1280),
      target_height_(720),
      frame_id_("camera_link"),
      quality_mode_("fast"),
      sws_context_(nullptr),
      is_initialized_(false),
      input_width_(0),
      input_height_(0),
      input_format_(AV_PIX_FMT_NONE),
      rgb_buffer_(nullptr),
      rgb_buffer_size_(0),
      frames_processed_(0),
      last_process_time_(std::chrono::steady_clock::now())
{
}

VideoProcessor::~VideoProcessor()
{
    cleanup();
}

void VideoProcessor::setTargetSize(int width, int height)
{
    if (width != target_width_ || height != target_height_) {
        target_width_ = width;
        target_height_ = height;
        
        // Reset to reinitialize with new size
        reset();
    }
}

bool VideoProcessor::processFrame(const uint8_t* input_data, int input_width, int input_height,
                                 int64_t timestamp, sensor_msgs::msg::Image& output_msg)
{
    if (!input_data) {
        return false;
    }

    // Initialize scaler if needed
    if (!is_initialized_ || input_width_ != input_width || input_height_ != input_height) {
        if (!initializeScaler(input_width, input_height)) {
            return false;
        }
    }

    // Prepare input frame info
    const uint8_t* src_data[4] = { input_data, nullptr, nullptr, nullptr };
    int src_linesize[4] = { input_width * 3, 0, 0, 0 }; // Assuming RGB24 input

    // Prepare output frame info
    uint8_t* dst_data[4] = { rgb_buffer_, nullptr, nullptr, nullptr };
    int dst_linesize[4] = { target_width_ * 3, 0, 0, 0 };

    // Scale the frame
    int result = sws_scale(sws_context_,
                          src_data, src_linesize, 0, input_height,
                          dst_data, dst_linesize);

    if (result <= 0) {
        return false;
    }

    // Fill ROS2 Image message
    fillImageMessage(output_msg, rgb_buffer_, target_width_, target_height_, timestamp);

    frames_processed_++;
    last_process_time_ = std::chrono::steady_clock::now();

    return true;
}

bool VideoProcessor::initializeScaler(int input_width, int input_height)
{
    // Clean up existing context
    cleanup();

    input_width_ = input_width;
    input_height_ = input_height;
    input_format_ = AV_PIX_FMT_RGB24; // Assuming RGB input

    // Create scaling context
    sws_context_ = createScaler(input_width_, input_height_, target_width_, target_height_);
    if (!sws_context_) {
        return false;
    }

    // Allocate output buffer
    rgb_buffer_size_ = target_width_ * target_height_ * 3; // RGB24
    rgb_buffer_ = new uint8_t[rgb_buffer_size_];
    
    if (!rgb_buffer_) {
        cleanup();
        return false;
    }

    is_initialized_ = true;
    return true;
}

SwsContext* VideoProcessor::createScaler(int src_width, int src_height, int dst_width, int dst_height)
{
    int sws_flags;
    
    // Choose scaling algorithm based on quality mode
    if (quality_mode_ == "fast") {
        sws_flags = SWS_FAST_BILINEAR;
    } else if (quality_mode_ == "quality") {
        sws_flags = SWS_BICUBIC;
    } else { // balanced
        sws_flags = SWS_BILINEAR;
    }

    return sws_getContext(
        src_width, src_height, input_format_,
        dst_width, dst_height, AV_PIX_FMT_RGB24,
        sws_flags, nullptr, nullptr, nullptr);
}

void VideoProcessor::fillImageMessage(sensor_msgs::msg::Image& msg, const uint8_t* data,
                                     int width, int height, int64_t timestamp)
{
    // Set header
    msg.header.stamp.sec = timestamp / 1000000;
    msg.header.stamp.nanosec = (timestamp % 1000000) * 1000;
    msg.header.frame_id = frame_id_;

    // Set image properties
    msg.height = height;
    msg.width = width;
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = width * 3; // 3 bytes per pixel for RGB

    // Copy image data
    size_t data_size = msg.step * msg.height;
    msg.data.resize(data_size);
    std::memcpy(msg.data.data(), data, data_size);
}

void VideoProcessor::reset()
{
    cleanup();
    is_initialized_ = false;
}

void VideoProcessor::cleanup()
{
    if (sws_context_) {
        sws_freeContext(sws_context_);
        sws_context_ = nullptr;
    }

    if (rgb_buffer_) {
        delete[] rgb_buffer_;
        rgb_buffer_ = nullptr;
    }

    rgb_buffer_size_ = 0;
    is_initialized_ = false;
}

} // namespace fv_rtmp_server 