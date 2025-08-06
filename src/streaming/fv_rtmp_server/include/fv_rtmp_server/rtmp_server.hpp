#pragma once

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/opt.h>
}

#include <functional>
#include <thread>
#include <atomic>
#include <string>
#include <memory>

namespace fv_rtmp_server
{

// Callback for frame data: (data, width, height, timestamp)
using FrameCallback = std::function<void(const uint8_t*, int, int, int64_t)>;

// Callback for status updates: (status, detail)
using StatusCallback = std::function<void(const std::string&, const std::string&)>;

class RTMPServer
{
public:
    RTMPServer();
    ~RTMPServer();

    // Configuration
    void setPort(int port) { port_ = port; }
    void setEndpoint(const std::string& endpoint) { endpoint_ = endpoint; }
    void setLowLatencyMode(bool enable) { low_latency_mode_ = enable; }
    
    // Callbacks
    void setFrameCallback(FrameCallback callback) { frame_callback_ = callback; }
    void setStatusCallback(StatusCallback callback) { status_callback_ = callback; }

    // Server control
    bool start();
    void stop();
    bool isRunning() const { return is_running_; }

    // Statistics
    uint64_t getFramesReceived() const { return frames_received_; }
    uint64_t getBytesReceived() const { return bytes_received_; }

private:
    // Server implementation
    void serverLoop();
    void cleanup();
    
    // RTMP stream handling
    bool openRTMPStream(const std::string& rtmp_url);
    bool processStream();
    
    // FFmpeg setup
    bool setupDecoder(AVStream* stream);
    void processFrame(AVFrame* frame);

    // Configuration
    int port_;
    std::string endpoint_;
    bool low_latency_mode_;

    // Callbacks
    FrameCallback frame_callback_;
    StatusCallback status_callback_;

    // Threading
    std::thread server_thread_;
    std::atomic<bool> should_stop_;
    std::atomic<bool> is_running_;

    // FFmpeg contexts
    AVFormatContext* format_context_;
    AVCodecContext* codec_context_;
    const AVCodec* codec_;
    AVFrame* frame_;
    AVPacket* packet_;
    
    // Stream info
    int video_stream_index_;
    
    // Statistics
    std::atomic<uint64_t> frames_received_;
    std::atomic<uint64_t> bytes_received_;

    // Network
    int server_socket_;
};

} // namespace fv_rtmp_server 