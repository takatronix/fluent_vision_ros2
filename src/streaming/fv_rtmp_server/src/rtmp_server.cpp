#include "fv_rtmp_server/rtmp_server.hpp"

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

#include <iostream>
#include <cstring>

namespace fv_rtmp_server
{

RTMPServer::RTMPServer()
    : port_(1935),
      endpoint_("/live/s"),
      low_latency_mode_(true),
      should_stop_(false),
      is_running_(false),
      format_context_(nullptr),
      codec_context_(nullptr),
      codec_(nullptr),
      frame_(nullptr),
      packet_(nullptr),
      video_stream_index_(-1),
      frames_received_(0),
      bytes_received_(0),
      server_socket_(-1)
{
    // Initialize FFmpeg
    av_log_set_level(AV_LOG_WARNING); // Reduce log noise
    avformat_network_init();
}

RTMPServer::~RTMPServer()
{
    stop();
    cleanup();
    avformat_network_deinit();
}

bool RTMPServer::start()
{
    if (is_running_) {
        return true;
    }

    should_stop_ = false;
    is_running_ = true;
    
    server_thread_ = std::thread(&RTMPServer::serverLoop, this);
    
    return true;
}

void RTMPServer::stop()
{
    should_stop_ = true;
    is_running_ = false;

    // Close server socket
    if (server_socket_ != -1) {
        close(server_socket_);
        server_socket_ = -1;
    }

    if (server_thread_.joinable()) {
        server_thread_.join();
    }

    cleanup();
}

void RTMPServer::serverLoop()
{
    if (status_callback_) {
        status_callback_("LISTENING", "RTMP server listening on port " + std::to_string(port_));
    }

    while (!should_stop_ && is_running_) {
        // Create RTMP URL for this server
        std::string rtmp_url = "rtmp://0.0.0.0:" + std::to_string(port_) + endpoint_;
        
        if (status_callback_) {
            status_callback_("WAITING", "Waiting for RTMP stream on " + rtmp_url);
        }

        // Try to open RTMP stream
        if (openRTMPStream(rtmp_url)) {
            if (status_callback_) {
                status_callback_("CONNECTED", "RTMP client connected");
            }
            
            // Process the stream
            processStream();
            
            if (status_callback_) {
                status_callback_("DISCONNECTED", "RTMP client disconnected");
            }
        }

        // Clean up and wait before trying again
        cleanup();
        
        if (!should_stop_) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

bool RTMPServer::openRTMPStream(const std::string& rtmp_url)
{
    // Allocate format context
    format_context_ = avformat_alloc_context();
    if (!format_context_) {
        if (status_callback_) {
            status_callback_("ERROR", "Failed to allocate format context");
        }
        return false;
    }

    // Set input format to FLV (RTMP uses FLV)
    AVInputFormat* input_format = av_find_input_format("flv");
    if (!input_format) {
        if (status_callback_) {
            status_callback_("ERROR", "FLV input format not found");
        }
        return false;
    }

    AVDictionary* options = nullptr;
    
    if (low_latency_mode_) {
        av_dict_set(&options, "fflags", "nobuffer", 0);
        av_dict_set(&options, "flags", "low_delay", 0);
        av_dict_set(&options, "analyzeduration", "1", 0);
        av_dict_set(&options, "probesize", "32", 0);
    }
    
    // Set RTMP options
    av_dict_set(&options, "listen", "1", 0);  // Listen mode
    av_dict_set(&options, "timeout", "5000000", 0);  // 5 second timeout

    // Open input stream
    int ret = avformat_open_input(&format_context_, rtmp_url.c_str(), input_format, &options);
    av_dict_free(&options);
    
    if (ret < 0) {
        char error_str[AV_ERROR_MAX_STRING_SIZE];
        av_strerror(ret, error_str, sizeof(error_str));
        if (status_callback_) {
            status_callback_("ERROR", "Failed to open RTMP stream: " + std::string(error_str));
        }
        return false;
    }

    // Find stream info
    ret = avformat_find_stream_info(format_context_, nullptr);
    if (ret < 0) {
        if (status_callback_) {
            status_callback_("ERROR", "Failed to find stream info");
        }
        return false;
    }

    // Find video stream
    for (unsigned int i = 0; i < format_context_->nb_streams; i++) {
        if (format_context_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_stream_index_ = i;
            break;
        }
    }

    if (video_stream_index_ == -1) {
        if (status_callback_) {
            status_callback_("ERROR", "No video stream found");
        }
        return false;
    }

    // Setup decoder
    if (!setupDecoder(format_context_->streams[video_stream_index_])) {
        return false;
    }

    return true;
}

bool RTMPServer::processStream()
{
    if (!format_context_ || !codec_context_) {
        return false;
    }

    // Allocate frame and packet
    packet_ = av_packet_alloc();
    frame_ = av_frame_alloc();
    
    if (!packet_ || !frame_) {
        if (status_callback_) {
            status_callback_("ERROR", "Failed to allocate frame/packet");
        }
        return false;
    }

    if (status_callback_) {
        status_callback_("STREAMING", "Processing RTMP video stream");
    }

    // Process packets
    while (!should_stop_ && is_running_) {
        int ret = av_read_frame(format_context_, packet_);
        
        if (ret < 0) {
            if (ret == AVERROR_EOF) {
                break; // End of stream
            } else if (ret == AVERROR(EAGAIN)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            } else {
                break; // Error
            }
        }

        if (packet_->stream_index == video_stream_index_) {
            // Send packet to decoder
            ret = avcodec_send_packet(codec_context_, packet_);
            if (ret < 0) {
                av_packet_unref(packet_);
                continue;
            }

            // Receive decoded frames
            while (ret >= 0 && !should_stop_) {
                ret = avcodec_receive_frame(codec_context_, frame_);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                    break;
                } else if (ret < 0) {
                    break;
                }

                // Process the decoded frame
                processFrame(frame_);
                frames_received_++;
            }
        }

        av_packet_unref(packet_);
    }

    return true;
}

bool RTMPServer::setupDecoder(AVStream* stream)
{
    // Find decoder
    codec_ = avcodec_find_decoder(stream->codecpar->codec_id);
    if (!codec_) {
        if (status_callback_) {
            status_callback_("ERROR", "Codec not found for codec_id: " + 
                           std::to_string(stream->codecpar->codec_id));
        }
        return false;
    }

    // Allocate codec context
    codec_context_ = avcodec_alloc_context3(codec_);
    if (!codec_context_) {
        if (status_callback_) {
            status_callback_("ERROR", "Failed to allocate codec context");
        }
        return false;
    }

    // Copy codec parameters
    if (avcodec_parameters_to_context(codec_context_, stream->codecpar) < 0) {
        if (status_callback_) {
            status_callback_("ERROR", "Failed to copy codec parameters");
        }
        return false;
    }

    // Set low latency options
    if (low_latency_mode_) {
        codec_context_->flags |= AV_CODEC_FLAG_LOW_DELAY;
        codec_context_->flags2 |= AV_CODEC_FLAG2_FAST;
        codec_context_->thread_count = 1; // Single thread for low latency
    }

    // Open codec
    if (avcodec_open2(codec_context_, codec_, nullptr) < 0) {
        if (status_callback_) {
            status_callback_("ERROR", "Failed to open codec");
        }
        return false;
    }

    if (status_callback_) {
        status_callback_("INFO", "Video decoder initialized: " + std::string(codec_->name) + 
                        " (" + std::to_string(codec_context_->width) + "x" + 
                        std::to_string(codec_context_->height) + ")");
    }

    return true;
}

void RTMPServer::processFrame(AVFrame* frame)
{
    if (!frame_callback_ || !frame) {
        return;
    }

    // Convert frame to RGB
    SwsContext* sws_ctx = sws_getContext(
        frame->width, frame->height, (AVPixelFormat)frame->format,
        frame->width, frame->height, AV_PIX_FMT_RGB24,
        SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);

    if (!sws_ctx) {
        return;
    }

    // Allocate RGB frame
    AVFrame* rgb_frame = av_frame_alloc();
    int num_bytes = av_image_get_buffer_size(AV_PIX_FMT_RGB24, frame->width, frame->height, 1);
    uint8_t* buffer = (uint8_t*)av_malloc(num_bytes);
    
    av_image_fill_arrays(rgb_frame->data, rgb_frame->linesize, buffer, 
                        AV_PIX_FMT_RGB24, frame->width, frame->height, 1);

    // Convert to RGB
    sws_scale(sws_ctx, frame->data, frame->linesize, 0, frame->height,
              rgb_frame->data, rgb_frame->linesize);

    // Generate timestamp (microseconds)
    int64_t timestamp = frame->pts != AV_NOPTS_VALUE ? frame->pts : 
                       std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::steady_clock::now().time_since_epoch()).count();

    // Call callback with RGB data
    frame_callback_(rgb_frame->data[0], frame->width, frame->height, timestamp);

    // Cleanup
    av_free(buffer);
    av_frame_free(&rgb_frame);
    sws_freeContext(sws_ctx);
}

void RTMPServer::cleanup()
{
    if (packet_) {
        av_packet_free(&packet_);
        packet_ = nullptr;
    }

    if (frame_) {
        av_frame_free(&frame_);
        frame_ = nullptr;
    }

    if (codec_context_) {
        avcodec_free_context(&codec_context_);
        codec_context_ = nullptr;
    }

    if (format_context_) {
        avformat_close_input(&format_context_);
        format_context_ = nullptr;
    }

    video_stream_index_ = -1;
}

} // namespace fv_rtmp_server 