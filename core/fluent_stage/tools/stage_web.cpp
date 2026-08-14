// stage_web — live camera effects demo in the browser, low latency.
//
//   ./stage_web [port] [ros_topic]
//     port       default 8790            → open http://<robot>:8790/
//     ros_topic  default /d405_color/image_raw/compressed
//
// Video source: a ROS 2 CompressedImage topic when built with rclcpp and
// messages arrive (the aspa camera, live); otherwise a synthetic pattern.
//
// The frame first crosses a classic water-surface simulation (the OpenGL
// demo algorithm: a damped wave-equation height field; pointer hover feeds
// drops, so fine wavelets trail the mouse and interfere realistically),
// then any of the 30 catalog filters — selected from a browser dropdown,
// tuned with a slider — runs on the GPU via the single-source pipeline.
//
// Transport: H.264 (ffmpeg h264_nvenc, libx264 fallback) over a WebSocket,
// decoded by WebCodecs onto a canvas; the same socket carries pointer
// events back. 60 fps, no B-frames, no buffering: glass-to-glass stays in
// the tens of milliseconds on a LAN. No WebCodecs → MJPEG fallback.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <jpeglib.h>

#include <fluent_stage/fluent_stage.hpp>
#ifdef FS_HAVE_VULKAN
#include <fluent_stage/vulkan_renderer.hpp>
#endif
#ifdef FS_HAVE_ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#endif

using namespace fluent_stage;

namespace {

constexpr uint32_t kW = 1280;
constexpr uint32_t kH = 720;
constexpr float kFps = 60;

// ---- pointer + control state from the page ---------------------------------

struct PointerEventIn {
    char phase;  // 'd'own 'm'ove 'u'p 'c'ancel 'h'over
    float x, y;
};

std::mutex g_events_mutex;
std::vector<PointerEventIn> g_events;
std::atomic<int> g_effect{0};        // 0 = none, 1.. = filterTable() index + 1
std::atomic<float> g_param{-1.0f};   // 0..1 slider; <0 = use the default
std::atomic<bool> g_water{true};

void handleMessage(const char* text, size_t len) {
    std::string s(text, len);
    PointerEventIn e{};
    int idx = 0;
    float v = 0;
    if (std::sscanf(s.c_str(), " %c %f %f", &e.phase, &e.x, &e.y) == 3 &&
        (e.phase == 'd' || e.phase == 'm' || e.phase == 'u' || e.phase == 'c' ||
         e.phase == 'h')) {
        std::lock_guard<std::mutex> lock(g_events_mutex);
        if (g_events.size() < 256) {
            g_events.push_back(e);
        }
    } else if (std::sscanf(s.c_str(), " e %d", &idx) == 1) {
        g_effect = idx;
        g_param = -1.0f;  // new effect starts from its defaults
    } else if (std::sscanf(s.c_str(), " p %f", &v) == 1) {
        g_param = v;
    } else if (std::sscanf(s.c_str(), " w %d", &idx) == 1) {
        g_water = idx != 0;
    }
}

std::atomic<bool> g_running{true};

// MJPEG fallback state (encoded only while such a client is connected).
std::mutex g_frame_mutex;
std::condition_variable g_frame_cv;
std::vector<uint8_t> g_jpeg;
uint64_t g_frame_seq = 0;
std::atomic<int> g_mjpeg_clients{0};

// ---- H.264 access units and their subscribers ------------------------------

struct AccessUnit {
    std::vector<uint8_t> data;  // Annex-B
    bool key = false;
    uint64_t t_us = 0;  // wall-clock at emission — fMP4 timestamps follow
                        // real time, so MSE playback never drifts from the
                        // true frame rate (declared-fps drift = latency)
};
using AuPtr = std::shared_ptr<const AccessUnit>;

struct VideoClient {
    std::mutex mutex;
    std::condition_variable cv;
    std::deque<AuPtr> queue;
    size_t queued_bytes = 0;
    bool wait_for_key = true;
    bool dead = false;
    bool mp4 = false;  // mux to fMP4 for the MSE path
};

std::mutex g_clients_mutex;
std::vector<std::shared_ptr<VideoClient>> g_clients;

void broadcastAu(AuPtr au) {
    std::lock_guard<std::mutex> lock(g_clients_mutex);
    for (auto& c : g_clients) {
        std::lock_guard<std::mutex> cl(c->mutex);
        if (c->wait_for_key && !au->key) {
            continue;
        }
        // A stalling client skips to the next keyframe — deltas cannot be
        // dropped individually and latency must never accumulate.
        if (c->queued_bytes > 4u << 20) {
            c->queue.clear();
            c->queued_bytes = 0;
            c->wait_for_key = true;
            if (!au->key) {
                continue;
            }
        }
        c->wait_for_key = false;
        c->queue.push_back(au);
        c->queued_bytes += au->data.size();
        c->cv.notify_one();
    }
}

// ---- encoder subprocess (ffmpeg) -------------------------------------------

struct Encoder {
    pid_t pid = -1;
    int stdin_fd = -1;
    int stdout_fd = -1;
};

Encoder spawnEncoder(bool nvenc) {
    int in_pipe[2], out_pipe[2];
    if (pipe(in_pipe) != 0 || pipe(out_pipe) != 0) {
        return {};
    }
    const std::string size = std::to_string(kW) + "x" + std::to_string(kH);
    const std::string fps = std::to_string(static_cast<int>(kFps));
    const std::string gop = std::to_string(static_cast<int>(kFps));  // 1 s
    const pid_t pid = fork();
    if (pid == 0) {
        dup2(in_pipe[0], 0);
        dup2(out_pipe[1], 1);
        close(in_pipe[0]);
        close(in_pipe[1]);
        close(out_pipe[0]);
        close(out_pipe[1]);
        if (nvenc) {
            execlp("ffmpeg", "ffmpeg", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt",
                   "rgba", "-s", size.c_str(), "-r", fps.c_str(), "-i", "pipe:0", "-c:v",
                   "h264_nvenc", "-preset", "p1", "-tune", "ull", "-zerolatency", "1",
                   "-delay", "0", "-bf", "0", "-g", gop.c_str(), "-b:v", "6M", "-maxrate",
                   "8M", "-bufsize", "1M", "-profile:v", "baseline", "-pix_fmt", "yuv420p",
                   "-f", "h264", "pipe:1", static_cast<char*>(nullptr));
        } else {
            execlp("ffmpeg", "ffmpeg", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt",
                   "rgba", "-s", size.c_str(), "-r", fps.c_str(), "-i", "pipe:0", "-c:v",
                   "libx264", "-preset", "ultrafast", "-tune", "zerolatency", "-bf", "0",
                   "-g", gop.c_str(), "-b:v", "6M", "-profile:v", "baseline", "-pix_fmt",
                   "yuv420p", "-f", "h264", "pipe:1", static_cast<char*>(nullptr));
        }
        _exit(127);
    }
    close(in_pipe[0]);
    close(out_pipe[1]);
    Encoder e;
    e.pid = pid;
    e.stdin_fd = in_pipe[1];
    e.stdout_fd = out_pipe[0];
    return e;
}

// Splits the Annex-B byte stream into access units (baseline, no B-frames).
class AuSplitter {
public:
    template <typename Emit>
    void feed(const uint8_t* data, size_t len, const Emit& emit) {
        buf_.insert(buf_.end(), data, data + len);
        size_t search_from = 0;
        while (true) {
            const size_t start = findStartCode(search_from);
            if (start == buf_.size()) {
                break;
            }
            const size_t code_len = buf_[start + 2] == 1 ? 3 : 4;
            const size_t next = findStartCode(start + code_len);
            if (next == buf_.size()) {
                if (start > 0) {
                    buf_.erase(buf_.begin(), buf_.begin() + static_cast<ptrdiff_t>(start));
                }
                break;
            }
            handleNal(&buf_[start], next - start, emit);
            search_from = next;
        }
    }

private:
    size_t findStartCode(size_t from) {
        for (size_t i = from; i + 3 < buf_.size(); ++i) {
            if (buf_[i] == 0 && buf_[i + 1] == 0 &&
                (buf_[i + 2] == 1 ||
                 (buf_[i + 2] == 0 && i + 3 < buf_.size() && buf_[i + 3] == 1))) {
                return i;
            }
        }
        return buf_.size();
    }

    template <typename Emit>
    void handleNal(const uint8_t* nal, size_t len, const Emit& emit) {
        const size_t code_len = nal[2] == 1 ? 3 : 4;
        if (len <= code_len) {
            return;
        }
        const int type = nal[code_len] & 0x1f;
        const bool slice = type == 1 || type == 5;
        const bool new_picture =
            slice && len > code_len + 1 && (nal[code_len + 1] & 0x80) != 0;
        const bool flush_before =
            (new_picture && au_has_slice_) ||
            (!slice && au_has_slice_ && (type == 6 || type == 7 || type == 8 || type == 9));
        if (flush_before && !au_.empty()) {
            auto out = std::make_shared<AccessUnit>();
            out->data.swap(au_);
            out->key = au_key_;
            emit(std::move(out));
            au_key_ = false;
            au_has_slice_ = false;
        }
        au_.insert(au_.end(), nal, nal + len);
        if (slice) {
            au_has_slice_ = true;
        }
        if (type == 5) {
            au_key_ = true;
        }
    }

    std::vector<uint8_t> buf_;
    std::vector<uint8_t> au_;
    bool au_key_ = false;
    bool au_has_slice_ = false;
};

// ---- websocket plumbing (RFC 6455) -----------------------------------------

void sha1(const uint8_t* data, size_t len, uint8_t out[20]) {
    uint32_t h[5] = {0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0};
    std::vector<uint8_t> msg(data, data + len);
    msg.push_back(0x80);
    while (msg.size() % 64 != 56) {
        msg.push_back(0);
    }
    const uint64_t bits = static_cast<uint64_t>(len) * 8;
    for (int i = 7; i >= 0; --i) {
        msg.push_back(static_cast<uint8_t>(bits >> (i * 8)));
    }
    for (size_t chunk = 0; chunk < msg.size(); chunk += 64) {
        uint32_t w[80];
        for (int i = 0; i < 16; ++i) {
            w[i] = (msg[chunk + i * 4] << 24) | (msg[chunk + i * 4 + 1] << 16) |
                   (msg[chunk + i * 4 + 2] << 8) | msg[chunk + i * 4 + 3];
        }
        for (int i = 16; i < 80; ++i) {
            const uint32_t v = w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16];
            w[i] = (v << 1) | (v >> 31);
        }
        uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
        for (int i = 0; i < 80; ++i) {
            uint32_t f, k;
            if (i < 20) {
                f = (b & c) | (~b & d);
                k = 0x5A827999;
            } else if (i < 40) {
                f = b ^ c ^ d;
                k = 0x6ED9EBA1;
            } else if (i < 60) {
                f = (b & c) | (b & d) | (c & d);
                k = 0x8F1BBCDC;
            } else {
                f = b ^ c ^ d;
                k = 0xCA62C1D6;
            }
            const uint32_t t = ((a << 5) | (a >> 27)) + f + e + k + w[i];
            e = d;
            d = c;
            c = (b << 30) | (b >> 2);
            b = a;
            a = t;
        }
        h[0] += a;
        h[1] += b;
        h[2] += c;
        h[3] += d;
        h[4] += e;
    }
    for (int i = 0; i < 5; ++i) {
        out[i * 4] = static_cast<uint8_t>(h[i] >> 24);
        out[i * 4 + 1] = static_cast<uint8_t>(h[i] >> 16);
        out[i * 4 + 2] = static_cast<uint8_t>(h[i] >> 8);
        out[i * 4 + 3] = static_cast<uint8_t>(h[i]);
    }
}

std::string base64(const uint8_t* data, size_t len) {
    static const char* alphabet =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    for (size_t i = 0; i < len; i += 3) {
        const uint32_t n = (data[i] << 16) | (i + 1 < len ? data[i + 1] << 8 : 0) |
                           (i + 2 < len ? data[i + 2] : 0);
        out.push_back(alphabet[(n >> 18) & 63]);
        out.push_back(alphabet[(n >> 12) & 63]);
        out.push_back(i + 1 < len ? alphabet[(n >> 6) & 63] : '=');
        out.push_back(i + 2 < len ? alphabet[n & 63] : '=');
    }
    return out;
}

bool sendAll(int fd, const void* data, size_t len) {  // sockets
    const char* p = static_cast<const char*>(data);
    while (len > 0) {
        const ssize_t n = ::send(fd, p, len, MSG_NOSIGNAL);
        if (n <= 0) {
            return false;
        }
        p += n;
        len -= static_cast<size_t>(n);
    }
    return true;
}

bool writeAll(int fd, const void* data, size_t len) {  // pipes
    const char* p = static_cast<const char*>(data);
    while (len > 0) {
        const ssize_t n = ::write(fd, p, len);
        if (n <= 0) {
            return false;
        }
        p += n;
        len -= static_cast<size_t>(n);
    }
    return true;
}

bool wsSendBinary(int fd, const uint8_t* payload, size_t len) {
    uint8_t head[10];
    size_t head_len = 2;
    head[0] = 0x82;
    if (len < 126) {
        head[1] = static_cast<uint8_t>(len);
    } else if (len < 65536) {
        head[1] = 126;
        head[2] = static_cast<uint8_t>(len >> 8);
        head[3] = static_cast<uint8_t>(len);
        head_len = 4;
    } else {
        head[1] = 127;
        for (int i = 0; i < 8; ++i) {
            head[2 + i] = static_cast<uint8_t>(static_cast<uint64_t>(len) >> ((7 - i) * 8));
        }
        head_len = 10;
    }
    return sendAll(fd, head, head_len) && sendAll(fd, payload, len);
}

// ---- fragmented MP4 muxing (for the MSE path) ------------------------------
//
// WebCodecs needs a secure context; plain-http robot pages fall back to
// Media Source Extensions, which accept fragmented MP4. We mux each H.264
// access unit into one moof+mdat fragment server-side so the page just
// appendBuffer()s whatever arrives.

class Mp4Muxer {
public:
    // Extracts SPS/PPS from a keyframe access unit; true once ready.
    bool prime(const std::vector<uint8_t>& annexb) {
        forEachNal(annexb, [&](const uint8_t* nal, size_t len) {
            const int type = nal[0] & 0x1f;
            if (type == 7) {
                sps_.assign(nal, nal + len);
            } else if (type == 8) {
                pps_.assign(nal, nal + len);
            }
        });
        return ready();
    }
    bool ready() const { return !sps_.empty() && !pps_.empty(); }

    std::vector<uint8_t> initSegment(uint32_t w, uint32_t h) const {
        std::vector<uint8_t> avcc;
        push8(avcc, 1);            // configurationVersion
        push8(avcc, sps_[1]);      // profile
        push8(avcc, sps_[2]);      // compat
        push8(avcc, sps_[3]);      // level
        push8(avcc, 0xff);         // 4-byte NAL lengths
        push8(avcc, 0xe1);         // 1 SPS
        push16(avcc, static_cast<uint16_t>(sps_.size()));
        avcc.insert(avcc.end(), sps_.begin(), sps_.end());
        push8(avcc, 1);            // 1 PPS
        push16(avcc, static_cast<uint16_t>(pps_.size()));
        avcc.insert(avcc.end(), pps_.begin(), pps_.end());

        const auto avc1 = box("avc1", [&](std::vector<uint8_t>& b) {
            for (int i = 0; i < 6; ++i) push8(b, 0);
            push16(b, 1);                       // data_reference_index
            for (int i = 0; i < 16; ++i) push8(b, 0);
            push16(b, static_cast<uint16_t>(w));
            push16(b, static_cast<uint16_t>(h));
            push32(b, 0x00480000);              // 72 dpi
            push32(b, 0x00480000);
            push32(b, 0);
            push16(b, 1);                       // frame count
            for (int i = 0; i < 32; ++i) push8(b, 0);  // compressor name
            push16(b, 0x0018);                  // depth
            push16(b, 0xffff);                  // pre-defined
            append(b, box("avcC", [&](std::vector<uint8_t>& c) { append(c, avcc); }));
        });
        const auto stbl = box("stbl", [&](std::vector<uint8_t>& b) {
            append(b, fullbox("stsd", 0, 0, [&](std::vector<uint8_t>& c) {
                push32(c, 1);
                append(c, avc1);
            }));
            append(b, fullbox("stts", 0, 0, [](std::vector<uint8_t>& c) { push32(c, 0); }));
            append(b, fullbox("stsc", 0, 0, [](std::vector<uint8_t>& c) { push32(c, 0); }));
            append(b, fullbox("stsz", 0, 0, [](std::vector<uint8_t>& c) {
                push32(c, 0);
                push32(c, 0);
            }));
            append(b, fullbox("stco", 0, 0, [](std::vector<uint8_t>& c) { push32(c, 0); }));
        });
        const auto minf = box("minf", [&](std::vector<uint8_t>& b) {
            append(b, fullbox("vmhd", 0, 1, [](std::vector<uint8_t>& c) {
                for (int i = 0; i < 8; ++i) push8(c, 0);
            }));
            append(b, box("dinf", [&](std::vector<uint8_t>& c) {
                append(c, fullbox("dref", 0, 0, [](std::vector<uint8_t>& d) {
                    push32(d, 1);
                    append(d, fullbox("url ", 0, 1, [](std::vector<uint8_t>&) {}));
                }));
            }));
            append(b, stbl);
        });
        const auto mdia = box("mdia", [&](std::vector<uint8_t>& b) {
            append(b, fullbox("mdhd", 0, 0, [&](std::vector<uint8_t>& c) {
                push32(c, 0);
                push32(c, 0);
                push32(c, kTimescale);
                push32(c, 0);
                push16(c, 0x55c4);  // language: und
                push16(c, 0);
            }));
            append(b, fullbox("hdlr", 0, 0, [](std::vector<uint8_t>& c) {
                push32(c, 0);
                c.insert(c.end(), {'v', 'i', 'd', 'e'});
                for (int i = 0; i < 12; ++i) push8(c, 0);
                push8(c, 0);  // empty name
            }));
            append(b, minf);
        });
        const auto trak = box("trak", [&](std::vector<uint8_t>& b) {
            append(b, fullbox("tkhd", 0, 7, [&](std::vector<uint8_t>& c) {
                push32(c, 0);
                push32(c, 0);
                push32(c, 1);  // track id
                push32(c, 0);
                push32(c, 0);
                for (int i = 0; i < 2; ++i) push32(c, 0);
                push16(c, 0);
                push16(c, 0);
                push16(c, 0);
                push16(c, 0);
                const uint32_t unity[9] = {0x10000, 0, 0, 0, 0x10000, 0, 0, 0, 0x40000000};
                for (uint32_t v : unity) push32(c, v);
                push32(c, w << 16);
                push32(c, h << 16);
            }));
            append(b, mdia);
        });
        const auto moov = box("moov", [&](std::vector<uint8_t>& b) {
            append(b, fullbox("mvhd", 0, 0, [&](std::vector<uint8_t>& c) {
                push32(c, 0);
                push32(c, 0);
                push32(c, kTimescale);
                push32(c, 0);
                push32(c, 0x00010000);
                push16(c, 0x0100);
                push16(c, 0);
                push32(c, 0);
                push32(c, 0);
                const uint32_t unity[9] = {0x10000, 0, 0, 0, 0x10000, 0, 0, 0, 0x40000000};
                for (uint32_t v : unity) push32(c, v);
                for (int i = 0; i < 6; ++i) push32(c, 0);
                push32(c, 2);  // next track id
            }));
            append(b, trak);
            append(b, box("mvex", [](std::vector<uint8_t>& b2) {
                append(b2, fullbox("trex", 0, 0, [](std::vector<uint8_t>& c) {
                    push32(c, 1);  // track id
                    push32(c, 1);  // default sample description index
                    push32(c, 0);
                    push32(c, 0);
                    push32(c, 0);
                }));
            }));
        });
        std::vector<uint8_t> out;
        append(out, box("ftyp", [](std::vector<uint8_t>& b) {
            b.insert(b.end(), {'i', 's', 'o', '5'});
            push32(b, 512);
            b.insert(b.end(), {'i', 's', 'o', '5'});
            b.insert(b.end(), {'a', 'v', 'c', '1'});
        }));
        append(out, moov);
        return out;
    }

    // One access unit → one moof+mdat fragment.
    std::vector<uint8_t> fragment(const std::vector<uint8_t>& annexb, bool key, uint32_t seq,
                                  uint64_t decode_time, uint32_t duration) const {
        std::vector<uint8_t> sample;  // AVCC: 4-byte length-prefixed NALs
        forEachNal(annexb, [&](const uint8_t* nal, size_t len) {
            push32(sample, static_cast<uint32_t>(len));
            sample.insert(sample.end(), nal, nal + len);
        });
        std::vector<uint8_t> trun_patch;
        const auto moof = box("moof", [&](std::vector<uint8_t>& b) {
            append(b, fullbox("mfhd", 0, 0, [&](std::vector<uint8_t>& c) { push32(c, seq); }));
            append(b, box("traf", [&](std::vector<uint8_t>& c) {
                append(c, fullbox("tfhd", 0, 0x020000, [](std::vector<uint8_t>& d) {
                    push32(d, 1);  // track id; default-base-is-moof
                }));
                append(c, fullbox("tfdt", 1, 0, [&](std::vector<uint8_t>& d) {
                    push32(d, static_cast<uint32_t>(decode_time >> 32));
                    push32(d, static_cast<uint32_t>(decode_time));
                }));
                // flags: data-offset | duration | size | flags per sample
                append(c, fullbox("trun", 0, 0x000701, [&](std::vector<uint8_t>& d) {
                    push32(d, 1);           // sample count
                    push32(d, 0);           // data offset (patched below)
                    push32(d, duration);
                    push32(d, static_cast<uint32_t>(sample.size()));
                    push32(d, key ? 0x02000000u : 0x01010000u);
                }));
            }));
        });
        std::vector<uint8_t> out = moof;
        // Patch the trun data offset: mdat payload begins right after moof.
        const uint32_t data_offset = static_cast<uint32_t>(moof.size() + 8);
        for (size_t i = 0; i + 16 <= out.size(); ++i) {
            if (std::memcmp(&out[i], "trun", 4) == 0) {
                const size_t off_at = i + 4 /*name*/ + 4 /*ver/flags*/ + 4 /*count*/;
                out[off_at] = static_cast<uint8_t>(data_offset >> 24);
                out[off_at + 1] = static_cast<uint8_t>(data_offset >> 16);
                out[off_at + 2] = static_cast<uint8_t>(data_offset >> 8);
                out[off_at + 3] = static_cast<uint8_t>(data_offset);
                break;
            }
        }
        append(out, box("mdat", [&](std::vector<uint8_t>& b) { append(b, sample); }));
        return out;
    }

    static constexpr uint32_t kTimescale = 90000;

private:
    template <typename Fn>
    static void forEachNal(const std::vector<uint8_t>& annexb, const Fn& fn) {
        size_t i = 0;
        while (i + 3 < annexb.size()) {
            size_t code = 0;
            if (annexb[i] == 0 && annexb[i + 1] == 0 && annexb[i + 2] == 1) {
                code = 3;
            } else if (i + 4 < annexb.size() && annexb[i] == 0 && annexb[i + 1] == 0 &&
                       annexb[i + 2] == 0 && annexb[i + 3] == 1) {
                code = 4;
            } else {
                ++i;
                continue;
            }
            const size_t start = i + code;
            size_t end = annexb.size();
            for (size_t j = start; j + 3 < annexb.size(); ++j) {
                if (annexb[j] == 0 && annexb[j + 1] == 0 &&
                    (annexb[j + 2] == 1 || (annexb[j + 2] == 0 && j + 4 <= annexb.size() &&
                                            annexb[j + 3] == 1))) {
                    end = j;
                    break;
                }
            }
            fn(&annexb[start], end - start);
            i = end;
        }
    }

    static void push8(std::vector<uint8_t>& b, uint8_t v) { b.push_back(v); }
    static void push16(std::vector<uint8_t>& b, uint16_t v) {
        b.push_back(static_cast<uint8_t>(v >> 8));
        b.push_back(static_cast<uint8_t>(v));
    }
    static void push32(std::vector<uint8_t>& b, uint32_t v) {
        b.push_back(static_cast<uint8_t>(v >> 24));
        b.push_back(static_cast<uint8_t>(v >> 16));
        b.push_back(static_cast<uint8_t>(v >> 8));
        b.push_back(static_cast<uint8_t>(v));
    }
    static void append(std::vector<uint8_t>& b, const std::vector<uint8_t>& v) {
        b.insert(b.end(), v.begin(), v.end());
    }
    template <typename Fill>
    static std::vector<uint8_t> box(const char name[4], const Fill& fill) {
        std::vector<uint8_t> body;
        fill(body);
        std::vector<uint8_t> out;
        push32(out, static_cast<uint32_t>(body.size() + 8));
        out.insert(out.end(), name, name + 4);
        append(out, body);
        return out;
    }
    template <typename Fill>
    static std::vector<uint8_t> fullbox(const char name[4], uint8_t version, uint32_t flags,
                                        const Fill& fill) {
        return box(name, [&](std::vector<uint8_t>& b) {
            push32(b, (static_cast<uint32_t>(version) << 24) | (flags & 0xffffff));
            fill(b);
        });
    }

    std::vector<uint8_t> sps_;
    std::vector<uint8_t> pps_;
};

// ---- the page --------------------------------------------------------------

const char* kPage = R"HTML(<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>fluent_stage fx</title>
<style>
  body { margin:0; background:#0d0f13; color:#dde; font:14px system-ui;
         display:flex; flex-direction:column; align-items:center; }
  #bar { display:flex; gap:10px; align-items:center; margin:8px; flex-wrap:wrap;
         justify-content:center; }
  select,input[type=range] { background:#1a1e26; color:#dde; border:1px solid #333a46;
         border-radius:6px; padding:4px 8px; }
  #v { max-width:98vw; max-height:86vh; border-radius:8px; background:#000;
       touch-action:none; user-select:none; -webkit-user-select:none; }
  #s { color:#678; font-size:12px; }
  label { color:#9ab; font-size:13px; }
</style>
<div id="bar">
  <label>effect <select id="fx"></select></label>
  <label>param <input id="pr" type="range" min="0" max="100" value="50"></label>
  <label><input id="wt" type="checkbox" checked> water</label>
  <span id="s">connecting…</span>
</div>
<canvas id="v" width="1280" height="720"></canvas>
<script>
const EFFECTS = %%EFFECTS%%;   // the filter catalog, embedded at serve time
const W = 1280, H = 720;
const canvas = document.getElementById('v');
const statusEl = document.getElementById('s');
const fxSel = document.getElementById('fx');
const paramEl = document.getElementById('pr');
const waterEl = document.getElementById('wt');
let sendMsg = () => {};

fxSel.onchange = () => { sendMsg('e ' + fxSel.selectedIndex); paramEl.value = 50; };
paramEl.oninput = () => sendMsg('p ' + (paramEl.value / 100));
waterEl.onchange = () => sendMsg('w ' + (waterEl.checked ? 1 : 0));

for (const n of ['effect: none', ...EFFECTS]) {
  const o = document.createElement('option');
  o.textContent = n;
  fxSel.appendChild(o);
}

function hookPointer(el) {
  let down = false, pending = null, last = 0;
  const send = (phase, e) => {
    const r = el.getBoundingClientRect();
    sendMsg(phase + ' ' + ((e.clientX - r.left) / r.width * W).toFixed(1) +
            ' ' + ((e.clientY - r.top) / r.height * H).toFixed(1));
  };
  const loop = ts => {
    if (pending && ts - last >= 16) { send(down ? 'm' : 'h', pending); pending = null; last = ts; }
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
  el.addEventListener('pointerdown', e => { down = true; el.setPointerCapture(e.pointerId); send('d', e); e.preventDefault(); });
  el.addEventListener('pointermove', e => { pending = e; });
  el.addEventListener('pointerup',   e => { down = false; send('u', e); });
  el.addEventListener('pointercancel', e => { down = false; send('c', e); });
}

function mjpegFallback(reason) {
  statusEl.textContent = 'mjpeg fallback (' + reason + ')';
  const img = document.createElement('img');
  img.id = 'v'; img.src = '/stream'; img.draggable = false;
  canvas.replaceWith(img);
  sendMsg = text => fetch('/pointer', {method: 'POST', body: text});
  hookPointer(img);
}

function webcodecsPath() {
  const ctx = canvas.getContext('2d');
  let frames = 0, seq = 0;
  const decoder = new VideoDecoder({
    output: f => { ctx.drawImage(f, 0, 0, W, H); f.close(); ++frames; },
    error: e => mjpegFallback(e.message),
  });
  decoder.configure({codec: 'avc1.42e01e', optimizeForLatency: true});
  const ws = new WebSocket(`ws://${location.host}/ws`);
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => { sendMsg = text => ws.send(text); hookPointer(canvas); };
  ws.onclose = () => statusEl.textContent = 'disconnected — reload to reconnect';
  ws.onmessage = e => {
    if (typeof e.data === 'string') return;
    const bytes = new Uint8Array(e.data);
    decoder.decode(new EncodedVideoChunk({
      type: bytes[0] === 1 ? 'key' : 'delta',
      timestamp: (seq++) * 16666,
      data: bytes.subarray(1),
    }));
  };
  setInterval(() => { statusEl.textContent = `h264/webcodecs ${frames} fps`; frames = 0; }, 1000);
}

function msePath() {
  // Plain-http pages have no WebCodecs (secure-context API); MSE takes the
  // same H.264, boxed as fMP4 by the robot. Still low latency: we chase
  // the live edge so no buffer builds up.
  const video = document.createElement('video');
  video.id = 'v'; video.muted = true; video.autoplay = true; video.playsInline = true;
  canvas.replaceWith(video);
  const ms = new MediaSource();
  video.src = URL.createObjectURL(ms);
  ms.addEventListener('sourceopen', () => {
    const sb = ms.addSourceBuffer('video/mp4; codecs="avc1.42e01e"');
    const q = [];
    sb.addEventListener('updateend', () => {
      if (q.length && !sb.updating) sb.appendBuffer(q.shift());
      if (!video.buffered.length) return;
      // Live-edge chase: soak small drift with playbackRate, jump hard past
      // 0.25 s — changing a filter must show up now, not seconds later.
      const lag = video.buffered.end(video.buffered.length - 1) - video.currentTime;
      if (lag > 0.25) {
        video.currentTime = video.buffered.end(video.buffered.length - 1) - 0.03;
      }
      video.playbackRate = lag > 0.1 ? 1.08 : 1.0;
      if (video.paused) video.play();
      statusEl.textContent = 'h264/mse  lag ' + Math.max(lag, 0).toFixed(2) + 's';
    });
    const ws = new WebSocket(`ws://${location.host}/ws?fmt=mp4`);
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => { sendMsg = t => ws.send(t); hookPointer(video); video.play(); };
    ws.onclose = () => statusEl.textContent = 'disconnected — reload to reconnect';
    ws.onmessage = e => {
      if (typeof e.data === 'string') return;
      if (sb.updating || q.length) q.push(e.data); else sb.appendBuffer(e.data);
    };
  });
  statusEl.textContent = 'h264/mse';
}

if ('VideoDecoder' in window) {
  webcodecsPath();
} else if (window.MediaSource && MediaSource.isTypeSupported('video/mp4; codecs="avc1.42e01e"')) {
  msePath();
} else {
  mjpegFallback('no decoder');
}
</script>
)HTML";

// ---- http / websocket serving ----------------------------------------------

void sendResponse(int fd, const char* status, const char* type, const std::string& body) {
    std::ostringstream head;
    head << "HTTP/1.1 " << status << "\r\nContent-Type: " << type
         << "\r\nContent-Length: " << body.size() << "\r\nConnection: close\r\n\r\n";
    const std::string h = head.str();
    sendAll(fd, h.data(), h.size());
    sendAll(fd, body.data(), body.size());
}

void streamMjpeg(int fd) {
    const char* head =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=fsframe\r\n"
        "Cache-Control: no-store\r\nConnection: close\r\n\r\n";
    if (!sendAll(fd, head, std::strlen(head))) {
        return;
    }
    ++g_mjpeg_clients;
    uint64_t last_seq = 0;
    std::vector<uint8_t> jpeg;
    while (g_running) {
        {
            std::unique_lock<std::mutex> lock(g_frame_mutex);
            g_frame_cv.wait_for(lock, std::chrono::milliseconds(500),
                                [&] { return g_frame_seq != last_seq || !g_running; });
            if (g_frame_seq == last_seq) {
                continue;
            }
            last_seq = g_frame_seq;
            jpeg = g_jpeg;
        }
        char part[128];
        const int n = std::snprintf(part, sizeof part,
                                    "--fsframe\r\nContent-Type: image/jpeg\r\n"
                                    "Content-Length: %zu\r\n\r\n",
                                    jpeg.size());
        if (!sendAll(fd, part, static_cast<size_t>(n)) ||
            !sendAll(fd, jpeg.data(), jpeg.size()) || !sendAll(fd, "\r\n", 2)) {
            break;
        }
    }
    --g_mjpeg_clients;
}

std::string effectCatalogJson() {
    std::string json = "[";
    for (const FilterSpec& spec : filterTable()) {
        if (json.size() > 1) {
            json += ",";
        }
        json += "\"";
        json += spec.name;
        json += "\"";
    }
    json += "]";
    return json;
}

void serveWebsocket(int fd, const std::string& request) {
    const size_t key_at = request.find("Sec-WebSocket-Key:");
    if (key_at == std::string::npos) {
        return;
    }
    size_t begin = key_at + 18;
    while (begin < request.size() && request[begin] == ' ') {
        ++begin;
    }
    const size_t end = request.find("\r\n", begin);
    const std::string accept_src =
        request.substr(begin, end - begin) + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    uint8_t digest[20];
    sha1(reinterpret_cast<const uint8_t*>(accept_src.data()), accept_src.size(), digest);
    const std::string resp = "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                             "Connection: Upgrade\r\nSec-WebSocket-Accept: " +
                             base64(digest, 20) + "\r\n\r\n";
    if (!sendAll(fd, resp.data(), resp.size())) {
        return;
    }

    auto client = std::make_shared<VideoClient>();
    client->mp4 = request.find("fmt=mp4") != std::string::npos;
    {
        std::lock_guard<std::mutex> lock(g_clients_mutex);
        g_clients.push_back(client);
    }

    std::thread sender([fd, client] {
        Mp4Muxer muxer;
        bool init_sent = false;
        uint32_t seq = 1;
        uint64_t dts = 0;
        uint64_t prev_t_us = 0;
        std::vector<uint8_t> frame;
        while (g_running) {
            AuPtr au;
            {
                std::unique_lock<std::mutex> lock(client->mutex);
                client->cv.wait_for(lock, std::chrono::milliseconds(500),
                                    [&] { return !client->queue.empty() || client->dead; });
                if (client->dead) {
                    return;
                }
                if (client->queue.empty()) {
                    continue;
                }
                au = client->queue.front();
                client->queue.pop_front();
                client->queued_bytes -= au->data.size();
            }
            bool ok;
            if (client->mp4) {
                if (!init_sent) {
                    if (!au->key || !muxer.prime(au->data)) {
                        continue;
                    }
                    const auto init = muxer.initSegment(kW, kH);
                    if (!wsSendBinary(fd, init.data(), init.size())) {
                        break;
                    }
                    init_sent = true;
                }
                // Real elapsed time per frame, clamped to something sane.
                uint32_t duration = Mp4Muxer::kTimescale / 60;
                if (prev_t_us != 0 && au->t_us > prev_t_us) {
                    const uint64_t d =
                        (au->t_us - prev_t_us) * Mp4Muxer::kTimescale / 1000000ull;
                    duration = static_cast<uint32_t>(
                        std::min<uint64_t>(std::max<uint64_t>(d, 900), 30000));
                }
                prev_t_us = au->t_us;
                const auto frag = muxer.fragment(au->data, au->key, seq++, dts, duration);
                dts += duration;
                ok = wsSendBinary(fd, frag.data(), frag.size());
            } else {
                frame.clear();
                frame.push_back(au->key ? 1 : 0);
                frame.insert(frame.end(), au->data.begin(), au->data.end());
                ok = wsSendBinary(fd, frame.data(), frame.size());
            }
            if (!ok) {
                break;
            }
        }
        std::lock_guard<std::mutex> lock(client->mutex);
        client->dead = true;
    });

    std::vector<uint8_t> buf;
    uint8_t chunk[2048];
    while (g_running) {
        const ssize_t n = ::recv(fd, chunk, sizeof chunk, 0);
        if (n <= 0) {
            break;
        }
        buf.insert(buf.end(), chunk, chunk + n);
        while (buf.size() >= 2) {
            const uint8_t opcode = buf[0] & 0x0f;
            const bool masked = (buf[1] & 0x80) != 0;
            uint64_t len = buf[1] & 0x7f;
            size_t at = 2;
            if (len == 126) {
                if (buf.size() < 4) {
                    break;
                }
                len = (buf[2] << 8) | buf[3];
                at = 4;
            } else if (len == 127) {
                if (buf.size() < 10) {
                    break;
                }
                len = 0;
                for (int i = 0; i < 8; ++i) {
                    len = (len << 8) | buf[2 + i];
                }
                at = 10;
            }
            const size_t mask_at = at;
            if (masked) {
                at += 4;
            }
            if (buf.size() < at + len) {
                break;
            }
            if (masked) {
                for (uint64_t i = 0; i < len; ++i) {
                    buf[at + i] ^= buf[mask_at + (i & 3)];
                }
            }
            if (opcode == 1) {
                handleMessage(reinterpret_cast<const char*>(&buf[at]), len);
            } else if (opcode == 9) {
                uint8_t pong[2] = {0x8a, 0};
                sendAll(fd, pong, 2);
            } else if (opcode == 8) {
                goto done;
            }
            buf.erase(buf.begin(), buf.begin() + static_cast<ptrdiff_t>(at + len));
        }
    }
done:
    {
        std::lock_guard<std::mutex> lock(client->mutex);
        client->dead = true;
    }
    client->cv.notify_all();
    sender.join();
    std::lock_guard<std::mutex> lock(g_clients_mutex);
    for (auto it = g_clients.begin(); it != g_clients.end(); ++it) {
        if (it->get() == client.get()) {
            g_clients.erase(it);
            break;
        }
    }
}

void handleConnection(int fd) {
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    std::string buf;
    char chunk[2048];
    while (true) {
        size_t head_end;
        while ((head_end = buf.find("\r\n\r\n")) == std::string::npos) {
            const ssize_t n = ::recv(fd, chunk, sizeof chunk, 0);
            if (n <= 0 || buf.size() > 65536) {
                ::close(fd);
                return;
            }
            buf.append(chunk, static_cast<size_t>(n));
        }
        if (buf.rfind("GET /ws", 0) == 0) {
            serveWebsocket(fd, buf.substr(0, head_end + 4));
            ::close(fd);
            return;
        }
        if (buf.rfind("GET /stream", 0) == 0) {
            streamMjpeg(fd);
            ::close(fd);
            return;
        }
        if (buf.rfind("POST /pointer", 0) == 0) {  // MJPEG-fallback input path
            const size_t body_at = head_end + 4;
            size_t want = 0;
            const size_t cl = buf.find("Content-Length:");
            if (cl != std::string::npos && cl < head_end) {
                want = static_cast<size_t>(std::atoi(buf.c_str() + cl + 15));
            }
            while (buf.size() - body_at < want) {
                const ssize_t n = ::recv(fd, chunk, sizeof chunk, 0);
                if (n <= 0) {
                    ::close(fd);
                    return;
                }
                buf.append(chunk, static_cast<size_t>(n));
            }
            handleMessage(buf.c_str() + body_at, want);
            const char* resp = "HTTP/1.1 204 No Content\r\nConnection: keep-alive\r\n\r\n";
            if (!sendAll(fd, resp, std::strlen(resp))) {
                ::close(fd);
                return;
            }
            buf.erase(0, body_at + want);
            continue;
        }
        {
            std::string page = kPage;
            const size_t token = page.find("%%EFFECTS%%");
            if (token != std::string::npos) {
                page.replace(token, 11, effectCatalogJson());
            }
            sendResponse(fd, "200 OK", "text/html; charset=utf-8", page);
        }
        ::close(fd);
        return;
    }
}

void serverLoop(uint16_t port) {
    const int listener = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    if (::bind(listener, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0 ||
        ::listen(listener, 16) != 0) {
        std::perror("bind/listen");
        g_running = false;
        return;
    }
    std::printf("stage_web: http://0.0.0.0:%u/\n", port);
    std::fflush(stdout);
    while (g_running) {
        const int fd = ::accept(listener, nullptr, nullptr);
        if (fd < 0) {
            continue;
        }
        std::thread(handleConnection, fd).detach();
    }
    ::close(listener);
}

// ---- MJPEG fallback encoder ------------------------------------------------

void encodeJpeg(const Surface& s, std::vector<uint8_t>& out) {
    jpeg_compress_struct cinfo{};
    jpeg_error_mgr jerr{};
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    unsigned char* mem = nullptr;
    unsigned long mem_size = 0;
    jpeg_mem_dest(&cinfo, &mem, &mem_size);
    cinfo.image_width = s.width;
    cinfo.image_height = s.height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, 75, TRUE);
    jpeg_start_compress(&cinfo, TRUE);
    std::vector<uint8_t> row(s.width * 3);
    while (cinfo.next_scanline < cinfo.image_height) {
        const uint8_t* src = s.row(cinfo.next_scanline);
        for (uint32_t x = 0; x < s.width; ++x) {
            const float a = src[x * 4 + 3] / 255.0f;
            row[x * 3 + 0] = static_cast<uint8_t>(src[x * 4 + 0] * a + 16 * (1 - a));
            row[x * 3 + 1] = static_cast<uint8_t>(src[x * 4 + 1] * a + 18 * (1 - a));
            row[x * 3 + 2] = static_cast<uint8_t>(src[x * 4 + 2] * a + 22 * (1 - a));
        }
        JSAMPROW rows[1] = {row.data()};
        jpeg_write_scanlines(&cinfo, rows, 1);
    }
    jpeg_finish_compress(&cinfo);
    out.assign(mem, mem + mem_size);
    jpeg_destroy_compress(&cinfo);
    free(mem);
}

// ---- classic water-surface simulation (the OpenGL demo algorithm) ----------
//
// A damped wave equation on a height grid at video resolution: every drop
// spreads, reflects, and interferes; the frame is warped by the height
// gradient with a touch of gradient shading. This is the exact algorithm
// behind the classic "water ripple" demos.

class WaterSim {
public:
    void reset(uint32_t w, uint32_t h) {
        w_ = w;
        h_ = h;
        prev_.assign(static_cast<size_t>(w) * h, 0);
        curr_.assign(static_cast<size_t>(w) * h, 0);
    }
    uint32_t width() const { return w_; }
    uint32_t height() const { return h_; }

    void drop(float fx, float fy, int strength, int radius) {
        const int cx = static_cast<int>(fx);
        const int cy = static_cast<int>(fy);
        for (int y = cy - radius; y <= cy + radius; ++y) {
            for (int x = cx - radius; x <= cx + radius; ++x) {
                if (x < 1 || y < 1 || x >= static_cast<int>(w_) - 1 ||
                    y >= static_cast<int>(h_) - 1) {
                    continue;
                }
                const float d2 = static_cast<float>((x - cx) * (x - cx) + (y - cy) * (y - cy));
                if (d2 <= static_cast<float>(radius * radius)) {
                    prev_[static_cast<size_t>(y) * w_ + x] -=
                        static_cast<int16_t>(strength * (1.0f - d2 / (radius * radius + 1)));
                }
            }
        }
    }

    void step() {
        for (uint32_t y = 1; y + 1 < h_; ++y) {
            const size_t row = static_cast<size_t>(y) * w_;
            for (uint32_t x = 1; x + 1 < w_; ++x) {
                const size_t i = row + x;
                int v = ((prev_[i - 1] + prev_[i + 1] + prev_[i - w_] + prev_[i + w_]) >> 1) -
                        curr_[i];
                v -= v >> 6;  // damping: long-lived, slowly fading wavelets
                curr_[i] = static_cast<int16_t>(v);
            }
        }
        prev_.swap(curr_);
    }

    // Refracts `src` into `dst` (both RGBA, w_×h_) by the height gradient.
    void warp(const uint8_t* src, uint8_t* dst) const {
        for (uint32_t y = 0; y < h_; ++y) {
            for (uint32_t x = 0; x < w_; ++x) {
                const size_t i = static_cast<size_t>(y) * w_ + x;
                int dx = 0, dy = 0;
                if (x > 0 && x + 1 < w_ && y > 0 && y + 1 < h_) {
                    dx = prev_[i - 1] - prev_[i + 1];
                    dy = prev_[i - w_] - prev_[i + w_];
                }
                int sx = static_cast<int>(x) + (dx >> 3);
                int sy = static_cast<int>(y) + (dy >> 3);
                sx = std::min(std::max(sx, 0), static_cast<int>(w_) - 1);
                sy = std::min(std::max(sy, 0), static_cast<int>(h_) - 1);
                const uint8_t* s = &src[(static_cast<size_t>(sy) * w_ + sx) * 4];
                uint8_t* d = &dst[i * 4];
                const int shade = dx >> 4;  // cheap specular from the slope
                d[0] = static_cast<uint8_t>(std::min(std::max(s[0] + shade, 0), 255));
                d[1] = static_cast<uint8_t>(std::min(std::max(s[1] + shade, 0), 255));
                d[2] = static_cast<uint8_t>(std::min(std::max(s[2] + shade, 0), 255));
                d[3] = s[3];
            }
        }
    }

    bool calm() const { return w_ == 0; }

private:
    uint32_t w_ = 0, h_ = 0;
    std::vector<int16_t> prev_, curr_;
};

// ---- video sources ---------------------------------------------------------

std::vector<uint8_t> makeTestPattern(uint32_t w, uint32_t h, uint32_t t) {
    // Fallback when no ROS frames arrive: a colorful plasma-style pattern
    // with enough structure for every effect to read clearly.
    std::vector<uint8_t> px(static_cast<size_t>(w) * h * 4);
    const float ft = t * 0.02f;
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            uint8_t* p = &px[(static_cast<size_t>(y) * w + x) * 4];
            const float fx = static_cast<float>(x) / w, fy = static_cast<float>(y) / h;
            const float v = std::sin(fx * 9 + ft) + std::sin(fy * 7 - ft * 1.3f) +
                            std::sin((fx + fy) * 11 + ft * 0.7f);
            const bool check = (((x / 40) + (y / 40)) & 1) != 0;
            p[0] = static_cast<uint8_t>(110 + 70 * std::sin(v * 1.7f) + (check ? 24 : 0));
            p[1] = static_cast<uint8_t>(110 + 70 * std::sin(v * 1.7f + 2.1f));
            p[2] = static_cast<uint8_t>(110 + 70 * std::sin(v * 1.7f + 4.2f) + (check ? 24 : 0));
            p[3] = 255;
        }
    }
    return px;
}

bool decodeJpegToRgba(const uint8_t* data, size_t len, std::vector<uint8_t>& out, uint32_t& w,
                      uint32_t& h) {
    jpeg_decompress_struct cinfo{};
    jpeg_error_mgr jerr{};
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, const_cast<uint8_t*>(data), len);
    if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK) {
        jpeg_destroy_decompress(&cinfo);
        return false;
    }
    cinfo.out_color_space = JCS_EXT_RGBA;
    jpeg_start_decompress(&cinfo);
    w = cinfo.output_width;
    h = cinfo.output_height;
    out.resize(static_cast<size_t>(w) * h * 4);
    while (cinfo.output_scanline < cinfo.output_height) {
        JSAMPROW row = &out[static_cast<size_t>(cinfo.output_scanline) * w * 4];
        jpeg_read_scanlines(&cinfo, &row, 1);
    }
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    return true;
}

#ifdef FS_HAVE_ROS
// Latest compressed frame per camera; the render loop decodes at its own
// pace (latest wins — a queue would be latency).
struct RosSource {
    std::mutex mutex;
    std::vector<uint8_t> jpeg;
    std::atomic<uint64_t> seq{0};
};
RosSource g_ros[2];

void rosThread(std::string main_topic, std::string pip_topic) {
    // Keep our own SIGINT/SIGTERM handling — rclcpp must not swallow them
    // (otherwise pkill leaves a zombie server holding the port).
    rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("stage_web_viewer");
    const std::string topics[2] = {main_topic, pip_topic};
    std::vector<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr> subs;
    for (int i = 0; i < 2; ++i) {
        if (topics[i].empty()) {
            continue;
        }
        RosSource* src = &g_ros[i];
        subs.push_back(node->create_subscription<sensor_msgs::msg::CompressedImage>(
            topics[i], rclcpp::SensorDataQoS(),
            [src](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(src->mutex);
                src->jpeg = msg->data;
                ++src->seq;
            }));
        std::printf("stage_web: subscribing %s\n", topics[i].c_str());
    }
    std::fflush(stdout);
    while (g_running && rclcpp::ok()) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    rclcpp::shutdown();
}
#endif

}  // namespace

int main(int argc, char** argv) {
    const uint16_t port = argc > 1 ? static_cast<uint16_t>(std::atoi(argv[1])) : 8790;
    const std::string topic = argc > 2 ? argv[2] : "/d405_color/image_raw/compressed";
    const std::string pip_topic = argc > 3 ? argv[3] : "/aspa/restamped/color_compressed";
    std::signal(SIGPIPE, SIG_IGN);
    std::signal(SIGINT, [](int) { std::_Exit(0); });
    std::signal(SIGTERM, [](int) { std::_Exit(0); });

    std::unique_ptr<Renderer> renderer;
#ifdef FS_HAVE_VULKAN
    try {
        renderer = std::make_unique<VulkanRenderer>();
        std::printf("stage_web: vulkan backend\n");
    } catch (const std::exception& e) {
        std::printf("stage_web: %s — falling back to cpu\n", e.what());
    }
#endif
    if (!renderer) {
        renderer = std::make_unique<CpuRenderer>();
    }

    // ---- encoder + AU fan-out ----
    bool nvenc = true;
    Encoder encoder = spawnEncoder(nvenc);
    std::atomic<bool> encoder_alive{true};
    const auto reader_fn = [&] {
        AuSplitter splitter;
        uint8_t chunk[65536];
        while (g_running) {
            const ssize_t n = ::read(encoder.stdout_fd, chunk, sizeof chunk);
            if (n <= 0) {
                encoder_alive = false;
                return;
            }
            splitter.feed(chunk, static_cast<size_t>(n), [](AuPtr au) {
                const_cast<AccessUnit*>(au.get())->t_us = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count());
                broadcastAu(au);
            });
        }
    };
    std::thread au_reader(reader_fn);

#ifdef FS_HAVE_ROS
    std::thread ros_thread(rosThread, topic, pip_topic);
#else
    (void)topic;
    (void)pip_topic;
#endif

    // ---- scene: main camera fills the canvas; the second camera floats as
    // a PiP layer that glides around at the full render rate ----
    Stage stage(kW, kH);
    auto pattern = makeTestPattern(640, 360, 0);
    auto& video = stage.image({640, 360, pattern.data(), 0}, Fit::Cover);
    auto& src_label = stage.text("no camera — test pattern", {24, 682})
                          .size(18)
                          .color({1, 1, 1, 0.55f});
    auto& pip = stage.image({640, 360, pattern.data(), 0}, Fit::Cover);
    pip.frame({kW - 360, 40, 320, 180})
        .cornerRadius(14)
        .masksToBounds(true)
        .border(2, {1, 1, 1, 0.6f})
        .shadow(0, 6, 14)
        .hidden(true);  // shown once the second camera delivers

    std::thread server(serverLoop, port);

    // ---- render loop (owns the Stage) ----
    WaterSim water;
    std::vector<uint8_t> cam_rgba;      // latest main camera frame
    std::vector<uint8_t> pip_rgba;      // latest second camera frame
    std::vector<uint8_t> warped;        // water-refracted copy handed to the layer
    uint32_t cam_w = 640, cam_h = 360;
    uint32_t pip_w = 0, pip_h = 0;
    cam_rgba = pattern;
    uint64_t seen_seq[2] = {0, 0};
    bool have_ros = false;
    bool have_pip = false;
    int current_effect = -1;
    float current_param = -2;
    uint32_t tick = 0;

    const auto frame_time = std::chrono::duration<double>(1.0 / kFps);
    auto next = std::chrono::steady_clock::now();
    while (g_running) {
        ++tick;

        // ---- latest camera frames (ROS latest-wins, else animated pattern) -
#ifdef FS_HAVE_ROS
        for (int cam = 0; cam < 2; ++cam) {
            if (g_ros[cam].seq == seen_seq[cam]) {
                continue;
            }
            std::vector<uint8_t> jpeg;
            {
                std::lock_guard<std::mutex> lock(g_ros[cam].mutex);
                jpeg = g_ros[cam].jpeg;
                seen_seq[cam] = g_ros[cam].seq;
            }
            uint32_t w = 0, h = 0;
            std::vector<uint8_t> rgba;
            if (!decodeJpegToRgba(jpeg.data(), jpeg.size(), rgba, w, h)) {
                continue;
            }
            if (cam == 0) {
                cam_rgba.swap(rgba);
                if (w != cam_w || h != cam_h) {
                    cam_w = w;
                    cam_h = h;
                    water.reset(cam_w, cam_h);
                }
                if (!have_ros) {
                    have_ros = true;
                    src_label.setText(topic + "  +  " + pip_topic);
                }
            } else {
                pip_rgba.swap(rgba);
                pip_w = w;
                pip_h = h;
                if (!have_pip) {
                    have_pip = true;
                    pip.hidden(false);
                }
            }
        }
#endif
        if (!have_ros) {
            cam_rgba = makeTestPattern(cam_w, cam_h, tick);
        }
        if (water.width() != cam_w) {
            water.reset(cam_w, cam_h);
        }

        // ---- pointer events: stage coords → camera pixels (Cover inverse) --
        const float cover = std::max(static_cast<float>(kW) / cam_w,
                                     static_cast<float>(kH) / cam_h);
        const float off_x = (cam_w * cover - kW) * 0.5f;
        const float off_y = (cam_h * cover - kH) * 0.5f;
        std::vector<PointerEventIn> events;
        {
            std::lock_guard<std::mutex> lock(g_events_mutex);
            events.swap(g_events);
        }
        for (const PointerEventIn& e : events) {
            const float cx = (e.x + off_x) / cover;
            const float cy = (e.y + off_y) / cover;
            if (e.phase == 'd') {
                water.drop(cx, cy, 900, 5);  // a real splash
            } else if (e.phase == 'm' || e.phase == 'h') {
                water.drop(cx, cy, 220, 2);  // fine wake trailing the pointer
            }
        }

        // ---- water simulation + warp --------------------------------------
        const bool water_on = g_water;
        if (water_on) {
            water.step();
            warped.resize(cam_rgba.size());
            water.warp(cam_rgba.data(), warped.data());
        }
        video.setImage({cam_w, cam_h,
                        water_on ? warped.data() : cam_rgba.data(), 0});

        // The PiP glides at the full render rate — layer motion is
        // independent of how fast its camera delivers frames.
        if (have_pip) {
            pip.setImage({pip_w, pip_h, pip_rgba.data(), 0});
            const float t = tick / kFps;
            pip.position(kW - 200 + 90 * std::sin(t * 0.55f),
                         150 + 70 * std::sin(t * 0.37f + 1.3f));
        }

        // ---- effect selection (browser dropdown + slider) ------------------
        const int effect = g_effect;
        const float param = g_param;
        if (effect != current_effect || param != current_param) {
            current_effect = effect;
            current_param = param;
            video.clearFilters();
            const auto& table = filterTable();
            if (effect > 0 && effect <= static_cast<int>(table.size())) {
                const FilterSpec& spec = table[static_cast<size_t>(effect - 1)];
                Filter f{spec.mode, {}};
                for (size_t i = 0; i < spec.params.size() && i < 5; ++i) {
                    f.values[i] = spec.params[i].default_value;
                }
                if (param >= 0 && !spec.params.empty()) {
                    // The slider sweeps the first parameter: 0 → 2×default
                    // (or 0..1 when the default is 0). Crude but live.
                    const float def = spec.params[0].default_value;
                    f.values[0] = def != 0 ? 2 * def * param : param;
                }
                video.filter(f);
            }
        }

        const float dt = static_cast<float>(frame_time.count());
        const Surface& frame = renderer->render(stage, dt);

        // ---- H.264 out (nvenc; one-shot fallback to libx264) ---------------
        if (!encoder_alive && nvenc) {
            // Closing stdin ends the child; the reader unblocks on EOF.
            close(encoder.stdin_fd);
            au_reader.join();
            close(encoder.stdout_fd);
            waitpid(encoder.pid, nullptr, 0);
            nvenc = false;
            encoder = spawnEncoder(false);
            encoder_alive = true;
            au_reader = std::thread(reader_fn);
            std::printf("stage_web: nvenc unavailable, using libx264\n");
            std::fflush(stdout);
        }
        if (encoder_alive) {
            for (uint32_t y = 0; y < frame.height; ++y) {
                if (!writeAll(encoder.stdin_fd, frame.row(y), frame.width * 4)) {
                    encoder_alive = false;
                    break;
                }
            }
        }
        if (g_mjpeg_clients > 0 && (tick & 1) == 0) {  // fallback runs at 30 fps
            std::lock_guard<std::mutex> lock(g_frame_mutex);
            encodeJpeg(frame, g_jpeg);
            ++g_frame_seq;
            g_frame_cv.notify_all();
        }

        // Loop health: real fps to the log every 5 s — declared 60 means
        // nothing if the loop can't hold it.
        static auto fps_t0 = std::chrono::steady_clock::now();
        static uint32_t fps_frames = 0;
        ++fps_frames;
        const auto now0 = std::chrono::steady_clock::now();
        if (now0 - fps_t0 >= std::chrono::seconds(5)) {
            std::printf("stage_web: render %.1f fps\n",
                        fps_frames / std::chrono::duration<double>(now0 - fps_t0).count());
            std::fflush(stdout);
            fps_t0 = now0;
            fps_frames = 0;
        }

        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_time);
        const auto now = std::chrono::steady_clock::now();
        if (now - next > std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                             frame_time * 2)) {
            next = now;  // fell behind: never burst-catch-up (that is latency)
        }
        std::this_thread::sleep_until(next);
    }
    server.join();
#ifdef FS_HAVE_ROS
    ros_thread.join();
#endif
    return 0;
}
