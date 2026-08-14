#pragma once

// wsvideo.hpp — the proven low-latency browser video transport, extracted
// from stage_web (which learned it the hard way): H.264 (ffmpeg h264_nvenc,
// libx264 fallback) split into access units and fanned out over WebSockets
// with bounded queues and skip-to-keyframe — latency can never accumulate.
// The MSE path muxes each AU into one fMP4 fragment server-side.
//
// MJPEG is not offered here on purpose: per-frame stills over WAN proxies
// queue without bound and the project has the scars to prove it.

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace wsvideo {

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
    bool mp4 = false;        // mux to fMP4 for the MSE path
    bool want_frame = false; // ack-paced stills: page consumed the last one
};

inline std::mutex g_clients_mutex;
inline std::vector<std::shared_ptr<VideoClient>> g_clients;

inline void broadcastAu(AuPtr au) {
    std::lock_guard<std::mutex> lock(g_clients_mutex);
    for (auto& c : g_clients) {
        std::lock_guard<std::mutex> cl(c->mutex);
        if (c->wait_for_key && !au->key) {
            continue;
        }
        // A stalling client skips to the next keyframe — deltas cannot be
        // dropped individually and latency must never accumulate. The cap is
        // deliberately tight (under a second even on a slow link): WAN viewers should skip
        // early rather than nurse a growing backlog.
        if (c->queued_bytes > 384u << 10) {
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

struct Encoder {
    pid_t pid = -1;
    int stdin_fd = -1;
    int stdout_fd = -1;
};

inline Encoder spawnEncoder(bool nvenc, uint32_t width, uint32_t height, int fps_i) {
    int in_pipe[2], out_pipe[2];
    if (pipe(in_pipe) != 0 || pipe(out_pipe) != 0) {
        return {};
    }
    const std::string size = std::to_string(width) + "x" + std::to_string(height);
    const std::string fps = std::to_string(fps_i);
    const std::string gop = std::to_string(fps_i);  // 1 s
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
                   "-delay", "0", "-bf", "0", "-g", gop.c_str(), "-b:v", "4M", "-maxrate",
                   "5M", "-bufsize", "1M", "-profile:v", "baseline", "-pix_fmt", "yuv420p",
                   "-f", "h264", "pipe:1", static_cast<char*>(nullptr));
        } else {
            execlp("ffmpeg", "ffmpeg", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt",
                   "rgba", "-s", size.c_str(), "-r", fps.c_str(), "-i", "pipe:0", "-c:v",
                   "libx264", "-preset", "ultrafast", "-tune", "zerolatency", "-bf", "0",
                   "-g", gop.c_str(), "-b:v", "4M", "-profile:v", "baseline", "-pix_fmt",
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

inline void sha1(const uint8_t* data, size_t len, uint8_t out[20]) {
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

inline std::string base64(const uint8_t* data, size_t len) {
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

inline bool sendAll(int fd, const void* data, size_t len) {  // sockets
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

inline bool writeAll(int fd, const void* data, size_t len) {  // pipes
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

inline bool wsSendBinary(int fd, const uint8_t* payload, size_t len) {
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

}  // namespace wsvideo
