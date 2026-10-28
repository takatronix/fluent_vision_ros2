#pragma once
// JPEG を開く・作るを Jetson (Thor / Orin) の専用回路 NVJPG に任せる (2026-10-29、ヘッダだけ)。
//
// 経路は GStreamer の nvjpegdec / nvjpegenc (JetPack 7.2 / L4T R39 では libnvmm_jpeg が裏にいる)。
// 要素が無い・初期化に失敗した・応答しなくなった時は OpenCV (libjpeg-turbo、CPU) に自動で戻る。
// どちらで動いているかは backend() で分かる。
//
// 使う側の CMake で pkg-config の gstreamer-1.0 / gstreamer-app-1.0 / gstreamer-video-1.0 を見つけたら
// FLUENT_HW_JPEG_GST=1 を定義してリンクする。無ければこのヘッダは OpenCV だけで組める。
// 1 インスタンスは 1 スレッドから使う。ROS には依存しない。
//
// 色のレンジ: JPEG (JFIF) の YUV はフルレンジ。nvvidconv に YUV↔RGB を任せると制限レンジとして伸び縮み
// して絵が歪む (1.23 倍 − 30、aspa2 実測) ので、YUV↔RGB は自前の CPU 変換で行い、専用回路には
// 開く (JPEG→I420)・縮小/回転 (VIC)・作る (I420→JPEG) だけ任せる。
//
// 同じ物が aspa-navigation の aspa_ui/hw_jpeg.{hpp,cpp} にもある (別リポジトリなので写し)。
//
// aspa2 (Thor、JetPack 7.2) の実測 (tools/hw_jpeg_check): 1280x720 を開く CPU 4.1→2.8 ms、1280x720 を作る 4.4→3.0 ms、
// 1080x1920 を作る 10.0→5.6 ms。1080x1920 を原寸で開く (回転 90) は 13.4→13.8 ms で得が無い (自前の YUV→BGR 変換が
// 画素数ぶん掛かる。VIC は制限レンジでしか変換できない)。開く側は縮小して使う所 (aspa_ui の合成) で効く。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#ifdef FLUENT_HW_JPEG_GST
#include <dlfcn.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/video.h>
#endif

namespace fluent {
namespace hwjpeg {

// JPEG の SOF から幅・高さを読む (baseline/progressive)。無効なら false。
inline bool sof_dimensions(const uint8_t* d, size_t n, int& width, int& height) {
  width = height = 0;
  if (n < 4 || d[0] != 0xFF || d[1] != 0xD8) return false;
  size_t i = 2;
  while (i + 4 <= n) {
    if (d[i] != 0xFF) { ++i; continue; }
    const uint8_t marker = d[i+1];
    if (marker == 0xFF) { ++i; continue; }
    if (marker == 0xD8 || (marker >= 0xD0 && marker <= 0xD7) || marker == 0x01) { i += 2; continue; }
    if (marker == 0xD9 || marker == 0xDA) return false;
    const size_t len = (size_t(d[i+2]) << 8) | d[i+3];
    if (len < 2 || i + 2 + len > n) return false;
    const bool sof = (marker >= 0xC0 && marker <= 0xCF) && marker != 0xC4 && marker != 0xC8 && marker != 0xCC;
    if (sof) {
      if (len < 7) return false;
      height = (d[i+5] << 8) | d[i+6];
      width = (d[i+7] << 8) | d[i+8];
      return width > 0 && height > 0;
    }
    i += 2 + len;
  }
  return false;
}

// SOI〜最後の EOI までの長さ (EOI の後の詰め物は数えない)。切れていれば 0。
// 切れた JPEG を nvjpegdec に入れるとプラグインの中で落ちる (aspa2 実測) ので、専用回路には揃った物だけ渡す。
inline size_t complete_length(const uint8_t* d, size_t n) {
  if (n < 4 || d[0] != 0xFF || d[1] != 0xD8) return 0;
  for (size_t i = n; i >= 2; --i) if (d[i-2] == 0xFF && d[i-1] == 0xD9) return i;
  return 0;
}

inline uint8_t clamp8(int v) { return uint8_t(v < 0 ? 0 : v > 255 ? 255 : v); }

// JFIF (フルレンジ BT.601) の I420 → BGR (CV_8UC3)。色差は最近傍で広げる。
inline void i420_full_to_bgr(const uint8_t* const planes[3], const int strides[3], int w, int h, cv::Mat& bgr) {
  bgr.create(h, w, CV_8UC3);
  for (int y = 0; y < h; ++y) {
    const uint8_t* yp = planes[0] + size_t(strides[0])*y;
    const uint8_t* up = planes[1] + size_t(strides[1])*(y/2);
    const uint8_t* vp = planes[2] + size_t(strides[2])*(y/2);
    uint8_t* out = bgr.ptr(y);
    for (int x = 0; x < w; ++x, out += 3) {
      const int Y = yp[x], U = up[x/2] - 128, V = vp[x/2] - 128;
      out[2] = clamp8(Y + ((359*V + 128) >> 8));
      out[1] = clamp8(Y - ((88*U + 183*V + 128) >> 8));
      out[0] = clamp8(Y + ((454*U + 128) >> 8));
    }
  }
}

// JFIF (フルレンジ BT.601) の BGR (CV_8UC3) → I420。U/V は 2×2 の平均。幅・高さは偶数。
inline void bgr_to_i420_full(const cv::Mat& bgr, uint8_t* dst) {
  const int w = bgr.cols, h = bgr.rows;
  uint8_t* y_plane = dst;
  uint8_t* u_plane = dst + size_t(w)*h;
  uint8_t* v_plane = u_plane + size_t(w/2)*(h/2);
  for (int y = 0; y < h; y += 2) {
    const uint8_t* r0 = bgr.ptr(y);
    const uint8_t* r1 = bgr.ptr(y+1);
    uint8_t* y0 = y_plane + size_t(y)*w;
    uint8_t* y1 = y0 + w;
    uint8_t* u = u_plane + size_t(y/2)*(w/2);
    uint8_t* v = v_plane + size_t(y/2)*(w/2);
    for (int x = 0; x < w; x += 2) {
      int sr = 0, sg = 0, sb = 0;
      const uint8_t* px[4] = {r0 + x*3, r0 + (x+1)*3, r1 + x*3, r1 + (x+1)*3};
      uint8_t* yo[4] = {y0 + x, y0 + x + 1, y1 + x, y1 + x + 1};
      for (int k = 0; k < 4; ++k) {
        const int b = px[k][0], g = px[k][1], r = px[k][2];
        *yo[k] = uint8_t((77*r + 150*g + 29*b + 128) >> 8);
        sr += r; sg += g; sb += b;
      }
      u[x/2] = uint8_t(((-43*sr - 85*sg + 128*sb + 512) >> 10) + 128);
      v[x/2] = uint8_t(((128*sr - 107*sg - 21*sb + 512) >> 10) + 128);
    }
  }
}

#ifdef FLUENT_HW_JPEG_GST
namespace detail {
constexpr int kMaxFailures = 3;          // 連続して応えない回数がこれを超えたら OpenCV に固定
constexpr int64_t kPullTimeoutMs = 1000; // 1 コマの応答待ち (NVJPG は 1080p でも 30 ms 程度)

inline bool gst_ready(std::string* why) {
  static std::once_flag once; static bool ok = false; static std::string reason;
  std::call_once(once, [] {
    // libgstnvjpeg.so は libnvmm_jpeg.so の jpeg_* (libjpeg-9f + Tegra) を呼ぶ。このプロセスには OpenCV 経由で
    // libjpeg.so.8 も居て同じ名前の jpeg_* が先に見えるため、普通に読み込むと nvjpegdec が落ちる (aspa2 で確認)。
    // 先に RTLD_DEEPBIND で読んでおくと両 .so が自分の依存先の jpeg_* を使う。
    const int flags = RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND;
    for (const char* p : {"libnvmm_jpeg.so", "/usr/lib/aarch64-linux-gnu/nvidia/libnvmm_jpeg.so"}) if (dlopen(p, flags)) break;
    for (const char* p : {"/usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstnvjpeg.so", "libgstnvjpeg.so"}) if (dlopen(p, flags)) break;
    GError* err = nullptr;
    if (!gst_init_check(nullptr, nullptr, &err)) {
      reason = std::string("gst_init failed: ") + (err ? err->message : "?");
      if (err) g_error_free(err);
      return;
    }
    for (const char* name : {"appsrc", "appsink", "nvjpegdec", "nvjpegenc", "nvvidconv"}) {
      GstElementFactory* f = gst_element_factory_find(name);
      if (!f) { reason = std::string("GStreamer element missing: ") + name; return; }
      gst_object_unref(f);
    }
    ok = true;
  });
  if (why) *why = reason;
  return ok;
}

// appsrc → … → appsink を 1 コマずつ同期で回す土台。
struct SyncPipeline {
  GstElement* pipeline = nullptr;
  GstAppSrc* src = nullptr;
  GstAppSink* sink = nullptr;
  int failures = 0;
  std::string error;
  ~SyncPipeline() { teardown(); }
  void teardown() {
    if (pipeline) {
      gst_element_set_state(pipeline, GST_STATE_NULL);
      if (src) gst_object_unref(src);
      if (sink) gst_object_unref(sink);
      gst_object_unref(pipeline);
    }
    pipeline = nullptr; src = nullptr; sink = nullptr;
  }
  bool build(const std::string& desc) {
    teardown();
    GError* err = nullptr;
    pipeline = gst_parse_launch(desc.c_str(), &err);
    if (!pipeline || err) {
      error = std::string("pipeline: ") + (err ? err->message : "?");
      if (err) g_error_free(err);
      teardown();
      return false;
    }
    src = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline), "src"));
    sink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline), "sink"));
    if (!src || !sink) { error = "pipeline: appsrc/appsink missing"; teardown(); return false; }
    gst_app_src_set_stream_type(src, GST_APP_STREAM_TYPE_STREAM);
    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      error = "pipeline: cannot start (" + bus_error() + ")";
      teardown();
      return false;
    }
    return true;
  }
  std::string bus_error() {
    std::string out;
    GstBus* bus = pipeline ? gst_element_get_bus(pipeline) : nullptr;
    if (!bus) return out;
    while (GstMessage* msg = gst_bus_pop_filtered(bus, GstMessageType(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING))) {
      GError* e = nullptr; gchar* dbg = nullptr;
      if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) gst_message_parse_error(msg, &e, &dbg);
      else gst_message_parse_warning(msg, &e, &dbg);
      if (e) { if (!out.empty()) out += "; "; out += e->message; g_error_free(e); }
      g_free(dbg);
      gst_message_unref(msg);
    }
    gst_object_unref(bus);
    return out;
  }
  GstSample* roundtrip(GstBuffer* in) {   // in の所有権は渡す
    if (gst_app_src_push_buffer(src, in) != GST_FLOW_OK) {
      error = "push failed (" + bus_error() + ")"; ++failures; return nullptr;
    }
    GstSample* sample = gst_app_sink_try_pull_sample(sink, kPullTimeoutMs*GST_MSECOND);
    if (!sample) {
      error = "no output within " + std::to_string(kPullTimeoutMs) + " ms (" + bus_error() + ")";
      ++failures; return nullptr;
    }
    failures = 0;
    return sample;
  }
};
}  // namespace detail
#endif

inline double ms_since(std::chrono::steady_clock::time_point t) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-t).count();
}

// JPEG → BGR (CV_8UC3)。max_w×max_h に収まるよう縮小 (0 = しない)。rotation は時計回りの度数 (0/90/180/270、VIC で回す)。
class Decoder {
public:
  explicit Decoder(bool allow_hw = true) : allow_hw_(allow_hw) {
#ifdef FLUENT_HW_JPEG_GST
    if (allow_hw_ && detail::gst_ready(&hw_error_)) backend_ = "nvjpeg";
#else
    hw_error_ = "built without GStreamer";
#endif
    if (!allow_hw_) hw_error_ = "disabled";
  }
  static bool hardware_available(std::string* why = nullptr) {
#ifdef FLUENT_HW_JPEG_GST
    return detail::gst_ready(why);
#else
    if (why) *why = "built without GStreamer";
    return false;
#endif
  }
  bool decode(const uint8_t* data, size_t size, cv::Mat& bgr, int rotation = 0, int max_w = 0, int max_h = 0,
              std::string* error = nullptr) {
    const auto started = std::chrono::steady_clock::now();
    int w = 0, h = 0;
    if (!sof_dimensions(data, size, w, h)) { if (error) *error = "invalid JPEG header"; last_ms_ = ms_since(started); return false; }
#ifdef FLUENT_HW_JPEG_GST
    if (backend_ == "nvjpeg") {
      const size_t complete = complete_length(data, size);
      std::string err;
      if (complete > 0 && hw_decode(data, complete, w, h, rotation, max_w, max_h, bgr, &err)) {
        last_backend_ = "nvjpeg"; last_ms_ = ms_since(started); return true;
      }
      if (complete > 0) {
        pipe_.teardown();   // 応えなかったパイプラインは作り直す
        if (pipe_.failures >= detail::kMaxFailures) { backend_ = "opencv"; hw_error_ = "nvjpeg gave up: " + err; }
      }
    }
#endif
    last_backend_ = "opencv";
    const bool ok = cpu_decode(data, size, rotation, max_w, max_h, bgr, error);
    last_ms_ = ms_since(started);
    return ok;
  }
  const char* backend() const { return backend_.c_str(); }
  const char* last_backend() const { return last_backend_.c_str(); }
  bool hardware() const { return backend_ == "nvjpeg"; }
  const std::string& hardware_error() const { return hw_error_; }
  double last_ms() const { return last_ms_; }

private:
  static void fit(int w, int h, int max_w, int max_h, int& ow, int& oh) {
    double scale = 1.0;
    if (max_w > 0) scale = std::min(scale, double(max_w)/w);
    if (max_h > 0) scale = std::min(scale, double(max_h)/h);
    ow = std::max(2, int(std::lround(w*scale)) & ~1);
    oh = std::max(2, int(std::lround(h*scale)) & ~1);
  }
  static bool cpu_decode(const uint8_t* data, size_t size, int rotation, int max_w, int max_h, cv::Mat& bgr, std::string* error) {
    const cv::Mat buf(1, int(size), CV_8U, const_cast<uint8_t*>(data));
    cv::Mat out = cv::imdecode(buf, cv::IMREAD_COLOR);
    if (out.empty()) { if (error) *error = "JPEG decode failed"; return false; }
    int ow = 0, oh = 0;
    fit(out.cols, out.rows, max_w, max_h, ow, oh);
    if (ow != out.cols || oh != out.rows) cv::resize(out, out, {ow, oh}, 0, 0, cv::INTER_AREA);
    switch (rotation) {
      case 90: cv::rotate(out, out, cv::ROTATE_90_CLOCKWISE); break;
      case 180: cv::rotate(out, out, cv::ROTATE_180); break;
      case 270: cv::rotate(out, out, cv::ROTATE_90_COUNTERCLOCKWISE); break;
      default: break;
    }
    bgr = std::move(out);
    return true;
  }
#ifdef FLUENT_HW_JPEG_GST
  bool hw_decode(const uint8_t* data, size_t size, int w, int h, int rotation, int max_w, int max_h, cv::Mat& bgr, std::string* error) {
    int ow = 0, oh = 0;
    fit(w, h, max_w, max_h, ow, oh);
    const bool swap = rotation == 90 || rotation == 270;
    const int rw = swap ? oh : ow, rh = swap ? ow : oh;   // 回した後の寸法
    // nvvidconv flip-method: 1 = 反時計回り 90、2 = 180、3 = 時計回り 90
    const int flip = rotation == 90 ? 3 : rotation == 180 ? 2 : rotation == 270 ? 1 : 0;
    if (!pipe_.pipeline || w != in_w_ || h != in_h_ || rw != out_w_ || rh != out_h_ || flip != flip_) {
      const std::string desc =
        "appsrc name=src is-live=true format=time do-timestamp=true caps=image/jpeg,framerate=0/1 ! "
        "nvjpegdec ! nvvidconv interpolation-method=5 flip-method=" + std::to_string(flip) +
        " ! video/x-raw,format=I420,width=" + std::to_string(rw) + ",height=" + std::to_string(rh) +
        " ! appsink name=sink sync=false max-buffers=1 drop=true";
      if (!pipe_.build(desc)) { if (error) *error = pipe_.error; ++pipe_.failures; return false; }
      in_w_ = w; in_h_ = h; out_w_ = rw; out_h_ = rh; flip_ = flip;
    }
    GstBuffer* in = gst_buffer_new_allocate(nullptr, size, nullptr);
    gst_buffer_fill(in, 0, data, size);
    GstSample* sample = pipe_.roundtrip(in);
    if (!sample) { if (error) *error = pipe_.error; return false; }
    bool ok = false;
    GstCaps* caps = gst_sample_get_caps(sample);
    GstBuffer* buf = gst_sample_get_buffer(sample);
    GstVideoInfo info;
    GstVideoFrame frame;
    if (caps && buf && gst_video_info_from_caps(&info, caps) && GST_VIDEO_INFO_FORMAT(&info) == GST_VIDEO_FORMAT_I420 &&
        GST_VIDEO_INFO_WIDTH(&info) == rw && GST_VIDEO_INFO_HEIGHT(&info) == rh &&
        gst_video_frame_map(&frame, &info, buf, GST_MAP_READ)) {
      const uint8_t* planes[3]; int strides[3];
      for (int p = 0; p < 3; ++p) {
        planes[p] = static_cast<const uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&frame, p));
        strides[p] = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, p);
      }
      i420_full_to_bgr(planes, strides, rw, rh, bgr);
      gst_video_frame_unmap(&frame);
      ok = true;
    } else {
      gchar* c = caps ? gst_caps_to_string(caps) : nullptr;
      pipe_.error = std::string("unexpected output ") + (c ? c : "(no caps)");
      g_free(c);
      ++pipe_.failures;
    }
    gst_sample_unref(sample);
    if (!ok && error) *error = pipe_.error;
    return ok;
  }
  detail::SyncPipeline pipe_;
  int in_w_ = 0, in_h_ = 0, out_w_ = 0, out_h_ = 0, flip_ = 0;
#endif
  bool allow_hw_;
  std::string backend_ = "opencv", last_backend_ = "opencv", hw_error_;
  double last_ms_ = 0;
};

// BGR (CV_8UC3、step は任意) → JPEG。quality 1..100。幅・高さが 8 の倍数でない絵はその 1 コマだけ OpenCV で作る。
class Encoder {
public:
  explicit Encoder(bool allow_hw = true) : allow_hw_(allow_hw) {
#ifdef FLUENT_HW_JPEG_GST
    if (allow_hw_ && detail::gst_ready(&hw_error_)) backend_ = "nvjpeg";
#else
    hw_error_ = "built without GStreamer";
#endif
    if (!allow_hw_) hw_error_ = "disabled";
  }
  static bool hardware_available(std::string* why = nullptr) { return Decoder::hardware_available(why); }
  bool encode(const cv::Mat& bgr, int quality, std::vector<uint8_t>& out, std::string* error = nullptr) {
    const auto started = std::chrono::steady_clock::now();
    if (bgr.empty() || bgr.type() != CV_8UC3) { if (error) *error = "encoder expects CV_8UC3"; return false; }
    quality = std::max(1, std::min(100, quality));
#ifdef FLUENT_HW_JPEG_GST
    if (backend_ == "nvjpeg" && bgr.cols % 8 == 0 && bgr.rows % 8 == 0) {
      std::string err;
      if (hw_encode(bgr, quality, out, &err)) { last_backend_ = "nvjpeg"; last_ms_ = ms_since(started); return true; }
      pipe_.teardown();
      if (pipe_.failures >= detail::kMaxFailures) { backend_ = "opencv"; hw_error_ = "nvjpeg gave up: " + err; }
    }
#endif
    last_backend_ = "opencv";
    const bool ok = cv::imencode(".jpg", bgr, out, {cv::IMWRITE_JPEG_QUALITY, quality});
    if (!ok && error) *error = "JPEG encode failed";
    last_ms_ = ms_since(started);
    return ok;
  }
  const char* backend() const { return backend_.c_str(); }
  const char* last_backend() const { return last_backend_.c_str(); }
  bool hardware() const { return backend_ == "nvjpeg"; }
  const std::string& hardware_error() const { return hw_error_; }
  double last_ms() const { return last_ms_; }

private:
#ifdef FLUENT_HW_JPEG_GST
  bool hw_encode(const cv::Mat& bgr, int q, std::vector<uint8_t>& out, std::string* error) {
    const int w = bgr.cols, h = bgr.rows;
    if (!pipe_.pipeline || w != in_w_ || h != in_h_ || q != quality_) {
      const std::string desc =
        "appsrc name=src is-live=true format=time do-timestamp=true caps=video/x-raw,format=I420,width=" +
        std::to_string(w) + ",height=" + std::to_string(h) + ",framerate=0/1 ! "
        "nvjpegenc quality=" + std::to_string(q) + " ! appsink name=sink sync=false max-buffers=1 drop=true";
      if (!pipe_.build(desc)) { if (error) *error = pipe_.error; ++pipe_.failures; return false; }
      in_w_ = w; in_h_ = h; quality_ = q;
    }
    GstBuffer* in = gst_buffer_new_allocate(nullptr, size_t(w)*h*3/2, nullptr);
    GstMapInfo map;
    if (!gst_buffer_map(in, &map, GST_MAP_WRITE)) { gst_buffer_unref(in); if (error) *error = "cannot map input"; return false; }
    bgr_to_i420_full(bgr, map.data);
    gst_buffer_unmap(in, &map);
    GstSample* sample = pipe_.roundtrip(in);
    if (!sample) { if (error) *error = pipe_.error; return false; }
    GstBuffer* buf = gst_sample_get_buffer(sample);
    bool ok = false;
    if (buf && gst_buffer_map(buf, &map, GST_MAP_READ)) {
      out.assign(map.data, map.data + map.size);
      gst_buffer_unmap(buf, &map);
      ok = out.size() > 4 && out[0] == 0xFF && out[1] == 0xD8;
      if (!ok) { pipe_.error = "output is not a JPEG"; ++pipe_.failures; }
    } else { pipe_.error = "cannot map output"; ++pipe_.failures; }
    gst_sample_unref(sample);
    if (!ok && error) *error = pipe_.error;
    return ok;
  }
  detail::SyncPipeline pipe_;
  int in_w_ = 0, in_h_ = 0, quality_ = 0;
#endif
  bool allow_hw_;
  std::string backend_ = "opencv", last_backend_ = "opencv", hw_error_;
  double last_ms_ = 0;
};

}  // namespace hwjpeg
}  // namespace fluent
