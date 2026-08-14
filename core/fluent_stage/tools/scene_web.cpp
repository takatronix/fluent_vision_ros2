// scene_web — live Scene (.fvs) preview in a browser: the L2 loop made
// tangible. Edit the file, save, and the picture changes **atomically at a
// frame boundary** (§2: validate → compile → activate); a broken edit keeps
// the last good picture on screen with an error banner instead of ever
// showing a broken frame.
//
//   scene_web scene.fvs [--port 8791] [--fps 30] [--image input=topic]...
//
//   GET /        minimal viewer page (stream + status readout)
//   GET /stream  MJPEG of the live scene
//   GET /status  JSON: digest, reload counter, errors, lint warnings
//
// `--image camera=/aspa/d405/color_compressed` feeds a ROS 2 CompressedImage
// topic into the declared `$inputs.camera` (build with the ROS environment
// sourced, like stage_web). Inputs without a feed show their built-in
// placeholder panels — which is itself the demo of the input contract.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <jpeglib.h>

#include <fluent_stage/cpu_renderer.hpp>
#include <fluent_stage/fluent_stage.hpp>
#include <fluent_stage/scene/compiler.hpp>
#include <fluent_stage/scene/document.hpp>
#include <fluent_stage/scene/inspector.hpp>
#include <fluent_stage/scene/linter.hpp>
#ifdef FS_HAVE_VULKAN
#include <fluent_stage/vulkan_renderer.hpp>
#endif
#ifdef FS_HAVE_ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#endif

using namespace fluent_stage;
namespace fsc = fluent_stage::scene;

namespace {

std::atomic<bool> g_running{true};

// ---- latest rendered JPEG, shared with the stream clients ------------------

std::mutex g_frame_mutex;
std::condition_variable g_frame_cv;
std::vector<uint8_t> g_jpeg;
uint64_t g_frame_seq = 0;

// ---- live status, shared with /status --------------------------------------

std::mutex g_status_mutex;
std::string g_status_json = "{}";

// ---- inspector snapshot, shared with /inspect and /at (§13-3) --------------
// Plain values only (serialized JSON + geometry): the HTTP threads never
// touch the live scene, so a reload swap can never race a query.

struct InspectEntry {
    std::string json;
    fluent_stage::Mat23 to_stage;
    fluent_stage::Rect bounds;
    float eff_opacity = 1;
    int paint_index = 0;
};
std::mutex g_inspect_mutex;
std::string g_inspect_json = "{}";
std::vector<InspectEntry> g_inspect_entries;

bool sendAll(int fd, const void* data, size_t size) {
    const char* p = static_cast<const char*>(data);
    while (size > 0) {
        const ssize_t n = ::send(fd, p, size, MSG_NOSIGNAL);
        if (n <= 0) {
            return false;
        }
        p += n;
        size -= static_cast<size_t>(n);
    }
    return true;
}

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
}

// The page is a viewer, nothing more: the picture is the product, and the
// status line just mirrors /status so a broken save is legible from the
// browser too.
const char* kPage = R"HTML(<!doctype html>
<meta charset="utf-8"><title>fluent scene — live</title>
<style>
  body { margin: 0; background: #0b0d10; color: #dfe5ec;
         font: 14px/1.5 system-ui, sans-serif; }
  img { display: block; width: 100vw; height: auto; }
  #bar { padding: 8px 14px; white-space: pre-wrap;
         font-family: ui-monospace, monospace; color: #9aa7b4; }
  #bar.err { color: #ff8a80; }
</style>
<img src="/stream">
<div id="bar">…</div>
<script>
async function tick() {
  try {
    const s = await (await fetch('/status')).json();
    const bar = document.getElementById('bar');
    bar.className = s.errors.length ? 'err' : '';
    bar.textContent = 'digest ' + s.digest.slice(0, 12) +
        '  reloads ' + s.reloads +
        (s.errors.length ? '\n' + s.errors.join('\n') : '') +
        (s.warnings.length ? '\n' + s.warnings.join('\n') : '');
  } catch (e) {}
  setTimeout(tick, 1000);
}
tick();
</script>
)HTML";

void serverLoop(uint16_t port) {
    const int listener = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    if (bind(listener, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0 ||
        listen(listener, 8) != 0) {
        std::fprintf(stderr, "scene_web: cannot listen on port %u\n", port);
        g_running = false;
        return;
    }
    std::printf("scene_web: http://0.0.0.0:%u/\n", port);
    std::fflush(stdout);
    while (g_running) {
        const int fd = ::accept(listener, nullptr, nullptr);
        if (fd < 0) {
            continue;
        }
        std::thread([fd] {
            char buf[2048];
            const ssize_t n = ::recv(fd, buf, sizeof buf - 1, 0);
            if (n <= 0) {
                ::close(fd);
                return;
            }
            buf[n] = '\0';
            const std::string req(buf);
            if (req.rfind("GET /stream", 0) == 0) {
                streamMjpeg(fd);
            } else if (req.rfind("GET /inspect", 0) == 0) {
                std::string body;
                {
                    std::lock_guard<std::mutex> lock(g_inspect_mutex);
                    body = g_inspect_json;
                }
                sendResponse(fd, "200 OK", "application/json", body);
            } else if (req.rfind("GET /at?", 0) == 0) {
                // /at?x=<logical>&y=<logical> — what is visible at a point.
                float x = 0, y = 0;
                const size_t xp = req.find("x="), yp = req.find("y=");
                if (xp != std::string::npos && yp != std::string::npos) {
                    x = std::strtof(req.c_str() + xp + 2, nullptr);
                    y = std::strtof(req.c_str() + yp + 2, nullptr);
                }
                std::string body = "{\"at\": [" + std::to_string(x) + ", " +
                                   std::to_string(y) + "], \"layers\": [";
                {
                    std::lock_guard<std::mutex> lock(g_inspect_mutex);
                    std::vector<const InspectEntry*> hits;
                    for (const InspectEntry& e : g_inspect_entries) {
                        if (e.eff_opacity <= 0 || e.bounds.w <= 0 || e.bounds.h <= 0) {
                            continue;
                        }
                        if (e.bounds.contains(e.to_stage.inverse().apply({x, y}))) {
                            hits.push_back(&e);
                        }
                    }
                    std::sort(hits.begin(), hits.end(),
                              [](const InspectEntry* a, const InspectEntry* b) {
                                  return a->paint_index > b->paint_index;
                              });
                    for (size_t i = 0; i < hits.size(); ++i) {
                        body += (i ? ", " : "") + hits[i]->json;
                    }
                }
                body += "]}";
                sendResponse(fd, "200 OK", "application/json", body);
            } else if (req.rfind("GET /status", 0) == 0) {
                std::string body;
                {
                    std::lock_guard<std::mutex> lock(g_status_mutex);
                    body = g_status_json;
                }
                sendResponse(fd, "200 OK", "application/json", body);
            } else if (req.rfind("GET / ", 0) == 0) {
                sendResponse(fd, "200 OK", "text/html; charset=utf-8", kPage);
            } else {
                sendResponse(fd, "404 Not Found", "text/plain", "not found\n");
            }
            ::close(fd);
        }).detach();
    }
    ::close(listener);
}

// ---- jpeg helpers (same shapes as stage_web) --------------------------------

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
    jpeg_set_quality(&cinfo, 80, TRUE);
    jpeg_start_compress(&cinfo, TRUE);
    std::vector<uint8_t> row(s.width * 3);
    while (cinfo.next_scanline < cinfo.image_height) {
        const uint8_t* src = s.row(cinfo.next_scanline);
        for (uint32_t x = 0; x < s.width; ++x) {
            const float a = src[x * 4 + 3] / 255.0f;
            row[x * 3 + 0] = static_cast<uint8_t>(src[x * 4 + 0] * a + 11 * (1 - a));
            row[x * 3 + 1] = static_cast<uint8_t>(src[x * 4 + 1] * a + 13 * (1 - a));
            row[x * 3 + 2] = static_cast<uint8_t>(src[x * 4 + 2] * a + 16 * (1 - a));
        }
        JSAMPROW rows[1] = {row.data()};
        jpeg_write_scanlines(&cinfo, rows, 1);
    }
    jpeg_finish_compress(&cinfo);
    out.assign(mem, mem + mem_size);
    jpeg_destroy_compress(&cinfo);
    free(mem);
}

#ifdef FS_HAVE_ROS
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

// Latest compressed frame per input; the render loop decodes at its own pace.
struct RosFeed {
    std::string input;
    std::mutex mutex;
    std::vector<uint8_t> jpeg;
    uint64_t seq = 0;
    // decoded pixels live here so the borrowed ImageView stays valid
    std::vector<uint8_t> rgba;
    uint32_t w = 0, h = 0;
    uint64_t decoded_seq = 0;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub;
};
#endif

// ---- scene loading ----------------------------------------------------------

std::string escapeJson(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '"' || c == '\\') {
            out += '\\';
            out += c;
        } else if (c == '\n') {
            out += "\\n";
        } else {
            out += c;
        }
    }
    return out;
}

struct LoadOutcome {
    std::unique_ptr<fsc::CompiledScene> scene;  ///< Null when rejected.
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
};

LoadOutcome loadScene(const std::string& path) {
    LoadOutcome out;
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        out.errors.push_back("cannot read " + path);
        return out;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    fsc::ParseResult parsed = fsc::parseScene(ss.str());
    for (const auto& d : parsed.diagnostics.items()) {
        const std::string line = "line " + std::to_string(d.span.begin_line) + ": [" +
                                 d.code + "] " + d.message;
        (d.severity == fsc::Severity::kError ? out.errors : out.warnings).push_back(line);
    }
    if (!parsed.ok()) {
        return out;
    }
    fsc::CompileResult compiled = fsc::compile(parsed.doc);
    for (const auto& d : compiled.diagnostics.items()) {
        out.errors.push_back("[" + d.code + "] " + d.message);
    }
    if (!compiled.ok()) {
        return out;
    }
    // Design lints on the candidate, before it goes live (§13-2).
    CpuRenderer lint_renderer;
    for (const auto& d : fsc::lint(*compiled.scene, lint_renderer).items()) {
        (d.severity == fsc::Severity::kError ? out.errors : out.warnings)
            .push_back("[" + d.code + "] " + d.message);
    }
    if (!compiled.ok() || !out.errors.empty()) {
        return out;
    }
    out.scene = std::move(compiled.scene);
    return out;
}

void publishStatus(const std::string& digest, uint64_t reloads,
                   const std::vector<std::string>& errors,
                   const std::vector<std::string>& warnings) {
    std::ostringstream json;
    json << "{\"digest\": \"" << digest << "\", \"reloads\": " << reloads
         << ", \"errors\": [";
    for (size_t i = 0; i < errors.size(); ++i) {
        json << (i ? ", " : "") << "\"" << escapeJson(errors[i]) << "\"";
    }
    json << "], \"warnings\": [";
    for (size_t i = 0; i < warnings.size(); ++i) {
        json << (i ? ", " : "") << "\"" << escapeJson(warnings[i]) << "\"";
    }
    json << "]}";
    std::lock_guard<std::mutex> lock(g_status_mutex);
    g_status_json = json.str();
}

/// A red banner on the live stage naming the failure. It is an ordinary
/// layer group added from C++ (the author privilege, §1.3) and removed on
/// the next good load.
void showErrorBanner(Stage& stage, const std::string& message) {
    if (Layer* old = stage.find("__scene_web_banner")) {
        old->remove();
    }
    auto& banner = stage.group("__scene_web_banner");
    const float w = stage.width();
    banner.rect({0, 0, w, 56}).color({0.75f, 0.11f, 0.11f, 0.92f});
    banner.text("scene error — showing the last good picture", {16, 6}).size(22);
    banner.text(message, {16, 34}).size(15).color({1, 1, 1, 0.85f});
}

void clearErrorBanner(Stage& stage) {
    if (Layer* banner = stage.find("__scene_web_banner")) {
        banner->remove();
    }
}

time_t fileMtime(const std::string& path) {
    struct stat st {};
    return ::stat(path.c_str(), &st) == 0 ? st.st_mtime : 0;
}

void onSignal(int) { g_running = false; }

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr,
                     "usage: scene_web <scene.fvs> [--port 8791] [--fps 30] "
                     "[--image input=topic]...\n");
        return 2;
    }
    const std::string path = argv[1];
    uint16_t port = 8791;
    float fps = 30;
    std::vector<std::pair<std::string, std::string>> image_feeds;  // input → topic
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            port = static_cast<uint16_t>(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--fps") == 0 && i + 1 < argc) {
            fps = std::max(1.0f, static_cast<float>(std::atof(argv[++i])));
        } else if (std::strcmp(argv[i], "--image") == 0 && i + 1 < argc) {
            const std::string spec = argv[++i];
            const size_t eq = spec.find('=');
            if (eq == std::string::npos) {
                std::fprintf(stderr, "scene_web: --image wants input=topic\n");
                return 2;
            }
            image_feeds.emplace_back(spec.substr(0, eq), spec.substr(eq + 1));
        } else {
            std::fprintf(stderr, "scene_web: unknown argument %s\n", argv[i]);
            return 2;
        }
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // First load must produce something to serve; a broken file still starts
    // the server with an empty stage and the error on screen.
    uint64_t reloads = 0;
    LoadOutcome current = loadScene(path);
    std::unique_ptr<fsc::CompiledScene> live = std::move(current.scene);
    std::unique_ptr<Stage> empty_stage;  // stand-in until the first good load
    if (live == nullptr) {
        empty_stage = std::make_unique<Stage>(1280, 720);
        showErrorBanner(*empty_stage, current.errors.empty() ? "?" : current.errors.front());
    }
    publishStatus(live ? live->digest() : "-", reloads, current.errors, current.warnings);
    for (const auto& e : current.errors) {
        std::fprintf(stderr, "scene_web: %s\n", e.c_str());
    }

#ifdef FS_HAVE_ROS
    rclcpp::InitOptions init;
    // scene_web owns shutdown; rclcpp's handler would race our loop.
    init.shutdown_on_signal = false;
    rclcpp::init(argc, argv, init, rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("scene_web");
    std::vector<std::unique_ptr<RosFeed>> feeds;
    for (const auto& [input, topic] : image_feeds) {
        auto feed = std::make_unique<RosFeed>();
        feed->input = input;
        RosFeed* raw = feed.get();
        feed->sub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
            topic, rclcpp::SensorDataQoS(),
            [raw](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(raw->mutex);
                raw->jpeg = msg->data;
                ++raw->seq;
            });
        std::printf("scene_web: $inputs.%s ← %s\n", input.c_str(), topic.c_str());
        feeds.push_back(std::move(feed));
    }
    std::thread spin([&] {
        while (g_running) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });
#else
    if (!image_feeds.empty()) {
        std::fprintf(stderr,
                     "scene_web: built without ROS — --image feeds are ignored "
                     "(configure with the ROS environment sourced)\n");
    }
#endif

    std::thread server(serverLoop, port);

    CpuRenderer cpu;
    Renderer* renderer = &cpu;
#ifdef FS_HAVE_VULKAN
    std::unique_ptr<VulkanRenderer> gpu;
    try {
        gpu = std::make_unique<VulkanRenderer>();
        renderer = gpu.get();
        std::printf("scene_web: vulkan\n");
    } catch (const std::exception& e) {
        std::printf("scene_web: cpu renderer (%s)\n", e.what());
    }
#endif
    std::fflush(stdout);

    time_t last_mtime = fileMtime(path);
    int inspect_tick = 0;
    const auto frame_time = std::chrono::duration<double>(1.0 / fps);
    auto next = std::chrono::steady_clock::now();
    auto last_frame = next;
    while (g_running) {
        // ---- hot reload: validate the candidate fully, then swap ----------
        const time_t mtime = fileMtime(path);
        if (mtime != last_mtime) {
            last_mtime = mtime;
            LoadOutcome candidate = loadScene(path);
            ++reloads;
            if (candidate.scene != nullptr) {
                live = std::move(candidate.scene);  // the frame-boundary swap
                empty_stage.reset();
                clearErrorBanner(live->stage());
                std::printf("scene_web: reload #%llu ok — digest %.12s\n",
                            static_cast<unsigned long long>(reloads),
                            live->digest().c_str());
            } else {
                Stage& s = live ? live->stage() : *empty_stage;
                showErrorBanner(
                    s, candidate.errors.empty() ? "?" : candidate.errors.front());
                std::printf("scene_web: reload #%llu rejected (%zu error(s)) — "
                            "keeping the last good scene\n",
                            static_cast<unsigned long long>(reloads),
                            candidate.errors.size());
            }
            publishStatus(live ? live->digest() : "-", reloads, candidate.errors,
                          candidate.warnings);
            std::fflush(stdout);
        }

        // ---- feed inputs (borrowed views: set every frame) ----------------
#ifdef FS_HAVE_ROS
        if (live) {
            for (auto& feed : feeds) {
                {
                    std::lock_guard<std::mutex> lock(feed->mutex);
                    if (feed->seq != feed->decoded_seq && !feed->jpeg.empty()) {
                        uint32_t w = 0, h = 0;
                        if (decodeJpegToRgba(feed->jpeg.data(), feed->jpeg.size(),
                                             feed->rgba, w, h)) {
                            feed->w = w;
                            feed->h = h;
                        }
                        feed->decoded_seq = feed->seq;
                    }
                }
                if (feed->w != 0) {
                    live->setImage(feed->input,
                                   {feed->w, feed->h, feed->rgba.data(), 0});
                }
            }
        }
#endif

        // ---- render, encode, publish --------------------------------------
        const auto now = std::chrono::steady_clock::now();
        const float dt = std::chrono::duration<float>(now - last_frame).count();
        last_frame = now;
        Stage& stage = live ? live->stage() : *empty_stage;
        const Surface& frame = renderer->render(stage, dt);
        {
            std::lock_guard<std::mutex> lock(g_frame_mutex);
            encodeJpeg(frame, g_jpeg);
            ++g_frame_seq;
        }
        g_frame_cv.notify_all();

        // ---- inspector snapshot (§13-3), refreshed ~2 Hz ------------------
        if (live && ++inspect_tick >= static_cast<int>(fps) / 2) {
            inspect_tick = 0;
            const auto placed = fsc::placeLayers(*live);
            std::vector<InspectEntry> entries;
            entries.reserve(placed.size());
            for (const auto& p : placed) {
                entries.push_back({fsc::placedLayerJson(p), p.to_stage, p.bounds,
                                   p.eff_opacity, p.paint_index});
            }
            std::lock_guard<std::mutex> lock(g_inspect_mutex);
            g_inspect_json = fsc::inspectJson(*live, placed);
            g_inspect_entries = std::move(entries);
        }

        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_time);
        std::this_thread::sleep_until(next);
    }

    g_frame_cv.notify_all();
    server.detach();
#ifdef FS_HAVE_ROS
    spin.join();
    rclcpp::shutdown();
#endif
    std::printf("scene_web: bye\n");
    return 0;
}
