// stage_web — the Stage in a browser: MJPEG stream out, pointer events in.
//
//   ./stage_web [port]      (default 8790)  →  open http://<robot>:8790/
//
// This is the §10-3 loop end to end: the browser is a thin viewer — it
// shows the rendered Surface and normalizes mouse/touch into three pointer
// calls. Every control (buttons, switches, slider, segmented, dropdown),
// the ripple wake, and all rendering run on the robot; the page holds zero
// UI logic. Swap the synthetic camera for a real one and this is the HUD.
//
// Plumbing: one render thread owns the Stage (single-thread rule) and
// drains a queue of pointer events each frame; HTTP connections run on
// their own threads. GET / serves the embedded page, GET /stream serves
// multipart MJPEG, POST /pointer enqueues "<phase> <x> <y>".

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstring>
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

using namespace fluent_stage;

namespace {

constexpr float kW = 1280;
constexpr float kH = 720;
constexpr float kFps = 30;

// ---- shared state ----------------------------------------------------------

struct PointerEventIn {
    char phase;  // 'd'own 'm'ove 'u'p 'c'ancel 'h'over
    float x, y;
};

std::mutex g_events_mutex;
std::vector<PointerEventIn> g_events;

std::mutex g_frame_mutex;
std::condition_variable g_frame_cv;
std::vector<uint8_t> g_jpeg;
uint64_t g_frame_seq = 0;
std::atomic<bool> g_running{true};

// ---- jpeg ------------------------------------------------------------------

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
    jpeg_set_quality(&cinfo, 75, TRUE);  // ~40% smaller than q85; Wi-Fi friendly
    jpeg_start_compress(&cinfo, TRUE);
    std::vector<uint8_t> row(s.width * 3);
    while (cinfo.next_scanline < cinfo.image_height) {
        const uint8_t* src = s.row(cinfo.next_scanline);
        for (uint32_t x = 0; x < s.width; ++x) {
            // Straight alpha over the page's dark background.
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

// ---- the page --------------------------------------------------------------

const char* kPage = R"HTML(<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>fluent_stage</title>
<style>
  body { margin:0; background:#101216; color:#dde; font:14px system-ui;
         display:flex; flex-direction:column; align-items:center; }
  h1 { font-size:15px; font-weight:600; margin:10px 0 6px; color:#9ab; }
  #v { max-width:96vw; max-height:88vh; border-radius:8px;
       touch-action:none; user-select:none; -webkit-user-select:none; }
</style>
<h1>fluent_stage — live</h1>
<img id="v" src="/stream" draggable="false">
<script>
const img = document.getElementById('v');
const W = 1280, H = 720;
function post(phase, e) {
  const r = img.getBoundingClientRect();
  const x = (e.clientX - r.left) / r.width * W;
  const y = (e.clientY - r.top) / r.height * H;
  fetch('/pointer', {method:'POST', body: phase + ' ' + x.toFixed(1) + ' ' + y.toFixed(1)});
}
let down = false, pendingMove = null, lastSend = 0;
function flushMove(ts) {
  // ~30 Hz is plenty for hover wake and drag; more just queues latency.
  if (pendingMove && ts - lastSend >= 33) {
    post(down ? 'm' : 'h', pendingMove); pendingMove = null; lastSend = ts;
  }
  requestAnimationFrame(flushMove);
}
requestAnimationFrame(flushMove);
img.addEventListener('pointerdown', e => { down = true; img.setPointerCapture(e.pointerId); post('d', e); e.preventDefault(); });
img.addEventListener('pointermove', e => { pendingMove = e; });
img.addEventListener('pointerup',   e => { down = false; post('u', e); });
img.addEventListener('pointercancel', e => { down = false; post('c', e); });
</script>
)HTML";

// ---- http ------------------------------------------------------------------

bool sendAll(int fd, const void* data, size_t len) {
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
            return;
        }
    }
}

void handleConnection(int fd) {
    // Latency rule: pointer connections are KEPT ALIVE — one TCP connection
    // carries the whole gesture stream. A connection per event stacks
    // handshakes against the browser's 6-connection limit (which the MJPEG
    // stream already occupies) and turns hover into visible input lag.
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
        if (buf.rfind("GET /stream", 0) == 0) {
            streamMjpeg(fd);
            ::close(fd);
            return;
        }
        if (buf.rfind("POST /pointer", 0) == 0) {
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
            PointerEventIn e{};
            if (std::sscanf(buf.c_str() + body_at, " %c %f %f", &e.phase, &e.x, &e.y) == 3) {
                std::lock_guard<std::mutex> lock(g_events_mutex);
                if (g_events.size() < 256) {
                    g_events.push_back(e);
                }
            }
            const char* resp =
                "HTTP/1.1 204 No Content\r\nConnection: keep-alive\r\n\r\n";
            if (!sendAll(fd, resp, std::strlen(resp))) {
                ::close(fd);
                return;
            }
            buf.erase(0, body_at + want);  // next request on the same socket
            continue;
        }
        sendResponse(fd, "200 OK", "text/html; charset=utf-8", kPage);
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
    while (g_running) {
        const int fd = ::accept(listener, nullptr, nullptr);
        if (fd < 0) {
            continue;
        }
        std::thread(handleConnection, fd).detach();
    }
    ::close(listener);
}

// ---- the demo HUD ----------------------------------------------------------

std::vector<uint8_t> makeCamera(uint32_t t) {
    // A synthetic asparagus field: sky gradient over ridged crop rows in
    // perspective — enough structure that the water refraction visibly
    // bends it. Swap for a real camera frame and nothing else changes.
    const uint32_t w = 640, h = 360;
    std::vector<uint8_t> px(w * h * 4);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            uint8_t* p = &px[(y * w + x) * 4];
            const float fy = static_cast<float>(y) / h;
            float r, g, b;
            if (fy < 0.32f) {  // sky band
                r = 96 + 40 * fy;
                g = 128 + 60 * fy;
                b = 120 + 30 * fy;
            } else {           // field: perspective crop rows + furrows
                const float depth = (fy - 0.32f) / 0.68f;             // 0 far → 1 near
                const float row_w = 14 + 70 * depth;                  // rows widen nearby
                const float cx = x - w * 0.5f;
                const float u = cx / (0.35f + depth) + t * 0.6f;      // slow drive-by
                const float row = std::fabs(std::fmod(u, row_w) / row_w - 0.5f) * 2;
                const float ridge = row < 0.42f ? 1.0f : 0.55f;       // plant vs furrow
                const float shade = 0.55f + 0.45f * depth;
                r = (34 + 26 * row) * ridge * shade + 12;
                g = (96 + 40 * row) * ridge * shade + 20;
                b = (52 + 22 * row) * ridge * shade + 14;
            }
            p[0] = static_cast<uint8_t>(std::min(r, 255.0f));
            p[1] = static_cast<uint8_t>(std::min(g, 255.0f));
            p[2] = static_cast<uint8_t>(std::min(b, 255.0f));
            p[3] = 255;
        }
    }
    return px;
}

}  // namespace

int main(int argc, char** argv) {
    const uint16_t port = argc > 1 ? static_cast<uint16_t>(std::atoi(argv[1])) : 8790;
    std::signal(SIGPIPE, SIG_IGN);

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

    // ---- scene ----
    Stage stage(kW, kH);
    auto cam = makeCamera(0);
    auto& video = stage.image({640, 360, cam.data(), 0}, Fit::Cover);
    auto& boxes = stage.boxes({}).color(Color::Teal).cornerRadius(8).smoothing(0.25f);

    auto& panel = stage.group("panel").position(24, 24);
    panel.rect({0, 0, 360, 118}).cornerRadius(14).color({0, 0, 0, 0.45f});
    auto& status = panel.text("待機中", {20, 14}).size(30);
    auto& speed_label = panel.text("SPEED 0.00 m/s", {20, 66}).size(22).color(Color::Teal);

    ui::Button start(stage.root(), {24, 170, 170, 56}, "収穫開始");
    ui::Button stop(stage.root(), {210, 170, 120, 56}, "停止");
    stop.enabled(false);
    ui::Switch light(stage.root(), {24, 250, 96, 48});
    stage.text("ライト", {134, 260}).size(22);
    auto& light_dot = stage.circle({260, 274}, 10).color({1, 1, 0.4f, 0.15f});
    ui::Slider speed(stage.root(), {24, 330, 300, 30}, 0.35f);
    ui::Segmented mode(stage.root(), {24, 390, 306, 44}, {"手動", "巡回", "追従"}, 1);
    ui::Dropdown profile(stage.root(), {24, 460, 220, 46}, {"標準", "低速", "高速", "点検"}, 0);
    ui::Gauge battery(stage.root(), {1180, 90}, 52);
    stage.text("BATT", {1180, 160}).size(16).align(Align::Center).color({1, 1, 1, 0.6f});
    fx::RippleStyle ripple_style;
    ripple_style.amplitude = 12;
    ripple_style.wavelength = 34;
    ripple_style.max_radius = 340;
    ripple_style.duration = 1.4f;
    // The whole screen is the water surface: a filter on the root applies
    // to the composited frame, so panels, buttons, and popups warp too.
    fx::Ripple ripple(stage.root(), ripple_style);

    bool harvesting = false;
    start.onTap([&] {
        harvesting = true;
        Transaction t(0.25f, Ease::InOut);
        status.setText("収穫中");
        status.color(Color::Teal);
        stop.enabled(true);
        start.enabled(false);
    });
    stop.onTap([&] {
        harvesting = false;
        status.setText("待機中");
        status.color(Color::White);
        start.enabled(true);
        stop.enabled(false);
    });
    light.onChange([&](bool on) {
        Transaction t(0.2f, Ease::Out);
        light_dot.color(on ? Color{1, 1, 0.4f, 1} : Color{1, 1, 0.4f, 0.15f});
    });
    speed.onChange([&](float v) {
        char text[32];
        std::snprintf(text, sizeof text, "SPEED %.2f m/s", v * 1.5f);
        speed_label.setText(text);
    });
    profile.onChange([&](int i) { std::printf("profile -> %d\n", i); });
    mode.onChange([&](int i) { std::printf("mode -> %d\n", i); });

    std::thread server(serverLoop, port);

    // ---- render loop (owns the Stage) ----
    uint32_t tick = 0;
    const auto frame_time = std::chrono::duration<double>(1.0 / kFps);
    auto next = std::chrono::steady_clock::now();
    while (g_running) {
        // Injected input, normalized by the page to logical coordinates.
        std::vector<PointerEventIn> events;
        {
            std::lock_guard<std::mutex> lock(g_events_mutex);
            events.swap(g_events);
        }
        for (const PointerEventIn& e : events) {
            switch (e.phase) {
                case 'd':
                    ripple.splash({e.x, e.y});
                    stage.pointerDown({e.x, e.y});
                    break;
                case 'm':
                    ripple.pointerMoved({e.x, e.y});
                    stage.pointerMove({e.x, e.y});
                    break;
                case 'h':
                    ripple.pointerMoved({e.x, e.y});
                    break;
                case 'u':
                    stage.pointerUp({e.x, e.y});
                    break;
                case 'c':
                    stage.pointerCancel();
                    break;
            }
        }

        // Live data: video, wandering detections, battery drain.
        ++tick;
        cam = makeCamera(tick);
        video.setImage({640, 360, cam.data(), 0});
        if (harvesting) {
            const float wob = std::sin(tick * 0.05f) * 40;
            boxes.setBoxes({{{520 + wob, 260, 200, 300}, 0.91f, "asparagus", 1},
                            {{860 - wob, 330, 170, 250}, 0.78f, "asparagus", 2}});
        } else {
            boxes.setBoxes({});
        }
        battery.setValue(0.9f - 0.1f * std::sin(tick * 0.004f));

        const float dt = static_cast<float>(frame_time.count());
        ripple.tick(dt);
        const Surface& frame = renderer->render(stage, dt);
        {
            std::lock_guard<std::mutex> lock(g_frame_mutex);
            encodeJpeg(frame, g_jpeg);
            ++g_frame_seq;
        }
        g_frame_cv.notify_all();

        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_time);
        std::this_thread::sleep_until(next);
    }
    server.join();
    return 0;
}
