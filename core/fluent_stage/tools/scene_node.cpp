// scene_node — the production ROS 2 adapter (Phase L3): a Scene document
// plus a binding document become a running HUD node.
//
//   scene_node scene.fvs binding.fvb [--fps 30]
//
// The scene declares what it needs ($inputs, types, fallbacks); the binding
// declares where that lives on this robot (topics, message types, QoS,
// converters) and where the rendered Surface goes. Both are validated
// against each other **before** anything subscribes — adapter validation
// fails before activation, never after (architecture doc §12).
//
// Inputs snapshot latest-wins at each frame; conversion happens on the
// render thread so callbacks stay cheap. The scene file hot-reloads like
// scene_web (validate → compile → re-check the binding → atomic swap); the
// binding document itself is fixed for the node's lifetime — changing
// deployment wiring is a restart, by design.

#include <sys/stat.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <jpeglib.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#ifdef FS_HAVE_VISION_MSGS
#include <vision_msgs/msg/detection2_d_array.hpp>
#endif
#include <geometry_msgs/msg/polygon.hpp>

#include <fluent_stage/cpu_renderer.hpp>
#include <fluent_stage/fluent_stage.hpp>
#include <fluent_stage/scene/binding.hpp>
#include <fluent_stage/scene/compiler.hpp>
#include <fluent_stage/scene/document.hpp>
#include <fluent_stage/scene/linter.hpp>
#ifdef FS_HAVE_VULKAN
#include <fluent_stage/vulkan_renderer.hpp>
#endif

using namespace fluent_stage;
namespace fsc = fluent_stage::scene;

namespace {

std::atomic<bool> g_running{true};
void onSignal(int) { g_running = false; }

// ---- file + diagnostics helpers --------------------------------------------

bool readFile(const std::string& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    out = ss.str();
    return true;
}

void printDiags(const fsc::DiagnosticList& diags, const char* what) {
    for (const auto& d : diags.items()) {
        std::fprintf(stderr, "scene_node: %s %s: [%s] %s\n", what,
                     fsc::toString(d.severity), d.code.c_str(), d.message.c_str());
    }
}

// ---- image conversion -------------------------------------------------------

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

/// sensor_msgs Image (the common encodings) → tightly packed RGBA8.
bool imageToRgba(const sensor_msgs::msg::Image& msg, std::vector<uint8_t>& out) {
    const uint32_t w = msg.width, h = msg.height;
    const std::string& enc = msg.encoding;
    out.resize(static_cast<size_t>(w) * h * 4);
    const size_t src_step = msg.step;
    auto rows = [&](auto&& per_pixel, uint32_t bpp) {
        for (uint32_t y = 0; y < h; ++y) {
            const uint8_t* src = msg.data.data() + y * src_step;
            uint8_t* dst = out.data() + static_cast<size_t>(y) * w * 4;
            for (uint32_t x = 0; x < w; ++x) {
                per_pixel(src + x * bpp, dst + x * 4);
            }
        }
    };
    if (enc == "rgba8") {
        rows([](const uint8_t* s, uint8_t* d) { std::memcpy(d, s, 4); }, 4);
    } else if (enc == "bgra8") {
        rows([](const uint8_t* s, uint8_t* d) { d[0] = s[2]; d[1] = s[1]; d[2] = s[0]; d[3] = s[3]; }, 4);
    } else if (enc == "rgb8") {
        rows([](const uint8_t* s, uint8_t* d) { d[0] = s[0]; d[1] = s[1]; d[2] = s[2]; d[3] = 255; }, 3);
    } else if (enc == "bgr8") {
        rows([](const uint8_t* s, uint8_t* d) { d[0] = s[2]; d[1] = s[1]; d[2] = s[0]; d[3] = 255; }, 3);
    } else if (enc == "mono8") {
        rows([](const uint8_t* s, uint8_t* d) { d[0] = d[1] = d[2] = s[0]; d[3] = 255; }, 1);
    } else {
        return false;
    }
    return true;
}

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
            row[x * 3 + 0] = src[x * 4 + 0];
            row[x * 3 + 1] = src[x * 4 + 1];
            row[x * 3 + 2] = src[x * 4 + 2];
        }
        JSAMPROW rows[1] = {row.data()};
        jpeg_write_scanlines(&cinfo, rows, 1);
    }
    jpeg_finish_compress(&cinfo);
    out.assign(mem, mem + mem_size);
    jpeg_destroy_compress(&cinfo);
    free(mem);
}

// ---- feeds: latest-wins slots the callbacks fill ---------------------------

struct Feed {
    const fsc::BindingDecl* decl = nullptr;
    std::mutex mutex;
    uint64_t seq = 0;

    // Exactly one of these carries data, per the converter.
    sensor_msgs::msg::Image::ConstSharedPtr image;
    sensor_msgs::msg::CompressedImage::ConstSharedPtr compressed;
    std::vector<Box> boxes;
    std::string text;
    std::vector<Vec2> points;

    // Render-thread state.
    uint64_t used_seq = 0;
    std::vector<uint8_t> rgba;  ///< Owned pixels behind the borrowed ImageView.
    uint32_t w = 0, h = 0;
    bool warned = false;

    rclcpp::SubscriptionBase::SharedPtr sub;
};

rclcpp::QoS qosOf(const std::string& word) {
    if (word == "transient_local") {
        return rclcpp::QoS(1).transient_local();
    }
    if (word == "default") {
        return rclcpp::QoS(10);
    }
    return rclcpp::SensorDataQoS();
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: scene_node <scene.fvs> <binding.fvb> [--fps 30]\n");
        return 2;
    }
    const std::string scene_path = argv[1];
    const std::string binding_path = argv[2];
    float fps = 30;
    for (int i = 3; i < argc; ++i) {
        if (std::strcmp(argv[i], "--fps") == 0 && i + 1 < argc) {
            fps = std::max(1.0f, static_cast<float>(std::atof(argv[++i])));
        }
    }

    // ---- load and cross-validate everything before touching ROS -----------
    std::string scene_text, binding_text;
    if (!readFile(scene_path, scene_text) || !readFile(binding_path, binding_text)) {
        std::fprintf(stderr, "scene_node: cannot read %s / %s\n", scene_path.c_str(),
                     binding_path.c_str());
        return 2;
    }
    fsc::ParseResult parsed = fsc::parseScene(scene_text);
    printDiags(parsed.diagnostics, "scene");
    if (!parsed.ok()) {
        return 1;
    }
    fsc::BindingParseResult binding = fsc::parseBinding(binding_text);
    printDiags(binding.diagnostics, "binding");
    if (!binding.ok()) {
        return 1;
    }
    fsc::DiagnosticList cross;
    fsc::validateBindingAgainstScene(binding.doc, parsed.doc, cross);
    printDiags(cross, "binding");
    if (cross.hasErrors()) {
        return 1;
    }
    fsc::CompileResult compiled = fsc::compile(parsed.doc);
    printDiags(compiled.diagnostics, "compile");
    if (!compiled.ok()) {
        return 1;
    }
    std::unique_ptr<fsc::CompiledScene> live = std::move(compiled.scene);

#ifdef FS_HAVE_VISION_MSGS
    const bool have_vision_msgs = true;
#else
    const bool have_vision_msgs = false;
#endif
    for (const auto& b : binding.doc.bindings) {
        if (!have_vision_msgs &&
            std::string(b.converter->name) == "ros_detections_to_detection2d") {
            std::fprintf(stderr,
                         "scene_node: built without vision_msgs; cannot bind '%s'\n",
                         b.input.c_str());
            return 1;
        }
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    rclcpp::InitOptions init;
    rclcpp::init(argc, argv, init, rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("fluent_scene_node");

    // ---- subscriptions per binding ----------------------------------------
    std::vector<std::unique_ptr<Feed>> feeds;
    for (const auto& b : binding.doc.bindings) {
        auto feed = std::make_unique<Feed>();
        feed->decl = &b;
        Feed* raw = feed.get();
        const std::string conv = b.converter->name;
        const rclcpp::QoS qos = qosOf(b.source.qos);
        if (conv == "ros_image_to_rgba8") {
            feed->sub = node->create_subscription<sensor_msgs::msg::Image>(
                b.source.topic, qos, [raw](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                    std::lock_guard<std::mutex> lock(raw->mutex);
                    raw->image = std::move(msg);
                    ++raw->seq;
                });
        } else if (conv == "ros_compressed_to_rgba8") {
            feed->sub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
                b.source.topic, qos,
                [raw](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
                    std::lock_guard<std::mutex> lock(raw->mutex);
                    raw->compressed = std::move(msg);
                    ++raw->seq;
                });
#ifdef FS_HAVE_VISION_MSGS
        } else if (conv == "ros_detections_to_detection2d") {
            feed->sub = node->create_subscription<vision_msgs::msg::Detection2DArray>(
                b.source.topic, qos,
                [raw](vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {
                    std::vector<Box> boxes;
                    boxes.reserve(msg->detections.size());
                    for (const auto& det : msg->detections) {
                        Box box;
                        const auto& bb = det.bbox;
                        box.rect = {static_cast<float>(bb.center.position.x - bb.size_x / 2),
                                    static_cast<float>(bb.center.position.y - bb.size_y / 2),
                                    static_cast<float>(bb.size_x),
                                    static_cast<float>(bb.size_y)};
                        for (const auto& hyp : det.results) {
                            if (hyp.hypothesis.score >= box.score) {
                                box.score = static_cast<float>(hyp.hypothesis.score);
                                box.label = hyp.hypothesis.class_id;
                            }
                        }
                        // Numeric track ids ride along; anything else falls
                        // back to nearest-neighbor smoothing (id 0).
                        box.id = static_cast<uint32_t>(std::strtoul(det.id.c_str(), nullptr, 10));
                        boxes.push_back(std::move(box));
                    }
                    std::lock_guard<std::mutex> lock(raw->mutex);
                    raw->boxes = std::move(boxes);
                    ++raw->seq;
                });
#endif
        } else if (conv == "ros_string_to_utf8") {
            feed->sub = node->create_subscription<std_msgs::msg::String>(
                b.source.topic, qos, [raw](std_msgs::msg::String::ConstSharedPtr msg) {
                    std::lock_guard<std::mutex> lock(raw->mutex);
                    raw->text = msg->data;
                    ++raw->seq;
                });
        } else if (conv == "ros_polygon_to_vec2") {
            feed->sub = node->create_subscription<geometry_msgs::msg::Polygon>(
                b.source.topic, qos, [raw](geometry_msgs::msg::Polygon::ConstSharedPtr msg) {
                    std::vector<Vec2> pts;
                    pts.reserve(msg->points.size());
                    for (const auto& p : msg->points) {
                        pts.push_back({p.x, p.y});
                    }
                    std::lock_guard<std::mutex> lock(raw->mutex);
                    raw->points = std::move(pts);
                    ++raw->seq;
                });
        }
        std::printf("scene_node: $inputs.%s ← %s (%s)\n", b.input.c_str(),
                    b.source.topic.c_str(), b.converter->name);
        feeds.push_back(std::move(feed));
    }

    // ---- output publishers -------------------------------------------------
    struct OutputPub {
        const fsc::OutputDecl* decl;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed;
    };
    std::vector<OutputPub> outputs;
    for (const auto& o : binding.doc.outputs) {
        OutputPub pub;
        pub.decl = &o;
        if (std::string(o.converter->name) == "rgba8_to_ros_image") {
            pub.image = node->create_publisher<sensor_msgs::msg::Image>(o.sink.topic,
                                                                        qosOf(o.sink.qos));
        } else {
            pub.compressed = node->create_publisher<sensor_msgs::msg::CompressedImage>(
                o.sink.topic, qosOf(o.sink.qos));
        }
        std::printf("scene_node: %s → %s (%s)\n", o.name.c_str(), o.sink.topic.c_str(),
                    o.converter->name);
        outputs.push_back(std::move(pub));
    }
    std::fflush(stdout);

    std::thread spin([&] {
        while (g_running) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });

    CpuRenderer cpu;
    Renderer* renderer = &cpu;
#ifdef FS_HAVE_VULKAN
    std::unique_ptr<VulkanRenderer> gpu;
    try {
        gpu = std::make_unique<VulkanRenderer>();
        renderer = gpu.get();
    } catch (const std::exception&) {
    }
#endif

    // ---- render loop --------------------------------------------------------
    auto mtimeOf = [](const std::string& p) {
        struct stat st {};
        return ::stat(p.c_str(), &st) == 0 ? st.st_mtime : 0;
    };
    time_t last_mtime = mtimeOf(scene_path);
    const auto frame_time = std::chrono::duration<double>(1.0 / fps);
    auto next = std::chrono::steady_clock::now();
    auto last_frame = next;
    while (g_running && rclcpp::ok()) {
        // Hot reload of the scene document (the binding is fixed).
        const time_t mtime = mtimeOf(scene_path);
        if (mtime != last_mtime) {
            last_mtime = mtime;
            std::string text;
            fsc::ParseResult reparsed;
            if (readFile(scene_path, text)) {
                reparsed = fsc::parseScene(text);
            }
            fsc::DiagnosticList recheck;
            if (reparsed.ok()) {
                fsc::validateBindingAgainstScene(binding.doc, reparsed.doc, recheck);
            }
            fsc::CompileResult recompiled;
            if (reparsed.ok() && !recheck.hasErrors()) {
                recompiled = fsc::compile(reparsed.doc);
            }
            if (recompiled.ok()) {
                live = std::move(recompiled.scene);
                std::printf("scene_node: reload ok — digest %.12s\n",
                            live->digest().c_str());
            } else {
                printDiags(reparsed.diagnostics, "reload");
                printDiags(recheck, "reload");
                printDiags(recompiled.diagnostics, "reload");
                std::printf("scene_node: reload rejected — keeping the last good scene\n");
            }
            std::fflush(stdout);
        }

        // Convert + feed the latest data.
        for (auto& feed : feeds) {
            sensor_msgs::msg::Image::ConstSharedPtr image;
            sensor_msgs::msg::CompressedImage::ConstSharedPtr compressed;
            std::vector<Box> boxes;
            std::string text;
            std::vector<Vec2> points;
            bool fresh = false;
            {
                std::lock_guard<std::mutex> lock(feed->mutex);
                if (feed->seq != feed->used_seq) {
                    fresh = true;
                    feed->used_seq = feed->seq;
                    image = feed->image;
                    compressed = feed->compressed;
                    boxes = feed->boxes;
                    text = feed->text;
                    points = feed->points;
                }
            }
            const std::string conv = feed->decl->converter->name;
            if (fresh) {
                if (image != nullptr) {
                    if (imageToRgba(*image, feed->rgba)) {
                        feed->w = image->width;
                        feed->h = image->height;
                    } else if (!feed->warned) {
                        feed->warned = true;
                        std::fprintf(stderr, "scene_node: unsupported encoding '%s' on %s\n",
                                     image->encoding.c_str(),
                                     feed->decl->source.topic.c_str());
                    }
                } else if (compressed != nullptr) {
                    uint32_t w = 0, h = 0;
                    if (decodeJpegToRgba(compressed->data.data(), compressed->data.size(),
                                         feed->rgba, w, h)) {
                        feed->w = w;
                        feed->h = h;
                    }
                } else if (conv == "ros_detections_to_detection2d") {
                    live->setBoxes(feed->decl->input, boxes);
                } else if (conv == "ros_string_to_utf8") {
                    live->setText(feed->decl->input, text);
                } else if (conv == "ros_polygon_to_vec2") {
                    live->setPoints(feed->decl->input, points);
                }
            }
            // Borrowed-view contract: image layers get their view every frame.
            if (feed->w != 0) {
                live->setImage(feed->decl->input, {feed->w, feed->h, feed->rgba.data(), 0});
            }
        }

        const auto now = std::chrono::steady_clock::now();
        const float dt = std::chrono::duration<float>(now - last_frame).count();
        last_frame = now;
        const Surface& frame = renderer->render(live->stage(), dt);

        for (auto& out : outputs) {
            const auto stamp = node->now();
            if (out.image != nullptr && out.image->get_subscription_count() > 0) {
                sensor_msgs::msg::Image msg;
                msg.header.stamp = stamp;
                msg.header.frame_id = "fluent_scene";
                msg.width = frame.width;
                msg.height = frame.height;
                msg.encoding = "rgba8";
                msg.step = frame.width * 4;
                msg.data.resize(static_cast<size_t>(frame.height) * msg.step);
                for (uint32_t y = 0; y < frame.height; ++y) {
                    std::memcpy(&msg.data[static_cast<size_t>(y) * msg.step], frame.row(y),
                                msg.step);
                }
                out.image->publish(std::move(msg));
            } else if (out.compressed != nullptr &&
                       out.compressed->get_subscription_count() > 0) {
                sensor_msgs::msg::CompressedImage msg;
                msg.header.stamp = stamp;
                msg.header.frame_id = "fluent_scene";
                msg.format = "jpeg";
                encodeJpeg(frame, msg.data);
                out.compressed->publish(std::move(msg));
            }
        }

        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_time);
        std::this_thread::sleep_until(next);
    }

    spin.join();
    rclcpp::shutdown();
    std::printf("scene_node: bye\n");
    return 0;
}
