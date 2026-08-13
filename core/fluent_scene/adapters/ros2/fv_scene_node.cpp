// fv_scene_node — ROS 2 adapter for the Fluent Scene runtime (stage 4).
//
// The adapter owns transport only (spec decision 2): it subscribes to the
// topics named by a separate binding document (fluent.binding/v1alpha1),
// converts messages into typed values, and pushes them into the stage-3
// binding table. A timer acquires a frame snapshot, renders it on the
// retained Vulkan backend, and publishes the composite image on the
// configured sink topic. The core never sees ROS types.
//
// Usage:
//   fv_scene_node --scene <file.fvs> --binding <file.yaml> [--rate HZ] [--cpu]
//
// Notes:
// - std_msgs/String has no header; those values are stamped at receipt time.
//   Every other converter uses header.stamp as the source event time.
// - The depth converter is validated but not yet implemented; depth values
//   are not pushed, so the scene's declared depth fallback stays in effect.

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "fluent_scene/binding.hpp"
#include "fluent_scene/binding_config.hpp"
#include "fluent_scene/diagnostics.hpp"
#include "fluent_scene/planner.hpp"
#include "fluent_scene/registry.hpp"
#include "fluent_scene/render/renderer.hpp"
#include "fluent_scene/validator.hpp"
#include "fluent_scene/yaml.hpp"

namespace {

std::string readFileOrEmpty(const std::string& path) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        return {};
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    return buffer.str();
}

void logDiagnostics(const rclcpp::Logger& logger, const fluent_scene::DiagnosticList& diagnostics) {
    for (const auto& diagnostic : diagnostics.items()) {
        const std::string line = std::string(toString(diagnostic.severity)) + ": " + diagnostic.code +
                                 ": " + diagnostic.message;
        if (diagnostic.severity == fluent_scene::Severity::kError) {
            RCLCPP_ERROR(logger, "%s", line.c_str());
        } else {
            RCLCPP_INFO(logger, "%s", line.c_str());
        }
    }
}

rclcpp::QoS qosFromName(const std::string& name) {
    if (name == "sensor_data") {
        return rclcpp::SensorDataQoS();
    }
    if (name == "transient_local") {
        return rclcpp::QoS(1).transient_local();
    }
    return rclcpp::QoS(10);
}

// Converts a sensor_msgs image into tightly packed RGBA8.
bool imageToRgba8(const sensor_msgs::msg::Image& message, fluent_scene::TypedValue& value) {
    const uint32_t width = message.width;
    const uint32_t height = message.height;
    if (width == 0 || height == 0) {
        return false;
    }
    value.kind = fluent_scene::TypedValue::Kind::kImage;
    value.width = width;
    value.height = height;
    value.pixels.resize(static_cast<size_t>(width) * height * 4);
    const std::string& encoding = message.encoding;
    int r = 0, g = 1, b = 2;
    size_t channels = 3;
    if (encoding == "rgb8") {
    } else if (encoding == "bgr8") {
        r = 2;
        b = 0;
    } else if (encoding == "rgba8") {
        channels = 4;
    } else if (encoding == "bgra8") {
        channels = 4;
        r = 2;
        b = 0;
    } else if (encoding == "mono8") {
        channels = 1;
    } else {
        return false;
    }
    for (uint32_t y = 0; y < height; ++y) {
        const uint8_t* src = message.data.data() + static_cast<size_t>(y) * message.step;
        uint8_t* dst = value.pixels.data() + static_cast<size_t>(y) * width * 4;
        for (uint32_t x = 0; x < width; ++x) {
            if (channels == 1) {
                dst[0] = dst[1] = dst[2] = src[x];
            } else {
                const uint8_t* pixel = src + static_cast<size_t>(x) * channels;
                dst[0] = pixel[r];
                dst[1] = pixel[g];
                dst[2] = pixel[b];
            }
            dst[3] = 0xff;
            dst += 4;
        }
    }
    return true;
}

class FvSceneNode : public rclcpp::Node {
public:
    FvSceneNode(const std::string& scene_path, const std::string& binding_path, double rate,
                bool use_cpu)
        : rclcpp::Node("fv_scene_node") {
        fluent_scene::DiagnosticList diagnostics;

        // Compile the scene: parse -> validate -> plan.
        const fluent_scene::YamlNode scene_root =
            fluent_scene::parseYaml(readFileOrEmpty(scene_path), diagnostics);
        if (!diagnostics.hasErrors()) {
            const fluent_scene::NodeRegistry registry = fluent_scene::NodeRegistry::builtinMvp();
            scene_ = fluent_scene::validateScene(scene_root, registry, diagnostics);
            if (scene_.ok) {
                plan_ = fluent_scene::planScene(scene_, diagnostics);
            }
        }
        if (!plan_.ok) {
            logDiagnostics(get_logger(), diagnostics);
            throw std::runtime_error("scene does not compile: " + scene_path);
        }

        // Adapter validation of the binding document (fails before activation).
        const fluent_scene::YamlNode binding_root =
            fluent_scene::parseYaml(readFileOrEmpty(binding_path), diagnostics);
        binding_ = fluent_scene::parseBindingDocument(binding_root, scene_, diagnostics);
        logDiagnostics(get_logger(), diagnostics);
        if (binding_ == nullptr) {
            throw std::runtime_error("binding document does not validate: " + binding_path);
        }

        // Retained renderer + runtime binding table.
        fluent_scene::DiagnosticList runtime_diagnostics;
        fluent_scene::RendererOptions renderer_options;
        renderer_ = use_cpu ? fluent_scene::createCpuRenderer(renderer_options, runtime_diagnostics)
                            : fluent_scene::createVulkanRenderer(renderer_options, runtime_diagnostics);
        if (renderer_ == nullptr || !renderer_->loadScene(scene_, plan_, runtime_diagnostics)) {
            logDiagnostics(get_logger(), runtime_diagnostics);
            throw std::runtime_error("renderer initialization failed");
        }
        table_ = fluent_scene::BindingTable::create(scene_, binding_->table_options,
                                                    runtime_diagnostics);
        if (table_ == nullptr) {
            logDiagnostics(get_logger(), runtime_diagnostics);
            throw std::runtime_error("binding table creation failed");
        }
        RCLCPP_INFO(get_logger(), "scene %s on %s backend (%s)", binding_->scene_name.c_str(),
                    renderer_->name(), plan_.digest.c_str());

        createSubscriptions();
        createPublishers();
        const auto period =
            std::chrono::duration<double>(1.0 / std::max(1.0, rate));
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() { onFrame(); });
    }

private:
    double nowSeconds() { return now().seconds(); }

    void createSubscriptions() {
        using fluent_scene::TypedValue;
        for (const fluent_scene::TopicBinding& binding : binding_->bindings) {
            const rclcpp::QoS qos = qosFromName(binding.qos);
            const std::string input = binding.input;
            if (binding.converter == "ros_image_to_rgba8") {
                image_subs_.push_back(create_subscription<sensor_msgs::msg::Image>(
                    binding.topic, qos,
                    [this, input](sensor_msgs::msg::Image::ConstSharedPtr message) {
                        TypedValue value;
                        if (!imageToRgba8(*message, value)) {
                            RCLCPP_WARN_ONCE(get_logger(), "unsupported image encoding \"%s\" on %s",
                                             message->encoding.c_str(), input.c_str());
                            return;
                        }
                        value.meta.has_timestamp = true;
                        value.meta.timestamp = rclcpp::Time(message->header.stamp).seconds();
                        value.meta.frame = message->header.frame_id;
                        value.meta.clock = "ros_time";
                        value.meta.sequence = ++sequence_;
                        fluent_scene::DiagnosticList diagnostics;
                        table_->push(input, std::move(value), diagnostics);
                    }));
            } else if (binding.converter == "ros_detections_to_Detection2D") {
                detection_subs_.push_back(create_subscription<vision_msgs::msg::Detection2DArray>(
                    binding.topic, qos,
                    [this, input](vision_msgs::msg::Detection2DArray::ConstSharedPtr message) {
                        TypedValue value;
                        value.kind = TypedValue::Kind::kDetections;
                        for (const auto& detection : message->detections) {
                            fluent_scene::DetectionInstance instance;
                            const auto& bbox = detection.bbox;
                            instance.bbox[0] = static_cast<float>(bbox.center.position.x -
                                                                  bbox.size_x * 0.5);
                            instance.bbox[1] = static_cast<float>(bbox.center.position.y -
                                                                  bbox.size_y * 0.5);
                            instance.bbox[2] = static_cast<float>(bbox.size_x);
                            instance.bbox[3] = static_cast<float>(bbox.size_y);
                            if (!detection.results.empty()) {
                                instance.score =
                                    static_cast<float>(detection.results.front().hypothesis.score);
                                instance.label = detection.results.front().hypothesis.class_id;
                            }
                            value.detections.push_back(std::move(instance));
                        }
                        value.meta.has_timestamp = true;
                        value.meta.timestamp = rclcpp::Time(message->header.stamp).seconds();
                        value.meta.frame = message->header.frame_id;
                        value.meta.clock = "ros_time";
                        value.meta.sequence = ++sequence_;
                        fluent_scene::DiagnosticList diagnostics;
                        table_->push(input, std::move(value), diagnostics);
                    }));
            } else if (binding.converter == "ros_string_to_utf8") {
                string_subs_.push_back(create_subscription<std_msgs::msg::String>(
                    binding.topic, qos, [this, input](std_msgs::msg::String::ConstSharedPtr message) {
                        TypedValue value;
                        value.kind = TypedValue::Kind::kString;
                        value.text = message->data;
                        value.meta.has_timestamp = true;  // stamped at receipt (no header)
                        value.meta.timestamp = nowSeconds();
                        value.meta.clock = "ros_time";
                        value.meta.sequence = ++sequence_;
                        fluent_scene::DiagnosticList diagnostics;
                        table_->push(input, std::move(value), diagnostics);
                    }));
            } else if (binding.converter == "ros_camera_info_to_calibration") {
                camera_info_subs_.push_back(create_subscription<sensor_msgs::msg::CameraInfo>(
                    binding.topic, qos,
                    [this, input](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
                        TypedValue value;
                        value.kind = TypedValue::Kind::kCalibration;
                        value.calibration_id = message->header.frame_id;
                        value.meta.has_timestamp = true;
                        value.meta.timestamp = rclcpp::Time(message->header.stamp).seconds();
                        value.meta.clock = "ros_time";
                        value.meta.sequence = ++sequence_;
                        fluent_scene::DiagnosticList diagnostics;
                        table_->push(input, std::move(value), diagnostics);
                    }));
            } else {
                RCLCPP_INFO(get_logger(),
                            "converter \"%s\" (input %s) validated but not implemented; the "
                            "declared fallback stays in effect",
                            binding.converter.c_str(), input.c_str());
            }
        }
    }

    void createPublishers() {
        for (const fluent_scene::OutputSink& sink : binding_->outputs) {
            if (sink.converter == "rgba8_to_ros_image" && !sink.topic.empty()) {
                image_pub_ = create_publisher<sensor_msgs::msg::Image>(sink.topic,
                                                                       qosFromName(sink.qos));
                RCLCPP_INFO(get_logger(), "publishing composite on %s", sink.topic.c_str());
                break;
            }
        }
    }

    void onFrame() {
        const double now_seconds = nowSeconds();
        const fluent_scene::FrameSnapshot snapshot = table_->acquireSnapshot(now_seconds);
        fluent_scene::DiagnosticList diagnostics;
        if (!renderer_->renderFrame(snapshot.toFrameInputs(), diagnostics)) {
            logDiagnostics(get_logger(), diagnostics);
            return;
        }
        ++frames_;
        if (image_pub_ != nullptr && image_pub_->get_subscription_count() > 0) {
            std::vector<uint8_t> pixels;
            uint32_t width = 0, height = 0;
            if (renderer_->readback(pixels, width, height)) {
                sensor_msgs::msg::Image message;
                message.header.stamp = now();
                message.header.frame_id = "fluent_scene";
                message.width = width;
                message.height = height;
                message.encoding = "rgba8";
                message.step = width * 4;
                message.data = std::move(pixels);
                image_pub_->publish(std::move(message));
            }
        }
        if (frames_ % 150 == 0) {
            const fluent_scene::BindingStats stats = table_->stats();
            const fluent_scene::RenderStats& render = renderer_->stats();
            RCLCPP_INFO(get_logger(),
                        "frames %lu | cpu %.2fms gpu %.2fms | snapshots %lu match_failures %lu",
                        static_cast<unsigned long>(frames_), render.last_cpu_ms, render.last_gpu_ms,
                        static_cast<unsigned long>(stats.snapshots),
                        static_cast<unsigned long>(stats.match_failures));
        }
    }

    fluent_scene::ValidationResult scene_;
    fluent_scene::PlanResult plan_;
    std::unique_ptr<fluent_scene::BindingDocument> binding_;
    std::unique_ptr<fluent_scene::Renderer> renderer_;
    std::unique_ptr<fluent_scene::BindingTable> table_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;
    std::vector<rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr> detection_subs_;
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> string_subs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subs_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t sequence_ = 0;
    uint64_t frames_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
    std::string scene_path;
    std::string binding_path;
    double rate = 30.0;
    bool use_cpu = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--scene" && i + 1 < argc) {
            scene_path = argv[++i];
        } else if (arg == "--binding" && i + 1 < argc) {
            binding_path = argv[++i];
        } else if (arg == "--rate" && i + 1 < argc) {
            rate = std::atof(argv[++i]);
        } else if (arg == "--cpu") {
            use_cpu = true;
        }
    }
    if (scene_path.empty() || binding_path.empty()) {
        std::fprintf(stderr,
                     "usage: fv_scene_node --scene <file.fvs> --binding <file.yaml> [--rate HZ] "
                     "[--cpu]\n");
        return 2;
    }
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        rclcpp::spin(std::make_shared<FvSceneNode>(scene_path, binding_path, rate, use_cpu));
    } catch (const std::exception& error) {
        std::fprintf(stderr, "fv_scene_node: %s\n", error.what());
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
