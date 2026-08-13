// Stage-3 exit evidence (spec section 14.3): freshness, synchronization,
// drop, and fallback behavior of the runtime binding layer — with no ROS 2
// and no GPU. Time is injected, so every case is deterministic.
//
// Usage: fvs_binding_tests <source_dir>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "fluent_scene/binding.hpp"
#include "fluent_scene/binding_config.hpp"
#include "fluent_scene/diagnostics.hpp"
#include "fluent_scene/registry.hpp"
#include "fluent_scene/validator.hpp"
#include "fluent_scene/yaml.hpp"

namespace {

int failures = 0;
std::string source_dir;

void check(bool condition, const std::string& what) {
    if (condition) {
        std::cout << "ok   " << what << '\n';
    } else {
        std::cout << "FAIL " << what << '\n';
        ++failures;
    }
}

std::string readFile(const std::string& relative) {
    std::ifstream stream(source_dir + "/" + relative, std::ios::binary);
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    return buffer.str();
}

fluent_scene::ValidationResult compileScene(const std::string& text) {
    fluent_scene::DiagnosticList diagnostics;
    const fluent_scene::YamlNode root = fluent_scene::parseYaml(text, diagnostics);
    fluent_scene::ValidationResult scene;
    if (!diagnostics.hasErrors()) {
        const fluent_scene::NodeRegistry registry = fluent_scene::NodeRegistry::builtinMvp();
        scene = fluent_scene::validateScene(root, registry, diagnostics);
    }
    if (!scene.ok) {
        for (const auto& diagnostic : diagnostics.items()) {
            std::cerr << diagnostic.code << ": " << diagnostic.message << '\n';
        }
    }
    return scene;
}

fluent_scene::TypedValue imageValue(double timestamp, uint64_t sequence) {
    fluent_scene::TypedValue value;
    value.kind = fluent_scene::TypedValue::Kind::kImage;
    value.width = 4;
    value.height = 4;
    value.pixels.assign(4 * 4 * 4, 0x80);
    value.meta.has_timestamp = true;
    value.meta.timestamp = timestamp;
    value.meta.sequence = sequence;
    value.meta.clock = "test";
    value.meta.frame = "camera_optical";
    return value;
}

fluent_scene::TypedValue detectionsValue(double timestamp, uint64_t sequence) {
    fluent_scene::TypedValue value;
    value.kind = fluent_scene::TypedValue::Kind::kDetections;
    fluent_scene::DetectionInstance detection;
    detection.bbox[0] = 1;
    detection.bbox[1] = 2;
    detection.bbox[2] = 3;
    detection.bbox[3] = 4;
    detection.score = 0.9f;
    value.detections.push_back(detection);
    value.meta.has_timestamp = true;
    value.meta.timestamp = timestamp;
    value.meta.sequence = sequence;
    return value;
}

fluent_scene::TypedValue stringValue(const std::string& text, double timestamp, uint64_t sequence) {
    fluent_scene::TypedValue value;
    value.kind = fluent_scene::TypedValue::Kind::kString;
    value.text = text;
    value.meta.has_timestamp = true;
    value.meta.timestamp = timestamp;
    value.meta.sequence = sequence;
    return value;
}

std::string entrySummary(const fluent_scene::FrameSnapshot& snapshot, const std::string& input) {
    const auto it = snapshot.entries.find(input);
    if (it == snapshot.entries.end()) {
        return "<absent>";
    }
    std::ostringstream out;
    out << toString(it->second.status) << (it->second.valid ? "/valid" : "/invalid") << "/seq"
        << it->second.sequence;
    return out.str();
}

std::string snapshotSummary(const fluent_scene::FrameSnapshot& snapshot) {
    std::ostringstream out;
    for (const auto& [name, entry] : snapshot.entries) {
        out << name << '=' << toString(entry.status) << ':' << (entry.valid ? 1 : 0) << ':'
            << entry.sequence << ':' << entry.age_seconds << ';';
    }
    out << "healthy=" << snapshot.healthy;
    return out.str();
}

// Minimal scene with a required per_frame input that declares NO fallback,
// to prove the unhealthy path.
const char* kNoFallbackScene = R"(schema: fluent.scene/v1alpha1
kind: Scene
metadata:
  name: no_fallback_fixture
inputs:
  camera:
    type: image.rgba8
    required: true
    update: per_frame
nodes:
  - id: camera_layer
    type: visual.image2d
    inputs:
      image: $inputs.camera
  - id: composite
    type: composite.layers
    inputs:
      layers:
        - $nodes.camera_layer.layer
outputs:
  frame:
    type: image.rgba8
    source: $nodes.composite.image
budgets:
  max_width: 640
  max_height: 360
  max_gpu_bytes: 67108864
  max_upload_bytes_per_frame: 8388608
  max_frames_in_flight: 2
)";

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: fvs_binding_tests <source_dir>\n";
        return 2;
    }
    source_dir = argv[1];
    using fluent_scene::BindingTable;
    using fluent_scene::BindingTableOptions;
    using fluent_scene::DiagnosticList;
    using fluent_scene::EntryStatus;
    using fluent_scene::FrameSnapshot;
    using fluent_scene::InputRuntimeConfig;
    using fluent_scene::QueuePolicy;

    const fluent_scene::ValidationResult scene =
        compileScene(readFile("examples/camera_detection_hud.fvs"));
    if (!scene.ok) {
        std::cout << "FAIL example scene compiles\n";
        return 1;
    }
    const BindingTableOptions default_options;

    // --- 1. matched group + declared fallbacks ------------------------------
    {
        DiagnosticList diagnostics;
        auto table = BindingTable::create(scene, default_options, diagnostics);
        check(table != nullptr, "binding table builds from the section-11 scene");
        table->push("camera", imageValue(1.000, 11), diagnostics);
        table->push("detections", detectionsValue(1.004, 21), diagnostics);
        const FrameSnapshot snapshot = table->acquireSnapshot(1.010);
        check(entrySummary(snapshot, "camera") == "fresh/valid/seq11",
              "camera matches within tolerance and is fresh");
        check(entrySummary(snapshot, "detections") == "fresh/valid/seq21",
              "detections match the camera timestamp (4 ms apart)");
        check(snapshot.entries.at("depth").status == EntryStatus::kFallbackValue &&
                  !snapshot.entries.at("depth").valid,
              "absent optional depth resolves to its declared hide fallback");
        const auto& status_text = snapshot.entries.at("status_text");
        check(status_text.status == EntryStatus::kFallbackValue && status_text.valid &&
                  status_text.value->text == "映像を待っています",
              "absent status_text serves its declared typed default");
        check(snapshot.entries.at("camera_calibration").status ==
                  EntryStatus::kFallbackUnavailable,
              "absent required calibration follows its declared behavior fallback");
        check(snapshot.healthy, "declared fallbacks keep the frame healthy");
        const auto inputs = snapshot.toFrameInputs();
        check(inputs.images.count("camera") == 1 && inputs.strings.count("status_text") == 1,
              "snapshot converts to renderer frame inputs");
    }

    // --- 2. synchronization: tolerance failure, then recovery ---------------
    {
        DiagnosticList diagnostics;
        auto table = BindingTable::create(scene, default_options, diagnostics);
        table->push("camera", imageValue(2.000, 1), diagnostics);
        table->push("detections", detectionsValue(2.020, 2), diagnostics);  // 20 ms > 12 ms
        const FrameSnapshot miss = table->acquireSnapshot(2.030);
        check(table->stats().match_failures == 1, "out-of-tolerance pair counts a match failure");
        check(miss.entries.at("camera").status == EntryStatus::kFallbackUnavailable,
              "unmatched camera falls back (no previous value to hold)");
        table->push("detections", detectionsValue(2.006, 3), diagnostics);  // within 6 ms
        const FrameSnapshot hit = table->acquireSnapshot(2.040);
        check(entrySummary(hit, "camera") == "fresh/valid/seq1" &&
                  entrySummary(hit, "detections") == "fresh/valid/seq3",
              "queues survive a failed match and pair on the next frame");
    }

    // --- 3. bounded queues: latest evicts and counts drops ------------------
    {
        DiagnosticList diagnostics;
        auto table = BindingTable::create(scene, default_options, diagnostics);
        table->push("status_text", stringValue("a", 1.0, 1), diagnostics);
        table->push("status_text", stringValue("b", 2.0, 2), diagnostics);
        table->push("status_text", stringValue("c", 3.0, 3), diagnostics);
        const FrameSnapshot snapshot = table->acquireSnapshot(3.1);
        check(snapshot.entries.at("status_text").value->text == "c" &&
                  table->stats().inputs.at("status_text").drops == 2,
              "latest policy keeps the newest value and counts evictions");
    }

    // --- 4. fifo policy drains in order and then holds ----------------------
    {
        BindingTableOptions options;
        options.inputs["status_text"] = InputRuntimeConfig{QueuePolicy::kFifo, 2, 0.0};
        DiagnosticList diagnostics;
        auto table = BindingTable::create(scene, options, diagnostics);
        table->push("status_text", stringValue("a", 1.0, 1), diagnostics);
        table->push("status_text", stringValue("b", 2.0, 2), diagnostics);
        table->push("status_text", stringValue("c", 3.0, 3), diagnostics);  // capacity 2: drops "a"
        const FrameSnapshot first = table->acquireSnapshot(3.1);
        const FrameSnapshot second = table->acquireSnapshot(3.2);
        const FrameSnapshot third = table->acquireSnapshot(3.3);
        check(table->stats().inputs.at("status_text").drops == 1 &&
                  first.entries.at("status_text").value->text == "b" &&
                  second.entries.at("status_text").value->text == "c" &&
                  third.entries.at("status_text").status == EntryStatus::kHeld,
              "fifo drains one value per frame, drops on overflow, then holds");
    }

    // --- 5. freshness: stale per_frame data resolves to fallbacks -----------
    {
        BindingTableOptions options;
        options.inputs["camera"] = InputRuntimeConfig{QueuePolicy::kMatched, 4, 0.1};
        DiagnosticList diagnostics;
        auto table = BindingTable::create(scene, options, diagnostics);
        table->push("camera", imageValue(1.0, 1), diagnostics);
        table->push("detections", detectionsValue(1.0, 2), diagnostics);
        const FrameSnapshot fresh = table->acquireSnapshot(1.05);
        check(fresh.entries.at("camera").status == EntryStatus::kFresh, "value inside max_age is fresh");
        const FrameSnapshot stale = table->acquireSnapshot(5.0);
        check(stale.entries.at("camera").status == EntryStatus::kFallbackUnavailable &&
                  table->stats().inputs.at("camera").stale == 1,
              "stale camera resolves to its behavior fallback and is counted");
        check(stale.healthy, "declared stale handling keeps the frame healthy");
    }

    // --- 6. undeclared missing required input makes the frame unhealthy -----
    {
        const fluent_scene::ValidationResult bare = compileScene(kNoFallbackScene);
        check(bare.ok, "no-fallback fixture compiles");
        DiagnosticList diagnostics;
        auto table = BindingTable::create(bare, default_options, diagnostics);
        const FrameSnapshot missing = table->acquireSnapshot(1.0);
        check(!missing.healthy &&
                  missing.entries.at("camera").status == EntryStatus::kMissing,
              "missing required input without a fallback marks the frame unhealthy");
        table->push("camera", imageValue(2.0, 1), diagnostics);
        const FrameSnapshot ok_frame = table->acquireSnapshot(2.01);
        check(ok_frame.healthy, "health recovers once required data arrives");
        const FrameSnapshot stale_frame = table->acquireSnapshot(9.0);
        check(!stale_frame.healthy, "stale required input without fallback is unhealthy again");
    }

    // --- 7. snapshots are immutable; mid-frame arrivals wait ----------------
    {
        DiagnosticList diagnostics;
        auto table = BindingTable::create(scene, default_options, diagnostics);
        table->push("camera", imageValue(1.00, 1), diagnostics);
        table->push("detections", detectionsValue(1.00, 2), diagnostics);
        const FrameSnapshot before = table->acquireSnapshot(1.01);
        table->push("camera", imageValue(1.05, 3), diagnostics);  // arrives mid-frame
        check(before.entries.at("camera").sequence == 1,
              "an already-acquired snapshot never changes");
        const FrameSnapshot after = table->acquireSnapshot(1.06);
        check(after.entries.at("camera").sequence == 3 &&
                  after.entries.at("detections").status == EntryStatus::kHeld,
              "the next snapshot sees the new value; optional peer holds");
    }

    // --- 8. contract violations produce bind-phase diagnostics --------------
    {
        DiagnosticList diagnostics;
        auto table = BindingTable::create(scene, default_options, diagnostics);
        check(!table->push("nonexistent", stringValue("x", 1.0, 1), diagnostics),
              "push to an undeclared input is rejected");
        check(!table->push("camera", stringValue("x", 1.0, 1), diagnostics),
              "kind mismatch is rejected");
        fluent_scene::TypedValue no_timestamp = imageValue(1.0, 1);
        no_timestamp.meta.has_timestamp = false;
        check(!table->push("camera", std::move(no_timestamp), diagnostics),
              "values without an explicit timestamp are rejected");
        size_t bind_errors = 0;
        for (const auto& diagnostic : diagnostics.items()) {
            if (diagnostic.phase == fluent_scene::Phase::kBind &&
                diagnostic.severity == fluent_scene::Severity::kError) {
                ++bind_errors;
            }
        }
        check(bind_errors == 3, "each violation is a structured bind-phase diagnostic");
    }

    // --- 9. determinism: identical schedules give identical snapshots -------
    {
        const auto run = [&]() {
            DiagnosticList diagnostics;
            auto table = BindingTable::create(scene, default_options, diagnostics);
            std::string log;
            for (int frame = 0; frame < 8; ++frame) {
                const double t = 1.0 + 0.033 * frame;
                table->push("camera", imageValue(t, static_cast<uint64_t>(frame * 2 + 1)),
                            diagnostics);
                if (frame % 2 == 0) {
                    table->push("detections",
                                detectionsValue(t + 0.004, static_cast<uint64_t>(frame * 2 + 2)),
                                diagnostics);
                }
                if (frame == 3) {
                    table->push("status_text", stringValue("収穫中", t, 99), diagnostics);
                }
                log += snapshotSummary(table->acquireSnapshot(t + 0.008)) + "\n";
            }
            return log;
        };
        check(run() == run(), "identical push/acquire schedules produce identical snapshots");
    }

    // --- 10. binding document (spec section 12) parses and validates --------
    {
        DiagnosticList diagnostics;
        const fluent_scene::YamlNode doc = fluent_scene::parseYaml(
            readFile("examples/camera_detection_hud_ros2.binding.yaml"), diagnostics);
        auto binding = fluent_scene::parseBindingDocument(doc, scene, diagnostics);
        check(binding != nullptr && !diagnostics.hasErrors(),
              "the section-12 binding document validates against the section-11 scene");
        if (binding != nullptr) {
            check(binding->scene_name == "camera_detection_hud" && binding->bindings.size() == 5,
                  "all five bindings are captured");
            const auto& camera =
                *std::find_if(binding->bindings.begin(), binding->bindings.end(),
                              [](const auto& b) { return b.input == "camera"; });
            check(camera.topic == "/camera/color/image_raw" &&
                      camera.converter == "ros_image_to_rgba8" && camera.qos == "sensor_data",
                  "camera binding carries topic, converter, and qos verbatim");
            const auto group = binding->table_options.groups.find("sensor_frame");
            check(group != binding->table_options.groups.end() &&
                      std::abs(group->second.tolerance_seconds - 0.012) < 1e-9,
                  "synchronization tolerance (12 ms) reaches the binding table options");
            const auto camera_config = binding->table_options.inputs.find("camera");
            check(camera_config != binding->table_options.inputs.end() &&
                      camera_config->second.capacity == 4,
                  "group queue_capacity is applied to member inputs");
        }
    }

    // --- 11. binding-document contract violations ----------------------------
    {
        const auto parseWith = [&](const std::string& patch_from, const std::string& patch_to) {
            std::string text = readFile("examples/camera_detection_hud_ros2.binding.yaml");
            const size_t at = text.find(patch_from);
            if (at != std::string::npos) {
                text.replace(at, patch_from.size(), patch_to);
            }
            DiagnosticList diagnostics;
            const fluent_scene::YamlNode doc = fluent_scene::parseYaml(text, diagnostics);
            auto binding = fluent_scene::parseBindingDocument(doc, scene, diagnostics);
            std::string first_error;
            for (const auto& diagnostic : diagnostics.items()) {
                if (diagnostic.severity == fluent_scene::Severity::kError) {
                    first_error = diagnostic.code;
                    break;
                }
            }
            return std::make_pair(binding == nullptr, first_error);
        };
        auto unknown_input = parseWith("  camera:\n    source:", "  cameraz:\n    source:");
        check(unknown_input.first && unknown_input.second == "bind.unknown_input",
              "binding an undeclared input is rejected");
        auto bad_converter =
            parseWith("converter: ros_image_to_rgba8", "converter: ros_image_to_nothing");
        check(bad_converter.first && bad_converter.second == "bind.unknown_converter",
              "an unregistered converter is rejected before activation");
        auto mismatched =
            parseWith("converter: ros_string_to_utf8", "converter: ros_image_to_rgba8");
        check(mismatched.first && mismatched.second == "bind.converter_mismatch",
              "a converter that cannot produce the input type is rejected");
        auto wrong_scene =
            parseWith("scene:\n  name: camera_detection_hud", "scene:\n  name: other_scene");
        check(wrong_scene.first && wrong_scene.second == "bind.scene_mismatch",
              "a binding document for a different scene is rejected");
    }

    if (failures == 0) {
        std::cout << "all binding tests passed\n";
        return 0;
    }
    std::cout << failures << " binding test(s) failed\n";
    return 1;
}
