// fv_render — renders a Fluent Scene headlessly with synthetic inputs and
// measures per-frame latency. This is the stage-2 demo/benchmark tool: it
// executes validate -> plan -> retained render on the chosen backend.
//
// Usage:
//   fv_render <scene.fvs> [--backend vulkan|cpu|both] [--frames N]
//             [--camera WxH] [--out file.ppm] [--validate]
//
// Exit codes: 0 ok, 1 scene/render error, 2 usage/io error.

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "fluent_scene/binding.hpp"
#include "fluent_scene/diagnostics.hpp"
#include "fluent_scene/planner.hpp"
#include "fluent_scene/registry.hpp"
#include "fluent_scene/render/renderer.hpp"
#include "fluent_scene/validator.hpp"
#include "fluent_scene/yaml.hpp"
#include "render/test_pattern.hpp"

namespace {

struct Percentiles {
    double p50 = 0, p95 = 0, p99 = 0, mean = 0;
};

Percentiles percentiles(std::vector<double> samples) {
    Percentiles result;
    if (samples.empty()) {
        return result;
    }
    std::sort(samples.begin(), samples.end());
    const auto at = [&](double q) {
        return samples[static_cast<size_t>(q * static_cast<double>(samples.size() - 1))];
    };
    result.p50 = at(0.50);
    result.p95 = at(0.95);
    result.p99 = at(0.99);
    double total = 0;
    for (double v : samples) {
        total += v;
    }
    result.mean = total / static_cast<double>(samples.size());
    return result;
}

void printDiagnostics(const fluent_scene::DiagnosticList& diagnostics) {
    for (const auto& diagnostic : diagnostics.items()) {
        std::cerr << toString(diagnostic.severity) << ": " << diagnostic.code << ": "
                  << diagnostic.message << '\n';
    }
}

// Renders through the stage-3 runtime binding table: sources push timestamped
// values, the scheduler snapshots at the frame boundary, and the snapshot
// feeds the retained renderer. status_text is intentionally never pushed so
// the declared default_status fallback is visible in the output.
int runWithBindings(const fluent_scene::ValidationResult& scene,
                    const fluent_scene::PlanResult& plan, int frames, uint32_t camera_w,
                    uint32_t camera_h, uint32_t out_w, uint32_t out_h, bool validate_layers,
                    const std::string& out_path) {
    fluent_scene::DiagnosticList diagnostics;
    fluent_scene::RendererOptions renderer_options;
    renderer_options.enable_validation = validate_layers;
    std::unique_ptr<fluent_scene::Renderer> renderer =
        fluent_scene::createVulkanRenderer(renderer_options, diagnostics);
    if (renderer == nullptr || !renderer->loadScene(scene, plan, diagnostics)) {
        printDiagnostics(diagnostics);
        return 1;
    }
    fluent_scene::BindingTableOptions binding_options;
    std::unique_ptr<fluent_scene::BindingTable> table =
        fluent_scene::BindingTable::create(scene, binding_options, diagnostics);
    if (table == nullptr) {
        printDiagnostics(diagnostics);
        return 1;
    }
    for (int frame = 0; frame < frames; ++frame) {
        const double t = static_cast<double>(frame) / 30.0;
        const fluent_scene::render::SyntheticFrame synthetic =
            fluent_scene::render::makeSyntheticFrame(static_cast<uint32_t>(frame), camera_w, camera_h,
                                                     out_w, out_h);
        fluent_scene::TypedValue camera;
        camera.kind = fluent_scene::TypedValue::Kind::kImage;
        camera.width = synthetic.camera_width;
        camera.height = synthetic.camera_height;
        camera.pixels = synthetic.camera;
        camera.meta.has_timestamp = true;
        camera.meta.timestamp = t;
        camera.meta.sequence = static_cast<uint64_t>(frame) * 2 + 1;
        table->push("camera", std::move(camera), diagnostics);
        fluent_scene::TypedValue detections;
        detections.kind = fluent_scene::TypedValue::Kind::kDetections;
        detections.detections = synthetic.detections;
        detections.meta.has_timestamp = true;
        detections.meta.timestamp = t + 0.004;  // within the 12 ms tolerance
        detections.meta.sequence = static_cast<uint64_t>(frame) * 2 + 2;
        table->push("detections", std::move(detections), diagnostics);

        const fluent_scene::FrameSnapshot snapshot = table->acquireSnapshot(t + 0.008);
        if (!renderer->renderFrame(snapshot.toFrameInputs(), diagnostics)) {
            printDiagnostics(diagnostics);
            return 1;
        }
    }
    const fluent_scene::BindingStats stats = table->stats();
    std::cout << "bindings: " << stats.snapshots << " snapshots, match_failures "
              << stats.match_failures << '\n';
    for (const auto& [name, input] : stats.inputs) {
        std::cout << "  " << name << ": pushes " << input.pushes << ", drops " << input.drops
                  << ", held " << input.held << ", stale " << input.stale << ", fallbacks "
                  << input.fallbacks << '\n';
    }
    if (!out_path.empty()) {
        std::vector<uint8_t> pixels;
        uint32_t width = 0, height = 0;
        if (renderer->readback(pixels, width, height)) {
            fluent_scene::render::writePpm(out_path, pixels, width, height);
            std::cout << "  wrote " << out_path << '\n';
        }
    }
    return 0;
}

int runBackend(const std::string& backend, const fluent_scene::ValidationResult& scene,
               const fluent_scene::PlanResult& plan, int frames, uint32_t camera_w, uint32_t camera_h,
               uint32_t out_w, uint32_t out_h, bool validate_layers, const std::string& out_path) {
    fluent_scene::DiagnosticList diagnostics;
    fluent_scene::RendererOptions options;
    options.enable_validation = validate_layers;
    std::unique_ptr<fluent_scene::Renderer> renderer;
    if (backend == "vulkan") {
        renderer = fluent_scene::createVulkanRenderer(options, diagnostics);
    } else {
        renderer = fluent_scene::createCpuRenderer(options, diagnostics);
    }
    if (renderer == nullptr) {
        printDiagnostics(diagnostics);
        std::cerr << "fv_render: cannot create " << backend << " renderer\n";
        return 1;
    }
    if (!renderer->loadScene(scene, plan, diagnostics)) {
        printDiagnostics(diagnostics);
        return 1;
    }
    std::vector<double> cpu_ms, gpu_ms;
    for (int frame = 0; frame < frames; ++frame) {
        const fluent_scene::render::SyntheticFrame synthetic =
            fluent_scene::render::makeSyntheticFrame(static_cast<uint32_t>(frame), camera_w, camera_h,
                                                     out_w, out_h);
        if (!renderer->renderFrame(synthetic.inputs(), diagnostics)) {
            printDiagnostics(diagnostics);
            return 1;
        }
        cpu_ms.push_back(renderer->stats().last_cpu_ms);
        gpu_ms.push_back(renderer->stats().last_gpu_ms);
    }
    const Percentiles cpu = percentiles(cpu_ms);
    const Percentiles gpu = percentiles(gpu_ms);
    const fluent_scene::RenderStats& stats = renderer->stats();
    std::cout << "backend " << renderer->name() << ": " << frames << " frames @ " << out_w << 'x'
              << out_h << "\n"
              << "  frame cpu ms: mean " << cpu.mean << ", p50 " << cpu.p50 << ", p95 " << cpu.p95
              << ", p99 " << cpu.p99 << '\n';
    if (backend == "vulkan") {
        std::cout << "  frame gpu ms: mean " << gpu.mean << ", p50 " << gpu.p50 << ", p95 " << gpu.p95
                  << ", p99 " << gpu.p99 << '\n';
    }
    std::cout << "  counters: pipeline_compiles " << stats.pipeline_compiles << ", image_uploads "
              << stats.image_uploads << ", buffer_uploads " << stats.buffer_uploads
              << ", atlas_updates " << stats.atlas_updates << ", missing_glyphs "
              << stats.missing_glyphs << '\n';
    if (!out_path.empty()) {
        std::vector<uint8_t> pixels;
        uint32_t width = 0, height = 0;
        if (!renderer->readback(pixels, width, height) ||
            !fluent_scene::render::writePpm(out_path, pixels, width, height)) {
            std::cerr << "fv_render: readback/write failed for " << out_path << '\n';
            return 1;
        }
        std::cout << "  wrote " << out_path << " (" << width << 'x' << height << ")\n";
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: fv_render <scene.fvs> [--backend vulkan|cpu|both] [--frames N] "
                     "[--camera WxH] [--out file.ppm] [--validate]\n";
        return 2;
    }
    const std::string scene_path = argv[1];
    std::string backend = "vulkan";
    std::string out_path;
    int frames = 120;
    uint32_t camera_w = 1280, camera_h = 720;
    bool validate_layers = false;
    bool use_bindings = false;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--bindings") {
            use_bindings = true;
        } else if (arg == "--backend" && i + 1 < argc) {
            backend = argv[++i];
        } else if (arg == "--frames" && i + 1 < argc) {
            frames = std::max(1, std::atoi(argv[++i]));
        } else if (arg == "--out" && i + 1 < argc) {
            out_path = argv[++i];
        } else if (arg == "--camera" && i + 1 < argc) {
            const std::string spec = argv[++i];
            const size_t x = spec.find('x');
            if (x != std::string::npos) {
                camera_w = static_cast<uint32_t>(std::atoi(spec.substr(0, x).c_str()));
                camera_h = static_cast<uint32_t>(std::atoi(spec.substr(x + 1).c_str()));
            }
        } else if (arg == "--validate") {
            validate_layers = true;
        } else {
            std::cerr << "fv_render: unknown option " << arg << '\n';
            return 2;
        }
    }

    std::ifstream stream(scene_path, std::ios::binary);
    if (!stream) {
        std::cerr << "fv_render: cannot read " << scene_path << '\n';
        return 2;
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();

    fluent_scene::DiagnosticList diagnostics;
    const fluent_scene::YamlNode root = fluent_scene::parseYaml(buffer.str(), diagnostics);
    fluent_scene::ValidationResult scene;
    fluent_scene::PlanResult plan;
    if (!diagnostics.hasErrors()) {
        const fluent_scene::NodeRegistry registry = fluent_scene::NodeRegistry::builtinMvp();
        scene = fluent_scene::validateScene(root, registry, diagnostics);
        if (scene.ok) {
            plan = fluent_scene::planScene(scene, diagnostics);
        }
    }
    if (!plan.ok) {
        diagnostics.sortCanonical();
        printDiagnostics(diagnostics);
        std::cerr << "fv_render: scene does not compile\n";
        return 1;
    }
    const fluent_scene::JsonValue* budgets = scene.ir.find("budgets");
    const uint32_t out_w = static_cast<uint32_t>(budgets->find("max_width")->uintValue());
    const uint32_t out_h = static_cast<uint32_t>(budgets->find("max_height")->uintValue());

    if (use_bindings) {
        return runWithBindings(scene, plan, frames, camera_w, camera_h, out_w, out_h,
                               validate_layers, out_path);
    }
    if (backend == "both") {
        const int vulkan_result = runBackend("vulkan", scene, plan, frames, camera_w, camera_h, out_w,
                                             out_h, validate_layers,
                                             out_path.empty() ? "" : out_path + ".vulkan.ppm");
        const int cpu_result = runBackend("cpu", scene, plan, frames, camera_w, camera_h, out_w, out_h,
                                          validate_layers,
                                          out_path.empty() ? "" : out_path + ".cpu.ppm");
        return vulkan_result != 0 ? vulkan_result : cpu_result;
    }
    return runBackend(backend, scene, plan, frames, camera_w, camera_h, out_w, out_h, validate_layers,
                      out_path);
}
