// Stage-2 renderer tests: retained-execution counters (no per-frame pipeline
// compiles, atlas updates only on text change), golden image with tolerance,
// and a Vulkan-versus-CPU cross-check. Skips (exit 77) when no Vulkan device
// is available so the suite stays runnable on GPU-less machines.
//
// Usage: fvs_render_tests <source_dir> [--regen]

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "fluent_scene/diagnostics.hpp"
#include "fluent_scene/planner.hpp"
#include "fluent_scene/registry.hpp"
#include "fluent_scene/render/renderer.hpp"
#include "fluent_scene/validator.hpp"
#include "fluent_scene/yaml.hpp"
#include "render/box_smoother.hpp"
#include "render/test_pattern.hpp"

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

void printDiagnostics(const fluent_scene::DiagnosticList& diagnostics) {
    for (const auto& diagnostic : diagnostics.items()) {
        std::cerr << toString(diagnostic.severity) << ": " << diagnostic.code << ": "
                  << diagnostic.message << '\n';
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: fvs_render_tests <source_dir> [--regen]\n";
        return 2;
    }
    source_dir = argv[1];
    const bool regen = argc > 2 && std::string(argv[2]) == "--regen";

    // Compile the fixture scene.
    fluent_scene::DiagnosticList diagnostics;
    const fluent_scene::YamlNode root =
        fluent_scene::parseYaml(readFile("tests/fixtures/render_scene.fvs"), diagnostics);
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
        printDiagnostics(diagnostics);
        std::cout << "FAIL render fixture does not compile\n";
        return 1;
    }

    // Vulkan device or skip.
    fluent_scene::RendererOptions options;
    fluent_scene::DiagnosticList vulkan_diagnostics;
    std::unique_ptr<fluent_scene::Renderer> vulkan =
        fluent_scene::createVulkanRenderer(options, vulkan_diagnostics);
    if (vulkan == nullptr) {
        std::cout << "SKIP: no usable Vulkan device on this machine\n";
        return 77;
    }
    if (!vulkan->loadScene(scene, plan, vulkan_diagnostics)) {
        printDiagnostics(vulkan_diagnostics);
        std::cout << "FAIL vulkan loadScene\n";
        return 1;
    }
    const uint64_t compiles_after_load = vulkan->stats().pipeline_compiles;
    check(compiles_after_load > 0, "pipelines are compiled during loadScene");

    // Three frames with constant text (frame 0 repeated) — the retained path.
    const fluent_scene::render::SyntheticFrame synthetic =
        fluent_scene::render::makeSyntheticFrame(0, 640, 360, 640, 360);
    for (int frame = 0; frame < 3; ++frame) {
        if (!vulkan->renderFrame(synthetic.inputs(), vulkan_diagnostics)) {
            printDiagnostics(vulkan_diagnostics);
            std::cout << "FAIL vulkan renderFrame\n";
            return 1;
        }
    }
    const fluent_scene::RenderStats& stats = vulkan->stats();
    check(stats.frames == 3, "three frames rendered");
    check(stats.pipeline_compiles == compiles_after_load,
          "no per-frame shader/pipeline compilation (spec section 8.2)");
    check(stats.atlas_updates == 1, "glyph atlas uploaded exactly once for constant text");
    check(stats.missing_glyphs == 0, "the CJK font covers the Japanese status text");
    check(stats.image_uploads == 3, "camera image streams every frame");

    std::vector<uint8_t> vulkan_pixels;
    uint32_t width = 0, height = 0;
    check(vulkan->readback(vulkan_pixels, width, height) && width == 640 && height == 360,
          "readback returns the 640x360 composite");

    const std::string golden_path = source_dir + "/tests/golden/render_scene.ppm";
    if (regen) {
        fluent_scene::render::writePpm(golden_path, vulkan_pixels, width, height);
        std::cout << "regenerated render golden\n";
    } else {
        std::vector<uint8_t> golden;
        uint32_t golden_w = 0, golden_h = 0;
        if (!fluent_scene::render::readPpm(golden_path, golden, golden_w, golden_h) ||
            golden_w != width || golden_h != height) {
            check(false, "golden image exists and matches dimensions");
        } else {
            double mean_diff = 0.0;
            uint32_t max_diff = 0;
            fluent_scene::render::diffRgbaVsRgb(vulkan_pixels, golden, mean_diff, max_diff);
            std::cout << "     golden diff: mean " << mean_diff << ", max " << max_diff << '\n';
            check(mean_diff <= 2.0 && max_diff <= 96,
                  "rendered composite matches the golden image within tolerance");
        }
    }

    // CPU reference cross-check on the same scene and inputs.
    fluent_scene::DiagnosticList cpu_diagnostics;
    std::unique_ptr<fluent_scene::Renderer> cpu =
        fluent_scene::createCpuRenderer(options, cpu_diagnostics);
    if (cpu == nullptr || !cpu->loadScene(scene, plan, cpu_diagnostics) ||
        !cpu->renderFrame(synthetic.inputs(), cpu_diagnostics)) {
        printDiagnostics(cpu_diagnostics);
        check(false, "cpu reference renders the fixture");
    } else {
        std::vector<uint8_t> cpu_pixels;
        uint32_t cpu_w = 0, cpu_h = 0;
        cpu->readback(cpu_pixels, cpu_w, cpu_h);
        std::vector<uint8_t> cpu_rgb(static_cast<size_t>(cpu_w) * cpu_h * 3);
        for (size_t i = 0; i < static_cast<size_t>(cpu_w) * cpu_h; ++i) {
            cpu_rgb[i * 3 + 0] = cpu_pixels[i * 4 + 0];
            cpu_rgb[i * 3 + 1] = cpu_pixels[i * 4 + 1];
            cpu_rgb[i * 3 + 2] = cpu_pixels[i * 4 + 2];
        }
        double mean_diff = 0.0;
        uint32_t max_diff = 0;
        fluent_scene::render::diffRgbaVsRgb(vulkan_pixels, cpu_rgb, mean_diff, max_diff);
        std::cout << "     vulkan-vs-cpu diff: mean " << mean_diff << ", max " << max_diff << '\n';
        check(mean_diff <= 10.0, "vulkan and cpu backends agree within a loose tolerance");
    }

    // --- box smoothing (EMA + nearest-neighbor association) -----------------
    {
        using fluent_scene::DetectionInstance;
        fluent_scene::render::BoxSmoother smoother;
        DetectionInstance det;
        det.bbox[0] = 100;
        det.bbox[1] = 100;
        det.bbox[2] = 80;
        det.bbox[3] = 60;
        auto first = smoother.update({det}, 0.033, 0.15f, 128);
        check(first.size() == 1 && first[0].bbox[0] == 100.0f,
              "a new track starts exactly at its target (no pop-in)");
        det.bbox[0] = 200;  // jump 100px to the right
        auto second = smoother.update({det}, 0.033, 0.15f, 128);
        check(second[0].bbox[0] > 100.0f && second[0].bbox[0] < 200.0f,
              "a matched track moves smoothly toward the new position");
        auto third = smoother.update({det}, 0.033, 0.15f, 128);
        check(third[0].bbox[0] > second[0].bbox[0] && third[0].bbox[0] < 200.0f,
              "successive frames keep converging on the target");
        auto passthrough = smoother.update({det}, 0.033, 0.0f, 128);
        check(passthrough[0].bbox[0] == 200.0f, "smoothing 0 passes detections through unchanged");
        det.bbox[0] = 1500;  // far beyond the association gate
        auto teleport = smoother.update({det}, 0.033, 0.15f, 128);
        check(teleport[0].bbox[0] == 1500.0f,
              "a distant detection is treated as a new object, not dragged across the screen");
    }

    // --- 2D primitives (circles + polyline) ---------------------------------
    {
        fluent_scene::DiagnosticList prim_diagnostics;
        const fluent_scene::YamlNode prim_root =
            fluent_scene::parseYaml(readFile("examples/primitives_demo.fvs"), prim_diagnostics);
        fluent_scene::ValidationResult prim_scene;
        fluent_scene::PlanResult prim_plan;
        if (!prim_diagnostics.hasErrors()) {
            const fluent_scene::NodeRegistry registry = fluent_scene::NodeRegistry::builtinMvp();
            prim_scene = fluent_scene::validateScene(prim_root, registry, prim_diagnostics);
            if (prim_scene.ok) {
                prim_plan = fluent_scene::planScene(prim_scene, prim_diagnostics);
            }
        }
        check(prim_plan.ok, "primitives scene compiles (circles2d + polyline2d)");
        std::unique_ptr<fluent_scene::Renderer> prim_renderer =
            fluent_scene::createVulkanRenderer(options, prim_diagnostics);
        bool rendered = prim_renderer != nullptr &&
                        prim_renderer->loadScene(prim_scene, prim_plan, prim_diagnostics);
        const fluent_scene::render::SyntheticFrame prim_frame =
            fluent_scene::render::makeSyntheticFrame(7, 640, 360, 1920, 1080);
        for (int frame = 0; rendered && frame < 2; ++frame) {
            rendered = prim_renderer->renderFrame(prim_frame.inputs(), prim_diagnostics);
        }
        if (!rendered) {
            printDiagnostics(prim_diagnostics);
        }
        check(rendered, "primitives scene renders on the Vulkan backend");
        if (rendered) {
            check(prim_renderer->stats().pipeline_compiles == 5,
                  "five pipelines (image/boxes/circles/polyline/text), all at loadScene");
            std::vector<uint8_t> pixels;
            uint32_t width = 0, height = 0;
            prim_renderer->readback(pixels, width, height);
            const std::string prim_golden = source_dir + "/tests/golden/primitives_demo.ppm";
            if (regen) {
                fluent_scene::render::writePpm(prim_golden, pixels, width, height);
                std::cout << "regenerated primitives golden\n";
            } else {
                std::vector<uint8_t> golden;
                uint32_t golden_w = 0, golden_h = 0;
                double mean_diff = 0.0;
                uint32_t max_diff = 0;
                if (fluent_scene::render::readPpm(prim_golden, golden, golden_w, golden_h) &&
                    golden_w == width && golden_h == height) {
                    fluent_scene::render::diffRgbaVsRgb(pixels, golden, mean_diff, max_diff);
                    std::cout << "     primitives golden diff: mean " << mean_diff << ", max "
                              << max_diff << '\n';
                    check(mean_diff <= 2.0 && max_diff <= 96,
                          "primitives render matches the golden image within tolerance");
                } else {
                    check(false, "primitives golden exists and matches dimensions");
                }
            }
        }
    }

    if (failures == 0) {
        std::cout << "all render tests passed\n";
        return 0;
    }
    std::cout << failures << " render test(s) failed\n";
    return 1;
}
