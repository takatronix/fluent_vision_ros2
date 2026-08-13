#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "fluent_scene/diagnostics.hpp"
#include "fluent_scene/planner.hpp"
#include "fluent_scene/validator.hpp"

namespace fluent_scene {

// CPU-side RGBA8 image view handed to the renderer as a frame input.
// Stage 3 will replace direct views with runtime bindings; the renderer
// interface itself is transport-agnostic (spec decision 3).
struct CpuImageView {
    uint32_t width = 0;
    uint32_t height = 0;
    const uint8_t* pixels = nullptr;  // tightly packed, width * height * 4 bytes
};

struct DetectionInstance {
    float bbox[4] = {0, 0, 0, 0};  // x, y, width, height in output pixels
    float score = 0.0f;
    std::string label;
};

// Per-frame external values keyed by scene input name.
struct FrameInputs {
    std::map<std::string, CpuImageView> images;
    std::map<std::string, std::vector<DetectionInstance>> detections;
    std::map<std::string, std::string> strings;
};

// Observable retained-execution counters (spec section 8.2: unchanged
// resources stay retained; stage 2 exit evidence requires reuse counters).
struct RenderStats {
    uint64_t frames = 0;
    uint64_t pipeline_compiles = 0;  // must not grow after loadScene
    uint64_t buffer_uploads = 0;
    uint64_t image_uploads = 0;
    uint64_t atlas_updates = 0;      // grows only when text/font state changes
    uint64_t missing_glyphs = 0;     // deterministic replacement glyph uses
    double last_cpu_ms = 0.0;        // CPU wall time of the last renderFrame
    double last_gpu_ms = 0.0;        // GPU execution time (0 for the CPU backend)
};

struct RendererOptions {
    // Maps allowlisted asset URIs (e.g. "builtin://fonts/default-cjk") to
    // filesystem font paths. Resolution authority stays with the host, never
    // with scene content (spec section 13).
    std::map<std::string, std::string> font_files;
    uint32_t font_pixel_size = 26;
    bool enable_validation = false;  // Vulkan validation layers when available
};

// Narrow retained-renderer interface. Backend-specific code implements this
// beneath the compiler and runtime (spec section 5.1); scenes and plans stay
// backend-neutral.
class Renderer {
public:
    virtual ~Renderer() = default;

    // Compiles pipelines and allocates retained resources for one scene.
    // Structural work happens here, never per frame.
    virtual bool loadScene(const ValidationResult& scene, const PlanResult& plan,
                           DiagnosticList& diagnostics) = 0;

    // Normal-frame path: snapshot inputs, upload changed data, reuse the
    // retained plan, submit, and wait (synchronous MVP).
    virtual bool renderFrame(const FrameInputs& inputs, DiagnosticList& diagnostics) = 0;

    // Copies the exported output image as tightly packed RGBA8.
    virtual bool readback(std::vector<uint8_t>& pixels, uint32_t& width, uint32_t& height) = 0;

    virtual const RenderStats& stats() const = 0;
    virtual const char* name() const = 0;
};

// Vulkan backend (headless offscreen; prefers a discrete GPU, falls back to
// any device with a graphics queue — integrated GPUs and MoltenVK included).
std::unique_ptr<Renderer> createVulkanRenderer(const RendererOptions& options,
                                               DiagnosticList& diagnostics);

// Scalar CPU reference backend implementing the same contract; exists to
// cross-check output and to measure the GPU-versus-CPU gap honestly.
std::unique_ptr<Renderer> createCpuRenderer(const RendererOptions& options,
                                            DiagnosticList& diagnostics);

}  // namespace fluent_scene
