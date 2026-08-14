#pragma once

/// \file vulkan_renderer.hpp
/// \brief VulkanRenderer — the production GPU backend (§11, Phase L1).
///
/// Same contract, same picture: `render(stage, dt)` returns a Surface that
/// matches the CPU reference within the golden tolerance, because both
/// backends walk the tree with the shared plan (src/render_shared.hpp) and
/// compile the very same shape/filter bodies (shared/*.h — as C++ on the
/// CPU, as GLSL→SPIR-V here). Shaders are compiled at build time; at
/// runtime there is zero shader compilation, per frame or otherwise.
///
/// ```cpp
/// std::unique_ptr<Renderer> renderer;
/// try {
///     renderer = std::make_unique<VulkanRenderer>();
/// } catch (const std::exception&) {
///     renderer = std::make_unique<CpuRenderer>();   // headless fallback
/// }
/// const Surface& frame = renderer->render(stage, dt);
/// ```
///
/// Construction throws `std::runtime_error` when no usable Vulkan device
/// exists — catch it and fall back to CpuRenderer.

#include <memory>
#include <string>

#include "fluent_stage/renderer.hpp"

namespace fluent_stage {

class VulkanRenderer final : public Renderer {
public:
    struct Options {
        /// Font file for text content; empty = search well-known system
        /// locations for a CJK-capable font (same rule as CpuRenderer).
        std::string font_file;
        /// Rasterized glyph size in atlas pixels.
        uint32_t font_pixel_size = 26;
    };

    /// Initializes the Vulkan device and builds every pipeline up front.
    /// Throws std::runtime_error when Vulkan is unavailable.
    VulkanRenderer();
    explicit VulkanRenderer(Options options);
    ~VulkanRenderer() override;
    VulkanRenderer(const VulkanRenderer&) = delete;
    VulkanRenderer& operator=(const VulkanRenderer&) = delete;

    const Surface& render(Stage& stage, float dt) override;
    const Surface& render(Stage& stage, uint32_t out_width, uint32_t out_height,
                          float dt) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace fluent_stage
