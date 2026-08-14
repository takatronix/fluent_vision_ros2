#pragma once

/// \file cpu_renderer.hpp
/// \brief CpuRenderer — the reference backend (§11, Phase L0).
///
/// Pure C++, no GPU: the readable, portable definition of what every frame
/// must look like. The Vulkan backend (L1) is validated against it via
/// golden images. It shares the filter and shape bodies with the GPU
/// through shared/filters_shared.h and shared/shapes_shared.h, so "the same
/// math" is literal, not aspirational.
///
/// Compositing model:
/// - plain leaf layers draw straight into the target (zero overhead);
/// - layers with filters, shadows, group opacity, or non-normal group
///   blending render into a transient intermediate buffer first (§11);
/// - rotation/scale evaluate SDFs through the inverse transform, so
///   transformed shapes stay vector-crisp; filtered/shadowed subtrees are
///   composited from their intermediate buffer with bilinear sampling.

#include <memory>
#include <string>

#include "fluent_stage/renderer.hpp"

namespace fluent_stage {

class CpuRenderer final : public Renderer {
public:
    struct Options {
        /// Font file for text content (TTF/TTC/OTF). Empty = search
        /// well-known system locations for a CJK-capable font (Noto CJK).
        std::string font_file;
        /// Rasterized glyph size in atlas pixels. Text at other logical
        /// sizes scales this bitmap; keep it at or above your largest
        /// common on-screen size.
        uint32_t font_pixel_size = 26;
    };

    CpuRenderer();
    explicit CpuRenderer(Options options);
    ~CpuRenderer() override;
    CpuRenderer(const CpuRenderer&) = delete;
    CpuRenderer& operator=(const CpuRenderer&) = delete;

    const Surface& render(Stage& stage, float dt) override;
    const Surface& render(Stage& stage, uint32_t out_width, uint32_t out_height,
                          float dt) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace fluent_stage
