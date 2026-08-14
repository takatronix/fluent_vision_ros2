#pragma once

/// \file surface.hpp
/// \brief Surface — the offscreen pixel result of rendering a Stage.
///
/// A Surface is a **borrowed view**: the renderer owns the memory and lends
/// it out (§15-5 of the design). The view stays valid until the next
/// `render()` call on the same renderer. Copy the pixels out if you need
/// them longer (e.g. to publish as a ROS image or encode to a file).

#include <cstddef>
#include <cstdint>

namespace fluent_stage {

/// Rendered pixels: RGBA8, straight (non-premultiplied) alpha, sRGB,
/// rows top-to-bottom — the same orientation as every coordinate in the API.
struct Surface {
    uint32_t width = 0;             ///< Width in pixels.
    uint32_t height = 0;            ///< Height in pixels.
    size_t strideBytes = 0;         ///< Bytes per row (>= width * 4).
    const uint8_t* pixels = nullptr;  ///< First byte of the top-left pixel.

    /// True when the view points at actual pixels.
    bool valid() const { return pixels != nullptr && width > 0 && height > 0; }

    /// Pointer to the start of row \p y (0 = top).
    const uint8_t* row(uint32_t y) const { return pixels + static_cast<size_t>(y) * strideBytes; }
};

}  // namespace fluent_stage
