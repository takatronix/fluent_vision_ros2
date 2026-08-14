// filters_def.h — the single source of truth for filter METADATA: the public
// name, the typed-struct name, and every parameter's name, default, and unit.
// filters.hpp includes this file several times with different definitions of
// the three macros to derive, from this one list:
//
//   - the typed chainable structs   (Bilateral().radius(4).sigma_color(0.2))
//   - their default values
//   - the runtime spec table        (Scene parsing, docs, capability JSON)
//
// Contract with filters_shared.h (the body source):
//   - MODE is a FS_* id declared there;
//   - parameters appear in slot order — FS_PARAM's `slot` must equal its
//     position in the block (0, 1, 2, …) and must match the p0..p4 order the
//     fs_apply dispatch passes to the body.
//
// Units: `Length` parameters are logical units — the renderer multiplies
// them by its logical→source-pixel factor before the body sees them, so
// `blur(4)` looks identical on a 640×480 and a 4K source (§5-3 of the
// design). `Scalar` parameters pass through untouched (ratios, angles,
// normalized 0–1 positions). `CoordX`/`CoordY` are positions in the
// layer's local logical space; the renderer maps them into the filter
// pass's normalized uv (so bodies get a ready-to-use uv center).
//
// No include guard: this file is included multiple times by design.
//
//   FS_FILTER(TypeName, method, MODE, summary_utf8)
//   FS_PARAM(slot, name, default, unit)
//   FS_END(TypeName)

// ---- color and tone --------------------------------------------------------

FS_FILTER(Grayscale, grayscale, FS_GRAYSCALE, "grayscale (luma)")
FS_END(Grayscale)

FS_FILTER(Sepia, sepia, FS_SEPIA, "sepia tone")
FS_PARAM(0, intensity, 1.0f, Scalar)
FS_END(Sepia)

FS_FILTER(Invert, invert, FS_INVERT, "invert (negative)")
FS_END(Invert)

FS_FILTER(Hue, hue, FS_HUE, "hue rotation (degrees)")
FS_PARAM(0, angle, 90.0f, Scalar)
FS_END(Hue)

FS_FILTER(Exposure, exposure, FS_EXPOSURE, "exposure (EV stops)")
FS_PARAM(0, stops, 0.5f, Scalar)
FS_END(Exposure)

FS_FILTER(WhiteBalance, white_balance, FS_WHITE_BALANCE, "white balance: temperature and tint")
FS_PARAM(0, temperature, 0.3f, Scalar)
FS_PARAM(1, tint, 0.0f, Scalar)
FS_END(WhiteBalance)

FS_FILTER(Levels, levels, FS_LEVELS, "levels (input range and gamma)")
FS_PARAM(0, min, 0.0f, Scalar)
FS_PARAM(1, max, 1.0f, Scalar)
FS_PARAM(2, gamma, 1.0f, Scalar)
FS_END(Levels)

FS_FILTER(Threshold, threshold, FS_THRESHOLD, "luma threshold (binarize)")
FS_PARAM(0, threshold, 0.5f, Scalar)
FS_END(Threshold)

FS_FILTER(Posterize, posterize, FS_POSTERIZE, "posterize (reduce tone levels)")
FS_PARAM(0, levels, 6.0f, Scalar)
FS_END(Posterize)

FS_FILTER(Solarize, solarize, FS_SOLARIZE, "solarize")
FS_PARAM(0, threshold, 0.5f, Scalar)
FS_END(Solarize)

FS_FILTER(Rgb, rgb, FS_RGB, "per-channel RGB gain")
FS_PARAM(0, red, 1.0f, Scalar)
FS_PARAM(1, green, 1.0f, Scalar)
FS_PARAM(2, blue, 1.0f, Scalar)
FS_END(Rgb)

FS_FILTER(Haze, haze, FS_HAZE, "haze removal / addition")
FS_PARAM(0, distance, 0.15f, Scalar)
FS_PARAM(1, slope, 0.0f, Scalar)
FS_END(Haze)

FS_FILTER(ColorTransform, color_transform, FS_COLOR_TRANSFORM,
          "brightness, contrast, saturation, gamma")
FS_PARAM(0, brightness, 0.0f, Scalar)
FS_PARAM(1, contrast, 1.0f, Scalar)
FS_PARAM(2, saturation, 1.0f, Scalar)
FS_PARAM(3, gamma, 1.0f, Scalar)
FS_END(ColorTransform)

FS_FILTER(Opacity, opacity, FS_OPACITY, "opacity (prefer the layer attribute)")
FS_PARAM(0, opacity, 0.7f, Scalar)
FS_END(Opacity)

FS_FILTER(Vignette, vignette, FS_VIGNETTE, "vignette")
FS_PARAM(0, start, 0.35f, Scalar)
FS_PARAM(1, end, 0.85f, Scalar)
FS_END(Vignette)

// ---- smoothing and denoise -------------------------------------------------

FS_FILTER(Blur, blur, FS_BLUR, "gaussian blur (separable, two passes)")
FS_PARAM(0, radius, 4.0f, Length)
FS_END(Blur)

FS_FILTER(Bilateral, bilateral, FS_BILATERAL, "bilateral (edge-preserving denoise)")
FS_PARAM(0, radius, 4.0f, Length)
FS_PARAM(1, sigma_color, 0.15f, Scalar)
FS_END(Bilateral)

FS_FILTER(Median, median, FS_MEDIAN, "median 3x3 (salt-and-pepper denoise)")
FS_END(Median)

// ---- edges and stylize -----------------------------------------------------

FS_FILTER(Sharpen, sharpen, FS_SHARPEN, "sharpen")
FS_PARAM(0, strength, 1.0f, Scalar)
FS_END(Sharpen)

FS_FILTER(Emboss, emboss, FS_EMBOSS, "emboss")
FS_PARAM(0, strength, 1.0f, Scalar)
FS_END(Emboss)

FS_FILTER(EdgeSobel, edge_sobel, FS_EDGE_SOBEL, "Sobel edge detection")
FS_PARAM(0, strength, 1.0f, Scalar)
FS_END(EdgeSobel)

FS_FILTER(Sketch, sketch, FS_SKETCH, "sketch (line drawing)")
FS_PARAM(0, strength, 1.0f, Scalar)
FS_END(Sketch)

FS_FILTER(Toon, toon, FS_TOON, "toon (posterize + edges)")
FS_PARAM(0, levels, 6.0f, Scalar)
FS_PARAM(1, edge, 0.2f, Scalar)
FS_END(Toon)

// ---- resampling ------------------------------------------------------------

FS_FILTER(Pixelate, pixelate, FS_PIXELATE, "pixelate (mosaic)")
FS_PARAM(0, size, 12.0f, Length)
FS_END(Pixelate)

FS_FILTER(Swirl, swirl, FS_SWIRL, "swirl distortion")
FS_PARAM(0, radius, 0.5f, Scalar)
FS_PARAM(1, angle, 1.0f, Scalar)
FS_END(Swirl)

FS_FILTER(MotionBlur, motion_blur, FS_MOTION_BLUR, "motion blur")
FS_PARAM(0, angle, 0.0f, Scalar)
FS_PARAM(1, distance, 12.0f, Length)
FS_END(MotionBlur)

FS_FILTER(ZoomBlur, zoom_blur, FS_ZOOM_BLUR, "zoom blur")
FS_PARAM(0, strength, 0.15f, Scalar)
FS_END(ZoomBlur)

FS_FILTER(Halftone, halftone, FS_HALFTONE, "halftone dots")
FS_PARAM(0, spacing, 8.0f, Length)
FS_END(Halftone)

FS_FILTER(RippleWave, ripple, FS_RIPPLE, "water refraction wave (expanding wavefront)")
FS_PARAM(0, center_x, 0.0f, CoordX)
FS_PARAM(1, center_y, 0.0f, CoordY)
FS_PARAM(2, radius, 60.0f, Length)
FS_PARAM(3, amplitude, 6.0f, Length)
FS_PARAM(4, wavelength, 26.0f, Length)
FS_END(RippleWave)
