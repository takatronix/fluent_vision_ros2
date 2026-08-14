// Scalar CPU reference backend. Implements the same narrow renderer contract
// as the Vulkan backend so output can be cross-checked and the GPU-versus-CPU
// gap can be measured honestly on the same scene and inputs.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "fluent_scene/render/renderer.hpp"
#include "render/box_smoother.hpp"
#include "render/scene_model.hpp"
#include "render/text_atlas.hpp"

namespace fluent_scene {
namespace {

constexpr float kBoxOutlineThickness = 3.0f;
constexpr float kShadowOffset = 1.5f;

class CpuRenderer final : public Renderer {
public:
    explicit CpuRenderer(const RendererOptions& options) : options_(options) {}

    bool loadScene(const ValidationResult& scene, const PlanResult& plan,
                   DiagnosticList& diagnostics) override {
        if (loaded_) {
            diagnostics.add("compile.invalid_input", Severity::kError, Phase::kCompile, Span{},
                            "renderer already has a loaded scene");
            return false;
        }
        if (!render::buildSceneModel(scene, plan, model_, diagnostics)) {
            return false;
        }
        framebuffer_.assign(static_cast<size_t>(model_.width) * model_.height * 4, 0);
        for (const render::DrawOp& draw : model_.draws) {
            if (draw.kind == render::DrawOp::Kind::kText && atlases_.count(draw.font_uri) == 0) {
                auto atlas = std::make_unique<render::TextAtlas>();
                render::TextAtlas::Options atlas_options;
                atlas_options.font_file = resolveFont(draw.font_uri);
                atlas_options.pixel_size = options_.font_pixel_size;
                atlas_options.glyph_capacity = static_cast<uint32_t>(
                    std::max<uint64_t>(1, std::min<uint64_t>(draw.glyph_capacity, 65536)));
                if (!atlas->init(atlas_options, diagnostics)) {
                    return false;
                }
                atlases_.emplace(draw.font_uri, std::move(atlas));
            }
        }
        stats_.pipeline_compiles = 0;  // nothing to compile on the CPU path
        runtime_params_ = model_.mutable_params;
        loaded_ = true;
        return true;
    }

    bool renderFrame(const FrameInputs& inputs, DiagnosticList& diagnostics) override {
        if (!loaded_) {
            diagnostics.add("compile.invalid_input", Severity::kError, Phase::kCompile, Span{},
                            "renderFrame called before loadScene");
            return false;
        }
        const auto cpu_start = std::chrono::steady_clock::now();
        if (has_last_frame_time_) {
            frame_dt_ = std::clamp(
                std::chrono::duration<double>(cpu_start - last_frame_time_).count(), 0.001, 0.1);
        }
        last_frame_time_ = cpu_start;
        has_last_frame_time_ = true;
        std::fill(framebuffer_.begin(), framebuffer_.end(), 0);
        // Opaque black background (alpha 255) to match the GPU clear.
        for (size_t i = 3; i < framebuffer_.size(); i += 4) {
            framebuffer_[i] = 0xff;
        }
        for (const render::DrawOp& op : model_.draws) {
            switch (op.kind) {
                case render::DrawOp::Kind::kImage: {
                    auto it = inputs.images.find(op.source_input);
                    if (it != inputs.images.end() && it->second.pixels != nullptr) {
                        CpuImageView view = it->second;
                        for (const std::string& effect_id : op.effect_chain) {
                            view = applyEffect(effect_id, view);
                        }
                        drawImage(view, op.fit);
                    }
                    break;
                }
                case render::DrawOp::Kind::kBoxes: {
                    auto it = inputs.detections.find(op.source_input);
                    if (it == inputs.detections.end()) {
                        break;
                    }
                    std::vector<DetectionInstance> detections = it->second;
                    if (detections.size() > op.max_instances) {
                        if (op.overflow == "drop_lowest_score") {
                            std::sort(detections.begin(), detections.end(),
                                      [](const DetectionInstance& a, const DetectionInstance& b) {
                                          return a.score > b.score;
                                      });
                            detections.resize(op.max_instances);
                        } else {
                            detections.erase(detections.begin(),
                                             detections.end() -
                                                 static_cast<ptrdiff_t>(op.max_instances));
                        }
                    }
                    if (op.smoothing > 0.0f) {
                        detections = smoothers_[op.node_id].update(std::move(detections), frame_dt_,
                                                                   op.smoothing, op.max_instances);
                    }
                    for (const DetectionInstance& detection : detections) {
                        drawBoxOutline(detection.bbox, op.color);
                    }
                    ++stats_.buffer_uploads;
                    break;
                }
                case render::DrawOp::Kind::kCircles:
                case render::DrawOp::Kind::kPolyline: {
                    auto it = inputs.points.find(op.source_input);
                    if (it == inputs.points.end()) {
                        break;
                    }
                    std::vector<Point2f> points = it->second;
                    if (points.size() > op.max_points) {
                        points.resize(op.max_points);  // declared truncate_end rule
                    }
                    if (op.kind == render::DrawOp::Kind::kCircles) {
                        for (const Point2f& point : points) {
                            drawCircle(point, op.radius, op.thickness, op.color);
                        }
                    } else {
                        for (size_t s = 0; s + 1 < points.size(); ++s) {
                            drawSegment(points[s], points[s + 1], op.thickness, op.color);
                        }
                    }
                    ++stats_.buffer_uploads;
                    break;
                }
                case render::DrawOp::Kind::kText: {
                    auto it = inputs.strings.find(op.source_input);
                    const std::string text =
                        it != inputs.strings.end() ? it->second : op.default_text;
                    render::TextAtlas& atlas = *atlases_.at(op.font_uri);
                    std::vector<render::TextAtlas::GlyphQuad> quads;
                    atlas.layout(text, op.position[0], op.position[1], op.max_glyphs, quads);
                    if (op.shadow) {
                        const float shadow_color[4] = {0.0f, 0.0f, 0.0f, 0.7f};
                        for (const auto& quad : quads) {
                            drawGlyph(atlas, quad, shadow_color, kShadowOffset, kShadowOffset);
                        }
                    }
                    for (const auto& quad : quads) {
                        drawGlyph(atlas, quad, op.color, 0.0f, 0.0f);
                    }
                    stats_.missing_glyphs = atlas.missingGlyphs();
                    break;
                }
            }
        }
        ++stats_.frames;
        stats_.last_cpu_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cpu_start)
                .count();
        stats_.last_gpu_ms = 0.0;
        return true;
    }

    bool readback(std::vector<uint8_t>& pixels, uint32_t& width, uint32_t& height) override {
        if (!loaded_) {
            return false;
        }
        pixels = framebuffer_;
        width = model_.width;
        height = model_.height;
        return true;
    }

    const RenderStats& stats() const override { return stats_; }
    const char* name() const override { return "cpu"; }

    bool setParam(const std::string& name, float value) override {
        auto it = runtime_params_.find(name);
        if (it == runtime_params_.end()) {
            return false;
        }
        it->second = value;
        return true;
    }

private:
    float effectValue(float literal, const std::string& param_ref) const {
        if (param_ref.empty()) {
            return literal;
        }
        auto it = runtime_params_.find(param_ref);
        return it != runtime_params_.end() ? it->second : literal;
    }

    void blendPixel(int x, int y, const float* rgba, float coverage) {
        if (x < 0 || y < 0 || x >= static_cast<int>(model_.width) ||
            y >= static_cast<int>(model_.height)) {
            return;
        }
        const float alpha = rgba[3] * coverage;
        if (alpha <= 0.0f) {
            return;
        }
        uint8_t* pixel =
            framebuffer_.data() + (static_cast<size_t>(y) * model_.width + x) * 4;
        for (int c = 0; c < 3; ++c) {
            const float src = rgba[c] * 255.0f;
            pixel[c] = static_cast<uint8_t>(std::lround(src * alpha + pixel[c] * (1.0f - alpha)));
        }
        pixel[3] = 0xff;
    }

    void drawImage(const CpuImageView& view, const std::string& fit) {
        const float target_w = static_cast<float>(model_.width);
        const float target_h = static_cast<float>(model_.height);
        const float source_w = static_cast<float>(view.width);
        const float source_h = static_cast<float>(view.height);
        float scale_x = target_w / source_w;
        float scale_y = target_h / source_h;
        if (fit == "contain") {
            scale_x = scale_y = std::min(scale_x, scale_y);
        } else if (fit == "cover") {
            scale_x = scale_y = std::max(scale_x, scale_y);
        }
        const float dst_w = source_w * scale_x;
        const float dst_h = source_h * scale_y;
        const float dst_x = (target_w - dst_w) * 0.5f;
        const float dst_y = (target_h - dst_h) * 0.5f;
        const int x0 = std::max(0, static_cast<int>(std::floor(dst_x)));
        const int y0 = std::max(0, static_cast<int>(std::floor(dst_y)));
        const int x1 = std::min(static_cast<int>(model_.width),
                                static_cast<int>(std::ceil(dst_x + dst_w)));
        const int y1 = std::min(static_cast<int>(model_.height),
                                static_cast<int>(std::ceil(dst_y + dst_h)));
        for (int y = y0; y < y1; ++y) {
            const float v = (static_cast<float>(y) + 0.5f - dst_y) / dst_h;
            const float src_y = std::clamp(v * source_h - 0.5f, 0.0f, source_h - 1.0f);
            const int sy0 = static_cast<int>(src_y);
            const int sy1 = std::min(sy0 + 1, static_cast<int>(view.height) - 1);
            const float fy = src_y - static_cast<float>(sy0);
            uint8_t* dst_row =
                framebuffer_.data() + (static_cast<size_t>(y) * model_.width + x0) * 4;
            for (int x = x0; x < x1; ++x) {
                const float u = (static_cast<float>(x) + 0.5f - dst_x) / dst_w;
                const float src_x = std::clamp(u * source_w - 0.5f, 0.0f, source_w - 1.0f);
                const int sx0 = static_cast<int>(src_x);
                const int sx1 = std::min(sx0 + 1, static_cast<int>(view.width) - 1);
                const float fx = src_x - static_cast<float>(sx0);
                const uint8_t* p00 = view.pixels + (static_cast<size_t>(sy0) * view.width + sx0) * 4;
                const uint8_t* p10 = view.pixels + (static_cast<size_t>(sy0) * view.width + sx1) * 4;
                const uint8_t* p01 = view.pixels + (static_cast<size_t>(sy1) * view.width + sx0) * 4;
                const uint8_t* p11 = view.pixels + (static_cast<size_t>(sy1) * view.width + sx1) * 4;
                for (int c = 0; c < 3; ++c) {
                    const float top = p00[c] * (1.0f - fx) + p10[c] * fx;
                    const float bottom = p01[c] * (1.0f - fx) + p11[c] * fx;
                    dst_row[c] = static_cast<uint8_t>(std::lround(top * (1.0f - fy) + bottom * fy));
                }
                dst_row[3] = 0xff;
                dst_row += 4;
            }
        }
    }

    void drawBoxOutline(const float* bbox, const float* color) {
        const int x0 = static_cast<int>(std::lround(bbox[0]));
        const int y0 = static_cast<int>(std::lround(bbox[1]));
        const int w = static_cast<int>(std::lround(bbox[2]));
        const int h = static_cast<int>(std::lround(bbox[3]));
        const int thickness = static_cast<int>(kBoxOutlineThickness);
        for (int y = y0; y < y0 + h; ++y) {
            for (int x = x0; x < x0 + w; ++x) {
                const int dx = std::min(x - x0, x0 + w - 1 - x);
                const int dy = std::min(y - y0, y0 + h - 1 - y);
                if (std::min(dx, dy) <= thickness) {
                    blendPixel(x, y, color, 1.0f);
                }
            }
        }
    }

    // Image-space effects, mirroring the GPU shaders (blur.frag / color.frag).
    CpuImageView applyEffect(const std::string& effect_id, const CpuImageView& source) {
        const render::EffectOp* op = nullptr;
        for (const render::EffectOp& candidate : model_.effects) {
            if (candidate.node_id == effect_id) {
                op = &candidate;
                break;
            }
        }
        if (op == nullptr || source.pixels == nullptr) {
            return source;
        }
        std::vector<uint8_t>& out = effect_pixels_[effect_id];
        out.resize(static_cast<size_t>(source.width) * source.height * 4);
        if (op->kind == render::EffectOp::Kind::kColorTransform) {
            const float brightness = effectValue(op->brightness, op->brightness_param);
            const float contrast = effectValue(op->contrast, op->contrast_param);
            const float saturation = effectValue(op->saturation, op->saturation_param);
            const float gamma = std::max(effectValue(op->gamma, op->gamma_param), 0.05f);
            const size_t count = static_cast<size_t>(source.width) * source.height;
            for (size_t i = 0; i < count; ++i) {
                const uint8_t* src = source.pixels + i * 4;
                uint8_t* dst = out.data() + i * 4;
                float c[3];
                for (int ch = 0; ch < 3; ++ch) {
                    c[ch] = (static_cast<float>(src[ch]) / 255.0f - 0.5f) * contrast + 0.5f +
                            brightness;
                }
                const float luma = 0.2126f * c[0] + 0.7152f * c[1] + 0.0722f * c[2];
                for (int ch = 0; ch < 3; ++ch) {
                    float v = luma + (c[ch] - luma) * saturation;
                    v = std::pow(std::clamp(v, 0.0f, 1.0f), 1.0f / gamma);
                    dst[ch] = static_cast<uint8_t>(std::lround(v * 255.0f));
                }
                dst[3] = 0xff;
            }
        } else {
            const float radius =
                std::clamp(effectValue(op->radius, op->radius_param), 0.5f, 16.0f);
            const float sigma = std::max(radius * 0.5f, 0.3f);
            const float two_sigma2 = 2.0f * sigma * sigma;
            const float step = std::max(radius / 6.0f, 0.001f);
            float weights[13];
            float weight_sum = 0.0f;
            for (int i = -6; i <= 6; ++i) {
                const float offset = static_cast<float>(i) * step;
                weights[i + 6] = std::exp(-(offset * offset) / two_sigma2);
                weight_sum += weights[i + 6];
            }
            std::vector<uint8_t>& tmp = effect_tmp_;
            tmp.resize(out.size());
            const int w = static_cast<int>(source.width);
            const int h = static_cast<int>(source.height);
            const auto pass = [&](const uint8_t* src, uint8_t* dst, bool horizontal) {
                for (int y = 0; y < h; ++y) {
                    for (int x = 0; x < w; ++x) {
                        float sum[3] = {0, 0, 0};
                        for (int i = -6; i <= 6; ++i) {
                            const int offset = static_cast<int>(std::lround(i * step));
                            const int sx = std::clamp(horizontal ? x + offset : x, 0, w - 1);
                            const int sy = std::clamp(horizontal ? y : y + offset, 0, h - 1);
                            const uint8_t* pixel = src + (static_cast<size_t>(sy) * w + sx) * 4;
                            const float weight = weights[i + 6];
                            sum[0] += pixel[0] * weight;
                            sum[1] += pixel[1] * weight;
                            sum[2] += pixel[2] * weight;
                        }
                        uint8_t* pixel = dst + (static_cast<size_t>(y) * w + x) * 4;
                        for (int ch = 0; ch < 3; ++ch) {
                            pixel[ch] = static_cast<uint8_t>(std::lround(sum[ch] / weight_sum));
                        }
                        pixel[3] = 0xff;
                    }
                }
            };
            pass(source.pixels, tmp.data(), /*horizontal=*/true);
            pass(tmp.data(), out.data(), /*horizontal=*/false);
        }
        return CpuImageView{source.width, source.height, out.data()};
    }

    // Same SDF + smoothstep rules as the GPU shaders so both backends agree.
    static float smoothCoverage(float distance) {
        const float t = std::clamp((distance + 0.75f) / 1.5f, 0.0f, 1.0f);
        return 1.0f - t * t * (3.0f - 2.0f * t);
    }

    void drawCircle(const Point2f& center, float radius, float thickness, const float* color) {
        const float reach = radius + thickness * 0.5f + 1.5f;
        const int x0 = static_cast<int>(std::floor(center.x - reach));
        const int x1 = static_cast<int>(std::ceil(center.x + reach));
        const int y0 = static_cast<int>(std::floor(center.y - reach));
        const int y1 = static_cast<int>(std::ceil(center.y + reach));
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                const float dx = static_cast<float>(x) + 0.5f - center.x;
                const float dy = static_cast<float>(y) + 0.5f - center.y;
                float d = std::sqrt(dx * dx + dy * dy) - radius;
                if (thickness > 0.0f) {
                    d = std::fabs(d) - thickness * 0.5f;
                }
                const float coverage = smoothCoverage(d);
                if (coverage > 0.0f) {
                    blendPixel(x, y, color, coverage);
                }
            }
        }
    }

    void drawSegment(const Point2f& a, const Point2f& b, float thickness, const float* color) {
        const float reach = thickness * 0.5f + 1.5f;
        const int x0 = static_cast<int>(std::floor(std::min(a.x, b.x) - reach));
        const int x1 = static_cast<int>(std::ceil(std::max(a.x, b.x) + reach));
        const int y0 = static_cast<int>(std::floor(std::min(a.y, b.y) - reach));
        const int y1 = static_cast<int>(std::ceil(std::max(a.y, b.y) + reach));
        const float bax = b.x - a.x;
        const float bay = b.y - a.y;
        const float len2 = std::max(bax * bax + bay * bay, 1e-6f);
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                const float px = static_cast<float>(x) + 0.5f - a.x;
                const float py = static_cast<float>(y) + 0.5f - a.y;
                const float h = std::clamp((px * bax + py * bay) / len2, 0.0f, 1.0f);
                const float dx = px - bax * h;
                const float dy = py - bay * h;
                const float d = std::sqrt(dx * dx + dy * dy) - thickness * 0.5f;
                const float coverage = smoothCoverage(d);
                if (coverage > 0.0f) {
                    blendPixel(x, y, color, coverage);
                }
            }
        }
    }

    void drawGlyph(const render::TextAtlas& atlas, const render::TextAtlas::GlyphQuad& quad,
                   const float* color, float offset_x, float offset_y) {
        const int w = static_cast<int>(quad.w);
        const int h = static_cast<int>(quad.h);
        const int atlas_x = static_cast<int>(quad.u0 * static_cast<float>(atlas.width()) + 0.5f);
        const int atlas_y = static_cast<int>(quad.v0 * static_cast<float>(atlas.height()) + 0.5f);
        const int dst_x = static_cast<int>(std::lround(quad.x + offset_x));
        const int dst_y = static_cast<int>(std::lround(quad.y + offset_y));
        for (int y = 0; y < h; ++y) {
            const uint8_t* src_row =
                atlas.pixels() + static_cast<size_t>(atlas_y + y) * atlas.width() + atlas_x;
            for (int x = 0; x < w; ++x) {
                const float coverage = static_cast<float>(src_row[x]) / 255.0f;
                if (coverage > 0.0f) {
                    blendPixel(dst_x + x, dst_y + y, color, coverage);
                }
            }
        }
    }

    std::string resolveFont(const std::string& uri) const {
        auto it = options_.font_files.find(uri);
        if (it != options_.font_files.end()) {
            return it->second;
        }
        static const char* kCandidates[] = {
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        };
        for (const char* candidate : kCandidates) {
            std::FILE* file = std::fopen(candidate, "rb");
            if (file != nullptr) {
                std::fclose(file);
                return candidate;
            }
        }
        return {};
    }

    RendererOptions options_;
    render::SceneModel model_;
    RenderStats stats_;
    bool loaded_ = false;
    std::vector<uint8_t> framebuffer_;
    std::map<std::string, std::unique_ptr<render::TextAtlas>> atlases_;
    std::map<std::string, render::BoxSmoother> smoothers_;
    std::map<std::string, std::vector<uint8_t>> effect_pixels_;
    std::vector<uint8_t> effect_tmp_;
    std::map<std::string, float> runtime_params_;
    std::chrono::steady_clock::time_point last_frame_time_;
    bool has_last_frame_time_ = false;
    double frame_dt_ = 1.0 / 30.0;
};

}  // namespace

std::unique_ptr<Renderer> createCpuRenderer(const RendererOptions& options,
                                            DiagnosticList& diagnostics) {
    (void)diagnostics;
    return std::make_unique<CpuRenderer>(options);
}

}  // namespace fluent_scene
