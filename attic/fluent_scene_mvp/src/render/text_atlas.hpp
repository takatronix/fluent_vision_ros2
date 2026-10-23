#pragma once

// Internal: glyph atlas shared by the Vulkan and CPU backends.
// FreeType rasterizes glyphs, HarfBuzz shapes UTF-8 (Japanese included) into
// positioned glyphs. The atlas uses fixed 32x32 R8 cells, 32 cells per row,
// so its byte size is exactly glyph_capacity * 1024 — the same worst-case
// formula the stage-1 planner charges against the GPU budget.

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "fluent_scene/diagnostics.hpp"

namespace fluent_scene {
namespace render {

class TextAtlas {
public:
    struct Options {
        std::string font_file;
        uint32_t pixel_size = 26;      // must fit the 32px cell
        uint32_t glyph_capacity = 2048;
    };

    struct GlyphQuad {
        float x = 0, y = 0, w = 0, h = 0;      // destination pixels
        float u0 = 0, v0 = 0, u1 = 0, v1 = 0;  // normalized atlas coords
    };

    TextAtlas() = default;
    ~TextAtlas();
    TextAtlas(const TextAtlas&) = delete;
    TextAtlas& operator=(const TextAtlas&) = delete;

    bool init(const Options& options, DiagnosticList& diagnostics);

    // Shapes `utf8` and appends up to `max_glyphs` quads positioned from the
    // top-left corner (x, y). Glyphs the font lacks use a deterministic
    // replacement box and increment missingGlyphs(). Truncation beyond
    // max_glyphs follows the declared truncate_end rule.
    size_t layout(const std::string& utf8, float x, float y, uint64_t max_glyphs,
                  std::vector<GlyphQuad>& out);

    const uint8_t* pixels() const { return pixels_.data(); }
    uint32_t width() const { return width_; }
    uint32_t height() const { return height_; }
    // Bumps whenever atlas pixel content changed (new glyph rasterized).
    uint64_t revision() const { return revision_; }
    uint64_t missingGlyphs() const { return missing_glyphs_; }
    float lineHeight() const { return line_height_; }

    static constexpr uint32_t kCell = 32;
    static constexpr uint32_t kCellsPerRow = 32;

private:
    struct Cell {
        uint32_t index = 0;          // linear cell index
        float bearing_x = 0;
        float bearing_y = 0;         // distance from baseline up to bitmap top
        uint32_t bitmap_w = 0;
        uint32_t bitmap_h = 0;
    };

    bool ensureGlyph(uint32_t glyph_index, Cell& out);
    void blitToCell(uint32_t cell_index, const uint8_t* src, uint32_t src_w, uint32_t src_h,
                    int src_pitch);
    void makeReplacementBitmap(std::vector<uint8_t>& bitmap, uint32_t& w, uint32_t& h) const;

    void* ft_library_ = nullptr;   // FT_Library
    void* ft_face_ = nullptr;      // FT_Face
    void* hb_font_ = nullptr;      // hb_font_t*

    std::vector<uint8_t> pixels_;
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    uint32_t capacity_ = 0;
    uint32_t next_cell_ = 0;
    uint64_t revision_ = 0;
    uint64_t missing_glyphs_ = 0;
    float ascender_ = 0;
    float line_height_ = 0;
    float replacement_advance_ = 0;
    std::map<uint32_t, Cell> cells_;  // FreeType glyph index -> cell
    bool has_replacement_ = false;
    Cell replacement_cell_;
};

}  // namespace render
}  // namespace fluent_scene
