#include "text_atlas.hpp"

#include <algorithm>
#include <cstring>

#include <ft2build.h>
#include FT_FREETYPE_H

#include <hb-ft.h>
#include <hb.h>

namespace fluent_stage {
namespace detail {

namespace {

FT_Library ftLib(void* p) { return static_cast<FT_Library>(p); }
FT_Face ftFace(void* p) { return static_cast<FT_Face>(p); }
hb_font_t* hbFont(void* p) { return static_cast<hb_font_t*>(p); }

}  // namespace

TextAtlas::~TextAtlas() {
    if (hb_font_ != nullptr) {
        hb_font_destroy(hbFont(hb_font_));
    }
    if (ft_face_ != nullptr) {
        FT_Done_Face(ftFace(ft_face_));
    }
    if (ft_library_ != nullptr) {
        FT_Done_FreeType(ftLib(ft_library_));
    }
}

bool TextAtlas::init(const Options& options, std::string& error) {
    const auto fail = [&](const std::string& message) {
        error = message;
        return false;
    };
    FT_Library library = nullptr;
    if (FT_Init_FreeType(&library) != 0) {
        return fail("FreeType initialization failed");
    }
    ft_library_ = library;

    // Font collections (.ttc) carry several faces; prefer a Japanese one.
    FT_Face face = nullptr;
    if (FT_New_Face(library, options.font_file.c_str(), 0, &face) != 0) {
        return fail("cannot open font file " + options.font_file);
    }
    const FT_Long num_faces = face->num_faces;
    for (FT_Long index = 1; index < num_faces; ++index) {
        if (face->family_name != nullptr &&
            std::string(face->family_name).find("JP") != std::string::npos) {
            break;
        }
        FT_Face candidate = nullptr;
        if (FT_New_Face(library, options.font_file.c_str(), index, &candidate) != 0) {
            break;
        }
        FT_Done_Face(face);
        face = candidate;
    }
    ft_face_ = face;
    pixel_size_ = std::min<uint32_t>(std::max<uint32_t>(options.pixel_size, 8), kCell);
    if (FT_Set_Pixel_Sizes(face, 0, pixel_size_) != 0) {
        return fail("cannot set pixel size on font " + options.font_file);
    }
    hb_font_ = hb_ft_font_create_referenced(face);
    if (hb_font_ == nullptr) {
        return fail("HarfBuzz font creation failed");
    }

    capacity_ = std::max<uint32_t>(1, options.glyph_capacity);
    width_ = kCellsPerRow * kCell;
    const uint32_t rows = (capacity_ + kCellsPerRow - 1) / kCellsPerRow;
    height_ = rows * kCell;
    pixels_.assign(static_cast<size_t>(width_) * height_, 0);
    revision_ = 1;

    ascender_ = static_cast<float>(face->size->metrics.ascender) / 64.0f;
    line_height_ = static_cast<float>(face->size->metrics.height) / 64.0f;
    replacement_advance_ = static_cast<float>(pixel_size_) * 0.75f;
    return true;
}

void TextAtlas::makeReplacementBitmap(std::vector<uint8_t>& bitmap, uint32_t& w,
                                      uint32_t& h) const {
    // Deterministic hollow box: missing glyphs are visible, identical
    // everywhere, and reported through missingGlyphs().
    w = std::min<uint32_t>(kCell, static_cast<uint32_t>(replacement_advance_) - 2);
    h = std::min<uint32_t>(kCell, static_cast<uint32_t>(ascender_));
    bitmap.assign(static_cast<size_t>(w) * h, 0);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            const bool border = x < 2 || y < 2 || x >= w - 2 || y >= h - 2;
            if (border) {
                bitmap[static_cast<size_t>(y) * w + x] = 0xff;
            }
        }
    }
}

void TextAtlas::blitToCell(uint32_t cell_index, const uint8_t* src, uint32_t src_w,
                           uint32_t src_h, int src_pitch) {
    const uint32_t cell_x = (cell_index % kCellsPerRow) * kCell;
    const uint32_t cell_y = (cell_index / kCellsPerRow) * kCell;
    const uint32_t copy_w = std::min(src_w, kCell);
    const uint32_t copy_h = std::min(src_h, kCell);
    for (uint32_t y = 0; y < copy_h; ++y) {
        const uint8_t* src_row = src + static_cast<ptrdiff_t>(y) * src_pitch;
        uint8_t* dst_row = pixels_.data() + (static_cast<size_t>(cell_y + y) * width_) + cell_x;
        std::memcpy(dst_row, src_row, copy_w);
    }
    ++revision_;
}

bool TextAtlas::ensureGlyph(uint32_t glyph_index, Cell& out) {
    auto it = cells_.find(glyph_index);
    if (it != cells_.end()) {
        out = it->second;
        return true;
    }
    FT_Face face = ftFace(ft_face_);
    if (glyph_index == 0) {
        if (!has_replacement_) {
            std::vector<uint8_t> bitmap;
            uint32_t w = 0, h = 0;
            makeReplacementBitmap(bitmap, w, h);
            replacement_cell_.index = next_cell_ < capacity_ ? next_cell_++ : capacity_ - 1;
            replacement_cell_.bearing_x = 1.0f;
            replacement_cell_.bearing_y = static_cast<float>(h);
            replacement_cell_.bitmap_w = w;
            replacement_cell_.bitmap_h = h;
            blitToCell(replacement_cell_.index, bitmap.data(), w, h, static_cast<int>(w));
            has_replacement_ = true;
        }
        out = replacement_cell_;
        return true;
    }
    if (next_cell_ >= capacity_) {
        // Atlas is full; fall back to the replacement glyph deterministically.
        return ensureGlyph(0, out);
    }
    if (FT_Load_Glyph(face, glyph_index, FT_LOAD_RENDER) != 0) {
        return ensureGlyph(0, out);
    }
    const FT_GlyphSlot slot = face->glyph;
    Cell cell;
    cell.index = next_cell_++;
    cell.bearing_x = static_cast<float>(slot->bitmap_left);
    cell.bearing_y = static_cast<float>(slot->bitmap_top);
    cell.bitmap_w = std::min<uint32_t>(slot->bitmap.width, kCell);
    cell.bitmap_h = std::min<uint32_t>(slot->bitmap.rows, kCell);
    if (slot->bitmap.buffer != nullptr && slot->bitmap.width > 0) {
        blitToCell(cell.index, slot->bitmap.buffer, slot->bitmap.width, slot->bitmap.rows,
                   slot->bitmap.pitch);
    }
    cells_.emplace(glyph_index, cell);
    out = cell;
    return true;
}

size_t TextAtlas::layout(const std::string& utf8, float x, float y, uint64_t max_glyphs,
                         std::vector<GlyphQuad>& out) {
    if (utf8.empty() || max_glyphs == 0 || !ready()) {
        return 0;
    }
    hb_buffer_t* buffer = hb_buffer_create();
    hb_buffer_add_utf8(buffer, utf8.c_str(), -1, 0, -1);
    hb_buffer_guess_segment_properties(buffer);
    hb_shape(hbFont(hb_font_), buffer, nullptr, 0);

    unsigned int glyph_count = 0;
    const hb_glyph_info_t* infos = hb_buffer_get_glyph_infos(buffer, &glyph_count);
    const hb_glyph_position_t* positions = hb_buffer_get_glyph_positions(buffer, &glyph_count);

    const float baseline = y + ascender_;
    float pen_x = x;
    size_t emitted = 0;
    for (unsigned int i = 0; i < glyph_count && emitted < max_glyphs; ++i) {
        const uint32_t glyph_index = infos[i].codepoint;  // post-shaping: glyph id
        if (glyph_index == 0) {
            ++missing_glyphs_;
        }
        Cell cell;
        if (!ensureGlyph(glyph_index, cell)) {
            continue;
        }
        const float offset_x = static_cast<float>(positions[i].x_offset) / 64.0f;
        const float offset_y = static_cast<float>(positions[i].y_offset) / 64.0f;
        float advance = static_cast<float>(positions[i].x_advance) / 64.0f;
        if (glyph_index == 0 && advance <= 0.0f) {
            advance = replacement_advance_;
        }
        if (cell.bitmap_w > 0 && cell.bitmap_h > 0) {
            GlyphQuad quad;
            quad.x = pen_x + offset_x + cell.bearing_x;
            quad.y = baseline - offset_y - cell.bearing_y;
            quad.w = static_cast<float>(cell.bitmap_w);
            quad.h = static_cast<float>(cell.bitmap_h);
            const float cell_u = static_cast<float>((cell.index % kCellsPerRow) * kCell);
            const float cell_v = static_cast<float>((cell.index / kCellsPerRow) * kCell);
            quad.u0 = cell_u / static_cast<float>(width_);
            quad.v0 = cell_v / static_cast<float>(height_);
            quad.u1 = (cell_u + quad.w) / static_cast<float>(width_);
            quad.v1 = (cell_v + quad.h) / static_cast<float>(height_);
            out.push_back(quad);
            ++emitted;
        }
        pen_x += advance;
    }
    hb_buffer_destroy(buffer);
    return emitted;
}

float TextAtlas::measure(const std::string& utf8) const {
    if (utf8.empty() || !ready()) {
        return 0;
    }
    hb_buffer_t* buffer = hb_buffer_create();
    hb_buffer_add_utf8(buffer, utf8.c_str(), -1, 0, -1);
    hb_buffer_guess_segment_properties(buffer);
    hb_shape(hbFont(hb_font_), buffer, nullptr, 0);

    unsigned int glyph_count = 0;
    const hb_glyph_info_t* infos = hb_buffer_get_glyph_infos(buffer, &glyph_count);
    const hb_glyph_position_t* positions = hb_buffer_get_glyph_positions(buffer, &glyph_count);
    float width = 0;
    for (unsigned int i = 0; i < glyph_count; ++i) {
        float advance = static_cast<float>(positions[i].x_advance) / 64.0f;
        if (infos[i].codepoint == 0 && advance <= 0.0f) {
            advance = replacement_advance_;
        }
        width += advance;
    }
    hb_buffer_destroy(buffer);
    return width;
}

bool initAtlasWithFallback(TextAtlas& atlas, const std::string& font_file,
                           uint32_t pixel_size) {
    static const char* const kFontSearch[] = {
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJKjp-Regular.otf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    };
    TextAtlas::Options options;
    options.pixel_size = pixel_size;
    std::string error;
    if (!font_file.empty()) {
        options.font_file = font_file;
        if (atlas.init(options, error)) {
            return true;
        }
        std::fprintf(stderr, "fluent_stage: %s — text will not render\n", error.c_str());
        return false;
    }
    for (const char* path : kFontSearch) {
        options.font_file = path;
        if (atlas.init(options, error)) {
            return true;
        }
    }
    std::fprintf(stderr,
                 "fluent_stage: no usable font found — text will not render "
                 "(set Options::font_file)\n");
    return false;
}

}  // namespace detail
}  // namespace fluent_stage
