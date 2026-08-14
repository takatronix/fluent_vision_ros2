// filters_tour — every filter in the catalog, applied to the same image,
// labeled from the same single-source metadata that defines it.
//
// The tile grid is generated from filterTable() (shared/filters_def.h), so
// this example can never drift from the real filter list: add a filter to
// the definition file and it appears here, in the typed API, and in the
// spec table at once.

#include <cstdio>
#include <vector>

#include <fluent_stage/fluent_stage.hpp>

using namespace fluent_stage;

int main() {
    // Test card: gradients + checkers stress color and kernel filters alike.
    const uint32_t img_w = 150, img_h = 100;
    std::vector<uint8_t> img(img_w * img_h * 4);
    for (uint32_t y = 0; y < img_h; ++y) {
        for (uint32_t x = 0; x < img_w; ++x) {
            uint8_t* p = &img[(y * img_w + x) * 4];
            p[0] = static_cast<uint8_t>(x * 255 / img_w);
            p[1] = static_cast<uint8_t>(y * 255 / img_h);
            p[2] = static_cast<uint8_t>(((x / 12) + (y / 12)) % 2 ? 220 : 40);
            p[3] = 255;
        }
    }
    const ImageView view{img_w, img_h, img.data(), 0};

    const auto& table = filterTable();
    const int cols = 6;
    const int rows = (static_cast<int>(table.size()) + 1 + cols - 1) / cols;
    const float tile_w = 160, tile_h = 130;
    Stage stage(cols * tile_w, rows * tile_h);

    // Tile 0: the unfiltered original.
    stage.image(view).frame({5, 5, tile_w - 10, tile_h - 30});
    stage.text("original", {5, tile_h - 24}).size(14).color(Color::Teal);

    for (size_t i = 0; i < table.size(); ++i) {
        const float x = static_cast<float>(((i + 1) % cols)) * tile_w;
        const float y = static_cast<float>(((i + 1) / cols)) * tile_h;
        Filter f{table[i].mode, {}};
        for (size_t p = 0; p < table[i].params.size() && p < 5; ++p) {
            f.values[p] = table[i].params[p].default_value;
        }
        stage.image(view).frame({x + 5, y + 5, tile_w - 10, tile_h - 30}).filter(f);
        stage.text(table[i].name, {x + 5, y + tile_h - 24}).size(14);
    }

    CpuRenderer renderer;
    const Surface& frame = renderer.render(stage, 0.0f);
    FILE* out = std::fopen("filters_tour.ppm", "wb");
    if (out == nullptr) {
        std::perror("filters_tour.ppm");
        return 1;
    }
    std::fprintf(out, "P6\n%u %u\n255\n", frame.width, frame.height);
    for (uint32_t y = 0; y < frame.height; ++y) {
        const uint8_t* row = frame.row(y);
        for (uint32_t x = 0; x < frame.width; ++x) {
            const uint8_t* p = &row[x * 4];
            const float a = p[3] / 255.0f;
            const uint8_t rgb[3] = {static_cast<uint8_t>(p[0] * a + 24 * (1 - a)),
                                    static_cast<uint8_t>(p[1] * a + 26 * (1 - a)),
                                    static_cast<uint8_t>(p[2] * a + 30 * (1 - a))};
            std::fwrite(rgb, 1, 3, out);
        }
    }
    std::fclose(out);
    std::printf("wrote filters_tour.ppm (%zu filters)\n", table.size());
    return 0;
}
