// ripple_demo — the real water ripple: fs_ripple refraction waves bending
// the image beneath (no rings drawn on top).
//
// A tap splash plus a hover trail run over a grid-and-panel scene; frames
// at t = 0.15 and t = 0.45 show the wavefronts expanding and the grid
// genuinely warping. Deterministic: run it twice, same bytes.

#include <cstdio>
#include <string>

#include <fluent_stage/fluent_stage.hpp>

using namespace fluent_stage;

namespace {

void writePpm(const std::string& path, const Surface& s) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (f == nullptr) {
        std::perror(path.c_str());
        return;
    }
    std::fprintf(f, "P6\n%u %u\n255\n", s.width, s.height);
    for (uint32_t y = 0; y < s.height; ++y) {
        const uint8_t* row = s.row(y);
        for (uint32_t x = 0; x < s.width; ++x) {
            const uint8_t* p = &row[x * 4];
            const float a = p[3] / 255.0f;
            const uint8_t rgb[3] = {static_cast<uint8_t>(p[0] * a + 20 * (1 - a)),
                                    static_cast<uint8_t>(p[1] * a + 24 * (1 - a)),
                                    static_cast<uint8_t>(p[2] * a + 30 * (1 - a))};
            std::fwrite(rgb, 1, 3, f);
        }
    }
    std::fclose(f);
}

}  // namespace

int main() {
    Stage stage(640, 360);
    // Everything inside `water` refracts; the HUD above stays crisp.
    auto& water = stage.group("water");
    water.bounds({0, 0, 640, 360});
    water.rect({0, 0, 640, 360}).color({0.07f, 0.11f, 0.15f, 1});
    water.grid(40).color({1, 1, 1, 0.22f});
    water.circle({480, 220}, 60).color(Color::Teal.faded(0.35f));
    water.text("水面", {60, 250}).size(60).color({1, 1, 1, 0.5f});

    auto& hud = stage.group("hud").position(24, 24);
    hud.rect({0, 0, 240, 64}).cornerRadius(12).color({0, 0, 0, 0.45f});
    hud.text("巡回中", {16, 16}).size(26);

    fx::Ripple ripple(water);

    CpuRenderer renderer;
    const auto frame = [&](float dt) {
        ripple.tick(dt);
        return renderer.render(stage, dt);  // one clock for everything
    };

    ripple.splash({320, 180});          // tap
    ripple.pointerMoved({520, 120});    // hover trail
    frame(0.075f);
    writePpm("ripple_young.ppm", frame(0.075f));   // t = 0.15
    frame(0.15f);
    writePpm("ripple_spread.ppm", frame(0.15f));   // t = 0.45
    std::printf("wrote ripple_young/spread.ppm\n");
    return 0;
}
