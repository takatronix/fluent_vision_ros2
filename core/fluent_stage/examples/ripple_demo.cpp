// ripple_demo — the pointer-wake effect (fx::Ripple) over a HUD scene.
//
// A scripted pointer path sweeps across the screen with a tap splash;
// frames at t = 0.12 and t = 0.35 show young and dissolving rings. Feed
// the effect real positions from your web viewer / touch input and it
// becomes the live hover wake.

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
    stage.rect({0, 0, 640, 360}).color({0.08f, 0.12f, 0.16f, 1});
    stage.grid(80).color({1, 1, 1, 0.07f});
    auto& hud = stage.group("hud").position(24, 24);
    hud.rect({0, 0, 240, 64}).cornerRadius(12).color({0, 0, 0, 0.45f});
    hud.text("巡回中", {16, 16}).size(26);

    fx::Ripple ripple(stage.root());  // after the UI: rings draw on top

    CpuRenderer renderer;
    const auto frame = [&](float dt) {
        ripple.tick(dt);
        return renderer.render(stage, dt);  // one clock for everything
    };

    // Scripted pointer sweep + a tap.
    const Vec2 path[] = {{80, 300}, {160, 260}, {240, 230}, {330, 210}, {420, 200}, {510, 180}};
    for (const Vec2& p : path) {
        ripple.pointerMoved(p);
    }
    ripple.splash({560, 120});

    frame(0.06f);
    writePpm("ripple_young.ppm", frame(0.06f));   // t = 0.12
    frame(0.12f);
    writePpm("ripple_fading.ppm", frame(0.11f));  // t = 0.35
    std::printf("wrote ripple_young/fading.ppm\n");
    return 0;
}
