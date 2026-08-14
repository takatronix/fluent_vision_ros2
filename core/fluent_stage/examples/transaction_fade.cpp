// transaction_fade — implicit animation (§9) with injected, deterministic
// time (§13-6).
//
// One Transaction retargets a HUD panel; the render loop then advances the
// stage by fixed dt steps and writes the frames at t = 0.0, 0.15, 0.3 s.
// Run it twice: the frames are byte-identical — time only moves through dt.

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
            const uint8_t rgb[3] = {static_cast<uint8_t>(p[0] * a + 24 * (1 - a)),
                                    static_cast<uint8_t>(p[1] * a + 26 * (1 - a)),
                                    static_cast<uint8_t>(p[2] * a + 30 * (1 - a))};
            std::fwrite(rgb, 1, 3, f);
        }
    }
    std::fclose(f);
}

}  // namespace

int main() {
    Stage stage(640, 360);
    stage.grid(80).color({1, 1, 1, 0.08f});

    auto& hud = stage.group("hud").position(24, 24);
    hud.rect({0, 0, 280, 80}).cornerRadius(12).color({0, 0, 0, 0.5f});
    hud.text("収穫モード", {16, 10}).size(26);
    hud.text("READY", {16, 46}).size(20).color(Color::Teal);

    auto& gauge = stage.arc({560, 300}, 40, 120, 380).thickness(9).color(Color::Teal);

    // One scope: everything inside animates over 0.3 s.
    {
        Transaction t(0.3f, Ease::InOut);
        stage.find("hud")->position(24, -120).opacity(0.0f);  // slide out + fade
        gauge.opacity(0.4f);
    }

    CpuRenderer renderer;
    writePpm("transaction_t000.ppm", renderer.render(stage, 0.0f));
    // Three 0.05 s steps → the reproducible screen at t = 0.15 s.
    renderer.render(stage, 0.05f);
    renderer.render(stage, 0.05f);
    writePpm("transaction_t015.ppm", renderer.render(stage, 0.05f));
    writePpm("transaction_t030.ppm", renderer.render(stage, 0.15f));
    std::printf("wrote transaction_t000/t015/t030.ppm\n");
    return 0;
}
