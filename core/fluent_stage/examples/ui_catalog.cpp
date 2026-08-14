// ui_catalog — every §10 control, driven by injected pointer gestures.
//
// Frame 1: the catalog at rest. Frame 2: after a scripted interaction —
// the slider dragged to 80%, the segmented control switched, the dropdown
// opened. Two runs produce identical bytes (time is only the dt below).

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
    Stage stage(640, 440);
    stage.rect({0, 0, 640, 440}).color({0.10f, 0.12f, 0.15f, 1});
    stage.text("speed", {24, 20}).size(16).color({1, 1, 1, 0.55f});

    ui::Slider speed(stage.root(), {24, 44, 240, 28}, 0.35f);
    speed.onChange([](float v) { std::printf("speed -> %.2f\n", v); });

    ui::Segmented mode(stage.root(), {24, 96, 280, 42}, {"手動", "巡回", "追従"}, 0);
    mode.onChange([](int i) { std::printf("mode -> %d\n", i); });

    ui::Gauge battery(stage.root(), {560, 80, }, 48);
    battery.setValue(0.64f);

    ui::Dropdown profile(stage.root(), {24, 168, 220, 46}, {"標準", "低速", "高速", "点検"}, 0);
    profile.onChange([](int i) { std::printf("profile -> %d\n", i); });

    ui::Button go(stage.root(), {24, 360, 180, 56}, "適用");
    ui::Switch light(stage.root(), {230, 364, 96, 48});

    CpuRenderer renderer;
    writePpm("ui_catalog_idle.ppm", renderer.render(stage, 0.0f));

    stage.pointerDown({100, 58});           // grab the slider…
    stage.pointerMove({230, 58});           // …drag to ~80%
    stage.pointerUp({230, 58});
    stage.pointerDown({250, 116});          // switch to 追従
    stage.pointerUp({250, 116});
    stage.pointerDown({100, 190});          // open the profile dropdown
    stage.pointerUp({100, 190});
    renderer.render(stage, 0.3f);           // settle transitions
    writePpm("ui_catalog_active.ppm", renderer.render(stage, 0.0f));
    std::printf("wrote ui_catalog_idle/active.ppm\n");
    return 0;
}
