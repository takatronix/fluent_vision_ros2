// ui_demo — buttons and switches reacting to injected pointer events (§10).
//
// The Stage owns no input device: a web viewer, a touch screen, or a VR
// controller ray all end up calling stage.pointerDown/Move/Up in logical
// coordinates. This demo scripts such a gesture sequence and writes three
// frames: idle → mid-press (t=0.05) → after the tap toggled the switch
// (t=0.15, knob mid-slide). Deterministic: run it twice, same bytes.

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
    stage.rect({0, 0, 640, 360}).color({0.10f, 0.12f, 0.15f, 1});
    stage.text("aspa control", {24, 18}).size(20).color({1, 1, 1, 0.6f});

    ui::Button start(stage.root(), {24, 60, 220, 64}, "収穫開始");
    ui::Button stop(stage.root(), {24, 140, 220, 64}, "停止");
    stop.enabled(false);

    ui::Switch light(stage.root(), {280, 68, 96, 48});
    stage.text("ライト", {390, 78}).size(22);
    ui::Switch follow(stage.root(), {280, 148, 96, 48});
    stage.text("追従", {390, 158}).size(22);
    follow.setOn(true, false);

    start.onTap([&] { std::printf("tap: 収穫開始\n"); });
    light.onChange([&](bool on) { std::printf("light -> %s\n", on ? "on" : "off"); });

    CpuRenderer renderer;
    writePpm("ui_idle.ppm", renderer.render(stage, 0.0f));

    stage.pointerDown({130, 90});                 // press 収穫開始
    writePpm("ui_pressed.ppm", renderer.render(stage, 0.05f));
    stage.pointerUp({130, 90});                   // tap fires here

    stage.pointerDown({328, 92});                 // tap the light switch
    stage.pointerUp({328, 92});
    writePpm("ui_toggling.ppm", renderer.render(stage, 0.075f));  // knob mid-slide
    renderer.render(stage, 0.3f);                 // settle (frame discarded)
    writePpm("ui_settled.ppm", renderer.render(stage, 0.0f));
    std::printf("wrote ui_idle/pressed/toggling/settled.ppm\n");
    return 0;
}
