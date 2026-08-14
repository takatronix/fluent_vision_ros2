// hud_basic — the design document's §2 demo, in C++ (Stage API).
//
// Draws a synthetic camera frame, detection boxes, and a HUD panel with
// Japanese text, then renders one frame and writes hud_basic.ppm next to
// the executable. Run it, open the image, and you have seen every core
// mechanism: content, styling chains, groups, shadows, and compositing.

#include <cstdio>
#include <vector>

#include <fluent_stage/fluent_stage.hpp>

using namespace fluent_stage;

int main() {
    Stage stage(1280, 720);

    // A synthetic "camera frame": vertical gradient with a horizon band.
    const uint32_t cam_w = 640, cam_h = 360;
    std::vector<uint8_t> cam(cam_w * cam_h * 4);
    for (uint32_t y = 0; y < cam_h; ++y) {
        for (uint32_t x = 0; x < cam_w; ++x) {
            uint8_t* p = &cam[(y * cam_w + x) * 4];
            const float t = static_cast<float>(y) / cam_h;
            p[0] = static_cast<uint8_t>(40 + 30 * t);
            p[1] = static_cast<uint8_t>(70 + 90 * t);
            p[2] = static_cast<uint8_t>(60 + 40 * t);
            p[3] = 255;
        }
    }
    ImageView camera{cam_w, cam_h, cam.data(), 0};
    stage.image(camera, Fit::Cover);

    // Detections, teal, with labels.
    stage.boxes({{{420, 250, 180, 260}, 0.91f, "asparagus", 0},
                 {{760, 300, 150, 220}, 0.78f, "asparagus", 0}})
        .color(Color::Teal)
        .cornerRadius(6);

    // A planned path and a heading arrow.
    stage.polyline({{140, 660}, {420, 600}, {760, 620}, {1120, 540}})
        .thickness(6)
        .dash(24)
        .color(Color::Orange.faded(0.9f));
    stage.arrow({640, 500}, {760, 430}).thickness(5).color(Color::Yellow);
    stage.crosshair({640, 360}, 26).color({1, 1, 1, 0.8f}).thickness(2);

    // The HUD panel from the design document.
    auto& hud = stage.group("hud").position(24, 24).opacity(0.9f).shadow();
    hud.rect({0, 0, 340, 96}).cornerRadius(12).color({0, 0, 0, 0.45f});
    hud.text("走行中", {16, 12}).size(28);
    hud.text("SPEED 1.2 m/s", {16, 52}).size(22).color(Color::Teal);

    // A gauge in the corner.
    stage.arc({1180, 640}, 44, 120, 420).thickness(10).color({1, 1, 1, 0.25f});
    stage.arc({1180, 640}, 44, 120, 320).thickness(10).color(Color::Teal);
    stage.text("64%", {1180, 628}).size(24).align(Align::Center);

    CpuRenderer renderer;
    const Surface& frame = renderer.render(stage, 0.0f);

    // PPM (RGB over a dark backdrop) — zero-dependency output.
    FILE* f = std::fopen("hud_basic.ppm", "wb");
    if (f == nullptr) {
        std::perror("hud_basic.ppm");
        return 1;
    }
    std::fprintf(f, "P6\n%u %u\n255\n", frame.width, frame.height);
    for (uint32_t y = 0; y < frame.height; ++y) {
        const uint8_t* row = frame.row(y);
        for (uint32_t x = 0; x < frame.width; ++x) {
            const uint8_t* p = &row[x * 4];
            const float a = p[3] / 255.0f;
            const uint8_t rgb[3] = {static_cast<uint8_t>(p[0] * a + 16 * (1 - a)),
                                    static_cast<uint8_t>(p[1] * a + 16 * (1 - a)),
                                    static_cast<uint8_t>(p[2] * a + 18 * (1 - a))};
            std::fwrite(rgb, 1, 3, f);
        }
    }
    std::fclose(f);
    std::printf("wrote hud_basic.ppm (%ux%u), %u layers\n", frame.width, frame.height,
                stage.layerCount());
    return 0;
}
