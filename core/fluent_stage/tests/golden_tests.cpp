// golden_tests — image-level contract tests (§11, §13-6).
//
// Each scene renders deterministically (injected dt only) and compares
// against a stored golden image in tests/golden/. The comparison allows a
// small per-channel tolerance so recompiles and minor FP differences don't
// flake, while real regressions (a shifted shape, a broken filter) fail.
//
//   ./golden_tests            compare against the goldens
//   ./golden_tests --update   regenerate the goldens from this build
//
// On failure the rendered image is written next to the golden as
// <name>.actual.ppm for visual diffing. The text scene depends on the
// system CJK font (goldens are generated on the robot's own image); when no
// font is available that scene is skipped with a notice.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <fluent_stage/fluent_stage.hpp>

using namespace fluent_stage;

namespace {

int g_failures = 0;
bool g_update = false;
std::string g_dir;  // tests/golden, from GOLDEN_DIR

// ---- ppm io ----------------------------------------------------------------

bool writePpm(const std::string& path, const Surface& s) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (f == nullptr) {
        return false;
    }
    std::fprintf(f, "P6\n%u %u\n255\n", s.width, s.height);
    for (uint32_t y = 0; y < s.height; ++y) {
        const uint8_t* row = s.row(y);
        for (uint32_t x = 0; x < s.width; ++x) {
            // Composite over mid-gray so alpha regressions are visible too.
            const uint8_t* p = &row[x * 4];
            const float a = p[3] / 255.0f;
            const uint8_t rgb[3] = {static_cast<uint8_t>(p[0] * a + 64 * (1 - a)),
                                    static_cast<uint8_t>(p[1] * a + 64 * (1 - a)),
                                    static_cast<uint8_t>(p[2] * a + 64 * (1 - a))};
            std::fwrite(rgb, 1, 3, f);
        }
    }
    std::fclose(f);
    return true;
}

bool readPpm(const std::string& path, uint32_t& w, uint32_t& h, std::vector<uint8_t>& rgb) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (f == nullptr) {
        return false;
    }
    char magic[3] = {};
    unsigned int pw = 0, ph = 0, maxval = 0;
    if (std::fscanf(f, "%2s %u %u %u", magic, &pw, &ph, &maxval) != 4 ||
        std::strcmp(magic, "P6") != 0 || maxval != 255) {
        std::fclose(f);
        return false;
    }
    std::fgetc(f);  // single whitespace after header
    rgb.resize(static_cast<size_t>(pw) * ph * 3);
    const bool ok = std::fread(rgb.data(), 1, rgb.size(), f) == rgb.size();
    std::fclose(f);
    w = pw;
    h = ph;
    return ok;
}

// ---- comparison ------------------------------------------------------------

void checkScene(const std::string& name, const Surface& s) {
    const std::string golden_path = g_dir + "/" + name + ".ppm";
    if (g_update) {
        if (writePpm(golden_path, s)) {
            std::printf("updated %s\n", golden_path.c_str());
        } else {
            std::fprintf(stderr, "FAIL cannot write %s\n", golden_path.c_str());
            ++g_failures;
        }
        return;
    }
    uint32_t gw = 0, gh = 0;
    std::vector<uint8_t> golden;
    if (!readPpm(golden_path, gw, gh, golden)) {
        std::fprintf(stderr, "FAIL %s: missing golden (run --update once)\n", name.c_str());
        ++g_failures;
        return;
    }
    if (gw != s.width || gh != s.height) {
        std::fprintf(stderr, "FAIL %s: size %ux%u != golden %ux%u\n", name.c_str(), s.width,
                     s.height, gw, gh);
        ++g_failures;
        return;
    }
    // Rebuild the composited rgb the same way writePpm does.
    size_t over_threshold = 0;
    int max_diff = 0;
    for (uint32_t y = 0; y < s.height; ++y) {
        const uint8_t* row = s.row(y);
        for (uint32_t x = 0; x < s.width; ++x) {
            const uint8_t* p = &row[x * 4];
            const float a = p[3] / 255.0f;
            const uint8_t rgb[3] = {static_cast<uint8_t>(p[0] * a + 64 * (1 - a)),
                                    static_cast<uint8_t>(p[1] * a + 64 * (1 - a)),
                                    static_cast<uint8_t>(p[2] * a + 64 * (1 - a))};
            const uint8_t* g = &golden[(static_cast<size_t>(y) * s.width + x) * 3];
            for (int c = 0; c < 3; ++c) {
                const int d = std::abs(static_cast<int>(rgb[c]) - g[c]);
                max_diff = std::max(max_diff, d);
                if (d > 2) {
                    ++over_threshold;
                }
            }
        }
    }
    const size_t total = static_cast<size_t>(s.width) * s.height * 3;
    const double over_ratio = static_cast<double>(over_threshold) / total;
    if (max_diff > 16 || over_ratio > 0.002) {
        std::fprintf(stderr, "FAIL %s: max_diff=%d over_ratio=%.4f%%\n", name.c_str(), max_diff,
                     over_ratio * 100.0);
        writePpm(g_dir + "/" + name + ".actual.ppm", s);
        ++g_failures;
    } else {
        std::printf("ok %s (max_diff=%d)\n", name.c_str(), max_diff);
    }
}

// ---- shared fixtures -------------------------------------------------------

std::vector<uint8_t> makeTestImage(uint32_t w, uint32_t h) {
    std::vector<uint8_t> px(w * h * 4);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            uint8_t* p = &px[(y * w + x) * 4];
            p[0] = static_cast<uint8_t>(x * 255 / w);
            p[1] = static_cast<uint8_t>(y * 255 / h);
            p[2] = static_cast<uint8_t>(((x / 16) + (y / 16)) % 2 ? 200 : 60);
            p[3] = 255;
        }
    }
    return px;
}

// ---- scenes ----------------------------------------------------------------

void sceneShapes(CpuRenderer& r) {
    Stage stage(480, 300);
    stage.grid(60).color({1, 1, 1, 0.12f});
    stage.line({20, 30}, {200, 60}).thickness(5).color(Color::Teal);
    stage.line({20, 60}, {200, 90}).thickness(5).dash(14).cap(Cap::Butt).color(Color::Orange);
    stage.polyline({{20, 130}, {80, 100}, {140, 140}, {200, 110}}).thickness(6);
    stage.polygon({{40, 200}, {110, 170}, {180, 210}, {120, 260}}).color(Color::Blue.faded(0.8f));
    stage.polygon({{40, 200}, {110, 170}, {180, 210}, {120, 260}})
        .thickness(2)
        .color(Color::White);
    stage.rect({230, 30, 100, 60}).cornerRadius(14).color(Color::Magenta.faded(0.9f));
    stage.rect({230, 110, 100, 60}).cornerRadius(14).thickness(4).color(Color::Yellow);
    stage.circle({280, 240}, 34).color(Color::Green);
    stage.circle({280, 240}, 34).thickness(3).color(Color::Black.faded(0.6f));
    stage.circles({{370, 40}, {395, 60}, {420, 45}, {440, 70}}, 7).color(Color::Red);
    stage.arc({400, 140}, 40, -60, 200).thickness(9).color(Color::Teal);
    stage.arrow({360, 220}, {450, 260}).thickness(5).color(Color::Orange);
    stage.crosshair({420, 200}, 22).thickness(2).color(Color::White);
    checkScene("shapes", r.render(stage, 0.0f));
}

void sceneAttributes(CpuRenderer& r) {
    Stage stage(480, 300);
    stage.rect({0, 0, 480, 300}).color({0.13f, 0.15f, 0.18f, 1});

    auto& panel = stage.group("panel");
    panel.bounds({0, 0, 180, 100}).position(24 + 90, 24 + 50);
    panel.background({0, 0, 0, 0.5f});
    panel.cornerRadius(16);
    panel.border(2, Color::Teal);
    panel.shadow(0, 6, 12);
    panel.masksToBounds(true);
    panel.circle({0, 50}, 40).color(Color::Magenta);  // clipped left half

    // Group opacity: overlapping children must fade as one.
    auto& g = stage.group("fade").opacity(0.5f);
    g.rect({240, 30, 90, 60}).color(Color::White);
    g.rect({280, 60, 90, 60}).color(Color::White);

    // Blend modes over a gradient bar.
    stage.rect({240, 160, 220, 30}).color({0.3f, 0.5f, 0.9f, 1});
    stage.rect({250, 150, 40, 50}).blend(Blend::Add).color({0.5f, 0.2f, 0.1f, 1});
    stage.rect({310, 150, 40, 50}).blend(Blend::Multiply).color({0.9f, 0.8f, 0.3f, 1});
    stage.rect({370, 150, 40, 50}).blend(Blend::Screen).color({0.2f, 0.6f, 0.4f, 1});

    // Border + background + shadow on a leaf with content.
    stage.rect({60, 200, 120, 70})
        .cornerRadius(10)
        .color({1, 1, 1, 0.25f})
        .shadow(4, 4, 8);
    checkScene("attributes", r.render(stage, 0.0f));
}

void sceneTransforms(CpuRenderer& r) {
    Stage stage(480, 300);
    stage.grid(50).color({1, 1, 1, 0.1f});
    stage.rect({0, 0, 120, 60}).position(110, 90).rotation(25).thickness(3).color(Color::Teal);
    stage.rect({0, 0, 120, 60}).position(110, 90).thickness(1).color({1, 1, 1, 0.3f});
    // Needle anchored at bottom-center.
    stage.rect({0, 0, 8, 90}).anchor(0.5f, 1.0f).position(300, 150).rotation(135).color(
        Color::Orange);
    stage.circle({300, 150}, 6).color(Color::White);
    // Mirrored + scaled group with nested rotation.
    auto& g = stage.group("g");
    g.bounds({0, 0, 100, 80}).position(400, 220).scale(-1.2f, 1.2f).rotation(-15);
    g.rect({10, 10, 80, 25}).cornerRadius(6).color(Color::Yellow);
    g.arrow({15, 55}, {85, 55}).thickness(4).color(Color::Red);
    checkScene("transforms", r.render(stage, 0.0f));
}

void sceneFilters(CpuRenderer& r) {
    Stage stage(480, 300);
    const auto img = makeTestImage(160, 120);
    const ImageView view{160, 120, img.data(), 0};
    stage.image(view).frame({10, 10, 150, 130});
    stage.image(view).frame({170, 10, 150, 130}).blur(5);
    stage.image(view).frame({330, 10, 140, 130}).grayscale();
    stage.image(view).frame({10, 160, 150, 130}).pixelate(10);
    stage.image(view).frame({170, 160, 150, 130}).filter(
        ColorTransform().contrast(1.6f).saturation(0.4f));
    // Group filter: applies to the composited subtree.
    auto& g = stage.group("g").invert();
    g.rect({340, 170, 60, 50}).color(Color::Teal);
    g.circle({430, 230}, 30).color(Color::Orange);
    checkScene("filters", r.render(stage, 0.0f));
}

void sceneAnimation(CpuRenderer& r) {
    // §13-6: "the screen at t = 0.15 s" must be reproducible.
    Stage stage(320, 200);
    auto& box = stage.rect({0, 0, 60, 60}).position(50, 100).color(Color::Teal);
    auto& dot = stage.circle({50, 40}, 12).color(Color::Orange);
    {
        Transaction t(0.3f, Ease::InOut);
        box.position(270, 100).rotation(90);
        dot.opacity(0.2f);
    }
    stage.advance(0.05f);
    stage.advance(0.05f);
    stage.advance(0.05f);  // t = 0.15 — mid-flight
    checkScene("animation_t015", r.render(stage, 0.0f));
}

void sceneImagePaste(CpuRenderer& r) {
    // §5-2b: partial copy & paste via sourceRect + frame, plus fit modes.
    Stage stage(480, 300);
    const auto img = makeTestImage(160, 120);
    const ImageView view{160, 120, img.data(), 0};
    stage.image(view, Fit::Contain).frame({10, 10, 140, 130}).background({1, 1, 1, 0.08f});
    stage.image(view, Fit::Cover).frame({160, 10, 140, 130});
    stage.image(view, Fit::Fill).frame({310, 10, 160, 130});
    stage.image(view).sourceRect({40, 30, 60, 40}).frame({60, 160, 180, 120});
    stage.image(view).sourceRect({40, 30, 60, 40}).frame({280, 160, 90, 120}).opacity(0.6f);
    checkScene("image_paste", r.render(stage, 0.0f));
}

void sceneUi(CpuRenderer& r) {
    // §10: controls are prefab subtrees; states are attribute overrides.
    Stage stage(480, 300);
    stage.rect({0, 0, 480, 300}).color({0.10f, 0.12f, 0.15f, 1});
    ui::Button normal(stage.root(), {24, 24, 180, 56}, "収穫開始");
    ui::Button pressed(stage.root(), {24, 100, 180, 56}, "PRESSED");
    ui::Button disabled(stage.root(), {24, 176, 180, 56}, "DISABLED");
    disabled.enabled(false);
    ui::Switch sw_off(stage.root(), {260, 28, 96, 48});
    ui::Switch sw_on(stage.root(), {260, 104, 96, 48});
    sw_on.setOn(true, false);
    stage.pointerDown({110, 128});  // hold the second button pressed
    stage.advance(0.5f);            // settle all state transitions
    checkScene("ui_controls", r.render(stage, 0.0f));
}

void sceneUiCatalog(CpuRenderer& r) {
    // The full §10 control catalog, one state each, dropdown open.
    Stage stage(480, 420);
    stage.rect({0, 0, 480, 420}).color({0.10f, 0.12f, 0.15f, 1});
    ui::Slider slider(stage.root(), {24, 24, 200, 28}, 0.6f);
    ui::Segmented seg(stage.root(), {24, 76, 240, 40}, {"手動", "巡回", "追従"}, 1);
    ui::Gauge gauge(stage.root(), {400, 70}, 46);
    gauge.setValue(0.64f);
    ui::Dropdown dd(stage.root(), {24, 140, 200, 44}, {"標準", "低速", "高速", "点検"}, 2);
    stage.pointerDown({100, 160});
    stage.pointerUp({100, 160});  // open the dropdown
    stage.advance(0.5f);          // settle transitions
    checkScene("ui_catalog", r.render(stage, 0.0f));
}

void sceneRipple(CpuRenderer& r) {
    // The pointer-wake effect at t = 0.15 s, deterministic like everything.
    Stage stage(480, 300);
    stage.rect({0, 0, 480, 300}).color({0.08f, 0.12f, 0.16f, 1});
    stage.grid(60).color({1, 1, 1, 0.08f});
    fx::Ripple ripple(stage.root());
    ripple.splash({120, 150});
    ripple.pointerMoved({240, 90});
    ripple.pointerMoved({300, 120});
    ripple.pointerMoved({360, 160});
    for (int i = 0; i < 3; ++i) {
        ripple.tick(0.05f);
        stage.advance(0.05f);
    }
    checkScene("ripple_t015", r.render(stage, 0.0f));
}

void sceneText(CpuRenderer& r) {
    Stage stage(480, 300);
    stage.rect({0, 0, 480, 300}).color({0.1f, 0.12f, 0.15f, 1});
    stage.text("走行中 — Stage L0", {20, 20}).size(30);
    stage.text("centered", {240, 90}).size(22).align(Align::Center).color(Color::Teal);
    stage.text("right", {460, 130}).size(22).align(Align::Right).color(Color::Orange);
    stage.text("影付きテキスト", {20, 180}).size(34).shadow(2, 3, 6);
    stage.text("small 12px 日本語", {20, 240}).size(12).color({1, 1, 1, 0.8f});
    const Surface& s = r.render(stage, 0.0f);
    checkScene("text", s);
}

}  // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--update") == 0) {
            g_update = true;
        }
    }
#ifdef GOLDEN_DIR
    g_dir = GOLDEN_DIR;
#else
    g_dir = "tests/golden";
#endif
    CpuRenderer r;
    sceneShapes(r);
    sceneAttributes(r);
    sceneTransforms(r);
    sceneFilters(r);
    sceneAnimation(r);
    sceneImagePaste(r);
    sceneUi(r);
    sceneUiCatalog(r);
    sceneRipple(r);
    sceneText(r);
    if (g_failures == 0) {
        std::printf("all golden_tests passed\n");
        return 0;
    }
    std::fprintf(stderr, "%d golden failure(s)\n", g_failures);
    return 1;
}
