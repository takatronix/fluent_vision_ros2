// stage_tests — L0 contract tests: geometry (CALayer semantics), implicit
// animation via Transaction with injected time (§13-6), hit-testing, limits,
// the filter metadata table, and render determinism.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <fluent_stage/fluent_stage.hpp>

using namespace fluent_stage;

namespace {

int g_failures = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                                    \
        }                                                                    \
    } while (0)

bool near(float a, float b, float eps = 1e-3f) { return std::fabs(a - b) < eps; }

void testGeometry() {
    Stage stage(1000, 800);
    // frame is sugar over bounds+position+anchor (CALayer §4.1).
    auto& panel = stage.rect({0, 0, 200, 100}).frame({100, 50, 200, 100});
    CHECK(near(panel.position().x, 200));  // anchor (0.5,0.5) → center
    CHECK(near(panel.position().y, 100));
    const Rect f = panel.frame();
    CHECK(near(f.x, 100) && near(f.y, 50) && near(f.w, 200) && near(f.h, 100));

    // position places the anchor, not the frame origin.
    panel.anchor(0, 0).position(300, 300);
    CHECK(near(panel.frame().x, 300) && near(panel.frame().y, 300));

    // Content bbox becomes automatic bounds (§15-1).
    auto& c = stage.circle({500, 400}, 50);
    const Rect cb = c.bounds();
    CHECK(near(cb.x, 450) && near(cb.y, 350) && near(cb.w, 100) && near(cb.h, 100));

    // frame() on a rotated layer is diagnosed.
    auto& r = stage.rect({0, 0, 10, 10}).rotation(30);
    r.frame({0, 0, 20, 20});
    bool diagnosed = false;
    for (const std::string& d : stage.drainDiagnostics()) {
        if (d.find("transformed layer") != std::string::npos) {
            diagnosed = true;
        }
    }
    CHECK(diagnosed);
}

void testTransaction() {
    Stage stage(100, 100);
    auto& r = stage.rect({0, 0, 10, 10}).id("r");

    // Outside a transaction: snap.
    r.opacity(0.5f);
    CHECK(near(r.presentedOpacity(), 0.5f));

    // Inside: animate; injected dt drives it deterministically.
    {
        Transaction t(1.0f, Ease::Linear);
        r.opacity(1.0f).position(50, 0);
    }
    CHECK(near(r.presentedOpacity(), 0.5f));  // not advanced yet
    stage.advance(0.5f);
    CHECK(near(r.presentedOpacity(), 0.75f));
    // position animates from its auto value (content center (5,5)) to (50,0).
    CHECK(near(r.presentedPosition().x, 27.5f));
    CHECK(near(r.presentedPosition().y, 2.5f));
    stage.advance(0.5f);
    CHECK(near(r.presentedOpacity(), 1.0f));
    CHECK(near(r.opacity(), 1.0f));  // model value was 1.0 all along

    // Retarget mid-flight continues from the presentation value (no jump).
    {
        Transaction t(1.0f, Ease::Linear);
        r.opacity(0.0f);
    }
    stage.advance(0.5f);
    CHECK(near(r.presentedOpacity(), 0.5f));
    {
        Transaction t(1.0f, Ease::Linear);
        r.opacity(1.0f);
    }
    stage.advance(0.0f);
    CHECK(near(r.presentedOpacity(), 0.5f));  // starts where it was
}

void testHitTest() {
    Stage stage(1000, 800);
    stage.rect({0, 0, 1000, 800}).id("bg").frame({0, 0, 1000, 800});
    auto& hud = stage.group("hud");
    hud.bounds({0, 0, 340, 96}).position(24 + 170, 24 + 48);  // frame at (24,24)
    hud.rect({0, 0, 340, 96}).id("panel");
    CHECK(stage.hitTest({30, 30}) != nullptr);
    CHECK(stage.hitTest({30, 30})->id() == "panel");
    CHECK(stage.hitTest({500, 500})->id() == "bg");
    hud.hidden(true);
    CHECK(stage.hitTest({30, 30})->id() == "bg");

    // A bare group is a coordinate space: position moves its origin (the
    // §2 example), it forwards hits but is never hit itself.
    auto& g = stage.group("g2").position(100, 100);
    g.rect({0, 0, 50, 50}).id("inner");
    CHECK(stage.hitTest({120, 120}) != nullptr);
    CHECK(stage.hitTest({120, 120})->id() == "inner");
    CHECK(stage.hitTest({99, 99})->id() != "g2");
}

void testLimits() {
    Stage stage(100, 100, StageLimits{4, 2, 8, 8, 16});
    stage.rect({0, 0, 1, 1});
    stage.rect({0, 0, 1, 1});
    stage.rect({0, 0, 1, 1});  // 4 layers total with root
    bool threw = false;
    try {
        stage.rect({0, 0, 1, 1});
    } catch (const std::length_error&) {
        threw = true;
    }
    CHECK(threw);

    // Data limits clamp with a diagnostic instead of throwing.
    Stage s2(100, 100, StageLimits{16, 4, 4, 8, 8});
    auto& p = s2.polyline({{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5}});
    CHECK(std::get<PolylineContent>(p.content()).points.size() == 4);
    CHECK(!s2.drainDiagnostics().empty());
}

void testFilterTable() {
    const FilterSpec* spec = findFilterSpec(FS_BILATERAL);
    CHECK(spec != nullptr);
    CHECK(std::string(spec->name) == "bilateral");
    CHECK(spec->params.size() == 2);
    CHECK(std::string(spec->params[0].name) == "radius");
    CHECK(spec->params[0].unit == FilterUnit::Length);
    CHECK(near(spec->params[1].default_value, 0.15f));

    // Typed structs carry defaults and setters from the same single source.
    const Filter f = Bilateral().sigma_color(0.3f);
    CHECK(f.mode == FS_BILATERAL);
    CHECK(near(f.values[0], 4.0f));  // default radius
    CHECK(near(f.values[1], 0.3f));

    // Every filter in the table resolves by mode.
    for (const FilterSpec& s : filterTable()) {
        CHECK(findFilterSpec(s.mode) == &s);
    }
}

void testRenderDeterminism() {
    Stage stage(320, 200);
    stage.rect({20, 20, 120, 80}).cornerRadius(10).color(Color::Teal);
    stage.circle({220, 100}, 40).thickness(6).color(Color::Orange);
    stage.line({10, 180}, {310, 150}).dash(12);
    auto& g = stage.group("g").opacity(0.8f).shadow();
    g.rect({160, 30, 100, 60}).color({1, 1, 1, 0.7f});
    g.rect({160, 30, 100, 60}).cornerRadius(8).blur(3);

    CpuRenderer r1;
    const Surface& a = r1.render(stage, 0.0f);
    CHECK(a.valid() && a.width == 320 && a.height == 200);
    std::vector<uint8_t> first(a.pixels, a.pixels + a.strideBytes * a.height);

    CpuRenderer r2;
    const Surface& b = r2.render(stage, 0.0f);
    CHECK(std::memcmp(first.data(), b.pixels, first.size()) == 0);

    // Something actually rendered.
    bool any = false;
    for (size_t i = 3; i < first.size(); i += 4) {
        if (first[i] != 0) {
            any = true;
            break;
        }
    }
    CHECK(any);

    // Scaled output renders the same scene at double size (SDF, no crash).
    const Surface& big = r1.render(stage, 640, 400, 0.0f);
    CHECK(big.width == 640 && big.height == 400);
}

}  // namespace

int main() {
    testGeometry();
    testTransaction();
    testHitTest();
    testLimits();
    testFilterTable();
    testRenderDeterminism();
    if (g_failures == 0) {
        std::printf("all stage_tests passed\n");
        return 0;
    }
    std::fprintf(stderr, "%d failure(s)\n", g_failures);
    return 1;
}
