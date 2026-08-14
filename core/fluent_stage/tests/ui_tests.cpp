// ui_tests — pointer injection (§10-3) and the Button/Switch state
// machines (§10-1/2): tap, capture tracking, slide-off, disabled
// fall-through, toggle events, and animated state transitions under
// injected time.

#include <cmath>
#include <cstdio>

#include <fluent_stage/fluent_stage.hpp>

using namespace fluent_stage;

namespace {

int g_failures = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                                        \
        }                                                                        \
    } while (0)

bool near(float a, float b, float eps = 1e-3f) { return std::fabs(a - b) < eps; }

void testButtonTap() {
    Stage stage(640, 360);
    ui::Button button(stage.root(), {100, 100, 220, 64}, "GO");
    int taps = 0;
    button.onTap([&] { ++taps; });

    CHECK(stage.pointerDown({150, 120}));
    CHECK(button.isPressed());
    stage.advance(0.2f);  // settle the press animation
    CHECK(near(button.layer().presentedScale().x, 0.96f));

    CHECK(stage.pointerUp({150, 120}));
    CHECK(taps == 1);
    CHECK(!button.isPressed());
    stage.advance(0.2f);
    CHECK(near(button.layer().presentedScale().x, 1.0f));

    // A press outside any control is not consumed.
    CHECK(!stage.pointerDown({600, 300}));
}

void testCaptureAndSlideOff() {
    Stage stage(640, 360);
    ui::Button button(stage.root(), {100, 100, 220, 64}, "GO");
    int taps = 0;
    button.onTap([&] { ++taps; });

    // Down inside, slide out, up outside: tracked but no tap.
    CHECK(stage.pointerDown({150, 120}));
    CHECK(stage.pointerMove({500, 300}));  // capture follows outside
    CHECK(!button.isPressed());            // visually released while outside
    CHECK(stage.pointerMove({150, 120}));
    CHECK(button.isPressed());             // re-enters
    CHECK(stage.pointerMove({500, 300}));
    CHECK(stage.pointerUp({500, 300}));
    CHECK(taps == 0);
    CHECK(!button.isPressed());

    // Cancel releases the press without firing.
    stage.pointerDown({150, 120});
    stage.pointerCancel();
    CHECK(taps == 0);
    CHECK(!button.isPressed());
}

void testDisabledFallsThrough() {
    Stage stage(640, 360);
    int background_hits = 0;
    stage.rect({0, 0, 640, 360}).color({0, 0, 0, 0.2f}).onPointer(
        [&](const PointerEvent& e) {
            if (e.phase == PointerPhase::Down) {
                ++background_hits;
            }
        });
    ui::Button button(stage.root(), {100, 100, 220, 64}, "GO");
    int taps = 0;
    button.onTap([&] { ++taps; });

    button.enabled(false);
    stage.advance(0.2f);
    CHECK(near(button.layer().presentedOpacity(), 0.4f));
    CHECK(stage.pointerDown({150, 120}));  // consumed by the background now
    stage.pointerUp({150, 120});
    CHECK(taps == 0);
    CHECK(background_hits == 1);

    button.enabled(true);
    stage.pointerDown({150, 120});
    stage.pointerUp({150, 120});
    CHECK(taps == 1);
    CHECK(background_hits == 1);
}

void testSwitchToggle() {
    Stage stage(640, 360);
    ui::Switch sw(stage.root(), {100, 100, 96, 48});
    bool last = false;
    int changes = 0;
    sw.onChange([&](bool on) {
        last = on;
        ++changes;
    });

    CHECK(!sw.isOn());
    const float knob_off = sw.layer().sublayers()[2]->presentedPosition().x;
    CHECK(near(knob_off, 24));  // radius

    stage.pointerDown({140, 120});
    stage.pointerUp({140, 120});
    CHECK(sw.isOn());
    CHECK(changes == 1 && last == true);
    stage.advance(0.075f);  // mid-toggle: knob between the endpoints
    const float knob_mid = sw.layer().sublayers()[2]->presentedPosition().x;
    CHECK(knob_mid > 25 && knob_mid < 71);
    stage.advance(0.2f);
    CHECK(near(sw.layer().sublayers()[2]->presentedPosition().x, 96 - 24));

    // Programmatic set does not fire onChange.
    sw.setOn(false, false);
    CHECK(changes == 1);
    CHECK(near(sw.layer().sublayers()[2]->presentedPosition().x, 24));
}

void testControlRemoveMidGesture() {
    Stage stage(640, 360);
    auto* button = new ui::Button(stage.root(), {100, 100, 220, 64}, "GO");
    stage.pointerDown({150, 120});
    button->remove();  // capture must be dropped, not dangle
    delete button;
    CHECK(!stage.pointerMove({150, 120}));
    CHECK(!stage.pointerUp({150, 120}));
    CHECK(stage.hitTest({150, 120}) == nullptr ||
          stage.hitTest({150, 120})->id() == "root");
}

void testSlider() {
    Stage stage(640, 360);
    ui::Slider slider(stage.root(), {100, 100, 240, 32}, 0.0f);
    float last = -1;
    int changes = 0;
    slider.onChange([&](float v) {
        last = v;
        ++changes;
    });

    // Down mid-track jumps there; drag tracks the pointer with no lag.
    stage.pointerDown({220, 116});  // local x = 120 → (120-16)/(240-32) = 0.5
    CHECK(near(slider.value(), 0.5f, 0.01f));
    CHECK(changes == 1);
    stage.pointerMove({324, 116});  // local x = 224 → 1.0
    CHECK(near(slider.value(), 1.0f, 0.01f));
    stage.pointerMove({0, 116});    // clamps at 0
    stage.pointerUp({0, 116});
    CHECK(near(slider.value(), 0.0f));
    CHECK(last >= 0 && changes >= 3);

    // Programmatic set animates and does not fire onChange.
    const int before = changes;
    slider.setValue(0.8f);
    CHECK(changes == before);
    stage.advance(0.075f);
    const float mid = slider.value();
    CHECK(near(mid, 0.8f));  // model value set immediately
}

void testSegmented() {
    Stage stage(640, 360);
    ui::Segmented seg(stage.root(), {100, 100, 300, 44}, {"手動", "巡回", "追従"}, 0);
    int last = -1;
    seg.onChange([&](int i) { last = i; });

    CHECK(seg.selected() == 0);
    stage.pointerDown({350, 120});  // third segment (local x = 250)
    stage.pointerUp({350, 120});
    CHECK(seg.selected() == 2);
    CHECK(last == 2);
    // Pill slides toward the third segment center (x = 250).
    stage.advance(0.075f);
    const float pill_x = seg.layer().sublayers()[1]->presentedPosition().x;
    CHECK(pill_x > 60 && pill_x < 250);
    stage.advance(0.2f);
    CHECK(near(seg.layer().sublayers()[1]->presentedPosition().x, 250, 0.5f));

    // Tapping the selected segment fires nothing.
    stage.pointerDown({350, 120});
    stage.pointerUp({350, 120});
    CHECK(last == 2);
}

void testGauge() {
    Stage stage(640, 360);
    ui::Gauge gauge(stage.root(), {320, 180}, 60);
    gauge.setValue(0.64f);
    CHECK(near(gauge.value(), 0.64f));
    const auto* arc = std::get_if<ArcContent>(&gauge.layer().sublayers()[1]->content());
    CHECK(arc != nullptr);
    CHECK(near(arc->end_deg - arc->start_deg, 270 * 0.64f, 0.1f));
    const auto* text =
        std::get_if<TextContent>(&gauge.layer().sublayers()[2]->content());
    CHECK(text != nullptr && text->utf8 == "64%");
}

void testDropdown() {
    Stage stage(640, 360);
    ui::Dropdown dd(stage.root(), {100, 40, 220, 48}, {"標準", "低速", "高速", "点検"}, 0);
    int last = -1;
    dd.onChange([&](int i) { last = i; });

    CHECK(!dd.isOpen());
    stage.pointerDown({150, 60});
    stage.pointerUp({150, 60});
    CHECK(dd.isOpen());

    // Pick the third row: popup starts at y = 40+48+4 = 92, rows are 44
    // tall after a 6px pad → row 2 spans y 186..230.
    stage.pointerDown({150, 200});
    stage.pointerUp({150, 200});
    CHECK(!dd.isOpen());
    CHECK(dd.selected() == 2);
    CHECK(last == 2);
    const auto* label =
        std::get_if<TextContent>(&dd.layer().sublayers()[1]->content());
    CHECK(label != nullptr && label->utf8 == "高速");

    // Open again, tap outside → closes without changing the value.
    stage.pointerDown({150, 60});
    stage.pointerUp({150, 60});
    CHECK(dd.isOpen());
    stage.pointerDown({600, 340});
    CHECK(!dd.isOpen());
    stage.pointerUp({600, 340});
    CHECK(dd.selected() == 2);
    CHECK(last == 2);
}

void testDropdownScroll() {
    Stage stage(640, 600);
    std::vector<std::string> options;
    for (int i = 0; i < 12; ++i) {
        options.push_back("opt" + std::to_string(i));
    }
    ui::Dropdown dd(stage.root(), {100, 40, 220, 48}, options, 0);
    int last = -1;
    dd.onChange([&](int i) { last = i; });

    stage.pointerDown({150, 60});
    stage.pointerUp({150, 60});
    CHECK(dd.isOpen());

    // Drag up 100 units: scrolls, and the release selects nothing.
    stage.pointerDown({150, 200});
    stage.pointerMove({150, 100});
    stage.pointerUp({150, 100});
    CHECK(dd.isOpen());
    CHECK(dd.selected() == 0 && last == -1);

    // Tap a row through the scrolled list: popup top is y=92, pad 6,
    // scroll −100 → y=120 lands on row (120−92−6+100)/44 = 2.77 → row 2.
    stage.pointerDown({150, 120});
    stage.pointerUp({150, 120});
    CHECK(!dd.isOpen());
    CHECK(dd.selected() == 2);
    CHECK(last == 2);
}

void testRipple() {
    Stage stage(640, 360);
    // A layer with an existing filter: the ripple must keep it as the base.
    auto& water = stage.rect({0, 0, 640, 360}).color({0.2f, 0.3f, 0.4f, 1}).blur(2);
    fx::RippleStyle style;
    style.max_waves = 3;
    {
        fx::Ripple ripple(water, style);
        CHECK(water.filters().size() == 1);

        ripple.splash({100, 100});
        CHECK(ripple.waveCount() == 1);
        CHECK(water.filters().size() == 2);  // base blur + one refraction wave
        CHECK(water.filters()[0].mode == FS_BLUR);
        CHECK(water.filters()[1].mode == FS_RIPPLE);

        ripple.pointerMoved({110, 100});  // 10 < spacing → no wave
        CHECK(ripple.waveCount() == 1);
        ripple.pointerMoved({160, 100});  // trail wave
        ripple.pointerMoved({220, 100});
        CHECK(ripple.waveCount() == 3);
        ripple.splash({300, 100});  // cap: oldest wave recycles
        CHECK(ripple.waveCount() == 3);

        // Waves advance only through tick and dissipate on schedule.
        ripple.tick(0.5f);
        CHECK(ripple.waveCount() == 3);
        ripple.tick(2.0f);
        CHECK(ripple.waveCount() == 0);
        CHECK(water.filters().size() == 1);
    }
    // Destruction restores the layer exactly as found.
    CHECK(water.filters().size() == 1);
    CHECK(water.filters()[0].mode == FS_BLUR);
}

}  // namespace

int main() {
    testButtonTap();
    testCaptureAndSlideOff();
    testDisabledFallsThrough();
    testSwitchToggle();
    testControlRemoveMidGesture();
    testSlider();
    testSegmented();
    testGauge();
    testDropdown();
    testDropdownScroll();
    testRipple();
    if (g_failures == 0) {
        std::printf("all ui_tests passed\n");
        return 0;
    }
    std::fprintf(stderr, "%d failure(s)\n", g_failures);
    return 1;
}
