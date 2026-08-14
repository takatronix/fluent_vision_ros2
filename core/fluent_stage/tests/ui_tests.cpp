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

}  // namespace

int main() {
    testButtonTap();
    testCaptureAndSlideOff();
    testDisabledFallsThrough();
    testSwitchToggle();
    testControlRemoveMidGesture();
    if (g_failures == 0) {
        std::printf("all ui_tests passed\n");
        return 0;
    }
    std::fprintf(stderr, "%d failure(s)\n", g_failures);
    return 1;
}
