// scene_tests — the L2 contract: parse/validate rejection, reorder-invariant
// digest, canonical fmt idempotence, metadata-table honesty against the C++
// structs, the §2 YAML ↔ C++ pixel-identity golden, $params animation, and
// the design linter (§13-2).

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <fluent_stage/cpu_renderer.hpp>
#include <fluent_stage/fluent_stage.hpp>
#include <fluent_stage/scene/binding.hpp>
#include <fluent_stage/scene/compiler.hpp>
#include <fluent_stage/scene/document.hpp>
#include <fluent_stage/scene/inspector.hpp>
#include <fluent_stage/scene/linter.hpp>

using namespace fluent_stage;
namespace fs = fluent_stage::scene;

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

// The §2 canonical document, exactly as the design writes it.
const char* kCanonicalScene = R"(schema: fluent.scene/v1alpha2
inputs:
  camera:     { type: image.rgba8, update: per_frame }
  detections: { type: "sequence<Detection2D, 128>", update: per_frame }
layers:
  - content: { image: { source: $inputs.camera } }
  - content: { boxes: { source: $inputs.detections,
                        color: [0.1, 0.9, 0.7, 1], smoothing: 0.2 } }
  - id: hud
    position: [24, 24]
    opacity: 0.9
    shadow: {}
    sublayers:
      - content: { rect: { size: [340, 96], corner_radius: 12,
                           color: [0, 0, 0, 0.45] } }
      - content: { text: { text: "走行中", position: [16, 12], size: 28 } }
)";

bool hasCode(const fs::DiagnosticList& diags, const std::string& code) {
    for (const fs::Diagnostic& d : diags.items()) {
        if (d.code == code) {
            return true;
        }
    }
    return false;
}

void testCanonicalSceneParses() {
    const fs::ParseResult r = fs::parseScene(kCanonicalScene);
    for (const auto& d : r.diagnostics.items()) {
        std::fprintf(stderr, "  diag: [%s] %s\n", d.code.c_str(), d.message.c_str());
    }
    CHECK(r.ok());
    CHECK(r.doc.layer_count == 5);
    CHECK(r.doc.inputs.size() == 2);
    CHECK(r.doc.layers.size() == 3);
    CHECK(r.doc.layers[2].id == "hud");
}

void testRejections() {
    auto errorsOf = [](const std::string& body) {
        std::string text = "schema: fluent.scene/v1alpha2\n" + body;
        return fs::parseScene(text).diagnostics;
    };
    // Unknown keys, at every level.
    CHECK(hasCode(errorsOf("layers:\n  - contnet: { rect: {} }\n"), "validate.unknown_key"));
    CHECK(hasCode(errorsOf("layers:\n  - content: { rects: {} }\n"), "validate.unknown_key"));
    CHECK(hasCode(errorsOf("layers:\n  - content: { rect: { radius: 4 } }\n"),
                  "validate.unknown_key"));
    // Type errors.
    CHECK(hasCode(errorsOf("layers:\n  - opacity: nope\n"), "validate.type"));
    CHECK(hasCode(errorsOf("layers:\n  - blend: darken\n"), "validate.type"));
    // Undeclared references are rejected before execution.
    CHECK(hasCode(errorsOf("layers:\n  - content: { image: { source: $inputs.cam } }\n"),
                  "validate.reference"));
    CHECK(hasCode(errorsOf("layers:\n  - opacity: $params.alpha\n"), "validate.reference"));
    // Input type must match the content that consumes it.
    CHECK(hasCode(errorsOf("inputs:\n  t: { type: text.utf8 }\nlayers:\n"
                           "  - content: { image: { source: $inputs.t } }\n"),
                  "validate.type"));
    // frame is sugar; spelling both is ambiguous.
    CHECK(hasCode(errorsOf("layers:\n  - frame: [0, 0, 10, 10]\n    bounds: [0, 0, 10, 10]\n"),
                  "validate.conflict"));
    // Required and exactly-one-of fields.
    CHECK(hasCode(errorsOf("layers:\n  - content: { circle: { radius: 5 } }\n"),
                  "validate.required"));
    CHECK(hasCode(errorsOf("layers:\n  - content: { text: {} }\n"), "validate.required"));
    // states is L4.
    CHECK(hasCode(errorsOf("layers:\n  - states: {}\n"), "validate.phase"));
    // Duplicate ids.
    CHECK(hasCode(errorsOf("layers:\n  - id: a\n  - id: a\n"), "validate.duplicate_id"));
    // A wrong schema is refused outright.
    CHECK(hasCode(fs::parseScene("schema: fluent.scene/v9\nlayers: []\n").diagnostics,
                  "validate.schema"));
}

void testDigestAndFmt() {
    const fs::ParseResult a = fs::parseScene(kCanonicalScene);
    CHECK(a.ok());

    // Reordering mappings cannot change the digest.
    const char* reordered = R"(layers:
  - content: { image: { source: $inputs.camera } }
  - content: { boxes: { smoothing: 0.2, color: [0.1, 0.9, 0.7, 1], source: $inputs.detections } }
  - shadow: {}
    opacity: 0.9
    position: [24, 24]
    id: hud
    sublayers:
      - content: { rect: { color: [0, 0, 0, 0.45], corner_radius: 12, size: [340, 96] } }
      - content: { text: { size: 28, position: [16, 12], text: "走行中" } }
inputs:
  detections: { type: "sequence<Detection2D, 128>", update: per_frame }
  camera: { type: image.rgba8, update: per_frame }
schema: fluent.scene/v1alpha2
)";
    const fs::ParseResult b = fs::parseScene(reordered);
    CHECK(b.ok());
    CHECK(fs::digest(a.doc) == fs::digest(b.doc));

    // A semantic change breaks it.
    std::string changed = kCanonicalScene;
    const size_t pos = changed.find("opacity: 0.9");
    changed.replace(pos, 12, "opacity: 0.8");
    const fs::ParseResult c = fs::parseScene(changed);
    CHECK(c.ok());
    CHECK(fs::digest(a.doc) != fs::digest(c.doc));

    // fmt is canonical and idempotent, and preserves the digest.
    const std::string once = fs::format(a.doc);
    const fs::ParseResult re = fs::parseScene(once);
    CHECK(re.ok());
    CHECK(fs::format(re.doc) == once);
    CHECK(fs::digest(re.doc) == fs::digest(a.doc));
}

// The metadata tables document defaults; the C++ structs own them. Keep the
// two honest against each other (single definition, enforced).
void testTableDefaultsMatchCpp() {
    auto fieldDefault = [](const char* content, const char* field) {
        const fs::FieldSpec* spec = fs::findContentSpec(content)->find(field);
        CHECK(spec != nullptr && spec->def.has_value);
        return spec->def;
    };
    const TextContent text{};
    CHECK(fieldDefault("text", "size").num[0] == text.size);
    CHECK(std::string(fieldDefault("text", "align").str) == "left");
    const CirclesContent circles{};
    CHECK(fieldDefault("circles", "radius").num[0] == circles.radius);
    const CrosshairContent crosshair{};
    CHECK(fieldDefault("crosshair", "size").num[0] == crosshair.size);
    const GridContent grid{};
    CHECK(fieldDefault("grid", "spacing").num[0] == grid.spacing);
    const ArrowContent arrow{};
    CHECK(fieldDefault("arrow", "head_size").num[0] == arrow.head_size);
    const ImageContent image{};
    CHECK(image.fit == Fit::Contain &&
          std::string(fieldDefault("image", "fit").str) == "contain");

    // Shadow/border table defaults are read from the structs directly; spot
    // check the wiring.
    const Shadow shadow{};
    CHECK(fs::shadowFields()[1].def.num[0] == shadow.radius);
    const Border border{};
    CHECK(fs::borderFields()[0].def.num[0] == border.width);
}

// -- the §2 golden: YAML and C++ must produce the identical picture ---------

std::vector<uint8_t> makeCameraPixels(uint32_t w, uint32_t h) {
    std::vector<uint8_t> pixels(static_cast<size_t>(w) * h * 4);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            uint8_t* p = &pixels[(static_cast<size_t>(y) * w + x) * 4];
            p[0] = static_cast<uint8_t>(x * 255 / w);
            p[1] = static_cast<uint8_t>(y * 255 / h);
            p[2] = ((x / 16 + y / 16) % 2) ? 200 : 60;
            p[3] = 255;
        }
    }
    return pixels;
}

std::vector<Box> makeDetections() {
    return {{{400, 300, 280, 220}, 0.92f, "asparagus", 7},
            {{900, 500, 200, 260}, 0.71f, "asparagus", 8}};
}

void testYamlCppParity() {
    const auto cam_pixels = makeCameraPixels(320, 240);
    const ImageView cam{320, 240, cam_pixels.data(), 0};
    const std::vector<Box> dets = makeDetections();

    // YAML side.
    const fs::ParseResult parsed = fs::parseScene(kCanonicalScene);
    CHECK(parsed.ok());
    fs::CompileResult compiled = fs::compile(parsed.doc);
    CHECK(compiled.ok());
    fs::CompiledScene& scene = *compiled.scene;
    CHECK(scene.setImage("camera", cam));
    CHECK(scene.setBoxes("detections", dets));

    // C++ side — §2 of the design, verbatim.
    Stage stage(1920, 1080);
    stage.image(cam);
    stage.boxes(dets).color(Color::Teal).smoothing(0.2f);
    auto& hud = stage.group("hud").position(24, 24).opacity(0.9f).shadow();
    hud.rect({0, 0, 340, 96}).cornerRadius(12).color({0, 0, 0, 0.45f});
    hud.text("走行中", {16, 12}).size(28);

    CpuRenderer ra, rb;
    const Surface& fa = ra.render(scene.stage(), 480, 270, 0.0f);
    const Surface& fb = rb.render(stage, 480, 270, 0.0f);
    CHECK(fa.width == fb.width && fa.height == fb.height);
    size_t diff = 0;
    for (uint32_t y = 0; y < fa.height; ++y) {
        if (std::memcmp(fa.row(y), fb.row(y), static_cast<size_t>(fa.width) * 4) != 0) {
            const uint8_t* pa = fa.row(y);
            const uint8_t* pb = fb.row(y);
            for (uint32_t i = 0; i < fa.width * 4; ++i) {
                diff += pa[i] != pb[i];
            }
        }
    }
    if (diff != 0) {
        std::fprintf(stderr, "  parity: %zu differing bytes\n", diff);
    }
    CHECK(diff == 0);

    // Feeding data replaced the placeholder (no leftover synthetic layers
    // beyond the declared 5 + root).
    CHECK(scene.stage().layerCount() == 6);
}

void testParamsAnimateAndInputs() {
    const char* text = R"(schema: fluent.scene/v1alpha2
params:
  hud_alpha: { type: f32, default: 1.0, runtime_mutable: true,
               animate: { duration: 0.4, ease: linear } }
  frozen: { type: f32, default: 3, runtime_mutable: false }
layers:
  - id: hud
    opacity: $params.hud_alpha
    content: { rect: { size: [100, 50] } }
)";
    const fs::ParseResult parsed = fs::parseScene(text);
    CHECK(parsed.ok());
    fs::CompileResult compiled = fs::compile(parsed.doc);
    CHECK(compiled.ok());
    fs::CompiledScene& scene = *compiled.scene;
    Layer* hud = scene.stage().find("hud");
    CHECK(hud != nullptr);
    CHECK(near(hud->opacity(), 1.0f));  // starts at the param default

    // setParam with an animate declaration interpolates (§9).
    CHECK(scene.setParam("hud_alpha", 0.0f));
    scene.stage().advance(0.2f);
    CHECK(near(hud->presentedOpacity(), 0.5f, 0.02f));
    scene.stage().advance(0.3f);
    CHECK(near(hud->presentedOpacity(), 0.0f));

    // Type and mutability gates hold.
    CHECK(!scene.setParam("hud_alpha", true));
    CHECK(!scene.setParam("frozen", 1.0f));
    CHECK(!scene.setParam("nope", 1.0f));
    CHECK(!scene.drainDiagnostics().empty());
}

void testLinter() {
    const char* text = R"(schema: fluent.scene/v1alpha2
stage: { size: [640, 360] }
layers:
  - content: { rect: { rect: [0, 0, 640, 360], color: [0.82, 0.82, 0.82, 1] } }
  - id: faint
    content: { text: { text: "low contrast", position: [40, 40], size: 30 } }
  - id: gone
    content: { circle: { center: [2000, 2000], radius: 20 } }
  - id: estop
    protected: true
    content: { rect: { rect: [500, 20, 100, 60], color: [0.9, 0.1, 0.1, 1] } }
  - content: { rect: { rect: [480, 0, 160, 120], color: [0, 0, 0, 1] } }
)";
    const fs::ParseResult parsed = fs::parseScene(text);
    CHECK(parsed.ok());
    fs::CompileResult compiled = fs::compile(parsed.doc);
    CHECK(compiled.ok());
    CpuRenderer renderer;
    const fs::DiagnosticList lints = fs::lint(*compiled.scene, renderer);
    // White text on a light plate: unreadable, flagged.
    CHECK(hasCode(lints, "lint.contrast"));
    // The circle at (2000, 2000) is outside a 640×360 canvas.
    CHECK(hasCode(lints, "lint.offscreen"));
    // The protected E-STOP rect is buried under an opaque plate — an error.
    CHECK(hasCode(lints, "lint.protected_occluded"));
    CHECK(lints.hasErrors());
}

void testInspector() {
    const fs::ParseResult parsed = fs::parseScene(kCanonicalScene);
    CHECK(parsed.ok());
    fs::CompileResult compiled = fs::compile(parsed.doc);
    CHECK(compiled.ok());
    fs::CompiledScene& scene = *compiled.scene;
    CpuRenderer renderer;
    renderer.render(scene.stage(), 0.0f);  // settle text bounds

    const auto placed = fs::placeLayers(scene);
    CHECK(placed.size() >= 6);  // root + 5 declared (placeholders extra)

    // The HUD panel rect wins over the image at a point inside the panel.
    const auto hits = fs::visibleAt(placed, {40, 40});
    CHECK(!hits.empty());
    bool saw_rect = false, saw_image = false;
    int rect_at = -1, image_at = -1;
    for (size_t i = 0; i < hits.size(); ++i) {
        if (std::holds_alternative<RectContent>(hits[i]->layer->content())) {
            saw_rect = true;
            rect_at = static_cast<int>(i);
        }
        if (std::holds_alternative<ImageContent>(hits[i]->layer->content())) {
            saw_image = true;
            image_at = static_cast<int>(i);
        }
    }
    CHECK(saw_rect && saw_image);
    CHECK(rect_at < image_at);  // topmost first

    // Effective opacity multiplies down the chain (hud declares 0.9).
    for (const auto* h : hits) {
        if (std::holds_alternative<RectContent>(h->layer->content())) {
            CHECK(near(h->eff_opacity, 0.9f));
        }
    }

    // A point outside everything hits only parent-filling layers.
    const auto far_hits = fs::visibleAt(placed, {1900, 1060});
    for (const auto* h : far_hits) {
        CHECK(!std::holds_alternative<RectContent>(h->layer->content()));
    }

    const std::string json = fs::inspectJson(scene, placed);
    CHECK(json.find("\"digest\"") != std::string::npos);
    CHECK(json.find("\"hud\"") != std::string::npos);
    CHECK(fs::atJson(placed, {40, 40}).find("\"rect\"") != std::string::npos);
}

void testBindingDoc() {
    const char* kBinding = R"(schema: fluent.binding/v1alpha1
kind: Binding
metadata: { name: hud_ros2 }
scene: { name: camera_detection_hud }
bindings:
  camera:
    source: { adapter: ros2, topic: /camera/color/image_raw,
              message_type: sensor_msgs/msg/Image, qos: sensor_data }
    converter: ros_image_to_rgba8
  detections:
    source: { adapter: ros2, topic: /perception/detections,
              message_type: vision_msgs/msg/Detection2DArray, qos: sensor_data }
    converter: ros_detections_to_detection2d
outputs:
  frame:
    sink: { adapter: ros2, topic: /visualization/composite,
            message_type: sensor_msgs/msg/Image, qos: sensor_data }
    converter: rgba8_to_ros_image
)";
    const fs::BindingParseResult b = fs::parseBinding(kBinding);
    for (const auto& d : b.diagnostics.items()) {
        std::fprintf(stderr, "  bind diag: [%s] %s\n", d.code.c_str(), d.message.c_str());
    }
    CHECK(b.ok());
    CHECK(b.doc.bindings.size() == 2);
    CHECK(b.doc.outputs.size() == 1);
    CHECK(b.doc.findBinding("camera") != nullptr);

    // Cross-validation against the §2 scene: clean.
    const fs::ParseResult scene = fs::parseScene(kCanonicalScene);
    CHECK(scene.ok());
    fs::DiagnosticList cross;
    fs::validateBindingAgainstScene(b.doc, scene.doc, cross);
    CHECK(!cross.hasErrors());

    // A binding naming an undeclared input is an error; a converter of the
    // wrong type is an error.
    const char* kBad = R"(schema: fluent.binding/v1alpha1
bindings:
  nonexistent:
    source: { adapter: ros2, topic: /t, message_type: std_msgs/msg/String }
    converter: ros_string_to_utf8
  camera:
    source: { adapter: ros2, topic: /t2, message_type: std_msgs/msg/String }
    converter: ros_string_to_utf8
)";
    const fs::BindingParseResult bad = fs::parseBinding(kBad);
    CHECK(bad.ok());
    fs::DiagnosticList cross2;
    fs::validateBindingAgainstScene(bad.doc, scene.doc, cross2);
    CHECK(hasCode(cross2, "bind.unknown_input"));
    CHECK(hasCode(cross2, "bind.type_mismatch"));
    CHECK(hasCode(cross2, "bind.unbound_input"));  // detections went unbound

    // Shape rejections: adapter, qos, message type vs converter, relative
    // topic names, unknown converters.
    auto errorsOf = [](const std::string& body) {
        return fs::parseBinding("schema: fluent.binding/v1alpha1\n" + body).diagnostics;
    };
    CHECK(hasCode(errorsOf("bindings:\n  a:\n    source: { adapter: dds, topic: /t }\n"
                           "    converter: ros_string_to_utf8\n"),
                  "bind.adapter"));
    CHECK(hasCode(errorsOf("bindings:\n  a:\n    source: { adapter: ros2, topic: t }\n"
                           "    converter: ros_string_to_utf8\n"),
                  "bind.topic"));
    CHECK(hasCode(errorsOf("bindings:\n  a:\n    source: { adapter: ros2, topic: /t, "
                           "qos: best }\n    converter: ros_string_to_utf8\n"),
                  "bind.qos"));
    CHECK(hasCode(errorsOf("bindings:\n  a:\n    source: { adapter: ros2, topic: /t, "
                           "message_type: sensor_msgs/msg/Image }\n"
                           "    converter: ros_string_to_utf8\n"),
                  "bind.message_type"));
    CHECK(hasCode(errorsOf("bindings:\n  a:\n    source: { adapter: ros2, topic: /t }\n"
                           "    converter: no_such_converter\n"),
                  "bind.converter"));
}

void testDescribe() {
    const std::string json = fs::describeJson();
    CHECK(json.find("\"contents\"") != std::string::npos);
    CHECK(json.find("\"boxes\"") != std::string::npos);
    CHECK(json.find("\"attributes\"") != std::string::npos);
    CHECK(json.find("\"corner_radius\"") != std::string::npos);
    CHECK(json.find("\"filters\"") != std::string::npos);
    // Every filter in the table appears.
    for (const FilterSpec& f : filterTable()) {
        CHECK(json.find("\"" + std::string(f.name) + "\"") != std::string::npos);
    }
}

}  // namespace

int main() {
    testCanonicalSceneParses();
    testRejections();
    testDigestAndFmt();
    testTableDefaultsMatchCpp();
    testYamlCppParity();
    testParamsAnimateAndInputs();
    testLinter();
    testInspector();
    testBindingDoc();
    testDescribe();
    if (g_failures == 0) {
        std::printf("scene_tests: all passed\n");
        return 0;
    }
    std::fprintf(stderr, "scene_tests: %d failure(s)\n", g_failures);
    return 1;
}
