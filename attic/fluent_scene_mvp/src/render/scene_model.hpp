#pragma once

// Internal: lowers a validated scene (canonical typed IR) plus its plan into
// the flat draw list both renderer backends execute. No GPU types here.

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "fluent_scene/diagnostics.hpp"
#include "fluent_scene/planner.hpp"
#include "fluent_scene/validator.hpp"

namespace fluent_scene {
namespace render {

struct DrawOp {
    enum class Kind { kImage, kBoxes, kCircles, kPolyline, kText };
    Kind kind = Kind::kImage;
    std::string node_id;
    std::string source_input;  // scene input feeding this draw

    // kImage
    std::string fit;  // contain | cover | fill
    std::vector<std::string> effect_chain;  // effect node ids, first-applied first

    // kBoxes
    float color[4] = {1, 1, 1, 1};
    uint64_t max_instances = 0;
    std::string overflow;  // drop_lowest_score | drop_oldest
    bool show_label = false;
    float smoothing = 0.0f;  // EMA time constant (s); 0 = no interpolation

    // kCircles / kPolyline
    uint64_t max_points = 0;
    float radius = 8.0f;      // kCircles
    float thickness = 3.0f;   // kCircles: 0 = filled; kPolyline: stroke width

    // kText
    std::string font_uri;
    float position[2] = {0, 0};
    bool shadow = false;
    std::string default_text;
    uint64_t max_glyphs = 0;
    uint64_t glyph_capacity = 0;
};

// Image-space effect (spec section 7.4 Effects): consumes an image, produces
// an image, with statically bounded kernels/passes.
struct EffectOp {
    enum class Kind { kBlur, kColorTransform };
    Kind kind = Kind::kBlur;
    std::string node_id;
    std::string source_input;   // set when the source is a scene input
    std::string source_effect;  // set when chained onto another effect
    float radius = 4.0f;        // kBlur (backend clamps to 16)
    float brightness = 0.0f;    // kColorTransform
    float contrast = 1.0f;
    float saturation = 1.0f;
    float gamma = 1.0f;
    // When a value came from a $params reference, the scene-parameter name is
    // recorded here so runtime_mutable updates flow through per frame.
    std::string radius_param;
    std::string brightness_param;
    std::string contrast_param;
    std::string saturation_param;
    std::string gamma_param;
};

struct SceneModel {
    std::string name;
    uint32_t width = 0;
    uint32_t height = 0;
    // runtime_mutable f32 scene params: name -> default value.
    std::map<std::string, float> mutable_params;
    std::vector<EffectOp> effects;  // dependency order (sources first)
    std::vector<DrawOp> draws;      // z-order of the exported composite
};

// Builds the model from the exported composite of `plan`. Reports
// compile-phase diagnostics (and returns false) for scenes the stage-2
// backends cannot render yet (e.g. nested composites).
bool buildSceneModel(const ValidationResult& scene, const PlanResult& plan, SceneModel& out,
                     DiagnosticList& diagnostics);

}  // namespace render
}  // namespace fluent_scene
