#pragma once

// Internal: lowers a validated scene (canonical typed IR) plus its plan into
// the flat draw list both renderer backends execute. No GPU types here.

#include <cstdint>
#include <string>
#include <vector>

#include "fluent_scene/diagnostics.hpp"
#include "fluent_scene/planner.hpp"
#include "fluent_scene/validator.hpp"

namespace fluent_scene {
namespace render {

struct DrawOp {
    enum class Kind { kImage, kBoxes, kText };
    Kind kind = Kind::kImage;
    std::string node_id;
    std::string source_input;  // scene input feeding this draw

    // kImage
    std::string fit;  // contain | cover | fill

    // kBoxes
    float color[4] = {1, 1, 1, 1};
    uint64_t max_instances = 0;
    std::string overflow;  // drop_lowest_score | drop_oldest
    bool show_label = false;

    // kText
    std::string font_uri;
    float position[2] = {0, 0};
    bool shadow = false;
    std::string default_text;
    uint64_t max_glyphs = 0;
    uint64_t glyph_capacity = 0;
};

struct SceneModel {
    std::string name;
    uint32_t width = 0;
    uint32_t height = 0;
    std::vector<DrawOp> draws;  // z-order of the exported composite
};

// Builds the model from the exported composite of `plan`. Reports
// compile-phase diagnostics (and returns false) for scenes the stage-2
// backends cannot render yet (e.g. nested composites).
bool buildSceneModel(const ValidationResult& scene, const PlanResult& plan, SceneModel& out,
                     DiagnosticList& diagnostics);

}  // namespace render
}  // namespace fluent_scene
