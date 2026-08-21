#pragma once

#include <string>

#include "fluent_scene/diagnostics.hpp"
#include "fluent_scene/json.hpp"
#include "fluent_scene/validator.hpp"

namespace fluent_scene {

// Identity of the backend-neutral plan format produced by this planner
// (roadmap stage 1: backend-neutral resource/pass IR and lifetime planner).
extern const char kSupportedPlanIdentity[];

struct PlanResult {
    bool ok = false;         // true only when no error diagnostics were produced
    JsonValue plan;          // canonical resource/pass plan (meaningful only when ok)
    std::string plan_text;   // deterministic serialization of `plan`
    std::string digest;      // "sha256:<hex>" of plan_text; empty unless ok
};

// Lowers a successfully validated scene (its canonical typed IR) into a
// backend-neutral resource/pass plan with lifetime intervals, and rejects
// scenes whose worst-case resource footprint exceeds their declared budgets.
// All problems are reported as compile-phase diagnostics; never throws.
//
// Sizing model (worst case, documented so budgets are auditable):
//   - imported/target images are sized at budgets.max_width x max_height
//   - image bytes/pixel: r8=1, rgba8=4, depth32f=4, rgba16f=8
//   - boxes instance buffer: bounds.max_instances x 32 bytes
//   - text glyph buffer: bounds.max_glyphs x 16 bytes
//   - font atlas: glyph_capacity x 1024 bytes (32x32 R8 per glyph), retained
//   - staging pool (host memory): budgets.max_upload_bytes_per_frame
// Per-frame upload budget counts only inputs declared `update: per_frame`.
PlanResult planScene(const ValidationResult& validated, DiagnosticList& diagnostics);

}  // namespace fluent_scene
