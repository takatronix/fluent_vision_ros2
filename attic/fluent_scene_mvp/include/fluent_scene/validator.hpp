#pragma once

#include <string>

#include "fluent_scene/diagnostics.hpp"
#include "fluent_scene/json.hpp"
#include "fluent_scene/registry.hpp"
#include "fluent_scene/yaml.hpp"

namespace fluent_scene {

// The exact schema identity supported by this validator (spec section 7.2:
// before v1, the validator must name the exact supported identities).
extern const char kSupportedSchemaIdentity[];

struct ValidationResult {
    bool ok = false;         // true only when no error diagnostics were produced
    JsonValue ir;            // canonical typed IR (meaningful only when ok)
    std::string ir_text;     // deterministic serialization of `ir`
    std::string digest;      // "sha256:<hex>" of ir_text; empty unless ok
};

// Validates a parsed `.fvs` document against the schema and `registry`, and on
// success emits the canonical typed IR plus its digest. All problems are
// reported as validate-phase diagnostics; the function never throws.
ValidationResult validateScene(const YamlNode& root, const NodeRegistry& registry,
                               DiagnosticList& diagnostics);

}  // namespace fluent_scene
