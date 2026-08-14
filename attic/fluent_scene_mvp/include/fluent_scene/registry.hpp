#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "fluent_scene/json.hpp"
#include "fluent_scene/types.hpp"

namespace fluent_scene {

// Declarative typed signature of an input port. Registration defines the type
// contract; scenes can never supply native code (spec section 7.3).
enum class PortPatternKind {
    kExact,              // exactly `exact_kind`
    kAnyImage,           // any image.* type
    kDetectionSequence,  // sequence<struct{bbox: vec4f, score: f32, label: string, ...}, N>
    kLayerList,          // list of `layer`-typed references
    kAnyPoints,          // points2d or polyline2d
};

struct PortSpec {
    std::string name;
    PortPatternKind pattern = PortPatternKind::kExact;
    TypeKind exact_kind = TypeKind::kString;
    bool required = true;
};

enum class ParamKind { kBool, kU32, kF32, kVec2f, kVec4f, kString, kEnum, kFontResource };

struct ParamSpec {
    std::string name;
    ParamKind kind = ParamKind::kString;
    bool required = false;
    std::vector<std::string> enum_values;
    JsonValue default_value;  // kNull when the parameter has no default
};

struct OutputSpec {
    std::string name;
    TypeKind type = TypeKind::kLayer;
};

// Dynamic-instance contract (spec section 7.5): node types that render dynamic
// counts must declare a maximum, and scenes must state capacity and overflow.
struct BoundsSpec {
    bool required = false;
    std::string count_key;                     // e.g. "max_instances"
    uint64_t max_count = 0;                    // registry hard cap
    std::vector<std::string> overflow_rules;   // deterministic rules only
};

struct NodeTypeSpec {
    std::string name;
    std::vector<PortSpec> inputs;
    std::vector<ParamSpec> params;
    std::vector<OutputSpec> outputs;
    BoundsSpec bounds;

    const PortSpec* findInput(const std::string& port) const;
    const ParamSpec* findParam(const std::string& param) const;
    const OutputSpec* findOutput(const std::string& output) const;
};

// In-memory node registry. The MVP registry covers the declarative signatures
// required by spec section 14.1: image, boxes, text, and composition.
class NodeRegistry {
public:
    void add(NodeTypeSpec spec);
    const NodeTypeSpec* find(const std::string& name) const;
    const std::map<std::string, NodeTypeSpec>& all() const { return specs_; }

    static NodeRegistry builtinMvp();

private:
    std::map<std::string, NodeTypeSpec> specs_;
};

// Maximum number of layers a composite node accepts.
constexpr uint64_t kMaxCompositeLayers = 64;

}  // namespace fluent_scene
