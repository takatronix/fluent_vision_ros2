#pragma once

/// \file document.hpp
/// \brief The validated Scene document model (§1.3) and the parse entry.
///
/// `parseScene()` turns Scene YAML (`.fvs`) into this typed model, running
/// the whole §1.3 contract up front: schema check, unknown-key rejection,
/// type checks, reference resolution ($inputs / $params), and structural
/// rules (required fields, mutually-exclusive fields, unique ids). A
/// document that parses without errors compiles without errors — every
/// rejection happens here, before anything executes.
///
/// The model is canonical: mappings lose their input order (inputs and
/// params are sorted by name, attributes and content fields by table order),
/// which is what makes the digest reorder-invariant and the formatter
/// deterministic. Filters keep declaration order — a filter chain is a
/// sequence, and its order is meaning, not style.

#include <optional>
#include <string>
#include <vector>

#include "fluent_stage/filters.hpp"
#include "fluent_stage/scene/diagnostics.hpp"
#include "fluent_stage/scene/schema.hpp"
#include "fluent_stage/scene/yaml.hpp"
#include "fluent_stage/types.hpp"

namespace fluent_stage::scene {

/// A validated Scene value. One compact struct covers every ValueKind; only
/// the members implied by `kind` are meaningful. `param` carries a
/// `$params.<name>` reference in attribute positions (the referenced param's
/// type has been checked against the attribute's kind).
struct Value {
    ValueKind kind = ValueKind::F32;
    float num[4] = {0, 0, 0, 0};   ///< F32 [0]; Vec2 [0..1]; Rect/Color [0..3].
    bool flag = false;             ///< Bool.
    int enum_value = 0;            ///< Enum* (underlying C++ enum value).
    std::string str;               ///< Str.
    std::vector<Vec2> points;      ///< Points.
    float mat[6] = {1, 0, 0, 1, 0, 0};  ///< Mat23 [a, b, c, d, tx, ty].
    std::string input;             ///< Input: referenced input name.
    std::string param;             ///< Non-empty: `$params.<name>` reference.

    Vec2 vec2() const { return {num[0], num[1]}; }
    Rect rect() const { return {num[0], num[1], num[2], num[3]}; }
    Color color() const { return {num[0], num[1], num[2], num[3]}; }
};

/// One explicitly-set field of a content declaration or shadow/border map.
struct MapField {
    const FieldSpec* spec = nullptr;
    Value value;
    Span span;
};

/// A layer attribute the document explicitly sets.
struct AttrDecl {
    const AttrSpec* spec = nullptr;
    Value value;                 ///< Scalar kinds ($params refs included).
    std::vector<MapField> map;   ///< MapShadow / MapBorder explicit fields.
    Span span;
};

/// A layer's content declaration: which content type, and the explicitly-set
/// fields (own fields and style keys), in table order.
struct ContentDecl {
    const ContentSpec* spec = nullptr;
    std::vector<MapField> fields;
    Span span;

    /// The set value for a field, or nullptr when the document omits it.
    const MapField* find(const std::string& name) const;
};

/// One filter in a layer's chain, resolved against the filter table.
struct FilterDecl {
    const FilterSpec* spec = nullptr;
    Filter value{};   ///< Mode + all parameter slots (defaults filled).
    std::vector<std::pair<int, float>> set_params;  ///< Explicit slots, table order.
    Span span;
};

/// A §9 transition declaration: attribute changes on this layer interpolate.
struct TransitionDecl {
    float duration = 0.3f;
    Ease ease = Ease::InOut;
    bool has_ease = false;  ///< Whether the document spelled the ease out.
};

/// One node of the declared layer tree.
struct LayerDecl {
    std::string id;               ///< Explicit id ("" = auto-assigned).
    std::string role;             ///< §13-5 semantic tag (free-form).
    bool protected_ = false;      ///< §13-4 protected layer (enforced by MCP).
    std::optional<ContentDecl> content;
    std::vector<AttrDecl> attrs;  ///< Table order.
    std::vector<FilterDecl> filters;  ///< Declaration order (chain order).
    std::optional<TransitionDecl> transition;
    std::vector<LayerDecl> sublayers;
    Span span;

    /// The declared attribute, or nullptr when the document omits it.
    const AttrDecl* find(AttrId id) const;
};

/// How often an input is expected to change (informational contract).
enum class UpdateRate { PerFrame, OnChange };

/// One declared input (§1.3 input contract + fallback).
struct InputDecl {
    std::string name;
    InputType type = InputType::ImageRgba8;
    uint32_t capacity = 0;        ///< Sequence bound N (0 for image/text).
    UpdateRate update = UpdateRate::PerFrame;
    InputFallback fallback = InputFallback::Placeholder;
    Span span;
};

/// One declared runtime parameter (§9 animate declaration included).
struct ParamDecl {
    std::string name;
    ParamType type = ParamType::F32;
    float def[4] = {0, 0, 0, 0};  ///< F32 [0]; Vec2 [0..1]; Color [0..3].
    bool def_flag = false;        ///< Bool default.
    bool runtime_mutable = true;
    bool has_animate = false;     ///< Whether setParam interpolates.
    float animate_duration = 0.3f;
    Ease animate_ease = Ease::InOut;
    Span span;
};

/// A validated Scene document — the unit of review, digest, and audit.
struct SceneDoc {
    std::string schema;           ///< Always kSchemaId after validation.
    float width = 1920;           ///< Logical canvas width (stage.size).
    float height = 1080;          ///< Logical canvas height.
    Fit fit = Fit::Contain;       ///< Logical→output aspect policy.
    bool has_stage = false;       ///< Whether the document had a `stage:` block.
    std::vector<InputDecl> inputs;   ///< Sorted by name.
    std::vector<ParamDecl> params;   ///< Sorted by name.
    std::vector<LayerDecl> layers;   ///< Root layers, compositing order.
    YamlNode meta;                ///< Free-form `meta:` block (generated_by, …).
    bool has_meta = false;
    uint32_t layer_count = 0;     ///< Total declared layers.
    uint32_t max_depth = 0;       ///< Deepest nesting (root layers = 1).

    /// The declared input, or nullptr.
    const InputDecl* findInput(const std::string& name) const;
    /// The declared param, or nullptr.
    const ParamDecl* findParam(const std::string& name) const;
};

/// The outcome of `parseScene`: the document plus every diagnostic, in
/// canonical order. The document is meaningful only when `ok()`.
struct ParseResult {
    SceneDoc doc;
    DiagnosticList diagnostics;

    /// True when the document validated without errors.
    bool ok() const { return !diagnostics.hasErrors(); }
};

/// Parses and validates Scene YAML. All rejection happens here: a result
/// with `ok()` compiles into a Stage without further errors (resource limits
/// aside, which are the compiler's CompileOptions).
ParseResult parseScene(const std::string& yaml_text);

/// The canonical, reorder-invariant SHA-256 digest of a validated document
/// (lowercase hex). Two documents that differ only in YAML mapping order,
/// comments, or formatting share a digest; any semantic change breaks it.
std::string digest(const SceneDoc& doc);

/// Serializes a validated document back to canonical Scene YAML (`fvsc
/// fmt`, §13-8): fixed key order, explicit fields only, stable number
/// formatting. Idempotent — formatting a formatted document is identity.
std::string format(const SceneDoc& doc);

/// The machine-readable capability catalog (§13-1): every content type,
/// attribute, filter, input/param type and limit this build understands,
/// as a JSON object (`fvsc describe --json`, MCP `node_types.list`).
std::string describeJson();

}  // namespace fluent_stage::scene
