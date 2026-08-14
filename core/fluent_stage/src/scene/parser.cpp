// parser.cpp — Scene YAML → validated SceneDoc (§1.3 contract).
//
// Everything is rejected here, before execution: schema mismatch, unknown
// keys, type errors, unresolved $inputs/$params references, conflicting or
// missing fields, duplicate ids. Top-level sections are processed in a fixed
// order (schema → stage → inputs → params → layers → meta) so reference
// resolution never depends on the document's mapping order.

#include <algorithm>
#include <cstdlib>
#include <set>
#include <string>

#include "fluent_stage/scene/document.hpp"

namespace fluent_stage::scene {

const MapField* ContentDecl::find(const std::string& name) const {
    for (const MapField& f : fields) {
        if (name == f.spec->name) {
            return &f;
        }
    }
    return nullptr;
}

const AttrDecl* LayerDecl::find(AttrId id) const {
    for (const AttrDecl& a : attrs) {
        if (a.spec->id == id) {
            return &a;
        }
    }
    return nullptr;
}

const InputDecl* SceneDoc::findInput(const std::string& name) const {
    for (const InputDecl& i : inputs) {
        if (i.name == name) {
            return &i;
        }
    }
    return nullptr;
}

const ParamDecl* SceneDoc::findParam(const std::string& name) const {
    for (const ParamDecl& p : params) {
        if (p.name == name) {
            return &p;
        }
    }
    return nullptr;
}

namespace {

constexpr const char* kPhaseNames = "";  // (diagnostics carry Phase enums)

bool isIdentifier(const std::string& s) {
    if (s.empty()) {
        return false;
    }
    auto head = [](char c) {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
    };
    auto body = [&](char c) {
        return head(c) || (c >= '0' && c <= '9') || c == '-';
    };
    if (!head(s[0])) {
        return false;
    }
    for (char c : s) {
        if (!body(c)) {
            return false;
        }
    }
    return true;
}

class Parser {
public:
    explicit Parser(DiagnosticList& diags) : d_(diags) {}

    SceneDoc parse(const YamlNode& root) {
        SceneDoc doc;
        if (!root.isMapping()) {
            error("validate.structure", root.span, "the document root must be a mapping");
            return doc;
        }
        static const std::set<std::string> kTopKeys = {"schema", "stage", "inputs",
                                                       "params", "layers", "meta"};
        for (const YamlMapEntry& e : root.entries) {
            if (!kTopKeys.count(e.key)) {
                error("validate.unknown_key", e.key_span,
                      "unknown top-level key '" + e.key +
                          "' (expected schema, stage, inputs, params, layers, meta)");
            }
        }

        parseSchema(root, doc);
        parseStage(root, doc);
        parseInputs(root, doc);
        parseParams(root, doc);
        parseLayers(root, doc);

        if (const YamlNode* meta = root.find("meta")) {
            if (!meta->isMapping()) {
                error("validate.type", meta->span, "meta must be a mapping");
            } else {
                doc.meta = *meta;
                doc.has_meta = true;
            }
        }
        return doc;
    }

private:
    // -- diagnostics --------------------------------------------------------

    void error(std::string code, Span span, std::string message) {
        d_.add(std::move(code), Severity::kError, Phase::kValidate, span, std::move(message));
    }

    // -- scalar readers -----------------------------------------------------

    bool readNumber(const YamlNode& n, float& out) {
        if (!n.isPlainScalar()) {
            return false;
        }
        const char* text = n.scalar.c_str();
        char* end = nullptr;
        const double v = std::strtod(text, &end);
        if (end == text || *end != '\0') {
            return false;
        }
        out = static_cast<float>(v);
        return true;
    }

    bool readBool(const YamlNode& n, bool& out) {
        if (!n.isPlainScalar()) {
            return false;
        }
        if (n.scalar == "true") {
            out = true;
            return true;
        }
        if (n.scalar == "false") {
            out = false;
            return true;
        }
        return false;
    }

    bool readNumberSeq(const YamlNode& n, size_t count, size_t min_count, float* out) {
        if (!n.isSequence() || n.elements.size() < min_count || n.elements.size() > count) {
            return false;
        }
        for (size_t i = 0; i < n.elements.size(); ++i) {
            if (!readNumber(n.elements[i], out[i])) {
                return false;
            }
        }
        return true;
    }

    // -- typed value parsing ------------------------------------------------

    /// Parses `n` as `kind` into `out`. When `allow_param` and the node is a
    /// `$params.<name>` scalar of a matching type, the reference is stored
    /// instead. Emits its own diagnostics; returns false on failure.
    bool parseValue(const YamlNode& n, ValueKind kind, const SceneDoc& doc, bool allow_param,
                    Value& out) {
        out = Value{};
        out.kind = kind;

        if (n.isPlainScalar() && n.scalar.rfind("$params.", 0) == 0) {
            if (!allow_param) {
                error("validate.reference", n.span,
                      "$params references are not allowed here");
                return false;
            }
            const std::string name = n.scalar.substr(8);
            const ParamDecl* p = doc.findParam(name);
            if (p == nullptr) {
                error("validate.reference", n.span, "undeclared param '" + name + "'");
                return false;
            }
            const bool matches = (kind == ValueKind::F32 && p->type == ParamType::F32) ||
                                 (kind == ValueKind::Vec2 && p->type == ParamType::Vec2) ||
                                 (kind == ValueKind::Color && p->type == ParamType::Color) ||
                                 (kind == ValueKind::Bool && p->type == ParamType::Bool);
            if (!matches) {
                error("validate.type", n.span,
                      "param '" + name + "' has type " + paramTypeName(p->type) +
                          " but a " + toString(kind) + " is required here");
                return false;
            }
            out.param = name;
            return true;
        }

        switch (kind) {
            case ValueKind::F32:
                if (!readNumber(n, out.num[0])) {
                    error("validate.type", n.span, "expected a number");
                    return false;
                }
                return true;
            case ValueKind::Vec2:
                if (!readNumberSeq(n, 2, 2, out.num)) {
                    error("validate.type", n.span, "expected [x, y]");
                    return false;
                }
                return true;
            case ValueKind::Rect:
                if (!readNumberSeq(n, 4, 4, out.num)) {
                    error("validate.type", n.span, "expected [x, y, w, h]");
                    return false;
                }
                return true;
            case ValueKind::Color: {
                out.num[3] = 1.0f;
                const bool ok3 = n.isSequence() && n.elements.size() == 3 &&
                                 readNumberSeq(n, 3, 3, out.num);
                const bool ok4 = !ok3 && n.isSequence() && n.elements.size() == 4 &&
                                 readNumberSeq(n, 4, 4, out.num);
                if (!ok3 && !ok4) {
                    error("validate.type", n.span,
                          "expected a color [r, g, b] or [r, g, b, a] with components 0-1");
                    return false;
                }
                return true;
            }
            case ValueKind::Bool:
                if (!readBool(n, out.flag)) {
                    error("validate.type", n.span, "expected true or false");
                    return false;
                }
                return true;
            case ValueKind::Str:
                if (!n.isScalar()) {
                    error("validate.type", n.span, "expected a string");
                    return false;
                }
                out.str = n.scalar;
                return true;
            case ValueKind::Points: {
                if (!n.isSequence()) {
                    error("validate.type", n.span, "expected a point list [[x, y], ...]");
                    return false;
                }
                out.points.reserve(n.elements.size());
                for (const YamlNode& e : n.elements) {
                    float xy[2] = {0, 0};
                    if (!readNumberSeq(e, 2, 2, xy)) {
                        error("validate.type", e.span, "expected a point [x, y]");
                        return false;
                    }
                    out.points.push_back({xy[0], xy[1]});
                }
                return true;
            }
            case ValueKind::Mat23:
                if (!readNumberSeq(n, 6, 6, out.mat)) {
                    error("validate.type", n.span, "expected [a, b, c, d, tx, ty]");
                    return false;
                }
                return true;
            case ValueKind::Input: {
                if (!n.isPlainScalar() || n.scalar.rfind("$inputs.", 0) != 0) {
                    error("validate.type", n.span,
                          "expected an input reference '$inputs.<name>'");
                    return false;
                }
                const std::string name = n.scalar.substr(8);
                if (doc.findInput(name) == nullptr) {
                    error("validate.reference", n.span, "undeclared input '" + name + "'");
                    return false;
                }
                out.input = name;
                return true;
            }
            case ValueKind::EnumFit:
            case ValueKind::EnumAlign:
            case ValueKind::EnumCap:
            case ValueKind::EnumBlend:
            case ValueKind::EnumEase: {
                if (!n.isPlainScalar() || !parseEnumWord(kind, n.scalar, out.enum_value)) {
                    std::string valid;
                    for (const char* w : enumWords(kind)) {
                        valid += valid.empty() ? w : (std::string(" | ") + w);
                    }
                    error("validate.type", n.span, "expected one of: " + valid);
                    return false;
                }
                return true;
            }
            case ValueKind::MapShadow:
            case ValueKind::MapBorder:
                error("validate.type", n.span, "internal: map kinds are parsed elsewhere");
                return false;
        }
        return false;
    }

    /// Parses a shadow/border-style mapping against `field_specs` into
    /// explicitly-set MapFields (table order).
    bool parseFieldMap(const YamlNode& n, const std::vector<FieldSpec>& field_specs,
                       const SceneDoc& doc, std::vector<MapField>& out) {
        if (n.isNull()) {
            return true;  // `shadow: {}` — all defaults.
        }
        if (!n.isMapping()) {
            error("validate.type", n.span, "expected a mapping");
            return false;
        }
        bool ok = true;
        for (const YamlMapEntry& e : n.entries) {
            const FieldSpec* spec = nullptr;
            for (const FieldSpec& f : field_specs) {
                if (e.key == f.name) {
                    spec = &f;
                    break;
                }
            }
            if (spec == nullptr) {
                std::string valid;
                for (const FieldSpec& f : field_specs) {
                    valid += valid.empty() ? f.name : (std::string(", ") + f.name);
                }
                error("validate.unknown_key", e.key_span,
                      "unknown key '" + e.key + "' (expected: " + valid + ")");
                ok = false;
                continue;
            }
            MapField field;
            field.spec = spec;
            field.span = e.value.span;
            if (!parseValue(e.value, spec->kind, doc, false, field.value)) {
                ok = false;
                continue;
            }
            out.push_back(std::move(field));
        }
        std::stable_sort(out.begin(), out.end(), [&](const MapField& a, const MapField& b) {
            return a.spec < b.spec;  // FieldSpecs live in one table vector: pointer order
        });                          // is table order.
        return ok;
    }

    // -- top-level sections -------------------------------------------------

    void parseSchema(const YamlNode& root, SceneDoc& doc) {
        const YamlNode* n = root.find("schema");
        if (n == nullptr) {
            error("validate.required", root.span, "missing required key 'schema'");
            return;
        }
        if (!n->isScalar() || n->scalar != kSchemaId) {
            error("validate.schema", n->span,
                  std::string("unsupported schema (this build accepts '") + kSchemaId + "')");
            return;
        }
        doc.schema = n->scalar;
    }

    void parseStage(const YamlNode& root, SceneDoc& doc) {
        const YamlNode* n = root.find("stage");
        if (n == nullptr) {
            return;  // defaults: 1920×1080, contain
        }
        if (!n->isMapping()) {
            error("validate.type", n->span, "stage must be a mapping { size, fit }");
            return;
        }
        doc.has_stage = true;
        for (const YamlMapEntry& e : n->entries) {
            if (e.key == "size") {
                float wh[2] = {0, 0};
                if (!readNumberSeq(e.value, 2, 2, wh) || wh[0] <= 0 || wh[1] <= 0) {
                    error("validate.type", e.value.span,
                          "stage.size must be [width, height] with positive numbers");
                    continue;
                }
                doc.width = wh[0];
                doc.height = wh[1];
            } else if (e.key == "fit") {
                Value v;
                if (parseValue(e.value, ValueKind::EnumFit, doc, false, v)) {
                    doc.fit = static_cast<Fit>(v.enum_value);
                }
            } else {
                error("validate.unknown_key", e.key_span,
                      "unknown stage key '" + e.key + "' (expected size, fit)");
            }
        }
    }

    void parseInputs(const YamlNode& root, SceneDoc& doc) {
        const YamlNode* n = root.find("inputs");
        if (n == nullptr) {
            return;
        }
        if (!n->isMapping()) {
            error("validate.type", n->span, "inputs must be a mapping of name → declaration");
            return;
        }
        for (const YamlMapEntry& e : n->entries) {
            if (!isIdentifier(e.key)) {
                error("validate.identifier", e.key_span,
                      "input name '" + e.key + "' is not a valid identifier");
                continue;
            }
            InputDecl decl;
            decl.name = e.key;
            decl.span = e.value.span;
            if (!e.value.isMapping()) {
                error("validate.type", e.value.span,
                      "input declaration must be a mapping { type, update, fallback }");
                continue;
            }
            bool has_type = false;
            for (const YamlMapEntry& f : e.value.entries) {
                if (f.key == "type") {
                    if (!f.value.isScalar() ||
                        !parseInputType(f.value.scalar, decl.type, decl.capacity)) {
                        error("validate.type", f.value.span,
                              "unknown input type (expected image.rgba8, text.utf8, "
                              "sequence<vec2, N>, sequence<detection2d, N>)");
                    } else {
                        has_type = true;
                    }
                } else if (f.key == "update") {
                    if (f.value.isPlainScalar() && f.value.scalar == "per_frame") {
                        decl.update = UpdateRate::PerFrame;
                    } else if (f.value.isPlainScalar() && f.value.scalar == "on_change") {
                        decl.update = UpdateRate::OnChange;
                    } else {
                        error("validate.type", f.value.span,
                              "update must be per_frame | on_change");
                    }
                } else if (f.key == "fallback") {
                    if (f.value.isPlainScalar() && f.value.scalar == "placeholder") {
                        decl.fallback = InputFallback::Placeholder;
                    } else if (f.value.isPlainScalar() && f.value.scalar == "hide") {
                        decl.fallback = InputFallback::Hide;
                    } else if (f.value.isPlainScalar() && f.value.scalar == "hold") {
                        decl.fallback = InputFallback::Hold;
                    } else {
                        error("validate.type", f.value.span,
                              "fallback must be placeholder | hide | hold");
                    }
                } else {
                    error("validate.unknown_key", f.key_span,
                          "unknown input key '" + f.key + "' (expected type, update, fallback)");
                }
            }
            if (!has_type) {
                error("validate.required", e.value.span,
                      "input '" + e.key + "' is missing required key 'type'");
                continue;
            }
            doc.inputs.push_back(std::move(decl));
        }
        std::sort(doc.inputs.begin(), doc.inputs.end(),
                  [](const InputDecl& a, const InputDecl& b) { return a.name < b.name; });
    }

    void parseParams(const YamlNode& root, SceneDoc& doc) {
        const YamlNode* n = root.find("params");
        if (n == nullptr) {
            return;
        }
        if (!n->isMapping()) {
            error("validate.type", n->span, "params must be a mapping of name → declaration");
            return;
        }
        for (const YamlMapEntry& e : n->entries) {
            if (!isIdentifier(e.key)) {
                error("validate.identifier", e.key_span,
                      "param name '" + e.key + "' is not a valid identifier");
                continue;
            }
            ParamDecl decl;
            decl.name = e.key;
            decl.span = e.value.span;
            if (!e.value.isMapping()) {
                error("validate.type", e.value.span,
                      "param declaration must be a mapping { type, default, runtime_mutable, "
                      "animate }");
                continue;
            }
            bool has_type = false;
            const YamlNode* default_node = nullptr;
            for (const YamlMapEntry& f : e.value.entries) {
                if (f.key == "type") {
                    if (!f.value.isPlainScalar() ||
                        !parseParamType(f.value.scalar, decl.type)) {
                        error("validate.type", f.value.span,
                              "param type must be f32 | vec2 | color | bool");
                    } else {
                        has_type = true;
                    }
                } else if (f.key == "default") {
                    default_node = &f.value;  // parsed after `type` is known
                } else if (f.key == "runtime_mutable") {
                    if (!readBool(f.value, decl.runtime_mutable)) {
                        error("validate.type", f.value.span,
                              "runtime_mutable must be true or false");
                    }
                } else if (f.key == "animate") {
                    if (!f.value.isMapping()) {
                        error("validate.type", f.value.span,
                              "animate must be a mapping { duration, ease }");
                        continue;
                    }
                    decl.has_animate = true;
                    bool has_duration = false;
                    for (const YamlMapEntry& a : f.value.entries) {
                        if (a.key == "duration") {
                            if (!readNumber(a.value, decl.animate_duration) ||
                                decl.animate_duration <= 0) {
                                error("validate.type", a.value.span,
                                      "animate.duration must be a positive number of seconds");
                            } else {
                                has_duration = true;
                            }
                        } else if (a.key == "ease") {
                            Value v;
                            if (parseValue(a.value, ValueKind::EnumEase, doc, false, v)) {
                                decl.animate_ease = static_cast<Ease>(v.enum_value);
                            }
                        } else {
                            error("validate.unknown_key", a.key_span,
                                  "unknown animate key '" + a.key +
                                      "' (expected duration, ease)");
                        }
                    }
                    if (!has_duration) {
                        error("validate.required", f.value.span,
                              "animate is missing required key 'duration'");
                    }
                } else {
                    error("validate.unknown_key", f.key_span,
                          "unknown param key '" + f.key +
                              "' (expected type, default, runtime_mutable, animate)");
                }
            }
            if (!has_type) {
                error("validate.required", e.value.span,
                      "param '" + e.key + "' is missing required key 'type'");
                continue;
            }
            if (default_node != nullptr) {
                Value v;
                const ValueKind kind = decl.type == ParamType::F32     ? ValueKind::F32
                                       : decl.type == ParamType::Vec2  ? ValueKind::Vec2
                                       : decl.type == ParamType::Color ? ValueKind::Color
                                                                       : ValueKind::Bool;
                if (parseValue(*default_node, kind, doc, false, v)) {
                    decl.def[0] = v.num[0];
                    decl.def[1] = v.num[1];
                    decl.def[2] = v.num[2];
                    decl.def[3] = v.num[3];
                    decl.def_flag = v.flag;
                }
            } else if (decl.type == ParamType::Color) {
                decl.def[3] = 1.0f;  // an omitted color default is opaque black
            }
            doc.params.push_back(std::move(decl));
        }
        std::sort(doc.params.begin(), doc.params.end(),
                  [](const ParamDecl& a, const ParamDecl& b) { return a.name < b.name; });
    }

    void parseLayers(const YamlNode& root, SceneDoc& doc) {
        const YamlNode* n = root.find("layers");
        if (n == nullptr) {
            error("validate.required", root.span, "missing required key 'layers'");
            return;
        }
        if (!n->isSequence()) {
            error("validate.type", n->span, "layers must be a sequence of layer mappings");
            return;
        }
        for (const YamlNode& e : n->elements) {
            doc.layers.push_back(parseLayer(e, doc, 1));
        }
    }

    // -- layers -------------------------------------------------------------

    LayerDecl parseLayer(const YamlNode& n, SceneDoc& doc, uint32_t depth) {
        LayerDecl layer;
        layer.span = n.span;
        doc.layer_count += 1;
        doc.max_depth = std::max(doc.max_depth, depth);
        if (!n.isMapping()) {
            error("validate.type", n.span, "a layer must be a mapping");
            return layer;
        }
        for (const YamlMapEntry& e : n.entries) {
            if (e.key == "id") {
                if (!e.value.isScalar() || !isIdentifier(e.value.scalar)) {
                    error("validate.identifier", e.value.span,
                          "id must be an identifier (letters, digits, _ , -)");
                    continue;
                }
                if (!seen_ids_.insert(e.value.scalar).second) {
                    error("validate.duplicate_id", e.value.span,
                          "duplicate layer id '" + e.value.scalar + "'");
                    continue;
                }
                layer.id = e.value.scalar;
            } else if (e.key == "role") {
                if (!e.value.isScalar()) {
                    error("validate.type", e.value.span, "role must be a string");
                    continue;
                }
                layer.role = e.value.scalar;
            } else if (e.key == "protected") {
                bool v = false;
                if (!readBool(e.value, v)) {
                    error("validate.type", e.value.span, "protected must be true or false");
                    continue;
                }
                layer.protected_ = v;
            } else if (e.key == "content") {
                parseContent(e.value, doc, layer);
            } else if (e.key == "sublayers") {
                if (!e.value.isSequence()) {
                    error("validate.type", e.value.span, "sublayers must be a sequence");
                    continue;
                }
                for (const YamlNode& s : e.value.elements) {
                    layer.sublayers.push_back(parseLayer(s, doc, depth + 1));
                }
            } else if (e.key == "filters") {
                parseFilters(e.value, layer);
            } else if (e.key == "transition") {
                parseTransition(e.value, doc, layer);
            } else if (e.key == "states") {
                error("validate.phase", e.key_span,
                      "'states' is Phase L4 (UI controls) and not part of fluent.scene/"
                      "v1alpha2");
            } else if (const AttrSpec* spec = findAttrSpec(e.key)) {
                AttrDecl attr;
                attr.spec = spec;
                attr.span = e.value.span;
                if (spec->kind == ValueKind::MapShadow) {
                    if (!parseFieldMap(e.value, shadowFields(), doc, attr.map)) {
                        continue;
                    }
                } else if (spec->kind == ValueKind::MapBorder) {
                    if (!parseFieldMap(e.value, borderFields(), doc, attr.map)) {
                        continue;
                    }
                } else if (spec->id == AttrId::Scale && e.value.isPlainScalar()) {
                    // A bare scalar s promotes to [s, s].
                    float s = 0;
                    if (!readNumber(e.value, s)) {
                        error("validate.type", e.value.span,
                              "scale must be a number or [sx, sy]");
                        continue;
                    }
                    attr.value.kind = ValueKind::Vec2;
                    attr.value.num[0] = s;
                    attr.value.num[1] = s;
                } else {
                    const bool bindable =
                        spec->kind == ValueKind::F32 || spec->kind == ValueKind::Vec2 ||
                        spec->kind == ValueKind::Color || spec->kind == ValueKind::Bool;
                    if (!parseValue(e.value, spec->kind, doc, bindable, attr.value)) {
                        continue;
                    }
                }
                layer.attrs.push_back(std::move(attr));
            } else {
                error("validate.unknown_key", e.key_span,
                      "unknown layer key '" + e.key + "'");
            }
        }

        // frame is sugar over bounds + position; both spellings at once are
        // ambiguous and rejected (same rule the C++ API documents).
        if (layer.find(AttrId::Frame) != nullptr) {
            if (layer.find(AttrId::Bounds) != nullptr) {
                error("validate.conflict", layer.span,
                      "frame and bounds are mutually exclusive (frame derives bounds)");
            }
            if (layer.find(AttrId::Position) != nullptr) {
                error("validate.conflict", layer.span,
                      "frame and position are mutually exclusive (frame derives position)");
            }
        }

        std::stable_sort(layer.attrs.begin(), layer.attrs.end(),
                         [](const AttrDecl& a, const AttrDecl& b) { return a.spec < b.spec; });
        return layer;
    }

    void parseContent(const YamlNode& n, SceneDoc& doc, LayerDecl& layer) {
        if (!n.isMapping() || n.entries.size() != 1) {
            error("validate.type", n.span,
                  "content must be a mapping with exactly one content type key");
            return;
        }
        const YamlMapEntry& e = n.entries.front();
        const ContentSpec* spec = findContentSpec(e.key);
        if (spec == nullptr) {
            std::string valid;
            for (const ContentSpec& c : contentTable()) {
                valid += valid.empty() ? c.name : (std::string(", ") + c.name);
            }
            error("validate.unknown_key", e.key_span,
                  "unknown content type '" + e.key + "' (expected: " + valid + ")");
            return;
        }
        ContentDecl content;
        content.spec = spec;
        content.span = n.span;
        if (!e.value.isNull()) {
            if (!e.value.isMapping()) {
                error("validate.type", e.value.span, "content fields must be a mapping");
                return;
            }
            for (const YamlMapEntry& f : e.value.entries) {
                const FieldSpec* field_spec = spec->find(f.key);
                if (field_spec == nullptr) {
                    std::string valid;
                    for (const FieldSpec& fs : spec->fields) {
                        valid += valid.empty() ? fs.name : (std::string(", ") + fs.name);
                    }
                    error("validate.unknown_key", f.key_span,
                          std::string("unknown ") + spec->name + " key '" + f.key +
                              "' (expected: " + valid + ")");
                    continue;
                }
                MapField field;
                field.spec = field_spec;
                field.span = f.value.span;
                if (!parseValue(f.value, field_spec->kind, doc, false, field.value)) {
                    continue;
                }
                content.fields.push_back(std::move(field));
            }
        }

        // Required fields.
        for (const FieldSpec& fs : spec->fields) {
            if (fs.required && content.find(fs.name) == nullptr) {
                error("validate.required", content.span,
                      std::string(spec->name) + " is missing required field '" + fs.name + "'");
            }
        }
        // Exactly-one-of rules.
        auto exactlyOne = [&](const char* a, const char* b) {
            const bool has_a = content.find(a) != nullptr;
            const bool has_b = content.find(b) != nullptr;
            if (has_a == has_b) {
                error(has_a ? "validate.conflict" : "validate.required", content.span,
                      std::string(spec->name) + " needs exactly one of '" + a + "' or '" + b +
                          "'");
            }
        };
        const std::string name = spec->name;
        if (name == "text") {
            exactlyOne("text", "source");
        } else if (name == "polyline" || name == "polygon" || name == "circles") {
            exactlyOne("points", "source");
        } else if (name == "rect") {
            if (content.find("rect") != nullptr && content.find("size") != nullptr) {
                error("validate.conflict", content.span,
                      "rect takes 'rect' or 'size', not both");
            }
        }
        // Input type compatibility.
        if (const MapField* src = content.find("source")) {
            if (!src->value.input.empty()) {
                const InputDecl* input = doc.findInput(src->value.input);
                InputType expected = InputType::ImageRgba8;
                if (name == "text") {
                    expected = InputType::TextUtf8;
                } else if (name == "boxes") {
                    expected = InputType::SeqDetection2D;
                } else if (name != "image") {
                    expected = InputType::SeqVec2;
                }
                if (input != nullptr && input->type != expected) {
                    error("validate.type", src->span,
                          "input '" + input->name + "' has type " +
                              inputTypeName(input->type, input->capacity) + " but " + name +
                              " needs " + inputTypeName(expected, input->capacity));
                }
            }
        }

        std::stable_sort(content.fields.begin(), content.fields.end(),
                         [](const MapField& a, const MapField& b) { return a.spec < b.spec; });
        layer.content = std::move(content);
    }

    void parseFilters(const YamlNode& n, LayerDecl& layer) {
        if (!n.isSequence()) {
            error("validate.type", n.span,
                  "filters must be a sequence of { name: { params } } entries");
            return;
        }
        for (const YamlNode& e : n.elements) {
            if (!e.isMapping() || e.entries.size() != 1) {
                error("validate.type", e.span,
                      "each filter must be a mapping with exactly one filter name key");
                continue;
            }
            const YamlMapEntry& f = e.entries.front();
            const FilterSpec* spec = nullptr;
            for (const FilterSpec& s : filterTable()) {
                if (f.key == s.name) {
                    spec = &s;
                    break;
                }
            }
            if (spec == nullptr) {
                error("validate.unknown_key", f.key_span,
                      "unknown filter '" + f.key + "' (see fvsc describe --json)");
                continue;
            }
            FilterDecl decl;
            decl.spec = spec;
            decl.span = e.span;
            decl.value.mode = spec->mode;
            for (size_t i = 0; i < spec->params.size(); ++i) {
                decl.value.values[i] = spec->params[i].default_value;
            }
            if (!f.value.isNull()) {
                if (!f.value.isMapping()) {
                    error("validate.type", f.value.span, "filter params must be a mapping");
                    continue;
                }
                for (const YamlMapEntry& p : f.value.entries) {
                    int slot = -1;
                    for (size_t i = 0; i < spec->params.size(); ++i) {
                        if (p.key == spec->params[i].name) {
                            slot = static_cast<int>(i);
                            break;
                        }
                    }
                    if (slot < 0) {
                        std::string valid;
                        for (const FilterParamSpec& ps : spec->params) {
                            valid += valid.empty() ? ps.name : (std::string(", ") + ps.name);
                        }
                        error("validate.unknown_key", p.key_span,
                              std::string("unknown ") + spec->name + " param '" + p.key +
                                  "'" + (valid.empty() ? " (it takes none)"
                                                       : " (expected: " + valid + ")"));
                        continue;
                    }
                    float v = 0;
                    if (!readNumber(p.value, v)) {
                        error("validate.type", p.value.span, "expected a number");
                        continue;
                    }
                    decl.value.values[slot] = v;
                    decl.set_params.emplace_back(slot, v);
                }
                std::sort(decl.set_params.begin(), decl.set_params.end());
            }
            layer.filters.push_back(std::move(decl));
        }
    }

    void parseTransition(const YamlNode& n, const SceneDoc& doc, LayerDecl& layer) {
        if (!n.isMapping()) {
            error("validate.type", n.span, "transition must be a mapping { duration, ease }");
            return;
        }
        TransitionDecl t;
        bool has_duration = false;
        for (const YamlMapEntry& e : n.entries) {
            if (e.key == "duration") {
                if (!readNumber(e.value, t.duration) || t.duration <= 0) {
                    error("validate.type", e.value.span,
                          "transition.duration must be a positive number of seconds");
                } else {
                    has_duration = true;
                }
            } else if (e.key == "ease") {
                Value v;
                if (parseValue(e.value, ValueKind::EnumEase, doc, false, v)) {
                    t.ease = static_cast<Ease>(v.enum_value);
                    t.has_ease = true;
                }
            } else {
                error("validate.unknown_key", e.key_span,
                      "unknown transition key '" + e.key + "' (expected duration, ease)");
            }
        }
        if (!has_duration) {
            error("validate.required", n.span, "transition is missing required key 'duration'");
            return;
        }
        layer.transition = t;
    }

    DiagnosticList& d_;
    std::set<std::string> seen_ids_;
};

}  // namespace

ParseResult parseScene(const std::string& yaml_text) {
    ParseResult result;
    const YamlNode root = parseYaml(yaml_text, result.diagnostics);
    if (!result.diagnostics.hasErrors()) {
        Parser parser(result.diagnostics);
        result.doc = parser.parse(root);
    }
    result.diagnostics.sortCanonical();
    return result;
}

}  // namespace fluent_stage::scene
