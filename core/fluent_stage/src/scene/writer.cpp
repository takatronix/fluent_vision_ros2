// writer.cpp — the canonical Scene form: format() (fvsc fmt, §13-8) and
// digest() (§1.3).
//
// There is exactly one canonical form. format() emits a validated document
// with a fixed key order (the metadata tables' order), explicit fields only,
// normalized numbers, and deterministic quoting; digest() is the SHA-256 of
// that text. Reordering mappings, changing comments or whitespace, or
// spelling a default differently cannot change the digest, because none of
// it survives into the canonical form.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "fluent_stage/scene/document.hpp"
#include "fluent_stage/scene/sha256.hpp"

namespace fluent_stage::scene {

namespace {

/// The shortest decimal spelling that parses back to exactly this float.
std::string formatNumber(float v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.6g", static_cast<double>(v));
    if (static_cast<float>(std::strtod(buf, nullptr)) != v) {
        std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(v));
    }
    return buf;
}

std::string quoted(const std::string& s) {
    std::string out = "\"";
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\t': out += "\\t"; break;
            default: out += c;
        }
    }
    out += "\"";
    return out;
}

bool isPlainSafe(const std::string& s) {
    if (s.empty() || s == "true" || s == "false" || s == "null") {
        return false;
    }
    auto safe = [](char c) {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
               c == '_' || c == '-' || c == '.' || c == '/';
    };
    for (char c : s) {
        if (!safe(c)) {
            return false;
        }
    }
    // Anything number-shaped stays quoted so it round-trips as a string.
    char* end = nullptr;
    std::strtod(s.c_str(), &end);
    return !(end != s.c_str() && *end == '\0');
}

std::string numSeq(const float* v, size_t n) {
    std::string out = "[";
    for (size_t i = 0; i < n; ++i) {
        out += (i != 0 ? ", " : "") + formatNumber(v[i]);
    }
    return out + "]";
}

/// One Scene value in flow style. Strings are always double-quoted;
/// identifiers, enums, and references stay plain.
std::string flowValue(const Value& v) {
    if (!v.param.empty()) {
        return "$params." + v.param;
    }
    switch (v.kind) {
        case ValueKind::F32: return formatNumber(v.num[0]);
        case ValueKind::Vec2: return numSeq(v.num, 2);
        case ValueKind::Rect:
        case ValueKind::Color: return numSeq(v.num, 4);
        case ValueKind::Bool: return v.flag ? "true" : "false";
        case ValueKind::Str: return quoted(v.str);
        case ValueKind::Points: {
            std::string out = "[";
            for (size_t i = 0; i < v.points.size(); ++i) {
                const float xy[2] = {v.points[i].x, v.points[i].y};
                out += (i != 0 ? ", " : "") + numSeq(xy, 2);
            }
            return out + "]";
        }
        case ValueKind::Mat23: return numSeq(v.mat, 6);
        case ValueKind::Input: return "$inputs." + v.input;
        case ValueKind::EnumFit:
        case ValueKind::EnumAlign:
        case ValueKind::EnumCap:
        case ValueKind::EnumBlend:
        case ValueKind::EnumEase: return enumWord(v.kind, v.enum_value);
        case ValueKind::MapShadow:
        case ValueKind::MapBorder: break;  // handled as field maps
    }
    return "{}";
}

std::string flowFieldMap(const std::vector<MapField>& fields) {
    if (fields.empty()) {
        return "{}";
    }
    std::string out = "{ ";
    for (size_t i = 0; i < fields.size(); ++i) {
        out += (i != 0 ? ", " : "") + std::string(fields[i].spec->name) + ": " +
               flowValue(fields[i].value);
    }
    return out + " }";
}

std::string flowTransition(const TransitionDecl& t) {
    return "{ duration: " + formatNumber(t.duration) + ", ease: " +
           enumWord(ValueKind::EnumEase, static_cast<int>(t.ease)) + " }";
}

std::string flowFilters(const std::vector<FilterDecl>& filters) {
    std::string out = "[";
    for (size_t i = 0; i < filters.size(); ++i) {
        const FilterDecl& f = filters[i];
        out += (i != 0 ? ", " : "") + std::string("{ ") + f.spec->name + ": ";
        if (f.set_params.empty()) {
            out += "{}";
        } else {
            out += "{ ";
            for (size_t j = 0; j < f.set_params.size(); ++j) {
                const auto& [slot, value] = f.set_params[j];
                out += (j != 0 ? ", " : "") +
                       std::string(f.spec->params[static_cast<size_t>(slot)].name) + ": " +
                       formatNumber(value);
            }
            out += " }";
        }
        out += " }";
    }
    return out + "]";
}

/// Generic canonical emission for the free-form `meta:` block: mapping keys
/// sorted, deterministic scalar spelling.
void emitMeta(const YamlNode& node, std::string& out, int indent) {
    const std::string pad(static_cast<size_t>(indent) * 2, ' ');
    switch (node.kind) {
        case YamlNode::Kind::kNull:
            out += " null\n";
            return;
        case YamlNode::Kind::kScalar: {
            const std::string& s = node.scalar;
            const bool plain = node.style == YamlNode::ScalarStyle::kPlain &&
                               (isPlainSafe(s) || s == "true" || s == "false" ||
                                [&] {
                                    char* end = nullptr;
                                    std::strtod(s.c_str(), &end);
                                    return end != s.c_str() && *end == '\0';
                                }());
            out += " " + (plain ? s : quoted(s)) + "\n";
            return;
        }
        case YamlNode::Kind::kSequence:
            if (node.elements.empty()) {
                out += " []\n";
                return;
            }
            out += "\n";
            for (const YamlNode& e : node.elements) {
                out += pad + "-";
                emitMeta(e, out, indent + 1);
            }
            return;
        case YamlNode::Kind::kMapping: {
            if (node.entries.empty()) {
                out += " {}\n";
                return;
            }
            out += "\n";
            std::vector<const YamlMapEntry*> sorted;
            sorted.reserve(node.entries.size());
            for (const YamlMapEntry& e : node.entries) {
                sorted.push_back(&e);
            }
            std::sort(sorted.begin(), sorted.end(),
                      [](const YamlMapEntry* a, const YamlMapEntry* b) {
                          return a->key < b->key;
                      });
            for (const YamlMapEntry* e : sorted) {
                out += pad + (isPlainSafe(e->key) ? e->key : quoted(e->key)) + ":";
                emitMeta(e->value, out, indent + 1);
            }
            return;
        }
    }
}

void emitLayer(const LayerDecl& layer, std::string& out, int indent) {
    const std::string pad(static_cast<size_t>(indent) * 2, ' ');
    std::string lead = pad + "- ";
    auto line = [&](const std::string& text) {
        out += lead + text + "\n";
        lead = pad + "  ";  // subsequent keys align under the first
    };

    if (!layer.id.empty()) {
        line("id: " + layer.id);
    }
    if (!layer.role.empty()) {
        line("role: " + (isPlainSafe(layer.role) ? layer.role : quoted(layer.role)));
    }
    if (layer.protected_) {
        line("protected: true");
    }
    for (const AttrDecl& attr : layer.attrs) {
        const ValueKind kind = attr.spec->kind;
        if (kind == ValueKind::MapShadow || kind == ValueKind::MapBorder) {
            line(std::string(attr.spec->name) + ": " + flowFieldMap(attr.map));
        } else {
            line(std::string(attr.spec->name) + ": " + flowValue(attr.value));
        }
    }
    if (layer.transition) {
        line("transition: " + flowTransition(*layer.transition));
    }
    if (layer.content) {
        line("content: { " + std::string(layer.content->spec->name) + ": " +
             flowFieldMap(layer.content->fields) + " }");
    }
    if (!layer.filters.empty()) {
        line("filters: " + flowFilters(layer.filters));
    }
    if (!layer.sublayers.empty()) {
        line("sublayers:");
        for (const LayerDecl& sub : layer.sublayers) {
            emitLayer(sub, out, indent + 2);
        }
    }
    if (lead == pad + "- ") {
        line("{}");  // a completely empty layer still needs a node
    }
}

}  // namespace

std::string format(const SceneDoc& doc) {
    std::string out;
    out += "schema: " + std::string(kSchemaId) + "\n";
    out += "stage: { size: [" + formatNumber(doc.width) + ", " + formatNumber(doc.height) +
           "], fit: " + enumWord(ValueKind::EnumFit, static_cast<int>(doc.fit)) + " }\n";

    if (!doc.inputs.empty()) {
        out += "inputs:\n";
        for (const InputDecl& in : doc.inputs) {
            const std::string type_name = inputTypeName(in.type, in.capacity);
            out += "  " + in.name + ": { type: " +
                   (isPlainSafe(type_name) ? type_name : quoted(type_name)) +
                   ", update: " +
                   (in.update == UpdateRate::PerFrame ? "per_frame" : "on_change") +
                   ", fallback: " +
                   (in.fallback == InputFallback::Placeholder ? "placeholder"
                    : in.fallback == InputFallback::Hide      ? "hide"
                                                              : "hold") +
                   " }\n";
        }
    }
    if (!doc.params.empty()) {
        out += "params:\n";
        for (const ParamDecl& p : doc.params) {
            out += "  " + p.name + ": { type: " + paramTypeName(p.type) + ", default: ";
            switch (p.type) {
                case ParamType::F32: out += formatNumber(p.def[0]); break;
                case ParamType::Vec2: out += numSeq(p.def, 2); break;
                case ParamType::Color: out += numSeq(p.def, 4); break;
                case ParamType::Bool: out += p.def_flag ? "true" : "false"; break;
            }
            out += std::string(", runtime_mutable: ") + (p.runtime_mutable ? "true" : "false");
            if (p.has_animate) {
                out += ", animate: { duration: " + formatNumber(p.animate_duration) +
                       ", ease: " +
                       enumWord(ValueKind::EnumEase, static_cast<int>(p.animate_ease)) + " }";
            }
            out += " }\n";
        }
    }

    out += "layers:\n";
    for (const LayerDecl& layer : doc.layers) {
        emitLayer(layer, out, 1);
    }

    if (doc.has_meta) {
        out += "meta:";
        emitMeta(doc.meta, out, 1);
    }
    return out;
}

std::string digest(const SceneDoc& doc) {
    return sha256Hex(format(doc));
}

}  // namespace fluent_stage::scene
