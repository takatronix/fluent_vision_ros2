// describe.cpp — the §13-1 capability catalog: everything this build's Scene
// vocabulary understands, serialized as JSON straight from the metadata
// tables (contents_def.h / attributes_def.h / filters_def.h). An AI reads
// this once and can write a valid document for exactly this robot's build —
// no guessing, no version skew.

#include <cstdio>
#include <string>

#include "fluent_stage/scene/document.hpp"

namespace fluent_stage::scene {

namespace {

std::string jsonEscape(const std::string& s) {
    std::string out;
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

std::string jsonNumber(float v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(v));
    return buf;
}

/// A field's documented default as a JSON value (null when it has none).
std::string defaultJson(const FieldSpec& f) {
    if (!f.def.has_value) {
        return "null";
    }
    if (f.def.str != nullptr) {
        return "\"" + jsonEscape(f.def.str) + "\"";
    }
    switch (f.kind) {
        case ValueKind::F32: return jsonNumber(f.def.num[0]);
        case ValueKind::Vec2:
            return "[" + jsonNumber(f.def.num[0]) + ", " + jsonNumber(f.def.num[1]) + "]";
        case ValueKind::Rect:
        case ValueKind::Color: {
            std::string out = "[";
            for (int i = 0; i < 4; ++i) {
                out += (i != 0 ? ", " : "") + jsonNumber(f.def.num[i]);
            }
            return out + "]";
        }
        case ValueKind::Bool: return f.def.num[0] != 0 ? "true" : "false";
        default: return "null";
    }
}

std::string typeJson(ValueKind kind) {
    std::string out = "\"" + std::string(toString(kind)) + "\"";
    return out;
}

std::string fieldJson(const FieldSpec& f) {
    std::string out = "{\"name\": \"" + std::string(f.name) + "\", \"type\": " +
                      typeJson(f.kind) + ", \"required\": " +
                      (f.required ? "true" : "false");
    if (f.is_style) {
        out += ", \"style\": true";
    }
    out += ", \"default\": " + defaultJson(f);
    const auto words = enumWords(f.kind);
    if (!words.empty()) {
        out += ", \"values\": [";
        for (size_t i = 0; i < words.size(); ++i) {
            out += (i != 0 ? ", " : "") + std::string("\"") + words[i] + "\"";
        }
        out += "]";
    }
    out += ", \"summary\": \"" + jsonEscape(f.summary) + "\"}";
    return out;
}

}  // namespace

std::string describeJson() {
    std::string out = "{\n";
    out += "  \"schema\": \"" + std::string(kSchemaId) + "\",\n";

    out += "  \"contents\": [\n";
    const auto& contents = contentTable();
    for (size_t i = 0; i < contents.size(); ++i) {
        const ContentSpec& c = contents[i];
        out += "    {\"name\": \"" + std::string(c.name) + "\", \"summary\": \"" +
               jsonEscape(c.summary) + "\", \"fields\": [\n";
        for (size_t j = 0; j < c.fields.size(); ++j) {
            out += "      " + fieldJson(c.fields[j]) +
                   (j + 1 < c.fields.size() ? ",\n" : "\n");
        }
        out += std::string("    ]}") + (i + 1 < contents.size() ? ",\n" : "\n");
    }
    out += "  ],\n";

    out += "  \"attributes\": [\n";
    const auto& attrs = attrTable();
    for (size_t i = 0; i < attrs.size(); ++i) {
        const AttrSpec& a = attrs[i];
        out += "    {\"name\": \"" + std::string(a.name) + "\", \"type\": " +
               typeJson(a.kind) + ", \"animatable\": " + (a.animatable ? "true" : "false");
        const auto words = enumWords(a.kind);
        if (!words.empty()) {
            out += ", \"values\": [";
            for (size_t j = 0; j < words.size(); ++j) {
                out += (j != 0 ? ", " : "") + std::string("\"") + words[j] + "\"";
            }
            out += "]";
        }
        if (a.kind == ValueKind::MapShadow || a.kind == ValueKind::MapBorder) {
            const auto& fields =
                a.kind == ValueKind::MapShadow ? shadowFields() : borderFields();
            out += ", \"fields\": [";
            for (size_t j = 0; j < fields.size(); ++j) {
                out += (j != 0 ? ", " : "") + fieldJson(fields[j]);
            }
            out += "]";
        }
        out += ", \"summary\": \"" + jsonEscape(a.summary) + "\"}";
        out += (i + 1 < attrs.size() ? ",\n" : "\n");
    }
    out += "  ],\n";

    out += "  \"filters\": [\n";
    const auto& filters = filterTable();
    for (size_t i = 0; i < filters.size(); ++i) {
        const FilterSpec& f = filters[i];
        out += "    {\"name\": \"" + std::string(f.name) + "\", \"summary\": \"" +
               jsonEscape(f.summary) + "\", \"params\": [";
        for (size_t j = 0; j < f.params.size(); ++j) {
            const FilterParamSpec& p = f.params[j];
            const char* unit = p.unit == FilterUnit::Length   ? "length"
                               : p.unit == FilterUnit::CoordX ? "coord_x"
                               : p.unit == FilterUnit::CoordY ? "coord_y"
                                                              : "scalar";
            out += (j != 0 ? ", " : "") + std::string("{\"name\": \"") + p.name +
                   "\", \"default\": " + jsonNumber(p.default_value) + ", \"unit\": \"" +
                   unit + "\"}";
        }
        out += "]}";
        out += (i + 1 < filters.size() ? ",\n" : "\n");
    }
    out += "  ],\n";

    out += "  \"input_types\": [\"image.rgba8\", \"text.utf8\", \"sequence<vec2, N>\", "
           "\"sequence<detection2d, N>\"],\n";
    out += "  \"input_fallbacks\": [\"placeholder\", \"hide\", \"hold\"],\n";
    out += "  \"param_types\": [\"f32\", \"vec2\", \"color\", \"bool\"],\n";

    const StageLimits limits{};
    out += "  \"limits\": {\"max_layers\": " + std::to_string(limits.max_layers) +
           ", \"max_depth\": " + std::to_string(limits.max_depth) +
           ", \"max_points\": " + std::to_string(limits.max_points) +
           ", \"max_boxes\": " + std::to_string(limits.max_boxes) +
           ", \"max_text_bytes\": " + std::to_string(limits.max_text_bytes) + "}\n";
    out += "}\n";
    return out;
}

}  // namespace fluent_stage::scene
