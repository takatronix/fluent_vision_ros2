#include "fluent_stage/scene/schema.hpp"

#include <cstring>

namespace fluent_stage::scene {

const char* toString(ValueKind kind) {
    switch (kind) {
        case ValueKind::F32: return "f32";
        case ValueKind::Vec2: return "vec2";
        case ValueKind::Rect: return "rect";
        case ValueKind::Color: return "color";
        case ValueKind::Bool: return "bool";
        case ValueKind::Str: return "string";
        case ValueKind::Points: return "points";
        case ValueKind::Mat23: return "mat23";
        case ValueKind::Input: return "input_ref";
        case ValueKind::EnumFit: return "fit";
        case ValueKind::EnumAlign: return "align";
        case ValueKind::EnumCap: return "cap";
        case ValueKind::EnumBlend: return "blend";
        case ValueKind::EnumEase: return "ease";
        case ValueKind::MapShadow: return "shadow";
        case ValueKind::MapBorder: return "border";
    }
    return "?";
}

const FieldSpec* ContentSpec::find(const std::string& key) const {
    for (const FieldSpec& f : fields) {
        if (key == f.name) {
            return &f;
        }
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// Content table — generated from shared/contents_def.h.
//
// The style table is expanded first into lookup helpers so FS_CSTYLE(name)
// inside a content block can pull the shared spec by name.
// ---------------------------------------------------------------------------

namespace {

#define FS_DNONE FieldDefault{}
#define FS_DF32(a) FieldDefault{true, {static_cast<float>(a), 0, 0, 0}, nullptr}
#define FS_DVEC2(a, b) FieldDefault{true, {static_cast<float>(a), static_cast<float>(b), 0, 0}, nullptr}
#define FS_DRECT(a, b, c, d)                                                          \
    FieldDefault{true,                                                                \
                 {static_cast<float>(a), static_cast<float>(b), static_cast<float>(c), \
                  static_cast<float>(d)},                                             \
                 nullptr}
#define FS_DSTR(s) FieldDefault{true, {0, 0, 0, 0}, #s}

// Pass 1 — one FieldSpec constant per shared style key.
#define FS_STYLE(name, KIND, summary) \
    const FieldSpec kStyle_##name{#name, ValueKind::KIND, false, true, FieldDefault{}, summary};
#define FS_CONTENT(name, summary)
#define FS_CFIELD(name, KIND, required, def, summary)
#define FS_CSTYLE(name)
#define FS_CONTENT_END(name)
#include "fluent_stage/shared/contents_def.h"
#undef FS_STYLE
#undef FS_CONTENT
#undef FS_CFIELD
#undef FS_CSTYLE
#undef FS_CONTENT_END

}  // namespace

const std::vector<ContentSpec>& contentTable() {
    // Pass 2 — the table itself.
#define FS_STYLE(name, KIND, summary)
#define FS_CONTENT(name, summary) {#name, summary, {
#define FS_CFIELD(name, KIND, required, def, summary) \
    {#name, ValueKind::KIND, required != 0, false, def, summary},
#define FS_CSTYLE(name) kStyle_##name,
#define FS_CONTENT_END(name) }},
    static const std::vector<ContentSpec> kTable = {
#include "fluent_stage/shared/contents_def.h"
    };
#undef FS_STYLE
#undef FS_CONTENT
#undef FS_CFIELD
#undef FS_CSTYLE
#undef FS_CONTENT_END
    return kTable;
}

const ContentSpec* findContentSpec(const std::string& name) {
    for (const ContentSpec& spec : contentTable()) {
        if (name == spec.name) {
            return &spec;
        }
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// Attribute table — generated from shared/attributes_def.h.
// ---------------------------------------------------------------------------

const std::vector<AttrSpec>& attrTable() {
#define FS_ATTR(Ident, yaml_name, KIND, animatable, summary) \
    {AttrId::Ident, #yaml_name, ValueKind::KIND, animatable != 0, summary},
    static const std::vector<AttrSpec> kTable = {
#include "fluent_stage/shared/attributes_def.h"
    };
#undef FS_ATTR
    return kTable;
}

const AttrSpec* findAttrSpec(const std::string& name) {
    for (const AttrSpec& spec : attrTable()) {
        if (name == spec.name) {
            return &spec;
        }
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// Shadow / border map fields — defaults read from the C++ structs themselves.
// ---------------------------------------------------------------------------

const std::vector<FieldSpec>& shadowFields() {
    static const Shadow d{};
    static const std::vector<FieldSpec> kFields = {
        {"offset", ValueKind::Vec2, false, false,
         FieldDefault{true, {d.offset.x, d.offset.y, 0, 0}, nullptr},
         "displacement in logical units (+y is down)"},
        {"radius", ValueKind::F32, false, false,
         FieldDefault{true, {d.radius, 0, 0, 0}, nullptr},
         "gaussian blur radius in logical units"},
        {"color", ValueKind::Color, false, false,
         FieldDefault{true, {d.color.r, d.color.g, d.color.b, d.color.a}, nullptr},
         "shadow color (alpha multiplies opacity)"},
        {"opacity", ValueKind::F32, false, false,
         FieldDefault{true, {d.opacity, 0, 0, 0}, nullptr}, "overall strength, 0-1"},
    };
    return kFields;
}

const std::vector<FieldSpec>& borderFields() {
    static const Border d{};
    static const std::vector<FieldSpec> kFields = {
        {"width", ValueKind::F32, false, false,
         FieldDefault{true, {d.width, 0, 0, 0}, nullptr}, "stroke width in logical units"},
        {"color", ValueKind::Color, false, false,
         FieldDefault{true, {d.color.r, d.color.g, d.color.b, d.color.a}, nullptr},
         "stroke color"},
    };
    return kFields;
}

// ---------------------------------------------------------------------------
// Enum spellings
// ---------------------------------------------------------------------------

namespace {

struct EnumEntry {
    const char* word;
    int value;
};

const EnumEntry kFitWords[] = {{"contain", static_cast<int>(Fit::Contain)},
                               {"cover", static_cast<int>(Fit::Cover)},
                               {"fill", static_cast<int>(Fit::Fill)}};
const EnumEntry kAlignWords[] = {{"left", static_cast<int>(Align::Left)},
                                 {"center", static_cast<int>(Align::Center)},
                                 {"right", static_cast<int>(Align::Right)}};
const EnumEntry kCapWords[] = {{"round", static_cast<int>(Cap::Round)},
                               {"butt", static_cast<int>(Cap::Butt)}};
const EnumEntry kBlendWords[] = {{"normal", static_cast<int>(Blend::Normal)},
                                 {"add", static_cast<int>(Blend::Add)},
                                 {"multiply", static_cast<int>(Blend::Multiply)},
                                 {"screen", static_cast<int>(Blend::Screen)}};
const EnumEntry kEaseWords[] = {{"linear", static_cast<int>(Ease::Linear)},
                                {"ease_in", static_cast<int>(Ease::In)},
                                {"ease_out", static_cast<int>(Ease::Out)},
                                {"ease_in_out", static_cast<int>(Ease::InOut)}};

const EnumEntry* enumEntries(ValueKind kind, size_t& count) {
    switch (kind) {
        case ValueKind::EnumFit: count = 3; return kFitWords;
        case ValueKind::EnumAlign: count = 3; return kAlignWords;
        case ValueKind::EnumCap: count = 2; return kCapWords;
        case ValueKind::EnumBlend: count = 4; return kBlendWords;
        case ValueKind::EnumEase: count = 4; return kEaseWords;
        default: count = 0; return nullptr;
    }
}

}  // namespace

bool parseEnumWord(ValueKind kind, const std::string& word, int& out) {
    size_t count = 0;
    const EnumEntry* entries = enumEntries(kind, count);
    for (size_t i = 0; i < count; ++i) {
        if (word == entries[i].word) {
            out = entries[i].value;
            return true;
        }
    }
    return false;
}

const char* enumWord(ValueKind kind, int value) {
    size_t count = 0;
    const EnumEntry* entries = enumEntries(kind, count);
    for (size_t i = 0; i < count; ++i) {
        if (value == entries[i].value) {
            return entries[i].word;
        }
    }
    return "?";
}

std::vector<const char*> enumWords(ValueKind kind) {
    size_t count = 0;
    const EnumEntry* entries = enumEntries(kind, count);
    std::vector<const char*> words;
    words.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        words.push_back(entries[i].word);
    }
    return words;
}

// ---------------------------------------------------------------------------
// Input and param types
// ---------------------------------------------------------------------------

bool parseInputType(const std::string& text, InputType& type, uint32_t& capacity) {
    capacity = 0;
    if (text == "image.rgba8") {
        type = InputType::ImageRgba8;
        return true;
    }
    if (text == "text.utf8") {
        type = InputType::TextUtf8;
        return true;
    }
    // sequence<elem, N> — spaces around the comma optional.
    const std::string prefix = "sequence<";
    if (text.size() > prefix.size() + 1 && text.compare(0, prefix.size(), prefix) == 0 &&
        text.back() == '>') {
        std::string inner = text.substr(prefix.size(), text.size() - prefix.size() - 1);
        const size_t comma = inner.find(',');
        if (comma == std::string::npos) {
            return false;
        }
        std::string elem = inner.substr(0, comma);
        std::string count = inner.substr(comma + 1);
        auto trim = [](std::string& s) {
            while (!s.empty() && s.front() == ' ') s.erase(s.begin());
            while (!s.empty() && s.back() == ' ') s.pop_back();
        };
        trim(elem);
        trim(count);
        if (count.empty() || count.size() > 9) {
            return false;
        }
        uint64_t n = 0;
        for (char c : count) {
            if (c < '0' || c > '9') {
                return false;
            }
            n = n * 10 + static_cast<uint64_t>(c - '0');
        }
        if (n == 0) {
            return false;
        }
        capacity = static_cast<uint32_t>(n);
        // Accept the historical CamelCase spellings alongside the canonical
        // lowercase ones (§2 wrote Detection2D).
        if (elem == "vec2" || elem == "Vec2") {
            type = InputType::SeqVec2;
            return true;
        }
        if (elem == "detection2d" || elem == "Detection2D") {
            type = InputType::SeqDetection2D;
            return true;
        }
    }
    return false;
}

std::string inputTypeName(InputType type, uint32_t capacity) {
    switch (type) {
        case InputType::ImageRgba8: return "image.rgba8";
        case InputType::TextUtf8: return "text.utf8";
        case InputType::SeqVec2: return "sequence<vec2, " + std::to_string(capacity) + ">";
        case InputType::SeqDetection2D:
            return "sequence<detection2d, " + std::to_string(capacity) + ">";
    }
    return "?";
}

bool parseParamType(const std::string& word, ParamType& out) {
    if (word == "f32") {
        out = ParamType::F32;
    } else if (word == "vec2") {
        out = ParamType::Vec2;
    } else if (word == "color") {
        out = ParamType::Color;
    } else if (word == "bool") {
        out = ParamType::Bool;
    } else {
        return false;
    }
    return true;
}

const char* paramTypeName(ParamType type) {
    switch (type) {
        case ParamType::F32: return "f32";
        case ParamType::Vec2: return "vec2";
        case ParamType::Color: return "color";
        case ParamType::Bool: return "bool";
    }
    return "?";
}

}  // namespace fluent_stage::scene
