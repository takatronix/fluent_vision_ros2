#include "fluent_scene/types.hpp"

#include <charconv>

namespace fluent_scene {

bool isImageKind(TypeKind kind) {
    return kind == TypeKind::kImageR8 || kind == TypeKind::kImageRgba8 || kind == TypeKind::kImageRgba16f ||
           kind == TypeKind::kImageDepth32f;
}

TypeTable::TypeTable() {
    builtin_names_ = {
        {"bool", TypeKind::kBool},
        {"i32", TypeKind::kI32},
        {"u32", TypeKind::kU32},
        {"f32", TypeKind::kF32},
        {"string", TypeKind::kString},
        {"vec2f", TypeKind::kVec2f},
        {"vec3f", TypeKind::kVec3f},
        {"vec4f", TypeKind::kVec4f},
        {"mat3f", TypeKind::kMat3f},
        {"mat4f", TypeKind::kMat4f},
        {"quatf", TypeKind::kQuatf},
        {"transform3d", TypeKind::kTransform3d},
        {"image.r8", TypeKind::kImageR8},
        {"image.rgba8", TypeKind::kImageRgba8},
        {"image.rgba16f", TypeKind::kImageRgba16f},
        {"image.depth32f", TypeKind::kImageDepth32f},
        {"points2d", TypeKind::kPoints2d},
        {"polyline2d", TypeKind::kPolyline2d},
        {"boxes2d", TypeKind::kBoxes2d},
        {"mesh3d", TypeKind::kMesh3d},
        {"pointcloud3d", TypeKind::kPointcloud3d},
        {"material", TypeKind::kMaterial},
        {"font", TypeKind::kFont},
        {"texture", TypeKind::kTexture},
        {"camera3d", TypeKind::kCamera3d},
        {"light", TypeKind::kLight},
        {"layer", TypeKind::kLayer},
        {"frame_meta", TypeKind::kFrameMeta},
        {"calibration", TypeKind::kCalibration},
        {"sync_state", TypeKind::kSyncState},
        {"diagnostic_set", TypeKind::kDiagnosticSet},
    };
}

bool TypeTable::declareStruct(const std::string& name, Span span) {
    if (builtin_names_.count(name) != 0 || structs_.count(name) != 0) {
        return false;
    }
    StructDecl decl;
    decl.name = name;
    decl.span = span;
    structs_.emplace(name, std::move(decl));
    return true;
}

StructDecl* TypeTable::findStruct(const std::string& name) {
    auto it = structs_.find(name);
    return it == structs_.end() ? nullptr : &it->second;
}

const StructDecl* TypeTable::findStruct(const std::string& name) const {
    auto it = structs_.find(name);
    return it == structs_.end() ? nullptr : &it->second;
}

TypePtr TypeTable::builtin(TypeKind kind) {
    auto type = std::make_shared<Type>();
    type->kind = kind;
    return type;
}

TypePtr TypeTable::makeStruct(const std::string& name) {
    auto type = std::make_shared<Type>();
    type->kind = TypeKind::kStruct;
    type->struct_name = name;
    return type;
}

TypePtr TypeTable::makeSequence(TypePtr element, uint64_t capacity) {
    auto type = std::make_shared<Type>();
    type->kind = TypeKind::kSequence;
    type->element = std::move(element);
    type->capacity = capacity;
    return type;
}

TypePtr TypeTable::makeOptional(TypePtr element) {
    auto type = std::make_shared<Type>();
    type->kind = TypeKind::kOptional;
    type->element = std::move(element);
    return type;
}

namespace {

const char* builtinName(TypeKind kind) {
    switch (kind) {
        case TypeKind::kBool: return "bool";
        case TypeKind::kI32: return "i32";
        case TypeKind::kU32: return "u32";
        case TypeKind::kF32: return "f32";
        case TypeKind::kString: return "string";
        case TypeKind::kVec2f: return "vec2f";
        case TypeKind::kVec3f: return "vec3f";
        case TypeKind::kVec4f: return "vec4f";
        case TypeKind::kMat3f: return "mat3f";
        case TypeKind::kMat4f: return "mat4f";
        case TypeKind::kQuatf: return "quatf";
        case TypeKind::kTransform3d: return "transform3d";
        case TypeKind::kImageR8: return "image.r8";
        case TypeKind::kImageRgba8: return "image.rgba8";
        case TypeKind::kImageRgba16f: return "image.rgba16f";
        case TypeKind::kImageDepth32f: return "image.depth32f";
        case TypeKind::kPoints2d: return "points2d";
        case TypeKind::kPolyline2d: return "polyline2d";
        case TypeKind::kBoxes2d: return "boxes2d";
        case TypeKind::kMesh3d: return "mesh3d";
        case TypeKind::kPointcloud3d: return "pointcloud3d";
        case TypeKind::kMaterial: return "material";
        case TypeKind::kFont: return "font";
        case TypeKind::kTexture: return "texture";
        case TypeKind::kCamera3d: return "camera3d";
        case TypeKind::kLight: return "light";
        case TypeKind::kLayer: return "layer";
        case TypeKind::kFrameMeta: return "frame_meta";
        case TypeKind::kCalibration: return "calibration";
        case TypeKind::kSyncState: return "sync_state";
        case TypeKind::kDiagnosticSet: return "diagnostic_set";
        case TypeKind::kStruct:
        case TypeKind::kSequence:
        case TypeKind::kOptional: return nullptr;
    }
    return nullptr;
}

void skipSpaces(const std::string& text, size_t& pos) {
    while (pos < text.size() && text[pos] == ' ') {
        ++pos;
    }
}

std::string readTypeName(const std::string& text, size_t& pos) {
    const size_t start = pos;
    while (pos < text.size()) {
        const char c = text[pos];
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '.') {
            ++pos;
        } else {
            break;
        }
    }
    return text.substr(start, pos - start);
}

}  // namespace

std::string TypeTable::canonicalName(const Type& type) {
    switch (type.kind) {
        case TypeKind::kStruct:
            return type.struct_name;
        case TypeKind::kSequence:
            return "sequence<" + canonicalName(*type.element) + "," + std::to_string(type.capacity) + ">";
        case TypeKind::kOptional:
            return "optional<" + canonicalName(*type.element) + ">";
        default:
            return builtinName(type.kind);
    }
}

bool TypeTable::equals(const Type& a, const Type& b) {
    if (a.kind != b.kind) {
        return false;
    }
    switch (a.kind) {
        case TypeKind::kStruct:
            return a.struct_name == b.struct_name;
        case TypeKind::kSequence:
            return a.capacity == b.capacity && equals(*a.element, *b.element);
        case TypeKind::kOptional:
            return equals(*a.element, *b.element);
        default:
            return true;
    }
}

TypePtr TypeTable::parse(const std::string& text, Span span, DiagnosticList& diagnostics) {
    size_t pos = 0;
    TypePtr type = parseInner(text, pos, span, diagnostics, 0);
    if (!type) {
        return nullptr;
    }
    skipSpaces(text, pos);
    if (pos != text.size()) {
        diagnostics.add("validate.unknown_type", Severity::kError, Phase::kValidate, span,
                        "trailing characters in type expression \"" + text + "\"");
        return nullptr;
    }
    return type;
}

TypePtr TypeTable::parseInner(const std::string& text, size_t& pos, Span span, DiagnosticList& diagnostics,
                              size_t depth) {
    if (depth > 8) {
        diagnostics.add("validate.unknown_type", Severity::kError, Phase::kValidate, span,
                        "type expression nests too deeply");
        return nullptr;
    }
    skipSpaces(text, pos);
    const std::string name = readTypeName(text, pos);
    if (name.empty()) {
        diagnostics.add("validate.unknown_type", Severity::kError, Phase::kValidate, span,
                        "expected a type name in \"" + text + "\"");
        return nullptr;
    }
    if (name == "sequence") {
        skipSpaces(text, pos);
        if (pos >= text.size() || text[pos] != '<') {
            diagnostics.add("validate.unbounded_sequence", Severity::kError, Phase::kValidate, span,
                            "sequence requires an element type and an explicit capacity: sequence<T, N>");
            return nullptr;
        }
        ++pos;
        TypePtr element = parseInner(text, pos, span, diagnostics, depth + 1);
        if (!element) {
            return nullptr;
        }
        skipSpaces(text, pos);
        if (pos >= text.size() || text[pos] != ',') {
            diagnostics.add("validate.unbounded_sequence", Severity::kError, Phase::kValidate, span,
                            "sequence requires an explicit capacity: sequence<T, N>");
            return nullptr;
        }
        ++pos;
        skipSpaces(text, pos);
        uint64_t capacity = 0;
        auto result = std::from_chars(text.data() + pos, text.data() + text.size(), capacity);
        if (result.ec != std::errc() || result.ptr == text.data() + pos) {
            diagnostics.add("validate.unbounded_sequence", Severity::kError, Phase::kValidate, span,
                            "sequence capacity must be a positive integer");
            return nullptr;
        }
        pos = static_cast<size_t>(result.ptr - text.data());
        skipSpaces(text, pos);
        if (pos >= text.size() || text[pos] != '>') {
            diagnostics.add("validate.unknown_type", Severity::kError, Phase::kValidate, span,
                            "expected '>' to close sequence<T, N>");
            return nullptr;
        }
        ++pos;
        if (capacity == 0 || capacity > kMaxSequenceCapacity) {
            diagnostics.add("validate.capacity_overflow", Severity::kError, Phase::kValidate, span,
                            "sequence capacity " + std::to_string(capacity) + " is outside the allowed range 1.." +
                                std::to_string(kMaxSequenceCapacity),
                            {{"capacity", std::to_string(capacity)},
                             {"limit", std::to_string(kMaxSequenceCapacity)}});
            return nullptr;
        }
        return makeSequence(std::move(element), capacity);
    }
    if (name == "optional") {
        skipSpaces(text, pos);
        if (pos >= text.size() || text[pos] != '<') {
            diagnostics.add("validate.unknown_type", Severity::kError, Phase::kValidate, span,
                            "optional requires an element type: optional<T>");
            return nullptr;
        }
        ++pos;
        TypePtr element = parseInner(text, pos, span, diagnostics, depth + 1);
        if (!element) {
            return nullptr;
        }
        skipSpaces(text, pos);
        if (pos >= text.size() || text[pos] != '>') {
            diagnostics.add("validate.unknown_type", Severity::kError, Phase::kValidate, span,
                            "expected '>' to close optional<T>");
            return nullptr;
        }
        ++pos;
        if (element->kind == TypeKind::kOptional) {
            diagnostics.add("validate.unknown_type", Severity::kError, Phase::kValidate, span,
                            "optional<optional<T>> is not a valid type");
            return nullptr;
        }
        return makeOptional(std::move(element));
    }
    auto builtin_it = builtin_names_.find(name);
    if (builtin_it != builtin_names_.end()) {
        return builtin(builtin_it->second);
    }
    auto struct_it = structs_.find(name);
    if (struct_it != structs_.end()) {
        struct_it->second.used = true;
        return makeStruct(name);
    }
    diagnostics.add("validate.unknown_type", Severity::kError, Phase::kValidate, span,
                    "unknown type \"" + name + "\"", {{"type", name}});
    return nullptr;
}

}  // namespace fluent_scene
