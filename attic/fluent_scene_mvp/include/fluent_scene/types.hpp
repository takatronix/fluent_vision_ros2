#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "fluent_scene/diagnostics.hpp"

namespace fluent_scene {

// Global static bound for declared sequence capacities (Decision 6: bounded behavior).
constexpr uint64_t kMaxSequenceCapacity = 65536;

enum class TypeKind {
    // Scalar
    kBool, kI32, kU32, kF32, kString,
    // Math
    kVec2f, kVec3f, kVec4f, kMat3f, kMat4f, kQuatf, kTransform3d,
    // Image
    kImageR8, kImageRgba8, kImageRgba16f, kImageDepth32f,
    // Geometry
    kPoints2d, kPolyline2d, kBoxes2d, kMesh3d, kPointcloud3d,
    // Visual
    kMaterial, kFont, kTexture, kCamera3d, kLight, kLayer,
    // Metadata
    kFrameMeta, kCalibration, kSyncState, kDiagnosticSet,
    // Structured
    kStruct, kSequence, kOptional,
};

struct Type;
using TypePtr = std::shared_ptr<const Type>;

struct Type {
    TypeKind kind = TypeKind::kBool;
    std::string struct_name;  // kStruct
    TypePtr element;          // kSequence / kOptional
    uint64_t capacity = 0;    // kSequence
};

struct StructField {
    std::string name;
    TypePtr type;
    Span span;
};

struct StructDecl {
    std::string name;
    std::vector<StructField> fields;  // declaration order (semantic)
    Span span;
    bool used = false;
};

bool isImageKind(TypeKind kind);

// Registry of built-in type names plus user-declared struct types.
class TypeTable {
public:
    TypeTable();

    // Registers a struct name (fields may be attached later). Returns false if
    // the name collides with a built-in type or another struct.
    bool declareStruct(const std::string& name, Span span);
    StructDecl* findStruct(const std::string& name);
    const StructDecl* findStruct(const std::string& name) const;
    const std::map<std::string, StructDecl>& structs() const { return structs_; }

    // Parses a type expression such as "image.rgba8", "sequence<Detection2D, 128>",
    // or "optional<f32>". Marks referenced structs as used. Returns nullptr and
    // reports a validate-phase diagnostic on failure.
    TypePtr parse(const std::string& text, Span span, DiagnosticList& diagnostics);

    static TypePtr builtin(TypeKind kind);
    static TypePtr makeStruct(const std::string& name);
    static TypePtr makeSequence(TypePtr element, uint64_t capacity);
    static TypePtr makeOptional(TypePtr element);

    static std::string canonicalName(const Type& type);
    static bool equals(const Type& a, const Type& b);

private:
    TypePtr parseInner(const std::string& text, size_t& pos, Span span, DiagnosticList& diagnostics,
                       size_t depth);

    std::map<std::string, TypeKind> builtin_names_;
    std::map<std::string, StructDecl> structs_;
};

}  // namespace fluent_scene
