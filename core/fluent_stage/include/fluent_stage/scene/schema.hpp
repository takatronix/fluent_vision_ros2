#pragma once

/// \file schema.hpp
/// \brief The Scene vocabulary as runtime metadata tables (§13-1).
///
/// Everything a Scene document may say — content types and their fields,
/// layer attributes, filters, param and input types — is described by the
/// tables in this file, generated from the single-definition headers
/// `shared/contents_def.h`, `shared/attributes_def.h`, and
/// `shared/filters_def.h`. The validator rejects unknown keys against these
/// tables, `fvsc describe --json` serializes them as the capability catalog,
/// and the canonical formatter takes its key order from them. Behavior is
/// never derived from here: the compiler applies documents through the same
/// Layer API a C++ author uses.

#include <string>
#include <vector>

#include "fluent_stage/filters.hpp"
#include "fluent_stage/types.hpp"

namespace fluent_stage::scene {

/// The kind of a Scene value — what shape of YAML is expected and how it is
/// stored in the document model.
enum class ValueKind {
    F32,        ///< A number.
    Vec2,       ///< [x, y].
    Rect,       ///< [x, y, w, h].
    Color,      ///< [r, g, b] or [r, g, b, a], components 0-1.
    Bool,       ///< true | false.
    Str,        ///< A string.
    Points,     ///< [[x, y], ...] — a point list.
    Mat23,      ///< [a, b, c, d, tx, ty] — an affine matrix.
    Input,      ///< "$inputs.<name>" — a declared-input reference.
    EnumFit,    ///< contain | cover | fill.
    EnumAlign,  ///< left | center | right.
    EnumCap,    ///< round | butt.
    EnumBlend,  ///< normal | add | multiply | screen.
    EnumEase,   ///< linear | ease_in | ease_out | ease_in_out.
    MapShadow,  ///< { offset, radius, color, opacity } — see shadowFields().
    MapBorder,  ///< { width, color } — see borderFields().
};

/// The YAML spelling of a value kind (for diagnostics and describe --json).
const char* toString(ValueKind kind);

/// A documented default value for a field. `has_value` is false for required
/// fields and for fields whose default is content-dependent (styles).
struct FieldDefault {
    bool has_value = false;
    float num[4] = {0, 0, 0, 0};  ///< F32 uses [0]; Vec2 [0..1]; Rect/Color [0..3].
    const char* str = nullptr;    ///< Enum/string defaults.
};

/// Metadata for one field of a content type (or of a shadow/border map).
struct FieldSpec {
    const char* name;      ///< Key as it appears in Scene YAML.
    ValueKind kind;        ///< Expected value shape.
    bool required;         ///< Whether the document must set it.
    bool is_style;         ///< True for shared style keys (map to Layer style setters).
    FieldDefault def;      ///< Documented default (verified against C++ by tests).
    const char* summary;   ///< One-line description (English; docs carry translations).
};

/// Metadata for one content type (§7 catalog).
struct ContentSpec {
    const char* name;                ///< Public name (`rect`, `boxes`, ...).
    const char* summary;             ///< One-line description.
    std::vector<FieldSpec> fields;   ///< Own fields first, then accepted styles.

    /// Finds a field by YAML key; nullptr when the content has no such key.
    const FieldSpec* find(const std::string& key) const;
};

/// All content types in §7 catalog order — the single machine-readable
/// catalog for validation, describe --json, and fmt.
const std::vector<ContentSpec>& contentTable();

/// Looks up a content type by its YAML name; nullptr when unknown.
const ContentSpec* findContentSpec(const std::string& name);

/// Identity of a layer attribute (generated from attributes_def.h).
enum class AttrId {
#define FS_ATTR(Ident, yaml_name, KIND, animatable, summary) Ident,
#include "fluent_stage/shared/attributes_def.h"
#undef FS_ATTR
};

/// Metadata for one layer attribute (§6.2).
struct AttrSpec {
    AttrId id;             ///< Stable identity for the compiler's dispatch.
    const char* name;      ///< YAML key.
    ValueKind kind;        ///< Expected value shape.
    bool animatable;       ///< Whether §9 implicit animation interpolates it.
    const char* summary;   ///< One-line description.
};

/// All layer attributes in declaration order.
const std::vector<AttrSpec>& attrTable();

/// Looks up an attribute by YAML key; nullptr when unknown.
const AttrSpec* findAttrSpec(const std::string& name);

/// Field metadata for the `shadow` attribute map. Defaults are read from
/// the C++ `Shadow{}` struct itself — a single definition by construction.
const std::vector<FieldSpec>& shadowFields();

/// Field metadata for the `border` attribute map (defaults from `Border{}`).
const std::vector<FieldSpec>& borderFields();

// ---------------------------------------------------------------------------
// Enum spellings — one place maps every YAML enum word to its C++ value.
// ---------------------------------------------------------------------------

/// Parses an enum word for `kind`; returns false when the word is not in the
/// vocabulary. The integer is the underlying C++ enum value.
bool parseEnumWord(ValueKind kind, const std::string& word, int& out);

/// The canonical YAML word for a C++ enum value of `kind` (fmt, describe).
const char* enumWord(ValueKind kind, int value);

/// All valid words for an enum kind, for diagnostics and describe --json.
std::vector<const char*> enumWords(ValueKind kind);

// ---------------------------------------------------------------------------
// Input and param types
// ---------------------------------------------------------------------------

/// The element type of a declared input (§1.3 input contract).
enum class InputType {
    ImageRgba8,     ///< `image.rgba8` — fed by CompiledScene::setImage.
    TextUtf8,       ///< `text.utf8` — fed by setText.
    SeqVec2,        ///< `sequence<vec2, N>` — fed by setPoints.
    SeqDetection2D, ///< `sequence<detection2d, N>` — fed by setBoxes.
};

/// What a layer shows before its input receives data (§1.3 fallback).
enum class InputFallback {
    Placeholder,  ///< A deterministic labeled placeholder panel (default).
    Hide,         ///< The layer stays hidden until data arrives.
    Hold,         ///< Nothing is drawn for data-less content; no placeholder.
};

/// The declared type of a runtime parameter.
enum class ParamType { F32, Vec2, Color, Bool };

/// Parses an input type spelling ("image.rgba8", "sequence<vec2, 128>").
/// `capacity` is the declared N for sequences (0 otherwise).
bool parseInputType(const std::string& text, InputType& type, uint32_t& capacity);

/// The canonical spelling of an input type (with capacity for sequences).
std::string inputTypeName(InputType type, uint32_t capacity);

/// Parses a param type word (f32 | vec2 | color | bool).
bool parseParamType(const std::string& word, ParamType& out);

/// The canonical spelling of a param type.
const char* paramTypeName(ParamType type);

/// The schema identifier this implementation accepts.
inline constexpr const char* kSchemaId = "fluent.scene/v1alpha2";

}  // namespace fluent_stage::scene
