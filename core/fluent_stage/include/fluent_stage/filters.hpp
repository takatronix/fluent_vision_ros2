#pragma once

/// \file filters.hpp
/// \brief Typed filter structs and the runtime filter value.
///
/// Filters apply to **any layer** — an image, a shape, text, or a whole
/// group (where they act on the composited result). The API has three tiers
/// (§8 of the design):
///
///   1. common filters as Layer methods:  `layer.blur(4)`
///   2. the full set as typed structs:    `layer.filter(Bilateral().radius(4))`
///   3. Scene YAML (Phase L2):            `filters: [ {blur: {radius: 4}} ]`
///
/// Every struct, default, and parameter name below is generated from
/// `shared/filters_def.h`, and every filter body lives once in
/// `shared/filters_shared.h` — there is no second definition anywhere.
///
/// Length-typed parameters (blur radius, pixelate size, …) are in the
/// Stage's logical units: `blur(4)` looks the same whether the layer's
/// source is 640×480 or 4K.

#include <vector>

#include "fluent_stage/shared/filters_shared.h"  // FS_* mode ids (no bodies without FS_SAMPLE)

namespace fluent_stage {

/// A filter instance ready to attach to a layer: a stable mode id from
/// filters_shared.h plus its parameter slots in table order. Users normally
/// construct these through the typed structs below rather than directly.
struct Filter {
    int mode = 0;           ///< FS_* id (see shared/filters_shared.h).
    float values[5] = {};   ///< Parameter slots, in filters_def.h order.
};

/// How a filter parameter is measured (drives logical-unit scaling, §5-3).
enum class FilterUnit {
    Scalar,  ///< Unitless: ratios, angles, normalized positions. Passed through.
    Length,  ///< Logical units; the renderer converts to source pixels.
};

/// Metadata for one filter parameter (generated from filters_def.h).
struct FilterParamSpec {
    const char* name;      ///< Parameter name as it appears in Scene YAML.
    float default_value;   ///< Value used when the parameter is omitted.
    FilterUnit unit;       ///< Scalar or Length (see FilterUnit).
};

/// Metadata for one filter (generated from filters_def.h). This single table
/// feeds Scene parsing (L2), the capability catalog (§13-1), and the docs.
struct FilterSpec {
    const char* name;      ///< Public snake_case name (`bilateral`).
    int mode;              ///< FS_* id.
    const char* summary;   ///< One-line description (English; docs carry
                           ///< translations, code carries one language).
    std::vector<FilterParamSpec> params;  ///< In slot order p0..p4.
};

// ---------------------------------------------------------------------------
// Pass 1 — default values: one function per filter building its Filter value.
// ---------------------------------------------------------------------------

namespace detail {

#define FS_FILTER(TypeName, method, MODE, summary) \
    constexpr Filter filterDefaults_##TypeName() { \
        Filter f{};                                \
        f.mode = MODE;
#define FS_PARAM(slot, name, default_value, unit) \
        f.values[slot] = default_value;
#define FS_END(TypeName) \
        return f;        \
    }

#include "fluent_stage/shared/filters_def.h"

#undef FS_FILTER
#undef FS_PARAM
#undef FS_END

}  // namespace detail

// ---------------------------------------------------------------------------
// Pass 2 — typed structs: one chainable setter per parameter, implicit
// conversion to Filter. `Bilateral().radius(4).sigma_color(0.2)` reads like
// the YAML it mirrors and cannot name a parameter that does not exist.
// ---------------------------------------------------------------------------

#define FS_FILTER(TypeName, method, MODE, summary) \
    struct TypeName {                              \
        Filter f_ = detail::filterDefaults_##TypeName();
#define FS_PARAM(slot, name, default_value, unit)  \
        auto& name(float v) {                      \
            f_.values[slot] = v;                   \
            return *this;                          \
        }
#define FS_END(TypeName)                           \
        constexpr operator Filter() const { return f_; } \
    };

#include "fluent_stage/shared/filters_def.h"

#undef FS_FILTER
#undef FS_PARAM
#undef FS_END

// ---------------------------------------------------------------------------
// Pass 3 — the runtime spec table.
// ---------------------------------------------------------------------------

/// All filters with their names, parameters, defaults, and units, in
/// filters_def.h order. The single machine-readable catalog.
inline const std::vector<FilterSpec>& filterTable() {
#define FS_FILTER(TypeName, method, MODE, summary) {#method, MODE, summary, {
#define FS_PARAM(slot, name, default_value, unit) {#name, default_value, FilterUnit::unit},
#define FS_END(TypeName) }},
    static const std::vector<FilterSpec> kTable = {
#include "fluent_stage/shared/filters_def.h"
    };
#undef FS_FILTER
#undef FS_PARAM
#undef FS_END
    return kTable;
}

/// Looks up a filter's metadata by mode id; nullptr when unknown.
inline const FilterSpec* findFilterSpec(int mode) {
    for (const FilterSpec& spec : filterTable()) {
        if (spec.mode == mode) {
            return &spec;
        }
    }
    return nullptr;
}

}  // namespace fluent_stage
