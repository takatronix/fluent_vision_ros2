#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "fluent_stage/scene/diagnostics.hpp"

namespace fluent_stage::scene {

struct YamlMapEntry;

// Parsed node of the bounded Fluent Scene YAML subset. Scalars keep their raw
// text plus quoting style; interpretation (null/bool/number) is schema-driven
// and happens in the validator, never in the parser.
struct YamlNode {
    enum class Kind { kNull, kScalar, kSequence, kMapping };
    enum class ScalarStyle { kPlain, kSingleQuoted, kDoubleQuoted };

    Kind kind = Kind::kNull;
    ScalarStyle style = ScalarStyle::kPlain;
    std::string scalar;
    std::vector<YamlNode> elements;      // kSequence
    std::vector<YamlMapEntry> entries;   // kMapping
    Span span;

    bool isNull() const { return kind == Kind::kNull; }
    bool isScalar() const { return kind == Kind::kScalar; }
    bool isSequence() const { return kind == Kind::kSequence; }
    bool isMapping() const { return kind == Kind::kMapping; }
    bool isPlainScalar() const { return kind == Kind::kScalar && style == ScalarStyle::kPlain; }

    const YamlNode* find(const std::string& key) const;
};

struct YamlMapEntry {
    std::string key;
    Span key_span;
    YamlNode value;
};

struct ParseLimits {
    size_t max_input_bytes = 1u << 20;  // 1 MiB
    size_t max_depth = 64;
    size_t max_nodes = 100000;
    size_t max_scalar_bytes = 4096;
};

// Parses `source` as the bounded declarative YAML subset used by Fluent Scene.
// Forbidden constructs (anchors, aliases, tags, block scalars, directives,
// multi-document markers, duplicate mapping keys, tab indentation) produce
// parse-phase diagnostics. On error the returned tree may be partial; callers
// must check `diagnostics.hasErrors()`.
YamlNode parseYaml(const std::string& source, DiagnosticList& diagnostics,
                   const ParseLimits& limits = ParseLimits());

}  // namespace fluent_stage::scene
