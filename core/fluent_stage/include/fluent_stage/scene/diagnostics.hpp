#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace fluent_stage::scene {

// 1-based, inclusive source range. A value of 0 means "no location".
struct Span {
    uint32_t begin_line = 0;
    uint32_t begin_col = 0;
    uint32_t end_line = 0;
    uint32_t end_col = 0;
};

enum class Severity { kError, kWarning, kInfo };

enum class Phase { kParse, kValidate, kCompile, kPreview, kActivate, kBind, kFrame };

const char* toString(Severity severity);
const char* toString(Phase phase);

struct Diagnostic {
    std::string code;        // stable identifier, e.g. "parse.duplicate_key"
    Severity severity = Severity::kError;
    Phase phase = Phase::kParse;
    Span span;
    std::string message;
    // Structured context as key/value pairs; keys are stable identifiers.
    std::vector<std::pair<std::string, std::string>> context;
};

// Bounded diagnostic collection with a canonical, deterministic ordering.
class DiagnosticList {
public:
    explicit DiagnosticList(size_t limit = 256) : limit_(limit) {}

    void add(Diagnostic diagnostic);
    void add(std::string code, Severity severity, Phase phase, Span span, std::string message,
             std::vector<std::pair<std::string, std::string>> context = {});

    bool hasErrors() const { return error_count_ > 0; }
    size_t errorCount() const { return error_count_; }
    bool overflowed() const { return overflowed_; }
    const std::vector<Diagnostic>& items() const { return items_; }

    // Canonical order: phase, begin_line, begin_col, code, message. Stable for equal keys.
    void sortCanonical();

private:
    std::vector<Diagnostic> items_;
    size_t limit_;
    size_t error_count_ = 0;
    bool overflowed_ = false;
};

}  // namespace fluent_stage::scene
