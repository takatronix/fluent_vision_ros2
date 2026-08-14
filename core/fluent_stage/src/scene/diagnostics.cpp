#include "fluent_stage/scene/diagnostics.hpp"

#include <algorithm>
#include <tuple>

namespace fluent_stage::scene {

const char* toString(Severity severity) {
    switch (severity) {
        case Severity::kError: return "error";
        case Severity::kWarning: return "warning";
        case Severity::kInfo: return "info";
    }
    return "unknown";
}

const char* toString(Phase phase) {
    switch (phase) {
        case Phase::kParse: return "parse";
        case Phase::kValidate: return "validate";
        case Phase::kCompile: return "compile";
        case Phase::kPreview: return "preview";
        case Phase::kActivate: return "activate";
        case Phase::kBind: return "bind";
        case Phase::kFrame: return "frame";
    }
    return "unknown";
}

void DiagnosticList::add(Diagnostic diagnostic) {
    if (diagnostic.severity == Severity::kError) {
        ++error_count_;
    }
    if (items_.size() >= limit_) {
        overflowed_ = true;
        return;
    }
    items_.push_back(std::move(diagnostic));
}

void DiagnosticList::add(std::string code, Severity severity, Phase phase, Span span, std::string message,
                         std::vector<std::pair<std::string, std::string>> context) {
    Diagnostic diagnostic;
    diagnostic.code = std::move(code);
    diagnostic.severity = severity;
    diagnostic.phase = phase;
    diagnostic.span = span;
    diagnostic.message = std::move(message);
    diagnostic.context = std::move(context);
    add(std::move(diagnostic));
}

void DiagnosticList::sortCanonical() {
    std::stable_sort(items_.begin(), items_.end(), [](const Diagnostic& a, const Diagnostic& b) {
        return std::make_tuple(static_cast<int>(a.phase), a.span.begin_line, a.span.begin_col, std::cref(a.code),
                               std::cref(a.message)) <
               std::make_tuple(static_cast<int>(b.phase), b.span.begin_line, b.span.begin_col, std::cref(b.code),
                               std::cref(b.message));
    });
}

}  // namespace fluent_stage::scene
