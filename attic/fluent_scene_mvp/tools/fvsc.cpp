// fvsc — Fluent Scene compiler front end (MVP slice).
//
// Commands:
//   fvsc parse <file>                 parse only (bounded YAML subset checks)
//   fvsc validate <file> [options]    parse + schema/type validation + canonical IR
//   fvsc compile <file> [options]     validate + backend-neutral resource/pass plan
//
// Options:
//   --ir <path|->        write the canonical typed IR JSON to a file or stdout
//   --plan <path|->      write the resource/pass plan JSON to a file or stdout (compile)
//   --diagnostics-json   print diagnostics (and digests when valid) as JSON to stdout
//
// Exit codes: 0 = no errors, 1 = diagnostics contain errors, 2 = usage or I/O error.
//
// This tool intentionally has no ROS 2, learning-framework, or GPU dependency.

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "fluent_scene/diagnostics.hpp"
#include "fluent_scene/json.hpp"
#include "fluent_scene/planner.hpp"
#include "fluent_scene/registry.hpp"
#include "fluent_scene/validator.hpp"
#include "fluent_scene/yaml.hpp"

namespace {

int usage() {
    std::cerr << "usage: fvsc parse <file>\n"
              << "       fvsc validate <file> [--ir <path|->] [--diagnostics-json]\n"
              << "       fvsc compile <file> [--ir <path|->] [--plan <path|->] [--diagnostics-json]\n";
    return 2;
}

bool writeOutput(const std::string& path, const std::string& content) {
    if (path == "-") {
        std::cout << content;
        return true;
    }
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        std::cerr << "fvsc: cannot write " << path << '\n';
        return false;
    }
    out << content;
    return true;
}

bool readFile(const std::string& path, std::string& out) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        return false;
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    out = buffer.str();
    return true;
}

void printDiagnosticsText(const std::string& path, const fluent_scene::DiagnosticList& diagnostics) {
    for (const auto& diagnostic : diagnostics.items()) {
        std::cerr << path << ':' << diagnostic.span.begin_line << ':' << diagnostic.span.begin_col << ": "
                  << toString(diagnostic.severity) << ": " << diagnostic.code << ": " << diagnostic.message;
        if (!diagnostic.context.empty()) {
            std::cerr << " [";
            for (size_t i = 0; i < diagnostic.context.size(); ++i) {
                if (i > 0) {
                    std::cerr << ", ";
                }
                std::cerr << diagnostic.context[i].first << '=' << diagnostic.context[i].second;
            }
            std::cerr << ']';
        }
        std::cerr << '\n';
    }
    if (diagnostics.overflowed()) {
        std::cerr << path << ": warning: diagnostic limit reached; further diagnostics were dropped\n";
    }
}

fluent_scene::JsonValue diagnosticsJson(const fluent_scene::DiagnosticList& diagnostics,
                                        const std::string& digest, const std::string& plan_digest) {
    using fluent_scene::JsonValue;
    JsonValue root = JsonValue::makeObject();
    JsonValue items = JsonValue::makeArray();
    for (const auto& diagnostic : diagnostics.items()) {
        JsonValue item = JsonValue::makeObject();
        item.set("code", JsonValue::makeString(diagnostic.code));
        item.set("severity", JsonValue::makeString(toString(diagnostic.severity)));
        item.set("phase", JsonValue::makeString(toString(diagnostic.phase)));
        JsonValue span = JsonValue::makeObject();
        span.set("begin_line", JsonValue::makeUInt(diagnostic.span.begin_line));
        span.set("begin_col", JsonValue::makeUInt(diagnostic.span.begin_col));
        span.set("end_line", JsonValue::makeUInt(diagnostic.span.end_line));
        span.set("end_col", JsonValue::makeUInt(diagnostic.span.end_col));
        item.set("span", std::move(span));
        item.set("message", JsonValue::makeString(diagnostic.message));
        if (!diagnostic.context.empty()) {
            JsonValue context = JsonValue::makeObject();
            for (const auto& [key, value] : diagnostic.context) {
                context.set(key, JsonValue::makeString(value));
            }
            item.set("context", std::move(context));
        }
        items.append(std::move(item));
    }
    root.set("diagnostics", std::move(items));
    if (!digest.empty()) {
        root.set("digest", JsonValue::makeString(digest));
    }
    if (!plan_digest.empty()) {
        root.set("plan_digest", JsonValue::makeString(plan_digest));
    }
    return root;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        return usage();
    }
    const std::string command = argv[1];
    const std::string path = argv[2];
    std::string ir_path;
    std::string plan_path;
    bool diagnostics_as_json = false;
    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--ir" && i + 1 < argc) {
            ir_path = argv[++i];
        } else if (arg == "--plan" && i + 1 < argc) {
            plan_path = argv[++i];
        } else if (arg == "--diagnostics-json") {
            diagnostics_as_json = true;
        } else {
            return usage();
        }
    }
    if (command != "parse" && command != "validate" && command != "compile") {
        return usage();
    }

    std::string source;
    if (!readFile(path, source)) {
        std::cerr << "fvsc: cannot read " << path << '\n';
        return 2;
    }

    fluent_scene::DiagnosticList diagnostics;
    const fluent_scene::YamlNode root = fluent_scene::parseYaml(source, diagnostics);

    fluent_scene::ValidationResult result;
    fluent_scene::PlanResult plan;
    if ((command == "validate" || command == "compile") && !diagnostics.hasErrors()) {
        const fluent_scene::NodeRegistry registry = fluent_scene::NodeRegistry::builtinMvp();
        result = fluent_scene::validateScene(root, registry, diagnostics);
        if (command == "compile" && result.ok) {
            plan = fluent_scene::planScene(result, diagnostics);
        }
    }
    diagnostics.sortCanonical();

    if (diagnostics_as_json) {
        std::cout << diagnosticsJson(diagnostics, result.digest, plan.digest).serialize();
    } else {
        printDiagnosticsText(path, diagnostics);
    }

    if (result.ok && !ir_path.empty() && !writeOutput(ir_path, result.ir_text)) {
        return 2;
    }
    if (plan.ok && !plan_path.empty() && !writeOutput(plan_path, plan.plan_text)) {
        return 2;
    }
    const bool emitted_to_stdout = ir_path == "-" || plan_path == "-";
    if (!diagnostics_as_json && !emitted_to_stdout && result.ok) {
        std::cout << "digest: " << result.digest << '\n';
        if (plan.ok) {
            std::cout << "plan_digest: " << plan.digest << '\n';
        }
    }
    return diagnostics.hasErrors() ? 1 : 0;
}
