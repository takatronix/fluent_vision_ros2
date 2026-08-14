// Test suite for the Fluent Scene MVP slice (spec section 14.2 acceptance
// criteria): deterministic canonical IR + digest, golden diagnostics, negative
// fixtures, and parse coverage of both bilingual specification examples.
//
// Usage: fvs_tests <source_dir> [--regen]
//   --regen rewrites the golden files from current behavior.

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "fluent_scene/diagnostics.hpp"
#include "fluent_scene/planner.hpp"
#include "fluent_scene/registry.hpp"
#include "fluent_scene/validator.hpp"
#include "fluent_scene/yaml.hpp"

namespace {

int failures = 0;
std::string source_dir;

void check(bool condition, const std::string& what) {
    if (condition) {
        std::cout << "ok   " << what << '\n';
    } else {
        std::cout << "FAIL " << what << '\n';
        ++failures;
    }
}

std::string readFile(const std::string& relative) {
    std::ifstream stream(source_dir + "/" + relative, std::ios::binary);
    if (!stream) {
        std::cout << "FAIL cannot read " << relative << '\n';
        ++failures;
        return {};
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    return buffer.str();
}

void writeFile(const std::string& relative, const std::string& content) {
    std::ofstream stream(source_dir + "/" + relative, std::ios::binary);
    stream << content;
}

struct RunResult {
    fluent_scene::DiagnosticList diagnostics;
    fluent_scene::ValidationResult validation;
    fluent_scene::PlanResult plan;
};

RunResult runValidate(const std::string& text) {
    RunResult result;
    const fluent_scene::YamlNode root = fluent_scene::parseYaml(text, result.diagnostics);
    if (!result.diagnostics.hasErrors()) {
        const fluent_scene::NodeRegistry registry = fluent_scene::NodeRegistry::builtinMvp();
        result.validation = fluent_scene::validateScene(root, registry, result.diagnostics);
    }
    result.diagnostics.sortCanonical();
    return result;
}

RunResult runCompile(const std::string& text) {
    RunResult result;
    const fluent_scene::YamlNode root = fluent_scene::parseYaml(text, result.diagnostics);
    if (!result.diagnostics.hasErrors()) {
        const fluent_scene::NodeRegistry registry = fluent_scene::NodeRegistry::builtinMvp();
        result.validation = fluent_scene::validateScene(root, registry, result.diagnostics);
        if (result.validation.ok) {
            result.plan = fluent_scene::planScene(result.validation, result.diagnostics);
        }
    }
    result.diagnostics.sortCanonical();
    return result;
}

std::string diagnosticsText(const fluent_scene::DiagnosticList& diagnostics) {
    std::ostringstream out;
    for (const auto& diagnostic : diagnostics.items()) {
        out << toString(diagnostic.phase) << ' ' << toString(diagnostic.severity) << ' ' << diagnostic.code
            << ' ' << diagnostic.span.begin_line << ':' << diagnostic.span.begin_col << ' '
            << diagnostic.message;
        for (const auto& [key, value] : diagnostic.context) {
            out << " [" << key << '=' << value << ']';
        }
        out << '\n';
    }
    return out.str();
}

bool hasErrorCode(const fluent_scene::DiagnosticList& diagnostics, const std::string& code) {
    for (const auto& diagnostic : diagnostics.items()) {
        if (diagnostic.code == code && diagnostic.severity == fluent_scene::Severity::kError) {
            return true;
        }
    }
    return false;
}

bool hasInfoCode(const fluent_scene::DiagnosticList& diagnostics, const std::string& code) {
    for (const auto& diagnostic : diagnostics.items()) {
        if (diagnostic.code == code && diagnostic.severity == fluent_scene::Severity::kInfo) {
            return true;
        }
    }
    return false;
}

void testParseExamples() {
    {
        fluent_scene::DiagnosticList diagnostics;
        fluent_scene::parseYaml(readFile("examples/camera_detection_hud.fvs"), diagnostics);
        check(!diagnostics.hasErrors(), "spec section 11 scene example parses without errors");
    }
    {
        fluent_scene::DiagnosticList diagnostics;
        fluent_scene::parseYaml(readFile("examples/camera_detection_hud_ros2.binding.yaml"), diagnostics);
        check(!diagnostics.hasErrors(), "spec section 12 binding example parses without errors");
    }
}

void testValidateExample(bool regen) {
    const RunResult run = runValidate(readFile("examples/camera_detection_hud.fvs"));
    check(run.validation.ok, "scene example validates without errors");
    check(hasInfoCode(run.diagnostics, "validate.unused_input"),
          "unused depth input yields an informational diagnostic (spec section 11)");
    const std::string diag_text = diagnosticsText(run.diagnostics);
    if (regen) {
        writeFile("tests/golden/camera_detection_hud.ir.json", run.validation.ir_text);
        writeFile("tests/golden/camera_detection_hud.digest.txt", run.validation.digest + "\n");
        writeFile("tests/golden/camera_detection_hud.diagnostics.txt", diag_text);
        std::cout << "regenerated golden files\n";
        return;
    }
    check(run.validation.ir_text == readFile("tests/golden/camera_detection_hud.ir.json"),
          "canonical IR is byte-identical to the golden file");
    check(run.validation.digest + "\n" == readFile("tests/golden/camera_detection_hud.digest.txt"),
          "digest matches the golden digest");
    check(diag_text == readFile("tests/golden/camera_detection_hud.diagnostics.txt"),
          "structured diagnostics match the golden file (stable ordering)");
}

void testPlanExample(bool regen) {
    const RunResult run = runCompile(readFile("examples/camera_detection_hud.fvs"));
    check(run.plan.ok, "scene example compiles to a resource/pass plan");
    if (regen) {
        writeFile("tests/golden/camera_detection_hud.plan.json", run.plan.plan_text);
        writeFile("tests/golden/camera_detection_hud.plan_digest.txt", run.plan.digest + "\n");
        std::cout << "regenerated plan golden files\n";
        return;
    }
    check(run.plan.plan_text == readFile("tests/golden/camera_detection_hud.plan.json"),
          "resource/pass plan is byte-identical to the golden plan");
    check(run.plan.digest + "\n" == readFile("tests/golden/camera_detection_hud.plan_digest.txt"),
          "plan digest matches the golden plan digest");
    // The plan must stay within the scene budgets it was compiled from.
    const fluent_scene::JsonValue* budgets = run.plan.plan.find("budgets");
    check(budgets != nullptr &&
              budgets->find("gpu_bytes_planned")->uintValue() <=
                  budgets->find("gpu_bytes_budget")->uintValue() &&
              budgets->find("upload_bytes_per_frame_planned")->uintValue() <=
                  budgets->find("upload_bytes_per_frame_budget")->uintValue(),
          "planned footprint is within the declared budgets");
}

void testBudgetRejection() {
    {
        const RunResult run = runCompile(readFile("tests/fixtures/budget_gpu_reject.fvs"));
        check(run.validation.ok, "budget_gpu_reject fixture validates (rejection is a compile decision)");
        check(!run.plan.ok && hasErrorCode(run.diagnostics, "compile.budget_exceeded"),
              "gpu memory over budget is rejected at compile with compile.budget_exceeded");
    }
    {
        const RunResult run = runCompile(readFile("tests/fixtures/budget_upload_reject.fvs"));
        check(run.validation.ok, "budget_upload_reject fixture validates");
        check(!run.plan.ok && hasErrorCode(run.diagnostics, "compile.budget_exceeded"),
              "per-frame upload over budget is rejected at compile with compile.budget_exceeded");
    }
}

void testDeterminism() {
    const RunResult base = runValidate(readFile("examples/camera_detection_hud.fvs"));
    const RunResult reordered = runValidate(readFile("tests/fixtures/reordered.fvs"));
    const RunResult commented = runValidate(readFile("tests/fixtures/commented.fvs"));
    check(reordered.validation.ok, "reordered fixture validates");
    check(commented.validation.ok, "commented fixture validates");
    check(base.validation.digest == reordered.validation.digest,
          "legal mapping/list reordering does not change the canonical digest");
    check(base.validation.ir_text == reordered.validation.ir_text,
          "legal mapping/list reordering does not change the canonical IR bytes");
    check(base.validation.digest == commented.validation.digest,
          "comments do not change the canonical digest");

    const RunResult again = runValidate(readFile("examples/camera_detection_hud.fvs"));
    check(base.validation.ir_text == again.validation.ir_text &&
              base.validation.digest == again.validation.digest &&
              diagnosticsText(base.diagnostics) == diagnosticsText(again.diagnostics),
          "repeated validation is byte-identical (IR, digest, diagnostics)");

    const RunResult plan_base = runCompile(readFile("examples/camera_detection_hud.fvs"));
    const RunResult plan_reordered = runCompile(readFile("tests/fixtures/reordered.fvs"));
    check(plan_base.plan.ok && plan_reordered.plan.ok &&
              plan_base.plan.plan_text == plan_reordered.plan.plan_text &&
              plan_base.plan.digest == plan_reordered.plan.digest,
          "legal reordering does not change the resource/pass plan bytes or digest");
}

void testNegativeFixtures() {
    struct Case {
        const char* file;
        const char* code;
    };
    static const Case kCases[] = {
        {"tests/fixtures/duplicate_key.fvs", "parse.duplicate_key"},
        {"tests/fixtures/alias_forbidden.fvs", "parse.anchor_forbidden"},
        {"tests/fixtures/tab_indent.fvs", "parse.tab_forbidden"},
        {"tests/fixtures/unknown_reference.fvs", "validate.unknown_reference"},
        {"tests/fixtures/type_mismatch.fvs", "validate.type_mismatch"},
        {"tests/fixtures/graph_cycle.fvs", "validate.graph_cycle"},
        {"tests/fixtures/missing_bounds.fvs", "validate.missing_bounds"},
        {"tests/fixtures/capacity_overflow.fvs", "validate.capacity_overflow"},
        {"tests/fixtures/unsupported_schema.fvs", "validate.unsupported_schema"},
    };
    for (const Case& c : kCases) {
        const RunResult run = runValidate(readFile(c.file));
        check(!run.validation.ok && hasErrorCode(run.diagnostics, c.code),
              std::string(c.file) + " reports " + c.code);
        for (const auto& diagnostic : run.diagnostics.items()) {
            if (diagnostic.code.empty() || diagnostic.message.empty() || diagnostic.span.begin_line == 0) {
                check(false, std::string(c.file) + ": diagnostic missing code/message/span");
                break;
            }
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: fvs_tests <source_dir> [--regen]\n";
        return 2;
    }
    source_dir = argv[1];
    const bool regen = argc > 2 && std::string(argv[2]) == "--regen";

    testParseExamples();
    testValidateExample(regen);
    testPlanExample(regen);
    if (!regen) {
        testDeterminism();
        testNegativeFixtures();
        testBudgetRejection();
    }

    if (failures == 0) {
        std::cout << "all tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
