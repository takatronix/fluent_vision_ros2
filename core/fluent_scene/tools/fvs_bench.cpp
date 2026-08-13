// fvs_bench — measures parse / validate latency of the Fluent Scene core.
//
// Usage: fvs_bench <file.fvs> [iterations]
//
// Reports per-iteration wall-clock statistics (min / mean / p95) for:
//   parse    bounded YAML subset -> AST
//   validate schema/type check -> canonical IR -> serialize -> sha256 digest
//   total    parse + validate
//
// The spec fixes no performance numbers (section 8.4); this tool exists so a
// platform profile can publish measured budgets instead of guesses.

#include <algorithm>
#include <chrono>
#include <cstdint>
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

struct Stats {
    double min_us = 0.0;
    double mean_us = 0.0;
    double p95_us = 0.0;
};

Stats summarize(std::vector<double>& samples_us) {
    Stats stats;
    if (samples_us.empty()) {
        return stats;
    }
    std::sort(samples_us.begin(), samples_us.end());
    stats.min_us = samples_us.front();
    double sum = 0.0;
    for (double v : samples_us) {
        sum += v;
    }
    stats.mean_us = sum / static_cast<double>(samples_us.size());
    stats.p95_us = samples_us[static_cast<size_t>(static_cast<double>(samples_us.size() - 1) * 0.95)];
    return stats;
}

void printRow(const char* label, const Stats& stats, double bytes) {
    std::cout << label << ": min " << stats.min_us << " us, mean " << stats.mean_us << " us, p95 "
              << stats.p95_us << " us";
    if (bytes > 0.0 && stats.mean_us > 0.0) {
        std::cout << "  (" << (bytes / stats.mean_us) << " MB/s at mean)";
    }
    std::cout << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: fvs_bench <file.fvs> [iterations]\n";
        return 2;
    }
    std::ifstream stream(argv[1], std::ios::binary);
    if (!stream) {
        std::cerr << "fvs_bench: cannot read " << argv[1] << '\n';
        return 2;
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    const std::string source = buffer.str();
    const int iterations = argc > 2 ? std::max(1, std::atoi(argv[2])) : 1000;

    const fluent_scene::NodeRegistry registry = fluent_scene::NodeRegistry::builtinMvp();

    // One checked run first: benchmarking an erroring document is meaningless.
    {
        fluent_scene::DiagnosticList diagnostics;
        const fluent_scene::YamlNode root = fluent_scene::parseYaml(source, diagnostics);
        if (diagnostics.hasErrors()) {
            std::cerr << "fvs_bench: input has parse errors; aborting\n";
            return 1;
        }
        const fluent_scene::ValidationResult result =
            fluent_scene::validateScene(root, registry, diagnostics);
        if (!result.ok) {
            std::cerr << "fvs_bench: input does not validate; aborting\n";
            return 1;
        }
        const fluent_scene::PlanResult plan = fluent_scene::planScene(result, diagnostics);
        if (!plan.ok) {
            std::cerr << "fvs_bench: input does not compile to a plan; aborting\n";
            return 1;
        }
        std::cout << "file: " << argv[1] << " (" << source.size() << " bytes)\n"
                  << "ir:   " << result.ir_text.size() << " bytes, digest " << result.digest << '\n'
                  << "plan: " << plan.plan_text.size() << " bytes, digest " << plan.digest << '\n'
                  << "iterations: " << iterations << "\n\n";
    }

    using Clock = std::chrono::steady_clock;
    std::vector<double> parse_us, validate_us, plan_us, total_us;
    parse_us.reserve(iterations);
    validate_us.reserve(iterations);
    plan_us.reserve(iterations);
    total_us.reserve(iterations);

    // Warm-up.
    for (int i = 0; i < 20; ++i) {
        fluent_scene::DiagnosticList diagnostics;
        const fluent_scene::YamlNode root = fluent_scene::parseYaml(source, diagnostics);
        const fluent_scene::ValidationResult result =
            fluent_scene::validateScene(root, registry, diagnostics);
        (void)fluent_scene::planScene(result, diagnostics);
    }

    for (int i = 0; i < iterations; ++i) {
        fluent_scene::DiagnosticList diagnostics;
        const auto t0 = Clock::now();
        const fluent_scene::YamlNode root = fluent_scene::parseYaml(source, diagnostics);
        const auto t1 = Clock::now();
        const fluent_scene::ValidationResult result =
            fluent_scene::validateScene(root, registry, diagnostics);
        const auto t2 = Clock::now();
        const fluent_scene::PlanResult plan = fluent_scene::planScene(result, diagnostics);
        const auto t3 = Clock::now();
        if (!plan.ok) {
            std::cerr << "fvs_bench: planning unexpectedly failed mid-run\n";
            return 1;
        }
        parse_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        validate_us.push_back(std::chrono::duration<double, std::micro>(t2 - t1).count());
        plan_us.push_back(std::chrono::duration<double, std::micro>(t3 - t2).count());
        total_us.push_back(std::chrono::duration<double, std::micro>(t3 - t0).count());
    }

    const double bytes = static_cast<double>(source.size());
    printRow("parse   ", summarize(parse_us), bytes);
    printRow("validate", summarize(validate_us), 0.0);
    printRow("plan    ", summarize(plan_us), 0.0);
    printRow("total   ", summarize(total_us), bytes);
    return 0;
}
