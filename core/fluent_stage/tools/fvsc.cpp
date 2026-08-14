// fvsc — the Fluent Scene compiler CLI (§13).
//
//   fvsc validate <scene.fvs>            type-check + design lints + digest
//   fvsc preview  <scene.fvs> [-o out.ppm] [--size WxH]   compile and render
//   fvsc fmt      <scene.fvs> [-i]       canonical form (§13-8)
//   fvsc digest   <scene.fvs>            canonical digest only
//   fvsc describe --json                 capability catalog (§13-1)
//
// Exit codes: 0 ok (warnings allowed), 1 errors in the document, 2 usage or
// I/O failure. `validate --strict` also fails (1) on warnings.

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#include "fluent_stage/cpu_renderer.hpp"
#include "fluent_stage/scene/compiler.hpp"
#include "fluent_stage/scene/document.hpp"
#include "fluent_stage/scene/linter.hpp"

using namespace fluent_stage;
using namespace fluent_stage::scene;

namespace {

int usage() {
    std::fprintf(stderr,
                 "usage:\n"
                 "  fvsc validate <scene.fvs> [--strict]\n"
                 "  fvsc preview  <scene.fvs> [-o out.ppm] [--size WxH]\n"
                 "  fvsc fmt      <scene.fvs> [-i]\n"
                 "  fvsc digest   <scene.fvs>\n"
                 "  fvsc describe --json\n");
    return 2;
}

bool readFile(const std::string& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::fprintf(stderr, "fvsc: cannot read %s\n", path.c_str());
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    out = ss.str();
    return true;
}

void printDiagnostics(const DiagnosticList& diags) {
    for (const Diagnostic& d : diags.items()) {
        if (d.span.begin_line != 0) {
            std::fprintf(stderr, "%s: line %u: [%s] %s\n", toString(d.severity),
                         d.span.begin_line, d.code.c_str(), d.message.c_str());
        } else {
            std::fprintf(stderr, "%s: [%s] %s\n", toString(d.severity), d.code.c_str(),
                         d.message.c_str());
        }
    }
}

bool writePpm(const Surface& s, const std::string& path) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        std::fprintf(stderr, "fvsc: cannot write %s\n", path.c_str());
        return false;
    }
    out << "P6\n" << s.width << " " << s.height << "\n255\n";
    for (uint32_t y = 0; y < s.height; ++y) {
        const uint8_t* row = s.row(y);
        for (uint32_t x = 0; x < s.width; ++x) {
            out.put(static_cast<char>(row[x * 4 + 0]));
            out.put(static_cast<char>(row[x * 4 + 1]));
            out.put(static_cast<char>(row[x * 4 + 2]));
        }
    }
    return static_cast<bool>(out);
}

int cmdValidate(const std::string& path, bool strict) {
    std::string text;
    if (!readFile(path, text)) {
        return 2;
    }
    ParseResult parsed = parseScene(text);
    printDiagnostics(parsed.diagnostics);
    if (!parsed.ok()) {
        std::fprintf(stderr, "fvsc: %zu error(s)\n", parsed.diagnostics.errorCount());
        return 1;
    }
    CompileResult compiled = compile(parsed.doc);
    printDiagnostics(compiled.diagnostics);
    if (!compiled.ok()) {
        return 1;
    }
    CpuRenderer renderer;
    const DiagnosticList lints = lint(*compiled.scene, renderer);
    printDiagnostics(lints);
    std::printf("digest: %s\n", compiled.scene->digest().c_str());
    if (lints.hasErrors() || (strict && !lints.items().empty())) {
        return 1;
    }
    std::printf("ok: %u layers\n", compiled.scene->stage().layerCount());
    return 0;
}

int cmdPreview(const std::string& path, const std::string& out_path, uint32_t w, uint32_t h) {
    std::string text;
    if (!readFile(path, text)) {
        return 2;
    }
    ParseResult parsed = parseScene(text);
    printDiagnostics(parsed.diagnostics);
    if (!parsed.ok()) {
        return 1;
    }
    CompileResult compiled = compile(parsed.doc);
    printDiagnostics(compiled.diagnostics);
    if (!compiled.ok()) {
        return 1;
    }
    Stage& stage = compiled.scene->stage();
    CpuRenderer renderer;
    const Surface& frame = (w != 0 && h != 0) ? renderer.render(stage, w, h, 0.0f)
                                              : renderer.render(stage, 0.0f);
    if (!writePpm(frame, out_path)) {
        return 2;
    }
    std::printf("%s: %ux%u (digest %s)\n", out_path.c_str(), frame.width, frame.height,
                compiled.scene->digest().c_str());
    return 0;
}

int cmdFmt(const std::string& path, bool in_place) {
    std::string text;
    if (!readFile(path, text)) {
        return 2;
    }
    ParseResult parsed = parseScene(text);
    if (!parsed.ok()) {
        printDiagnostics(parsed.diagnostics);
        return 1;
    }
    const std::string canonical = format(parsed.doc);
    if (in_place) {
        std::ofstream out(path, std::ios::binary);
        if (!out) {
            std::fprintf(stderr, "fvsc: cannot write %s\n", path.c_str());
            return 2;
        }
        out << canonical;
    } else {
        std::fputs(canonical.c_str(), stdout);
    }
    return 0;
}

int cmdDigest(const std::string& path) {
    std::string text;
    if (!readFile(path, text)) {
        return 2;
    }
    ParseResult parsed = parseScene(text);
    if (!parsed.ok()) {
        printDiagnostics(parsed.diagnostics);
        return 1;
    }
    std::printf("%s\n", digest(parsed.doc).c_str());
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        return usage();
    }
    const std::string cmd = argv[1];

    if (cmd == "describe") {
        // --json is the only format; requiring the flag keeps room for a
        // human-readable default later.
        if (argc != 3 || std::strcmp(argv[2], "--json") != 0) {
            return usage();
        }
        std::fputs(describeJson().c_str(), stdout);
        return 0;
    }
    if (argc < 3) {
        return usage();
    }
    const std::string path = argv[2];

    if (cmd == "validate") {
        bool strict = false;
        for (int i = 3; i < argc; ++i) {
            if (std::strcmp(argv[i], "--strict") == 0) {
                strict = true;
            } else {
                return usage();
            }
        }
        return cmdValidate(path, strict);
    }
    if (cmd == "preview") {
        std::string out_path = "preview.ppm";
        uint32_t w = 0, h = 0;
        for (int i = 3; i < argc; ++i) {
            if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
                out_path = argv[++i];
            } else if (std::strcmp(argv[i], "--size") == 0 && i + 1 < argc) {
                unsigned pw = 0, ph = 0;
                if (std::sscanf(argv[++i], "%ux%u", &pw, &ph) != 2 || pw == 0 || ph == 0) {
                    return usage();
                }
                w = pw;
                h = ph;
            } else {
                return usage();
            }
        }
        return cmdPreview(path, out_path, w, h);
    }
    if (cmd == "fmt") {
        bool in_place = false;
        for (int i = 3; i < argc; ++i) {
            if (std::strcmp(argv[i], "-i") == 0) {
                in_place = true;
            } else {
                return usage();
            }
        }
        return cmdFmt(path, in_place);
    }
    if (cmd == "digest") {
        return cmdDigest(path);
    }
    return usage();
}
