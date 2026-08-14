#pragma once

/// \file compiler.hpp
/// \brief Scene document → live Stage (§1.3 compile step).
///
/// `compile()` builds a Stage from a validated SceneDoc through the same
/// Layer API a C++ author uses — the §2 contract that YAML and C++ produce
/// the identical picture is upheld by sharing the code path, not by
/// imitating it. The result, `CompiledScene`, owns the Stage plus the
/// document's runtime surface:
///
///   - **inputs** — `setImage` / `setText` / `setPoints` / `setBoxes` feed
///     declared `$inputs` into every layer bound to them (type-checked
///     against the declaration). Until the first data arrives, a layer shows
///     its declared fallback: a deterministic labeled placeholder panel
///     (images), stays hidden, or simply holds empty.
///   - **params** — `setParam` writes a declared `$params` value through to
///     every bound attribute. A param with an `animate` declaration (§9)
///     interpolates through a Transaction; otherwise a bound layer's
///     `transition` applies; otherwise the change snaps.
///
/// Atomic replacement (§2: validate → compile → preview → activate) is the
/// caller's frame loop: compile the new document off to the side, then swap
/// which CompiledScene you render at a frame boundary.

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "fluent_stage/scene/document.hpp"
#include "fluent_stage/stage.hpp"

namespace fluent_stage::scene {

/// Resource bounds for compilation. The declared tree (placeholders
/// included) is checked against `limits` up front and rejected with
/// diagnostics — never by throwing mid-build.
struct CompileOptions {
    StageLimits limits{};
};

/// A compiled, runnable Scene: the Stage plus the document's input/param
/// runtime surface. Move-only; render it like any Stage.
class CompiledScene {
public:
    CompiledScene(const CompiledScene&) = delete;
    CompiledScene& operator=(const CompiledScene&) = delete;

    /// The live layer tree. Render with any Renderer; mutating it directly
    /// is the C++ author's privilege (§1.3) — inputs/params stay coherent
    /// only for layers the document declared.
    Stage& stage() { return *stage_; }
    const Stage& stage() const { return *stage_; }

    /// The validated document this Stage was built from (owned copy).
    const SceneDoc& doc() const { return doc_; }

    /// The document's canonical digest (§1.3) — the audit identity of what
    /// is on screen.
    const std::string& digest() const { return digest_; }

    /// \name Input feeds
    /// Each returns false (with a diagnostic) when the name is undeclared
    /// or the declared type does not match the call. Sequence inputs clamp
    /// to their declared capacity with a diagnostic.
    /// @{
    bool setImage(const std::string& input, const ImageView& view);
    bool setText(const std::string& input, const std::string& utf8);
    bool setPoints(const std::string& input, const std::vector<Vec2>& points);
    bool setBoxes(const std::string& input, const std::vector<Box>& boxes);
    /// @}

    /// \name Param writes (§9 animate declarations honored)
    /// Each returns false (with a diagnostic) when the name is undeclared,
    /// the type does not match, or the param is not runtime_mutable.
    /// @{
    bool setParam(const std::string& name, float value);
    bool setParam(const std::string& name, Vec2 value);
    bool setParam(const std::string& name, Color value);
    bool setParam(const std::string& name, bool value);
    /// @}

    /// Runtime diagnostics (bad input names, type mismatches, clamps),
    /// oldest first; clears the queue. Stage diagnostics are included.
    std::vector<std::string> drainDiagnostics();

    /// The compiled layer for a document node (linter/inspector use);
    /// nullptr for nodes that failed to build.
    Layer* layerFor(const LayerDecl* decl) const;

    /// Document node ↔ live layer pairs in tree order (§13-3 inspector
    /// surface; the linter walks this).
    const std::vector<std::pair<const LayerDecl*, Layer*>>& nodes() const { return nodes_; }

    /// One `$params` attribute binding (compiler bookkeeping).
    struct ParamBinding {
        Layer* layer;
        AttrId attr;
    };
    /// One `$inputs` content binding with its fallback presentation
    /// (compiler bookkeeping).
    struct InputBinding {
        Layer* layer;
        InputFallback fallback;
        std::vector<Layer*> placeholder_parts;  ///< Removed on first data.
        bool background_was_placeholder = false;
        bool fed = false;
    };

private:
    friend struct CompileAccess;
    CompiledScene() = default;

    void diagnose(std::string message);
    void applyParam(const ParamDecl& decl, const Value& value);

    SceneDoc doc_;
    std::string digest_;
    std::unique_ptr<Stage> stage_;
    std::vector<std::pair<const LayerDecl*, Layer*>> nodes_;

    std::map<std::string, std::vector<ParamBinding>> param_bindings_;
    std::map<std::string, std::vector<InputBinding>> input_bindings_;
    std::map<const Layer*, TransitionDecl> layer_transitions_;
    std::vector<std::string> diagnostics_;
};

/// The outcome of `compile`. `scene` is null when compilation was rejected.
struct CompileResult {
    std::unique_ptr<CompiledScene> scene;
    DiagnosticList diagnostics;

    /// True when the document compiled.
    bool ok() const { return scene != nullptr && !diagnostics.hasErrors(); }
};

/// Builds a Stage from a validated document. `doc` must come from a
/// `parseScene` result with `ok()` — compiling an erroneous document is
/// rejected outright.
CompileResult compile(const SceneDoc& doc, const CompileOptions& options = {});

}  // namespace fluent_stage::scene
