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

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "fluent_stage/scene/document.hpp"
#include "fluent_stage/stage.hpp"
#include "fluent_stage/ui.hpp"

namespace fluent_stage::scene {

/// One user gesture leaving a declared UI control (§10-4). The host decides
/// where it goes from here — a C++ callback, a ROS topic, a log.
struct UiEvent {
    std::string id;       ///< The control layer's id.
    const char* control;  ///< "button" | "switch" | "slider".
    float value = 0;      ///< Slider value (switch: 1/0, button: 1).
    bool flag = false;    ///< Switch state (button taps: true).
};

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

    /// Installs the single UI event handler (§10-4): every declared
    /// button/switch/slider routes user gestures here. Pointer injection is
    /// the host's job (`stage().pointerDown/Move/Up`); programmatic changes
    /// (setParam on a bound control) do not fire events, exactly like the
    /// C++ controls.
    void onUiEvent(std::function<void(const UiEvent&)> handler) {
        ui_handler_ = std::move(handler);
    }

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
    /// One `$params` binding to a UI control's state (compiler bookkeeping).
    struct ControlBinding {
        enum class Kind { SwitchOn, SliderValue, GaugeValue };
        Kind kind;
        void* control;  ///< The ui:: instance owned by controls_.
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
    std::map<std::string, std::vector<ControlBinding>> control_bindings_;
    std::map<const Layer*, TransitionDecl> layer_transitions_;
    std::vector<std::string> diagnostics_;

    // Declared UI controls (§10). The objects carry the behavior; their
    // layers live in the Stage like everything else.
    std::vector<std::unique_ptr<ui::Button>> buttons_;
    std::vector<std::unique_ptr<ui::Switch>> switches_;
    std::vector<std::unique_ptr<ui::Slider>> sliders_;
    std::vector<std::unique_ptr<ui::Gauge>> gauges_;
    std::function<void(const UiEvent&)> ui_handler_;
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
