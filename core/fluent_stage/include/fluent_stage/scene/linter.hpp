#pragma once

/// \file linter.hpp
/// \brief The §13-2 design linter — "good", checked by machine, past what
///        the type system can reject.
///
/// `lint()` inspects a compiled scene with the renderer's help (it renders
/// the stage to measure text and to sample what is actually behind each
/// text run) and returns warnings for pictures that are valid but bad:
///
///   - `lint.contrast` — text/background contrast below WCAG 4.5:1,
///     measured against the rendered pixels behind the text.
///   - `lint.text_overflow` — text that escapes its layer's declared bounds.
///   - `lint.occlusion` — a layer completely covered by a later opaque
///     layer (`lint.protected_occluded`, an **error**, when the covered
///     layer is `protected: true` — §13-4).
///   - `lint.offscreen` — a layer placed entirely outside the canvas.
///
/// Feeding the scene's inputs first (placeholders otherwise stand in) makes
/// the contrast check reflect the live picture. The linter renders the
/// stage; run it on a scene you are previewing, not mid-frame.

#include "fluent_stage/renderer.hpp"
#include "fluent_stage/scene/compiler.hpp"
#include "fluent_stage/scene/diagnostics.hpp"

namespace fluent_stage::scene {

/// Runs every design lint against `scene`, rendering through `renderer`.
/// Returns diagnostics in canonical order (warnings, plus errors for
/// protected-layer occlusion).
DiagnosticList lint(CompiledScene& scene, Renderer& renderer);

}  // namespace fluent_stage::scene
