#pragma once

/// \file fluent_stage.hpp
/// \brief Umbrella header — include this and draw.
///
/// ```cpp
/// #include <fluent_stage/fluent_stage.hpp>
/// using namespace fluent_stage;
///
/// Stage stage(1920, 1080);
/// stage.text("こんにちは", {40, 40}).size(48).shadow();
///
/// CpuRenderer renderer;
/// const Surface& frame = renderer.render(stage, 0.0f);
/// ```
///
/// fluent_stage is the runtime layer tree of Fluent Vision (§1 of
/// docs/design/fluent_stage.ja.md): Scene (declarative documents) compiles
/// into a Stage (this library), which renders into a Surface.

#include "fluent_stage/animation.hpp"
#include "fluent_stage/content.hpp"
#include "fluent_stage/cpu_renderer.hpp"
#include "fluent_stage/filters.hpp"
#include "fluent_stage/geometry.hpp"
#include "fluent_stage/layer.hpp"
#include "fluent_stage/renderer.hpp"
#include "fluent_stage/stage.hpp"
#include "fluent_stage/surface.hpp"
#include "fluent_stage/transaction.hpp"
#include "fluent_stage/types.hpp"
#include "fluent_stage/ui.hpp"
