#include "render/scene_model.hpp"

#include <algorithm>

namespace fluent_scene {
namespace render {
namespace {

void err(DiagnosticList& diags, std::string code, std::string message) {
    diags.add(std::move(code), Severity::kError, Phase::kCompile, Span{}, std::move(message));
}

const JsonValue* findByName(const JsonValue* array, const char* key, const std::string& value) {
    if (array == nullptr || !array->isArray()) {
        return nullptr;
    }
    for (const JsonValue& element : array->elements()) {
        const JsonValue* name = element.find(key);
        if (name != nullptr && name->kind() == JsonValue::Kind::kString && name->stringValue() == value) {
            return &element;
        }
    }
    return nullptr;
}

std::vector<std::string> refParts(const std::string& ref) {
    std::vector<std::string> parts;
    size_t start = 0;
    while (start <= ref.size()) {
        const size_t dot = ref.find('.', start);
        if (dot == std::string::npos) {
            parts.push_back(ref.substr(start));
            break;
        }
        parts.push_back(ref.substr(start, dot - start));
        start = dot + 1;
    }
    return parts;
}

std::string portRef(const JsonValue& node, const std::string& port) {
    const JsonValue* binding = findByName(node.find("inputs"), "port", port);
    if (binding == nullptr) {
        return {};
    }
    const JsonValue* source = binding->find("source");
    if (source == nullptr || !source->isObject()) {
        return {};
    }
    const JsonValue* ref = source->find("ref");
    return ref != nullptr ? ref->stringValue() : std::string{};
}

// Resolves a node parameter to its literal JsonValue, following one level of
// $params references to the parameter's declared default.
const JsonValue* paramLiteral(const JsonValue& ir, const JsonValue& node, const std::string& name) {
    const JsonValue* param = findByName(node.find("params"), "name", name);
    if (param == nullptr) {
        return nullptr;
    }
    const JsonValue* value = param->find("value");
    if (value == nullptr) {
        return nullptr;
    }
    if (value->isObject()) {
        const JsonValue* ref = value->find("ref");
        if (ref == nullptr) {
            return nullptr;
        }
        const std::vector<std::string> parts = refParts(ref->stringValue());
        if (parts.size() == 2 && parts[0] == "$params") {
            const JsonValue* decl = findByName(ir.find("params"), "name", parts[1]);
            return decl != nullptr ? decl->find("default") : nullptr;
        }
        return nullptr;  // $resources refs are handled by the caller
    }
    return value;
}

// Returns the scene-parameter name when a node parameter is a $params
// reference, empty otherwise.
std::string paramRefName(const JsonValue& node, const std::string& name) {
    const JsonValue* param = findByName(node.find("params"), "name", name);
    if (param == nullptr) {
        return {};
    }
    const JsonValue* value = param->find("value");
    if (value == nullptr || !value->isObject()) {
        return {};
    }
    const JsonValue* ref = value->find("ref");
    if (ref == nullptr) {
        return {};
    }
    const std::vector<std::string> parts = refParts(ref->stringValue());
    return parts.size() == 2 && parts[0] == "$params" ? parts[1] : std::string{};
}

void readVec(const JsonValue* value, float* out, size_t count) {
    if (value == nullptr || !value->isArray() || value->elements().size() != count) {
        return;
    }
    for (size_t i = 0; i < count; ++i) {
        out[i] = static_cast<float>(value->elements()[i].floatValue());
    }
}

bool readBool(const JsonValue* value, bool fallback) {
    return value != nullptr && value->kind() == JsonValue::Kind::kBool ? value->boolValue() : fallback;
}

std::string readString(const JsonValue* value, const std::string& fallback) {
    return value != nullptr && value->kind() == JsonValue::Kind::kString ? value->stringValue() : fallback;
}

uint64_t boundsCount(const JsonValue& node, const char* key) {
    const JsonValue* bounds = node.find("bounds");
    if (bounds == nullptr) {
        return 0;
    }
    const JsonValue* count = bounds->find(key);
    return count != nullptr ? count->uintValue() : 0;
}

std::string boundsOverflow(const JsonValue& node) {
    const JsonValue* bounds = node.find("bounds");
    if (bounds == nullptr) {
        return {};
    }
    const JsonValue* rule = bounds->find("overflow");
    return rule != nullptr ? rule->stringValue() : std::string{};
}

}  // namespace

bool buildSceneModel(const ValidationResult& scene, const PlanResult& plan, SceneModel& out,
                     DiagnosticList& diagnostics) {
    const JsonValue& ir = scene.ir;
    const JsonValue* metadata = ir.find("metadata");
    if (metadata != nullptr) {
        const JsonValue* name = metadata->find("name");
        if (name != nullptr) {
            out.name = name->stringValue();
        }
    }
    const JsonValue* budgets = ir.find("budgets");
    if (budgets == nullptr) {
        err(diagnostics, "compile.invalid_input", "scene IR is missing budgets");
        return false;
    }
    out.width = static_cast<uint32_t>(budgets->find("max_width")->uintValue());
    out.height = static_cast<uint32_t>(budgets->find("max_height")->uintValue());

    // Collect runtime_mutable f32 scene params (the live-tunable knobs).
    if (const JsonValue* params = ir.find("params")) {
        for (const JsonValue& param : params->elements()) {
            const JsonValue* mutable_flag = param.find("runtime_mutable");
            const JsonValue* type = param.find("type");
            if (mutable_flag == nullptr || !mutable_flag->boolValue() || type == nullptr ||
                type->stringValue() != "f32") {
                continue;
            }
            float value = 0.0f;
            const JsonValue* default_value = param.find("default");
            if (default_value != nullptr) {
                value = static_cast<float>(default_value->floatValue());
            }
            out.mutable_params[param.find("name")->stringValue()] = value;
        }
    }

    // The stage-2 backends render the first exported image output.
    const JsonValue* plan_outputs = plan.plan.find("outputs");
    if (plan_outputs == nullptr || !plan_outputs->isArray() || plan_outputs->elements().empty()) {
        err(diagnostics, "compile.unsupported_output",
            "the scene exports no image output; nothing to render");
        return false;
    }
    const std::string target_resource = plan_outputs->elements()[0].find("resource")->stringValue();

    const JsonValue* passes = plan.plan.find("passes");
    const JsonValue* render_pass = nullptr;
    if (passes != nullptr && passes->isArray()) {
        for (const JsonValue& pass : passes->elements()) {
            const JsonValue* target = pass.find("target");
            if (target != nullptr && target->stringValue() == target_resource) {
                render_pass = &pass;
                break;
            }
        }
    }
    if (render_pass == nullptr) {
        err(diagnostics, "compile.invalid_input", "plan has no render pass for the exported output");
        return false;
    }

    const JsonValue* nodes = ir.find("nodes");
    const JsonValue* draws = render_pass->find("draws");
    if (draws == nullptr || !draws->isArray()) {
        err(diagnostics, "compile.invalid_input", "render pass has no draw list");
        return false;
    }
    for (const JsonValue& plan_draw : draws->elements()) {
        const std::string node_id = plan_draw.find("node")->stringValue();
        const std::string op = plan_draw.find("op")->stringValue();
        const JsonValue* node = findByName(nodes, "id", node_id);
        if (node == nullptr) {
            err(diagnostics, "compile.invalid_input", "plan draw references unknown node " + node_id);
            return false;
        }
        DrawOp draw;
        draw.node_id = node_id;
        if (op == "draw_image") {
            draw.kind = DrawOp::Kind::kImage;
            draw.fit = readString(paramLiteral(ir, *node, "fit"), "contain");
            // Follow the image reference through any effect chain back to a
            // scene input; composite-produced images are still unsupported.
            std::string ref = portRef(*node, "image");
            std::vector<std::string> chain;  // draw-side first while walking
            bool resolved = false;
            for (int depth = 0; depth < 8; ++depth) {
                const std::vector<std::string> parts = refParts(ref);
                if (parts.size() == 2 && parts[0] == "$inputs") {
                    draw.source_input = parts[1];
                    resolved = true;
                    break;
                }
                if (parts.size() != 3 || parts[0] != "$nodes") {
                    break;
                }
                const JsonValue* upstream = findByName(nodes, "id", parts[1]);
                if (upstream == nullptr) {
                    break;
                }
                const std::string upstream_type = upstream->find("type")->stringValue();
                if (upstream_type != "effects.blur" && upstream_type != "effects.color_transform") {
                    break;  // e.g. a composite image — not supported yet
                }
                chain.push_back(parts[1]);
                EffectOp effect;
                effect.node_id = parts[1];
                if (upstream_type == "effects.blur") {
                    effect.kind = EffectOp::Kind::kBlur;
                    const JsonValue* radius = paramLiteral(ir, *upstream, "radius");
                    if (radius != nullptr) {
                        effect.radius = static_cast<float>(radius->floatValue());
                    }
                    effect.radius_param = paramRefName(*upstream, "radius");
                } else {
                    effect.kind = EffectOp::Kind::kColorTransform;
                    const JsonValue* v = paramLiteral(ir, *upstream, "brightness");
                    if (v != nullptr) effect.brightness = static_cast<float>(v->floatValue());
                    v = paramLiteral(ir, *upstream, "contrast");
                    if (v != nullptr) effect.contrast = static_cast<float>(v->floatValue());
                    v = paramLiteral(ir, *upstream, "saturation");
                    if (v != nullptr) effect.saturation = static_cast<float>(v->floatValue());
                    v = paramLiteral(ir, *upstream, "gamma");
                    if (v != nullptr) effect.gamma = static_cast<float>(v->floatValue());
                    effect.brightness_param = paramRefName(*upstream, "brightness");
                    effect.contrast_param = paramRefName(*upstream, "contrast");
                    effect.saturation_param = paramRefName(*upstream, "saturation");
                    effect.gamma_param = paramRefName(*upstream, "gamma");
                }
                ref = portRef(*upstream, "image");
                const std::vector<std::string> src = refParts(ref);
                if (src.size() == 2 && src[0] == "$inputs") {
                    effect.source_input = src[1];
                } else if (src.size() == 3 && src[0] == "$nodes") {
                    effect.source_effect = src[1];
                }
                bool known = false;
                for (const EffectOp& existing : out.effects) {
                    if (existing.node_id == effect.node_id) {
                        known = true;
                        break;
                    }
                }
                if (!known) {
                    // Insert before any effect that consumes this one so the
                    // list stays in dependency order (sources first).
                    size_t at = out.effects.size();
                    for (size_t i = 0; i < out.effects.size(); ++i) {
                        if (out.effects[i].source_effect == effect.node_id) {
                            at = i;
                            break;
                        }
                    }
                    out.effects.insert(out.effects.begin() + static_cast<ptrdiff_t>(at),
                                       std::move(effect));
                }
            }
            if (!resolved) {
                err(diagnostics, "compile.unsupported_output",
                    "node \"" + node_id +
                        "\": image sources must be a scene input, optionally through "
                        "effects.* nodes (composite chaining is not supported yet)");
                return false;
            }
            std::reverse(chain.begin(), chain.end());  // first-applied first
            draw.effect_chain = std::move(chain);
        } else if (op == "draw_boxes") {
            draw.kind = DrawOp::Kind::kBoxes;
            const std::vector<std::string> parts = refParts(portRef(*node, "detections"));
            draw.source_input = parts.size() == 2 ? parts[1] : "";
            readVec(paramLiteral(ir, *node, "color"), draw.color, 4);
            draw.show_label = readBool(paramLiteral(ir, *node, "show_label"), false);
            draw.max_instances = boundsCount(*node, "max_instances");
            draw.overflow = boundsOverflow(*node);
            const JsonValue* smoothing = paramLiteral(ir, *node, "smoothing");
            if (smoothing != nullptr) {
                draw.smoothing = static_cast<float>(smoothing->floatValue());
            }
        } else if (op == "draw_circles" || op == "draw_polyline") {
            draw.kind = op == "draw_circles" ? DrawOp::Kind::kCircles : DrawOp::Kind::kPolyline;
            const std::vector<std::string> parts = refParts(portRef(*node, "points"));
            draw.source_input = parts.size() == 2 ? parts[1] : "";
            readVec(paramLiteral(ir, *node, "color"), draw.color, 4);
            draw.max_points = boundsCount(*node, "max_points");
            const JsonValue* thickness = paramLiteral(ir, *node, "thickness");
            if (thickness != nullptr) {
                draw.thickness = static_cast<float>(thickness->floatValue());
            }
            if (draw.kind == DrawOp::Kind::kCircles) {
                const JsonValue* radius = paramLiteral(ir, *node, "radius");
                if (radius != nullptr) {
                    draw.radius = static_cast<float>(radius->floatValue());
                }
            }
        } else if (op == "draw_text") {
            draw.kind = DrawOp::Kind::kText;
            const std::vector<std::string> parts = refParts(portRef(*node, "text"));
            draw.source_input = parts.size() == 2 ? parts[1] : "";
            readVec(paramLiteral(ir, *node, "position"), draw.position, 2);
            readVec(paramLiteral(ir, *node, "color"), draw.color, 4);
            draw.shadow = readBool(paramLiteral(ir, *node, "shadow"), false);
            draw.default_text = readString(paramLiteral(ir, *node, "default_text"), "");
            draw.max_glyphs = boundsCount(*node, "max_glyphs");
            // Font resource: uri + glyph capacity from the IR resources table.
            const JsonValue* param = findByName(node->find("params"), "name", "font");
            if (param != nullptr && param->find("value") != nullptr && param->find("value")->isObject()) {
                const JsonValue* ref = param->find("value")->find("ref");
                if (ref != nullptr) {
                    const std::vector<std::string> font_parts = refParts(ref->stringValue());
                    if (font_parts.size() == 2 && font_parts[0] == "$resources") {
                        const JsonValue* resource =
                            findByName(ir.find("resources"), "name", font_parts[1]);
                        if (resource != nullptr) {
                            draw.font_uri = readString(resource->find("uri"), "");
                            const JsonValue* capacity = resource->find("glyph_capacity");
                            draw.glyph_capacity = capacity != nullptr ? capacity->uintValue() : 0;
                        }
                    }
                }
            }
            if (draw.font_uri.empty()) {
                err(diagnostics, "compile.invalid_input",
                    "node \"" + node_id + "\" has no resolvable font resource");
                return false;
            }
        } else {
            err(diagnostics, "compile.invalid_input",
                "unknown plan draw op \"" + op + "\" for node " + node_id);
            return false;
        }
        out.draws.push_back(std::move(draw));
    }
    return true;
}

}  // namespace render
}  // namespace fluent_scene
