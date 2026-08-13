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
            const std::string ref = portRef(*node, "image");
            const std::vector<std::string> parts = refParts(ref);
            if (parts.size() != 2 || parts[0] != "$inputs") {
                err(diagnostics, "compile.unsupported_output",
                    "node \"" + node_id +
                        "\": the stage-2 backend renders images from scene inputs only "
                        "(nested composites are not supported yet)");
                return false;
            }
            draw.source_input = parts[1];
            draw.fit = readString(paramLiteral(ir, *node, "fit"), "contain");
        } else if (op == "draw_boxes") {
            draw.kind = DrawOp::Kind::kBoxes;
            const std::vector<std::string> parts = refParts(portRef(*node, "detections"));
            draw.source_input = parts.size() == 2 ? parts[1] : "";
            readVec(paramLiteral(ir, *node, "color"), draw.color, 4);
            draw.show_label = readBool(paramLiteral(ir, *node, "show_label"), false);
            draw.max_instances = boundsCount(*node, "max_instances");
            draw.overflow = boundsOverflow(*node);
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
