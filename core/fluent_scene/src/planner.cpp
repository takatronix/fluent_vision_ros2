#include "fluent_scene/planner.hpp"

#include <algorithm>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "fluent_scene/sha256.hpp"

namespace fluent_scene {

const char kSupportedPlanIdentity[] = "fluent.plan/v1alpha1";

namespace {

constexpr uint64_t kBoxInstanceBytes = 32;    // bbox vec4 + color + flags
constexpr uint64_t kGlyphQuadBytes = 16;      // position + uv + glyph index
constexpr uint64_t kAtlasBytesPerGlyph = 1024;  // 32x32 R8 cell

struct ImageFormat {
    const char* name;
    uint64_t bytes_per_pixel;
};

bool imageFormatOf(const std::string& type_name, ImageFormat& out) {
    if (type_name == "image.r8") {
        out = {"r8", 1};
        return true;
    }
    if (type_name == "image.rgba8") {
        out = {"rgba8", 4};
        return true;
    }
    if (type_name == "image.rgba16f") {
        out = {"rgba16f", 8};
        return true;
    }
    if (type_name == "image.depth32f") {
        out = {"depth32f", 4};
        return true;
    }
    return false;
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

const JsonValue* findByName(const JsonValue* array, const char* name_key, const std::string& value) {
    if (array == nullptr || !array->isArray()) {
        return nullptr;
    }
    for (const JsonValue& element : array->elements()) {
        const JsonValue* name = element.find(name_key);
        if (name != nullptr && name->kind() == JsonValue::Kind::kString && name->stringValue() == value) {
            return &element;
        }
    }
    return nullptr;
}

// Returns the single {"ref": ...} string of a node port, or empty.
std::string portRef(const JsonValue& node, const std::string& port) {
    const JsonValue* inputs = node.find("inputs");
    const JsonValue* binding = findByName(inputs, "port", port);
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

std::vector<std::string> portRefList(const JsonValue& node, const std::string& port) {
    std::vector<std::string> refs;
    const JsonValue* inputs = node.find("inputs");
    const JsonValue* binding = findByName(inputs, "port", port);
    if (binding == nullptr) {
        return refs;
    }
    const JsonValue* source = binding->find("source");
    if (source == nullptr || !source->isArray()) {
        return refs;
    }
    for (const JsonValue& element : source->elements()) {
        const JsonValue* ref = element.find("ref");
        if (ref != nullptr) {
            refs.push_back(ref->stringValue());
        }
    }
    return refs;
}

std::string paramRef(const JsonValue& node, const std::string& name) {
    const JsonValue* params = node.find("params");
    const JsonValue* param = findByName(params, "name", name);
    if (param == nullptr) {
        return {};
    }
    const JsonValue* value = param->find("value");
    if (value == nullptr || !value->isObject()) {
        return {};
    }
    const JsonValue* ref = value->find("ref");
    return ref != nullptr ? ref->stringValue() : std::string{};
}

uint64_t boundsCount(const JsonValue& node, const std::string& key) {
    const JsonValue* bounds = node.find("bounds");
    if (bounds == nullptr) {
        return 0;
    }
    const JsonValue* count = bounds->find(key);
    return count != nullptr ? count->uintValue() : 0;
}

struct PlanResource {
    std::string id;
    std::string kind;      // image | buffer | font_atlas | staging_pool
    std::string format;    // images only
    uint64_t width = 0;
    uint64_t height = 0;
    uint64_t glyphs = 0;
    uint64_t bytes = 0;
    std::string residency;  // device | host
    std::string update;     // per_frame | on_change | static (streamed resources)
    std::string source;     // originating $inputs/$resources/$nodes reference
    int first_use = -1;
    int last_use = -1;
    bool exported = false;
    bool aliasable = false;
    bool per_frame_upload = false;
};

struct PlanDraw {
    std::string node;
    std::string op;
    std::vector<std::string> reads;
    uint64_t max_instances = 0;
    uint64_t max_glyphs = 0;
};

struct PlanPass {
    std::string id;
    std::string kind;  // upload | render
    std::string target;
    std::vector<std::string> writes;
    std::vector<PlanDraw> draws;
    uint64_t bytes_per_frame = 0;
};

class ScenePlanner {
public:
    ScenePlanner(const ValidationResult& validated, DiagnosticList& diagnostics)
        : ir_(validated.ir), scene_digest_(validated.digest), diags_(diagnostics) {}

    PlanResult run() {
        PlanResult result;
        const size_t errors_before = diags_.errorCount();
        readBudgets();
        collectNodes();
        collectSceneOutputs();
        computeLiveSet();
        lowerLiveComposites();
        buildUploadPass();
        computeLifetimes();
        checkBudgets();
        if (diags_.errorCount() != errors_before) {
            return result;
        }
        result.plan = buildPlanJson();
        result.plan_text = result.plan.serialize();
        result.digest = "sha256:" + sha256Hex(result.plan_text);
        result.ok = true;
        return result;
    }

private:
    void err(std::string code, std::string message,
             std::vector<std::pair<std::string, std::string>> context = {}) {
        diags_.add(std::move(code), Severity::kError, Phase::kCompile, Span{}, std::move(message),
                   std::move(context));
    }

    void info(std::string code, std::string message,
              std::vector<std::pair<std::string, std::string>> context = {}) {
        diags_.add(std::move(code), Severity::kInfo, Phase::kCompile, Span{}, std::move(message),
                   std::move(context));
    }

    void readBudgets() {
        const JsonValue* budgets = ir_.find("budgets");
        if (budgets == nullptr) {
            err("compile.invalid_input", "canonical IR is missing budgets");
            return;
        }
        const auto get = [&](const char* key) -> uint64_t {
            const JsonValue* value = budgets->find(key);
            return value != nullptr ? value->uintValue() : 0;
        };
        max_width_ = get("max_width");
        max_height_ = get("max_height");
        max_gpu_bytes_ = get("max_gpu_bytes");
        max_upload_bytes_ = get("max_upload_bytes_per_frame");
    }

    void collectNodes() {
        const JsonValue* nodes = ir_.find("nodes");
        if (nodes == nullptr || !nodes->isArray()) {
            return;
        }
        for (const JsonValue& node : nodes->elements()) {
            const JsonValue* id = node.find("id");
            const JsonValue* type = node.find("type");
            if (id == nullptr || type == nullptr) {
                continue;
            }
            node_order_.push_back(id->stringValue());
            nodes_.emplace(id->stringValue(), &node);
            node_types_.emplace(id->stringValue(), type->stringValue());
        }
    }

    void collectSceneOutputs() {
        const JsonValue* outputs = ir_.find("outputs");
        if (outputs == nullptr || !outputs->isArray()) {
            return;
        }
        for (const JsonValue& output : outputs->elements()) {
            const JsonValue* name = output.find("name");
            const JsonValue* type = output.find("type");
            const JsonValue* source = output.find("source");
            if (name == nullptr || type == nullptr || source == nullptr) {
                continue;
            }
            const JsonValue* ref = source->find("ref");
            if (ref == nullptr) {
                continue;
            }
            const std::vector<std::string> parts = refParts(ref->stringValue());
            if (parts.size() >= 2 && parts[0] == "$runtime") {
                continue;  // diagnostic outputs need no GPU resources
            }
            const std::string& type_name = type->stringValue();
            if (parts.size() != 3 || parts[0] != "$nodes") {
                continue;
            }
            if (type_name.rfind("image.", 0) != 0) {
                err("compile.unsupported_output",
                    "scene output \"" + name->stringValue() + "\" has type " + type_name +
                        ", which cannot be presented; composite layers into an image first",
                    {{"output", name->stringValue()}, {"type", type_name}});
                continue;
            }
            exported_nodes_.insert(parts[1]);
            output_bindings_.emplace_back(name->stringValue(), parts[1]);
        }
        std::sort(output_bindings_.begin(), output_bindings_.end());
    }

    // Liveness: nodes reachable from exported nodes through $nodes references.
    void computeLiveSet() {
        std::vector<std::string> stack(exported_nodes_.begin(), exported_nodes_.end());
        while (!stack.empty()) {
            const std::string id = stack.back();
            stack.pop_back();
            if (!live_.insert(id).second) {
                continue;
            }
            auto it = nodes_.find(id);
            if (it == nodes_.end()) {
                continue;
            }
            const JsonValue* inputs = it->second->find("inputs");
            if (inputs == nullptr || !inputs->isArray()) {
                continue;
            }
            for (const JsonValue& binding : inputs->elements()) {
                const JsonValue* source = binding.find("source");
                if (source == nullptr) {
                    continue;
                }
                const auto follow = [&](const JsonValue& ref_object) {
                    const JsonValue* ref = ref_object.find("ref");
                    if (ref == nullptr) {
                        return;
                    }
                    const std::vector<std::string> parts = refParts(ref->stringValue());
                    if (parts.size() == 3 && parts[0] == "$nodes") {
                        stack.push_back(parts[1]);
                    }
                };
                if (source->isObject()) {
                    follow(*source);
                } else if (source->isArray()) {
                    for (const JsonValue& element : source->elements()) {
                        follow(element);
                    }
                }
            }
        }
        for (const std::string& id : node_order_) {
            if (live_.count(id) == 0) {
                info("compile.dead_node",
                     "node \"" + id + "\" does not contribute to any scene output and was not planned",
                     {{"node", id}});
            }
        }
    }

    PlanResource& ensureResource(const std::string& id) {
        auto it = resources_.find(id);
        if (it == resources_.end()) {
            PlanResource resource;
            resource.id = id;
            it = resources_.emplace(id, std::move(resource)).first;
        }
        return it->second;
    }

    // Imported image for a scene input; returns the resource id.
    std::string ensureImportedImage(const std::string& input_name) {
        const std::string id = "img.in." + input_name;
        if (resources_.count(id) != 0) {
            return id;
        }
        const JsonValue* input = findByName(ir_.find("inputs"), "name", input_name);
        ImageFormat format{"rgba8", 4};
        std::string update = "per_frame";
        if (input != nullptr) {
            const JsonValue* type = input->find("type");
            if (type != nullptr) {
                imageFormatOf(type->stringValue(), format);
            }
            const JsonValue* update_value = input->find("update");
            if (update_value != nullptr) {
                update = update_value->stringValue();
            }
        }
        PlanResource& resource = ensureResource(id);
        resource.kind = "image";
        resource.format = format.name;
        resource.width = max_width_;
        resource.height = max_height_;
        resource.bytes = max_width_ * max_height_ * format.bytes_per_pixel;
        resource.residency = "device";
        resource.update = update;
        resource.source = "$inputs." + input_name;
        resource.per_frame_upload = (update == "per_frame");
        return id;
    }

    std::string ensureStreamedBuffer(const std::string& id, uint64_t bytes, const std::string& source,
                                     const std::string& update) {
        PlanResource& resource = ensureResource(id);
        resource.kind = "buffer";
        resource.bytes = bytes;
        resource.residency = "device";
        resource.update = update;
        resource.source = source;
        resource.per_frame_upload = (update == "per_frame");
        return id;
    }

    std::string ensureFontAtlas(const std::string& resource_name) {
        const std::string id = "atlas." + resource_name;
        if (resources_.count(id) != 0) {
            return id;
        }
        const JsonValue* declared = findByName(ir_.find("resources"), "name", resource_name);
        uint64_t glyphs = 0;
        if (declared != nullptr) {
            const JsonValue* capacity = declared->find("glyph_capacity");
            if (capacity != nullptr) {
                glyphs = capacity->uintValue();
            }
        }
        PlanResource& resource = ensureResource(id);
        resource.kind = "font_atlas";
        resource.format = "r8";
        resource.glyphs = glyphs;
        resource.bytes = glyphs * kAtlasBytesPerGlyph;
        resource.residency = "device";
        resource.update = "on_change";
        resource.source = "$resources." + resource_name;
        return id;
    }

    std::string inputUpdateOf(const std::string& ref) {
        const std::vector<std::string> parts = refParts(ref);
        if (parts.size() == 2 && parts[0] == "$inputs") {
            const JsonValue* input = findByName(ir_.find("inputs"), "name", parts[1]);
            if (input != nullptr) {
                const JsonValue* update = input->find("update");
                if (update != nullptr) {
                    return update->stringValue();
                }
            }
        }
        return "per_frame";
    }

    bool lowerDraw(const std::string& node_id, PlanDraw& draw) {
        const JsonValue& node = *nodes_.at(node_id);
        const std::string& type = node_types_.at(node_id);
        draw.node = node_id;
        if (type == "visual.image2d") {
            draw.op = "draw_image";
            const std::string ref = portRef(node, "image");
            const std::vector<std::string> parts = refParts(ref);
            if (parts.size() == 2 && parts[0] == "$inputs") {
                draw.reads.push_back(ensureImportedImage(parts[1]));
            } else if (parts.size() == 3 && parts[0] == "$nodes") {
                draw.reads.push_back("img.node." + parts[1]);
            } else {
                err("compile.invalid_input", "node \"" + node_id + "\" has an unplannable image source");
                return false;
            }
            return true;
        }
        if (type == "visual.boxes2d") {
            draw.op = "draw_boxes";
            draw.max_instances = boundsCount(node, "max_instances");
            const std::string source = portRef(node, "detections");
            draw.reads.push_back(ensureStreamedBuffer("buf." + node_id + ".instances",
                                                      draw.max_instances * kBoxInstanceBytes, source,
                                                      inputUpdateOf(source)));
            return true;
        }
        if (type == "text.dynamic") {
            draw.op = "draw_text";
            draw.max_glyphs = boundsCount(node, "max_glyphs");
            const std::string font_ref = paramRef(node, "font");
            const std::vector<std::string> font_parts = refParts(font_ref);
            if (font_parts.size() == 2 && font_parts[0] == "$resources") {
                draw.reads.push_back(ensureFontAtlas(font_parts[1]));
            }
            const std::string source = portRef(node, "text");
            draw.reads.push_back(ensureStreamedBuffer("buf." + node_id + ".glyphs",
                                                      draw.max_glyphs * kGlyphQuadBytes, source,
                                                      inputUpdateOf(source)));
            return true;
        }
        err("compile.invalid_input",
            "node \"" + node_id + "\" of type " + type + " cannot be lowered as a layer draw");
        return false;
    }

    void lowerLiveComposites() {
        for (const std::string& id : node_order_) {
            if (live_.count(id) == 0 || node_types_.at(id) != "composite.layers") {
                continue;
            }
            const JsonValue& node = *nodes_.at(id);
            PlanPass pass;
            pass.id = "pass.render." + id;
            pass.kind = "render";
            const std::string target_id = "img.node." + id;
            PlanResource& target = ensureResource(target_id);
            target.kind = "image";
            target.format = "rgba8";
            target.width = max_width_;
            target.height = max_height_;
            target.bytes = max_width_ * max_height_ * 4;
            target.residency = "device";
            target.source = "$nodes." + id + ".image";
            target.exported = exported_nodes_.count(id) != 0;
            target.aliasable = !target.exported;
            pass.target = target_id;
            pass.writes.push_back(target_id);
            for (const std::string& layer_ref : portRefList(node, "layers")) {
                const std::vector<std::string> parts = refParts(layer_ref);
                if (parts.size() != 3 || parts[0] != "$nodes") {
                    continue;
                }
                PlanDraw draw;
                if (lowerDraw(parts[1], draw)) {
                    pass.draws.push_back(std::move(draw));
                }
            }
            render_passes_.push_back(std::move(pass));
        }
    }

    void buildUploadPass() {
        PlanPass upload;
        upload.id = "pass.upload";
        upload.kind = "upload";
        std::vector<std::string> streamed;
        for (const auto& [id, resource] : resources_) {
            if (!resource.update.empty() && resource.kind != "font_atlas") {
                streamed.push_back(id);
                if (resource.per_frame_upload) {
                    upload.bytes_per_frame += resource.bytes;
                }
            }
        }
        if (streamed.empty()) {
            passes_ = render_passes_;
            return;
        }
        std::sort(streamed.begin(), streamed.end());
        upload.writes = streamed;
        upload_bytes_planned_ = upload.bytes_per_frame;

        PlanResource& staging = ensureResource("staging.upload");
        staging.kind = "staging_pool";
        staging.bytes = max_upload_bytes_;
        staging.residency = "host";

        passes_.clear();
        passes_.push_back(std::move(upload));
        for (PlanPass& pass : render_passes_) {
            passes_.push_back(std::move(pass));
        }
        render_passes_.clear();
    }

    void computeLifetimes() {
        for (size_t index = 0; index < passes_.size(); ++index) {
            const PlanPass& pass = passes_[index];
            const auto touch = [&](const std::string& id) {
                PlanResource& resource = resources_.at(id);
                if (resource.first_use < 0) {
                    resource.first_use = static_cast<int>(index);
                }
                resource.last_use = static_cast<int>(index);
            };
            for (const std::string& id : pass.writes) {
                touch(id);
            }
            if (pass.kind == "upload") {
                touch("staging.upload");
            }
            for (const PlanDraw& draw : pass.draws) {
                for (const std::string& id : draw.reads) {
                    touch(id);
                }
            }
        }
    }

    void checkBudgets() {
        for (const auto& [id, resource] : resources_) {
            if (resource.residency == "device") {
                gpu_bytes_planned_ += resource.bytes;
            }
        }
        if (gpu_bytes_planned_ > max_gpu_bytes_) {
            err("compile.budget_exceeded",
                "planned device memory " + std::to_string(gpu_bytes_planned_) +
                    " bytes exceeds budgets.max_gpu_bytes " + std::to_string(max_gpu_bytes_),
                {{"budget", "max_gpu_bytes"},
                 {"limit", std::to_string(max_gpu_bytes_)},
                 {"planned", std::to_string(gpu_bytes_planned_)}});
        }
        if (upload_bytes_planned_ > max_upload_bytes_) {
            err("compile.budget_exceeded",
                "planned per-frame upload " + std::to_string(upload_bytes_planned_) +
                    " bytes exceeds budgets.max_upload_bytes_per_frame " +
                    std::to_string(max_upload_bytes_),
                {{"budget", "max_upload_bytes_per_frame"},
                 {"limit", std::to_string(max_upload_bytes_)},
                 {"planned", std::to_string(upload_bytes_planned_)}});
        }
    }

    JsonValue buildPlanJson() {
        JsonValue plan = JsonValue::makeObject();
        plan.set("schema", JsonValue::makeString(kSupportedPlanIdentity));
        const JsonValue* metadata = ir_.find("metadata");
        const JsonValue* name = metadata != nullptr ? metadata->find("name") : nullptr;
        plan.set("scene", JsonValue::makeString(name != nullptr ? name->stringValue() : ""));
        plan.set("scene_digest", JsonValue::makeString(scene_digest_));

        JsonValue passes = JsonValue::makeArray();
        for (size_t index = 0; index < passes_.size(); ++index) {
            const PlanPass& pass = passes_[index];
            JsonValue pass_value = JsonValue::makeObject();
            pass_value.set("id", JsonValue::makeString(pass.id));
            pass_value.set("kind", JsonValue::makeString(pass.kind));
            pass_value.set("index", JsonValue::makeUInt(index));
            if (!pass.target.empty()) {
                pass_value.set("target", JsonValue::makeString(pass.target));
            }
            JsonValue writes = JsonValue::makeArray();
            for (const std::string& id : pass.writes) {
                writes.append(JsonValue::makeString(id));
            }
            pass_value.set("writes", std::move(writes));
            if (pass.kind == "render") {
                JsonValue draws = JsonValue::makeArray();
                for (const PlanDraw& draw : pass.draws) {
                    JsonValue draw_value = JsonValue::makeObject();
                    draw_value.set("node", JsonValue::makeString(draw.node));
                    draw_value.set("op", JsonValue::makeString(draw.op));
                    JsonValue reads = JsonValue::makeArray();
                    for (const std::string& id : draw.reads) {
                        reads.append(JsonValue::makeString(id));
                    }
                    draw_value.set("reads", std::move(reads));
                    if (draw.max_instances > 0) {
                        draw_value.set("max_instances", JsonValue::makeUInt(draw.max_instances));
                    }
                    if (draw.max_glyphs > 0) {
                        draw_value.set("max_glyphs", JsonValue::makeUInt(draw.max_glyphs));
                    }
                    draws.append(std::move(draw_value));
                }
                pass_value.set("draws", std::move(draws));
            }
            if (pass.kind == "upload") {
                pass_value.set("bytes_per_frame", JsonValue::makeUInt(pass.bytes_per_frame));
            }
            passes.append(std::move(pass_value));
        }
        plan.set("passes", std::move(passes));

        JsonValue resources = JsonValue::makeArray();
        for (const auto& [id, resource] : resources_) {
            JsonValue value = JsonValue::makeObject();
            value.set("id", JsonValue::makeString(resource.id));
            value.set("kind", JsonValue::makeString(resource.kind));
            if (!resource.format.empty()) {
                value.set("format", JsonValue::makeString(resource.format));
            }
            if (resource.width > 0) {
                value.set("width", JsonValue::makeUInt(resource.width));
                value.set("height", JsonValue::makeUInt(resource.height));
            }
            if (resource.glyphs > 0) {
                value.set("glyphs", JsonValue::makeUInt(resource.glyphs));
            }
            value.set("bytes", JsonValue::makeUInt(resource.bytes));
            value.set("residency", JsonValue::makeString(resource.residency));
            if (!resource.update.empty()) {
                value.set("update", JsonValue::makeString(resource.update));
            }
            if (!resource.source.empty()) {
                value.set("source", JsonValue::makeString(resource.source));
            }
            value.set("first_use", JsonValue::makeInt(resource.first_use));
            value.set("last_use", JsonValue::makeInt(resource.last_use));
            value.set("exported", JsonValue::makeBool(resource.exported));
            value.set("aliasable", JsonValue::makeBool(resource.aliasable));
            resources.append(std::move(value));
        }
        plan.set("resources", std::move(resources));

        JsonValue outputs = JsonValue::makeArray();
        for (const auto& [output_name, node_id] : output_bindings_) {
            JsonValue value = JsonValue::makeObject();
            value.set("name", JsonValue::makeString(output_name));
            value.set("resource", JsonValue::makeString("img.node." + node_id));
            outputs.append(std::move(value));
        }
        plan.set("outputs", std::move(outputs));

        JsonValue budgets = JsonValue::makeObject();
        budgets.set("gpu_bytes_budget", JsonValue::makeUInt(max_gpu_bytes_));
        budgets.set("gpu_bytes_planned", JsonValue::makeUInt(gpu_bytes_planned_));
        budgets.set("upload_bytes_per_frame_budget", JsonValue::makeUInt(max_upload_bytes_));
        budgets.set("upload_bytes_per_frame_planned", JsonValue::makeUInt(upload_bytes_planned_));
        plan.set("budgets", std::move(budgets));
        return plan;
    }

    const JsonValue& ir_;
    const std::string scene_digest_;
    DiagnosticList& diags_;

    uint64_t max_width_ = 0;
    uint64_t max_height_ = 0;
    uint64_t max_gpu_bytes_ = 0;
    uint64_t max_upload_bytes_ = 0;

    std::vector<std::string> node_order_;
    std::map<std::string, const JsonValue*> nodes_;
    std::map<std::string, std::string> node_types_;
    std::set<std::string> exported_nodes_;
    std::vector<std::pair<std::string, std::string>> output_bindings_;
    std::set<std::string> live_;

    std::map<std::string, PlanResource> resources_;
    std::vector<PlanPass> render_passes_;
    std::vector<PlanPass> passes_;
    uint64_t gpu_bytes_planned_ = 0;
    uint64_t upload_bytes_planned_ = 0;
};

}  // namespace

PlanResult planScene(const ValidationResult& validated, DiagnosticList& diagnostics) {
    if (!validated.ok) {
        diagnostics.add("compile.invalid_input", Severity::kError, Phase::kCompile, Span{},
                        "cannot plan a scene that did not validate");
        return PlanResult{};
    }
    ScenePlanner planner(validated, diagnostics);
    return planner.run();
}

}  // namespace fluent_scene
