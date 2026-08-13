#include "fluent_scene/binding_config.hpp"

#include <algorithm>
#include <map>

namespace fluent_scene {

const char kSupportedBindingIdentity[] = "fluent.binding/v1alpha1";

namespace {

void err(DiagnosticList& diagnostics, std::string code, Span span, std::string message,
         std::vector<std::pair<std::string, std::string>> context = {}) {
    diagnostics.add(std::move(code), Severity::kError, Phase::kBind, span, std::move(message),
                    std::move(context));
}

const JsonValue* findIrEntry(const JsonValue& ir, const char* section, const std::string& name) {
    const JsonValue* array = ir.find(section);
    if (array == nullptr || !array->isArray()) {
        return nullptr;
    }
    for (const JsonValue& element : array->elements()) {
        const JsonValue* entry_name = element.find("name");
        if (entry_name != nullptr && entry_name->stringValue() == name) {
            return &element;
        }
    }
    return nullptr;
}

TypedValue::Kind inputKind(const JsonValue& input) {
    const std::string type_name = input.find("type")->stringValue();
    if (type_name.rfind("image.", 0) == 0) {
        return TypedValue::Kind::kImage;
    }
    if (type_name.rfind("sequence<", 0) == 0) {
        return TypedValue::Kind::kDetections;
    }
    if (type_name == "calibration") {
        return TypedValue::Kind::kCalibration;
    }
    return TypedValue::Kind::kString;
}

std::string scalarOrEmpty(const YamlNode* node) {
    return node != nullptr && node->isScalar() ? node->scalar : std::string{};
}

}  // namespace

bool converterProduces(const std::string& converter, TypedValue::Kind& out_kind) {
    static const std::map<std::string, TypedValue::Kind> kRegistry = {
        {"ros_image_to_rgba8", TypedValue::Kind::kImage},
        {"ros_depth_to_depth32f_meters", TypedValue::Kind::kImage},
        {"ros_detections_to_Detection2D", TypedValue::Kind::kDetections},
        {"ros_string_to_utf8", TypedValue::Kind::kString},
        {"ros_camera_info_to_calibration", TypedValue::Kind::kCalibration},
    };
    auto it = kRegistry.find(converter);
    if (it == kRegistry.end()) {
        return false;
    }
    out_kind = it->second;
    return true;
}

std::unique_ptr<BindingDocument> parseBindingDocument(const YamlNode& root,
                                                      const ValidationResult& scene,
                                                      DiagnosticList& diagnostics) {
    if (!scene.ok) {
        err(diagnostics, "bind.invalid_input", Span{}, "cannot bind a scene that did not validate");
        return nullptr;
    }
    if (!root.isMapping()) {
        err(diagnostics, "bind.invalid_input", root.span, "a binding document must be a mapping");
        return nullptr;
    }
    const YamlNode* schema = root.find("schema");
    if (schema == nullptr || !schema->isPlainScalar() || schema->scalar != kSupportedBindingIdentity) {
        err(diagnostics, "bind.unsupported_schema", schema != nullptr ? schema->span : root.span,
            std::string("binding documents must declare schema ") + kSupportedBindingIdentity);
        return nullptr;
    }
    const YamlNode* kind = root.find("kind");
    if (kind == nullptr || !kind->isPlainScalar() || kind->scalar != "Binding") {
        err(diagnostics, "bind.unsupported_schema", kind != nullptr ? kind->span : root.span,
            "binding documents must declare kind: Binding");
        return nullptr;
    }

    auto document = std::make_unique<BindingDocument>();
    if (const YamlNode* metadata = root.find("metadata")) {
        document->name = scalarOrEmpty(metadata->find("name"));
    }
    const std::string scene_name = [&]() {
        const JsonValue* metadata = scene.ir.find("metadata");
        const JsonValue* name = metadata != nullptr ? metadata->find("name") : nullptr;
        return name != nullptr ? name->stringValue() : std::string{};
    }();
    const YamlNode* scene_ref = root.find("scene");
    const std::string bound_scene =
        scene_ref != nullptr && scene_ref->isMapping() ? scalarOrEmpty(scene_ref->find("name")) : "";
    if (bound_scene != scene_name) {
        err(diagnostics, "bind.scene_mismatch", scene_ref != nullptr ? scene_ref->span : root.span,
            "binding document targets scene \"" + bound_scene + "\" but the loaded scene is \"" +
                scene_name + "\"",
            {{"bound", bound_scene}, {"scene", scene_name}});
        return nullptr;
    }
    document->scene_name = scene_name;

    bool ok = true;

    // ---- bindings ----------------------------------------------------------
    const YamlNode* bindings = root.find("bindings");
    if (bindings != nullptr && bindings->isMapping()) {
        for (const auto& entry : bindings->entries) {
            const JsonValue* input = findIrEntry(scene.ir, "inputs", entry.key);
            if (input == nullptr) {
                err(diagnostics, "bind.unknown_input", entry.key_span,
                    "binding targets undeclared scene input \"" + entry.key + "\"",
                    {{"input", entry.key}});
                ok = false;
                continue;
            }
            if (!entry.value.isMapping()) {
                err(diagnostics, "bind.invalid_input", entry.value.span,
                    "binding for \"" + entry.key + "\" must be a mapping");
                ok = false;
                continue;
            }
            TopicBinding binding;
            binding.input = entry.key;
            binding.kind = inputKind(*input);
            const YamlNode* source = entry.value.find("source");
            if (source == nullptr || !source->isMapping()) {
                err(diagnostics, "bind.invalid_input", entry.value.span,
                    "binding for \"" + entry.key + "\" requires a source mapping");
                ok = false;
                continue;
            }
            binding.adapter = scalarOrEmpty(source->find("adapter"));
            binding.topic = scalarOrEmpty(source->find("topic"));
            binding.message_type = scalarOrEmpty(source->find("message_type"));
            binding.qos = scalarOrEmpty(source->find("qos"));
            if (binding.adapter != "ros2") {
                err(diagnostics, "bind.unsupported_adapter", source->span,
                    "unsupported adapter \"" + binding.adapter + "\" (supported: ros2)");
                ok = false;
            }
            if (binding.topic.empty()) {
                err(diagnostics, "bind.invalid_input", source->span,
                    "binding for \"" + entry.key + "\" requires a topic");
                ok = false;
            }
            binding.converter = scalarOrEmpty(entry.value.find("converter"));
            TypedValue::Kind produced = TypedValue::Kind::kString;
            if (!converterProduces(binding.converter, produced)) {
                err(diagnostics, "bind.unknown_converter", entry.value.span,
                    "converter \"" + binding.converter + "\" is not registered (input \"" +
                        entry.key + "\")",
                    {{"converter", binding.converter}});
                ok = false;
            } else if (produced != binding.kind) {
                err(diagnostics, "bind.converter_mismatch", entry.value.span,
                    "converter \"" + binding.converter +
                        "\" does not produce the declared type of input \"" + entry.key + "\"",
                    {{"converter", binding.converter}, {"input", entry.key}});
                ok = false;
            }
            document->bindings.push_back(std::move(binding));
        }
    }
    std::sort(document->bindings.begin(), document->bindings.end(),
              [](const TopicBinding& a, const TopicBinding& b) { return a.input < b.input; });

    // Required scene inputs without any binding still work through declared
    // fallbacks; surface them as informational diagnostics.
    if (const JsonValue* inputs = scene.ir.find("inputs")) {
        for (const JsonValue& input : inputs->elements()) {
            const std::string name = input.find("name")->stringValue();
            const bool bound =
                std::any_of(document->bindings.begin(), document->bindings.end(),
                            [&](const TopicBinding& binding) { return binding.input == name; });
            if (!bound && input.find("required")->boolValue()) {
                diagnostics.add("bind.unbound_input", Severity::kInfo, Phase::kBind, Span{},
                                "required input \"" + name +
                                    "\" has no binding; its declared fallback will serve every frame",
                                {{"input", name}});
            }
        }
    }

    // ---- synchronization ----------------------------------------------------
    if (const YamlNode* synchronization = root.find("synchronization")) {
        if (synchronization->isMapping()) {
            for (const auto& group : synchronization->entries) {
                if (!group.value.isMapping()) {
                    continue;
                }
                SyncGroupConfig config;
                const std::string policy = scalarOrEmpty(group.value.find("policy"));
                if (!policy.empty() && policy != "approximate") {
                    err(diagnostics, "bind.unsupported_policy", group.value.span,
                        "unsupported synchronization policy \"" + policy + "\" (supported: approximate)");
                    ok = false;
                }
                const std::string tolerance = scalarOrEmpty(group.value.find("tolerance_ms"));
                if (!tolerance.empty()) {
                    config.tolerance_seconds = std::atof(tolerance.c_str()) / 1000.0;
                }
                document->table_options.groups[group.key] = config;
                const std::string capacity = scalarOrEmpty(group.value.find("queue_capacity"));
                if (!capacity.empty()) {
                    const size_t queue_capacity =
                        static_cast<size_t>(std::max(1, std::atoi(capacity.c_str())));
                    // Apply the group's queue capacity to its member inputs.
                    if (const JsonValue* inputs = scene.ir.find("inputs")) {
                        for (const JsonValue& input : inputs->elements()) {
                            const JsonValue* metadata = input.find("metadata");
                            const JsonValue* input_group =
                                metadata != nullptr ? metadata->find("synchronization_group") : nullptr;
                            if (input_group != nullptr && input_group->stringValue() == group.key) {
                                InputRuntimeConfig runtime;
                                runtime.policy = QueuePolicy::kMatched;
                                runtime.capacity = queue_capacity;
                                document->table_options
                                    .inputs[input.find("name")->stringValue()] = runtime;
                            }
                        }
                    }
                }
            }
        }
    }

    // ---- outputs -------------------------------------------------------------
    if (const YamlNode* outputs = root.find("outputs")) {
        if (outputs->isMapping()) {
            for (const auto& entry : outputs->entries) {
                if (findIrEntry(scene.ir, "outputs", entry.key) == nullptr) {
                    err(diagnostics, "bind.unknown_output", entry.key_span,
                        "sink targets undeclared scene output \"" + entry.key + "\"",
                        {{"output", entry.key}});
                    ok = false;
                    continue;
                }
                if (!entry.value.isMapping()) {
                    continue;
                }
                OutputSink sink;
                sink.output = entry.key;
                if (const YamlNode* sink_node = entry.value.find("sink")) {
                    if (sink_node->isMapping()) {
                        sink.adapter = scalarOrEmpty(sink_node->find("adapter"));
                        sink.topic = scalarOrEmpty(sink_node->find("topic"));
                        sink.message_type = scalarOrEmpty(sink_node->find("message_type"));
                        sink.qos = scalarOrEmpty(sink_node->find("qos"));
                    }
                }
                sink.converter = scalarOrEmpty(entry.value.find("converter"));
                document->outputs.push_back(std::move(sink));
            }
        }
    }
    std::sort(document->outputs.begin(), document->outputs.end(),
              [](const OutputSink& a, const OutputSink& b) { return a.output < b.output; });

    if (!ok) {
        return nullptr;
    }
    return document;
}

}  // namespace fluent_scene
