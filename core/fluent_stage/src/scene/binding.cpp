// binding.cpp — the fluent.binding/v1alpha1 document: parse, the converter
// catalog, and validation against a scene. Adapter validation happens here,
// before anything subscribes or publishes (architecture doc §12: "アダプター
// 検証は activation 前に失敗しなければならない").

#include <algorithm>

#include "fluent_stage/scene/binding.hpp"
#include "fluent_stage/scene/yaml.hpp"

namespace fluent_stage::scene {

const std::vector<ConverterSpec>& converterTable() {
    static const std::vector<ConverterSpec> kTable = {
        {"ros_image_to_rgba8", "sensor_msgs/msg/Image", InputType::ImageRgba8, false,
         "sensor_msgs Image (rgb8/bgr8/rgba8/bgra8/mono8) to image.rgba8"},
        {"ros_compressed_to_rgba8", "sensor_msgs/msg/CompressedImage", InputType::ImageRgba8,
         false, "JPEG/PNG CompressedImage to image.rgba8"},
        {"ros_detections_to_detection2d", "vision_msgs/msg/Detection2DArray",
         InputType::SeqDetection2D, false,
         "Detection2DArray to sequence<detection2d> (center/size to logical rect)"},
        {"ros_string_to_utf8", "std_msgs/msg/String", InputType::TextUtf8, false,
         "String to text.utf8"},
        {"aspa_json_to_detection2d", "std_msgs/msg/String", InputType::SeqDetection2D, false,
         "aspa perception JSON (class/conf/box_xyxy in image pixels, scaled by "
         "image_size to the stage canvas) to sequence<detection2d>"},
        {"ros_polygon_to_vec2", "geometry_msgs/msg/Polygon", InputType::SeqVec2, false,
         "Polygon points (x, y) to sequence<vec2> in logical units"},
        {"rgba8_to_ros_image", "sensor_msgs/msg/Image", InputType::ImageRgba8, true,
         "rendered Surface to sensor_msgs Image (rgba8)"},
        {"rgba8_to_ros_compressed", "sensor_msgs/msg/CompressedImage", InputType::ImageRgba8,
         true, "rendered Surface to JPEG CompressedImage"},
    };
    return kTable;
}

const ConverterSpec* findConverter(const std::string& name) {
    for (const ConverterSpec& c : converterTable()) {
        if (name == c.name) {
            return &c;
        }
    }
    return nullptr;
}

const BindingDecl* BindingDoc::findBinding(const std::string& input) const {
    for (const BindingDecl& b : bindings) {
        if (b.input == input) {
            return &b;
        }
    }
    return nullptr;
}

namespace {

bool isQosWord(const std::string& word) {
    return word == "sensor_data" || word == "default" || word == "transient_local";
}

class Parser {
public:
    explicit Parser(DiagnosticList& diags) : d_(diags) {}

    BindingDoc parse(const YamlNode& root) {
        BindingDoc doc;
        if (!root.isMapping()) {
            error("validate.structure", root.span, "the document root must be a mapping");
            return doc;
        }
        for (const YamlMapEntry& e : root.entries) {
            if (e.key == "schema") {
                if (!e.value.isScalar() || e.value.scalar != kBindingSchemaId) {
                    error("validate.schema", e.value.span,
                          std::string("unsupported schema (this build accepts '") +
                              kBindingSchemaId + "')");
                } else {
                    doc.schema = e.value.scalar;
                }
            } else if (e.key == "kind") {
                if (!e.value.isScalar() || e.value.scalar != "Binding") {
                    error("validate.type", e.value.span, "kind must be Binding");
                }
            } else if (e.key == "metadata") {
                if (e.value.isMapping()) {
                    if (const YamlNode* n = e.value.find("name")) {
                        if (n->isScalar()) {
                            doc.name = n->scalar;
                        }
                    }
                }
            } else if (e.key == "scene") {
                if (e.value.isMapping()) {
                    if (const YamlNode* n = e.value.find("name")) {
                        if (n->isScalar()) {
                            doc.scene_name = n->scalar;
                        }
                    }
                }
            } else if (e.key == "bindings") {
                parseBindings(e.value, doc);
            } else if (e.key == "outputs") {
                parseOutputs(e.value, doc);
            } else if (e.key == "synchronization") {
                d_.add("bind.not_implemented", Severity::kInfo, Phase::kBind, e.key_span,
                       "synchronization groups are declared but not implemented yet; "
                       "inputs snapshot latest-wins");
            } else {
                error("validate.unknown_key", e.key_span,
                      "unknown top-level key '" + e.key +
                          "' (expected schema, kind, metadata, scene, bindings, "
                          "synchronization, outputs)");
            }
        }
        if (doc.schema.empty()) {
            error("validate.required", root.span, "missing required key 'schema'");
        }
        if (doc.bindings.empty()) {
            error("validate.required", root.span, "a binding document needs 'bindings'");
        }
        std::sort(doc.bindings.begin(), doc.bindings.end(),
                  [](const BindingDecl& a, const BindingDecl& b) { return a.input < b.input; });
        std::sort(doc.outputs.begin(), doc.outputs.end(),
                  [](const OutputDecl& a, const OutputDecl& b) { return a.name < b.name; });
        return doc;
    }

private:
    void error(std::string code, Span span, std::string message) {
        d_.add(std::move(code), Severity::kError, Phase::kBind, span, std::move(message));
    }

    /// Parses a source/sink endpoint mapping. Returns false on failure.
    bool parseEndpoint(const YamlNode& n, const char* what, Endpoint& out) {
        if (!n.isMapping()) {
            error("validate.type", n.span,
                  std::string(what) + " must be a mapping { adapter, topic, message_type, qos }");
            return false;
        }
        bool ok = true;
        for (const YamlMapEntry& e : n.entries) {
            if (e.key == "adapter") {
                if (!e.value.isScalar() || e.value.scalar != "ros2") {
                    error("bind.adapter", e.value.span,
                          "unknown adapter (this build provides: ros2)");
                    ok = false;
                } else {
                    out.adapter = e.value.scalar;
                }
            } else if (e.key == "topic") {
                if (!e.value.isScalar() || e.value.scalar.empty() ||
                    e.value.scalar[0] != '/') {
                    error("bind.topic", e.value.span,
                          "topic must be an absolute ROS name (starting with /)");
                    ok = false;
                } else {
                    out.topic = e.value.scalar;
                }
            } else if (e.key == "message_type") {
                if (!e.value.isScalar()) {
                    error("validate.type", e.value.span, "message_type must be a string");
                    ok = false;
                } else {
                    out.message_type = e.value.scalar;
                }
            } else if (e.key == "qos") {
                if (!e.value.isScalar() || !isQosWord(e.value.scalar)) {
                    error("bind.qos", e.value.span,
                          "qos must be sensor_data | default | transient_local");
                    ok = false;
                } else {
                    out.qos = e.value.scalar;
                }
            } else {
                error("validate.unknown_key", e.key_span,
                      std::string("unknown ") + what + " key '" + e.key + "'");
                ok = false;
            }
        }
        if (out.adapter.empty()) {
            error("validate.required", n.span, std::string(what) + " is missing 'adapter'");
            ok = false;
        }
        if (out.topic.empty()) {
            error("validate.required", n.span, std::string(what) + " is missing 'topic'");
            ok = false;
        }
        return ok;
    }

    /// Shared shape of bindings and outputs entries: endpoint + converter.
    /// `metadata` blocks are accepted on bindings (declared contracts like
    /// timestamp paths) and not interpreted in this build.
    void parseBindings(const YamlNode& n, BindingDoc& doc) {
        if (!n.isMapping()) {
            error("validate.type", n.span, "bindings must be a mapping of input → binding");
            return;
        }
        for (const YamlMapEntry& e : n.entries) {
            BindingDecl decl;
            decl.input = e.key;
            decl.span = e.value.span;
            if (!e.value.isMapping()) {
                error("validate.type", e.value.span,
                      "a binding must be a mapping { source, converter }");
                continue;
            }
            bool ok = true;
            for (const YamlMapEntry& f : e.value.entries) {
                if (f.key == "source") {
                    ok = parseEndpoint(f.value, "source", decl.source) && ok;
                } else if (f.key == "converter") {
                    if (!f.value.isScalar() ||
                        (decl.converter = findConverter(f.value.scalar)) == nullptr ||
                        decl.converter->is_output) {
                        error("bind.converter", f.value.span,
                              "unknown input converter (see converterTable in describe)");
                        decl.converter = nullptr;
                        ok = false;
                    }
                } else if (f.key == "metadata") {
                    // Declared metadata contract; carried, not interpreted yet.
                } else {
                    error("validate.unknown_key", f.key_span,
                          "unknown binding key '" + f.key +
                              "' (expected source, converter, metadata)");
                    ok = false;
                }
            }
            if (decl.converter == nullptr) {
                if (ok) {
                    error("validate.required", e.value.span,
                          "binding '" + e.key + "' is missing 'converter'");
                }
                continue;
            }
            if (!decl.source.message_type.empty() &&
                decl.source.message_type != decl.converter->message_type) {
                error("bind.message_type", decl.span,
                      "converter " + std::string(decl.converter->name) + " expects " +
                          decl.converter->message_type + " but the source declares " +
                          decl.source.message_type);
                continue;
            }
            if (ok) {
                doc.bindings.push_back(std::move(decl));
            }
        }
    }

    void parseOutputs(const YamlNode& n, BindingDoc& doc) {
        if (!n.isMapping()) {
            error("validate.type", n.span, "outputs must be a mapping of name → output");
            return;
        }
        for (const YamlMapEntry& e : n.entries) {
            OutputDecl decl;
            decl.name = e.key;
            decl.span = e.value.span;
            if (!e.value.isMapping()) {
                error("validate.type", e.value.span,
                      "an output must be a mapping { sink, converter }");
                continue;
            }
            bool ok = true;
            for (const YamlMapEntry& f : e.value.entries) {
                if (f.key == "sink") {
                    ok = parseEndpoint(f.value, "sink", decl.sink) && ok;
                } else if (f.key == "converter") {
                    if (!f.value.isScalar() ||
                        (decl.converter = findConverter(f.value.scalar)) == nullptr ||
                        !decl.converter->is_output) {
                        error("bind.converter", f.value.span, "unknown output converter");
                        decl.converter = nullptr;
                        ok = false;
                    }
                } else {
                    error("validate.unknown_key", f.key_span,
                          "unknown output key '" + f.key + "' (expected sink, converter)");
                    ok = false;
                }
            }
            if (decl.converter == nullptr) {
                if (ok) {
                    error("validate.required", e.value.span,
                          "output '" + e.key + "' is missing 'converter'");
                }
                continue;
            }
            if (!decl.sink.message_type.empty() &&
                decl.sink.message_type != decl.converter->message_type) {
                error("bind.message_type", decl.span,
                      "converter " + std::string(decl.converter->name) + " emits " +
                          decl.converter->message_type + " but the sink declares " +
                          decl.sink.message_type);
                continue;
            }
            if (ok) {
                doc.outputs.push_back(std::move(decl));
            }
        }
    }

    DiagnosticList& d_;
};

}  // namespace

BindingParseResult parseBinding(const std::string& yaml_text) {
    BindingParseResult result;
    const YamlNode root = parseYaml(yaml_text, result.diagnostics);
    if (!result.diagnostics.hasErrors()) {
        Parser parser(result.diagnostics);
        result.doc = parser.parse(root);
    }
    result.diagnostics.sortCanonical();
    return result;
}

void validateBindingAgainstScene(const BindingDoc& binding, const SceneDoc& scene,
                                 DiagnosticList& diagnostics) {
    for (const BindingDecl& b : binding.bindings) {
        const InputDecl* input = scene.findInput(b.input);
        if (input == nullptr) {
            diagnostics.add("bind.unknown_input", Severity::kError, Phase::kBind, b.span,
                            "binding names input '" + b.input +
                                "' which the scene does not declare");
            continue;
        }
        if (b.converter != nullptr && b.converter->produces != input->type) {
            diagnostics.add(
                "bind.type_mismatch", Severity::kError, Phase::kBind, b.span,
                "converter " + std::string(b.converter->name) + " produces " +
                    inputTypeName(b.converter->produces, input->capacity) + " but input '" +
                    b.input + "' declares " + inputTypeName(input->type, input->capacity));
        }
    }
    for (const InputDecl& input : scene.inputs) {
        if (binding.findBinding(input.name) == nullptr) {
            diagnostics.add("bind.unbound_input", Severity::kInfo, Phase::kBind, Span{},
                            "scene input '" + input.name +
                                "' has no binding; it will present its fallback");
        }
    }
    diagnostics.sortCanonical();
}

}  // namespace fluent_stage::scene
