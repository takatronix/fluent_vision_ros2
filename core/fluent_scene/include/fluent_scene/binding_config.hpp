#pragma once

// Stage 4 (transport-neutral half): parses and validates a separate binding
// document (`fluent.binding/v1alpha1`, spec section 12) against a validated
// scene. The core never interprets ROS message types or QoS beyond carrying
// their names; adapter validation semantics (converter existence, type
// compatibility, synchronization configuration) live here so they are
// testable without ROS 2.

#include <memory>
#include <string>
#include <vector>

#include "fluent_scene/binding.hpp"
#include "fluent_scene/diagnostics.hpp"
#include "fluent_scene/validator.hpp"
#include "fluent_scene/yaml.hpp"

namespace fluent_scene {

// The binding-document identity supported by this parser.
extern const char kSupportedBindingIdentity[];

struct TopicBinding {
    std::string input;         // declared scene input
    std::string adapter;       // e.g. "ros2"
    std::string topic;
    std::string message_type;  // carried verbatim for the adapter
    std::string qos;           // sensor_data | transient_local | default
    std::string converter;     // registered converter name
    TypedValue::Kind kind = TypedValue::Kind::kString;  // resolved input kind
};

struct OutputSink {
    std::string output;  // declared scene output
    std::string adapter;
    std::string topic;
    std::string message_type;
    std::string qos;
    std::string converter;
};

struct BindingDocument {
    std::string name;
    std::string scene_name;
    std::vector<TopicBinding> bindings;  // sorted by input name
    std::vector<OutputSink> outputs;     // sorted by output name
    BindingTableOptions table_options;   // sync tolerance and queue capacities
};

// Registered converter names and the input kind each produces. A converter
// that is absent from this registry fails validation before activation
// (spec section 12).
bool converterProduces(const std::string& converter, TypedValue::Kind& out_kind);

// Parses `root` (an already-parsed YAML document) as a binding document and
// validates it against `scene`. Returns nullptr and bind-phase diagnostics on
// contract violations.
std::unique_ptr<BindingDocument> parseBindingDocument(const YamlNode& root,
                                                      const ValidationResult& scene,
                                                      DiagnosticList& diagnostics);

}  // namespace fluent_scene
