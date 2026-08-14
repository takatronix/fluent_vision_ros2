#pragma once

/// \file binding.hpp
/// \brief The deployment-side binding document (`fluent.binding/v1alpha1`).
///
/// A Scene declares **what** it needs (`inputs: camera: image.rgba8`); a
/// binding document declares **where that comes from on this robot** — the
/// ROS 2 topic, message type, QoS, and converter per input, and where the
/// rendered Surface goes. The two are separate files on purpose
/// (architecture doc §12): the scene stays portable and never names a
/// topic, and the binding carries deployment privilege.
///
/// ```yaml
/// schema: fluent.binding/v1alpha1
/// kind: Binding
/// metadata: { name: hud_ros2 }
/// scene: { name: camera_detection_hud }
/// bindings:
///   camera:
///     source: { adapter: ros2, topic: /camera/color/image_raw,
///               message_type: sensor_msgs/msg/Image, qos: sensor_data }
///     converter: ros_image_to_rgba8
/// outputs:
///   frame:
///     sink: { adapter: ros2, topic: /visualization/composite,
///             message_type: sensor_msgs/msg/Image, qos: sensor_data }
///     converter: rgba8_to_ros_image
/// ```
///
/// `parseBinding` validates the document shape and the converter table;
/// `validateBindingAgainstScene` checks it against a validated SceneDoc
/// (every binding names a declared input of the converter's type; unbound
/// scene inputs get an information diagnostic — they will show their
/// fallback). Adapter validation fails **before** activation, never after.

#include <string>
#include <vector>

#include "fluent_stage/scene/diagnostics.hpp"
#include "fluent_stage/scene/document.hpp"

namespace fluent_stage::scene {

/// The schema identifier this implementation accepts.
inline constexpr const char* kBindingSchemaId = "fluent.binding/v1alpha1";

/// What a converter turns a ROS message into (input side) or consumes
/// (output side).
struct ConverterSpec {
    const char* name;          ///< Public converter name.
    const char* message_type;  ///< The ROS 2 message type it expects/emits.
    InputType produces;        ///< Input converters: the Scene-side type.
    bool is_output;            ///< True for Surface→message converters.
    const char* summary;
};

/// The converter catalog — the single machine-readable list of what this
/// build can adapt (extends describe --json).
const std::vector<ConverterSpec>& converterTable();

/// Looks up a converter by name; nullptr when unknown.
const ConverterSpec* findConverter(const std::string& name);

/// Transport endpoint description (source or sink).
struct Endpoint {
    std::string adapter;       ///< Only "ros2" in this build.
    std::string topic;
    std::string message_type;  ///< Must match the converter's expectation.
    std::string qos = "sensor_data";  ///< sensor_data | default | transient_local.
};

/// One input binding: scene input name ← endpoint via converter.
struct BindingDecl {
    std::string input;
    Endpoint source;
    const ConverterSpec* converter = nullptr;
    Span span;
};

/// One output: a rendered Surface → endpoint via converter.
struct OutputDecl {
    std::string name;
    Endpoint sink;
    const ConverterSpec* converter = nullptr;
    Span span;
};

/// A validated binding document.
struct BindingDoc {
    std::string schema;      ///< Always kBindingSchemaId after validation.
    std::string name;        ///< metadata.name (may be empty).
    std::string scene_name;  ///< scene.name (matched informationally).
    std::vector<BindingDecl> bindings;  ///< Sorted by input name.
    std::vector<OutputDecl> outputs;    ///< Sorted by name.

    /// The binding for a scene input, or nullptr.
    const BindingDecl* findBinding(const std::string& input) const;
};

/// The outcome of `parseBinding`.
struct BindingParseResult {
    BindingDoc doc;
    DiagnosticList diagnostics;

    /// True when the document validated without errors.
    bool ok() const { return !diagnostics.hasErrors(); }
};

/// Parses and validates a binding document in isolation (shape, adapters,
/// converters, message types, QoS words).
BindingParseResult parseBinding(const std::string& yaml_text);

/// Checks a binding document against a validated scene: unknown inputs and
/// type mismatches are errors; scene inputs without a binding get an info
/// diagnostic (they will present their declared fallback).
void validateBindingAgainstScene(const BindingDoc& binding, const SceneDoc& scene,
                                 DiagnosticList& diagnostics);

}  // namespace fluent_stage::scene
