#include "fluent_scene/registry.hpp"

namespace fluent_scene {

const PortSpec* NodeTypeSpec::findInput(const std::string& port) const {
    for (const auto& spec : inputs) {
        if (spec.name == port) {
            return &spec;
        }
    }
    return nullptr;
}

const ParamSpec* NodeTypeSpec::findParam(const std::string& param) const {
    for (const auto& spec : params) {
        if (spec.name == param) {
            return &spec;
        }
    }
    return nullptr;
}

const OutputSpec* NodeTypeSpec::findOutput(const std::string& output) const {
    for (const auto& spec : outputs) {
        if (spec.name == output) {
            return &spec;
        }
    }
    return nullptr;
}

void NodeRegistry::add(NodeTypeSpec spec) { specs_[spec.name] = std::move(spec); }

const NodeTypeSpec* NodeRegistry::find(const std::string& name) const {
    auto it = specs_.find(name);
    return it == specs_.end() ? nullptr : &it->second;
}

namespace {

JsonValue vecDefault(std::initializer_list<double> values) {
    JsonValue array = JsonValue::makeArray();
    for (double v : values) {
        array.append(JsonValue::makeFloat(v));
    }
    return array;
}

NodeTypeSpec makeImage2d() {
    NodeTypeSpec spec;
    spec.name = "visual.image2d";
    spec.inputs.push_back(PortSpec{"image", PortPatternKind::kAnyImage, TypeKind::kImageRgba8, true});
    ParamSpec fit;
    fit.name = "fit";
    fit.kind = ParamKind::kEnum;
    fit.enum_values = {"contain", "cover", "fill"};
    fit.default_value = JsonValue::makeString("contain");
    spec.params.push_back(std::move(fit));
    spec.outputs.push_back(OutputSpec{"layer", TypeKind::kLayer});
    return spec;
}

NodeTypeSpec makeBoxes2d() {
    NodeTypeSpec spec;
    spec.name = "visual.boxes2d";
    spec.inputs.push_back(PortSpec{"detections", PortPatternKind::kDetectionSequence, TypeKind::kBoxes2d, true});
    ParamSpec color;
    color.name = "color";
    color.kind = ParamKind::kVec4f;
    color.default_value = vecDefault({1.0, 1.0, 1.0, 1.0});
    spec.params.push_back(std::move(color));
    ParamSpec show_label;
    show_label.name = "show_label";
    show_label.kind = ParamKind::kBool;
    show_label.default_value = JsonValue::makeBool(false);
    spec.params.push_back(std::move(show_label));
    ParamSpec smoothing;  // EMA time constant in seconds; 0 = boxes jump to new data
    smoothing.name = "smoothing";
    smoothing.kind = ParamKind::kF32;
    smoothing.default_value = JsonValue::makeFloat(0.0);
    spec.params.push_back(std::move(smoothing));
    spec.outputs.push_back(OutputSpec{"layer", TypeKind::kLayer});
    spec.bounds.required = true;
    spec.bounds.count_key = "max_instances";
    spec.bounds.max_count = 4096;
    spec.bounds.overflow_rules = {"drop_lowest_score", "drop_oldest"};
    return spec;
}

NodeTypeSpec makeTextDynamic() {
    NodeTypeSpec spec;
    spec.name = "text.dynamic";
    spec.inputs.push_back(PortSpec{"text", PortPatternKind::kExact, TypeKind::kString, true});
    ParamSpec font;
    font.name = "font";
    font.kind = ParamKind::kFontResource;
    font.required = true;
    spec.params.push_back(std::move(font));
    ParamSpec position;
    position.name = "position";
    position.kind = ParamKind::kVec2f;
    position.default_value = vecDefault({0.0, 0.0});
    spec.params.push_back(std::move(position));
    ParamSpec color;
    color.name = "color";
    color.kind = ParamKind::kVec4f;
    color.default_value = vecDefault({1.0, 1.0, 1.0, 1.0});
    spec.params.push_back(std::move(color));
    ParamSpec shadow;
    shadow.name = "shadow";
    shadow.kind = ParamKind::kBool;
    shadow.default_value = JsonValue::makeBool(false);
    spec.params.push_back(std::move(shadow));
    ParamSpec default_text;
    default_text.name = "default_text";
    default_text.kind = ParamKind::kString;
    default_text.default_value = JsonValue::makeString("");
    spec.params.push_back(std::move(default_text));
    spec.outputs.push_back(OutputSpec{"layer", TypeKind::kLayer});
    spec.bounds.required = true;
    spec.bounds.count_key = "max_glyphs";
    spec.bounds.max_count = 8192;
    spec.bounds.overflow_rules = {"truncate_end"};
    return spec;
}

NodeTypeSpec makeCircles2d() {
    NodeTypeSpec spec;
    spec.name = "visual.circles2d";
    spec.inputs.push_back(PortSpec{"points", PortPatternKind::kAnyPoints, TypeKind::kPoints2d, true});
    ParamSpec color;
    color.name = "color";
    color.kind = ParamKind::kVec4f;
    color.default_value = vecDefault({1.0, 1.0, 1.0, 1.0});
    spec.params.push_back(std::move(color));
    ParamSpec radius;
    radius.name = "radius";
    radius.kind = ParamKind::kF32;
    radius.default_value = JsonValue::makeFloat(8.0);
    spec.params.push_back(std::move(radius));
    ParamSpec thickness;  // 0 = filled disc, > 0 = ring outline
    thickness.name = "thickness";
    thickness.kind = ParamKind::kF32;
    thickness.default_value = JsonValue::makeFloat(0.0);
    spec.params.push_back(std::move(thickness));
    spec.outputs.push_back(OutputSpec{"layer", TypeKind::kLayer});
    spec.bounds.required = true;
    spec.bounds.count_key = "max_points";
    spec.bounds.max_count = 4096;
    spec.bounds.overflow_rules = {"truncate_end"};
    return spec;
}

NodeTypeSpec makePolyline2d() {
    NodeTypeSpec spec;
    spec.name = "visual.polyline2d";
    spec.inputs.push_back(
        PortSpec{"points", PortPatternKind::kAnyPoints, TypeKind::kPolyline2d, true});
    ParamSpec color;
    color.name = "color";
    color.kind = ParamKind::kVec4f;
    color.default_value = vecDefault({1.0, 1.0, 1.0, 1.0});
    spec.params.push_back(std::move(color));
    ParamSpec thickness;
    thickness.name = "thickness";
    thickness.kind = ParamKind::kF32;
    thickness.default_value = JsonValue::makeFloat(3.0);
    spec.params.push_back(std::move(thickness));
    spec.outputs.push_back(OutputSpec{"layer", TypeKind::kLayer});
    spec.bounds.required = true;
    spec.bounds.count_key = "max_points";
    spec.bounds.max_count = 4096;
    spec.bounds.overflow_rules = {"truncate_end"};
    return spec;
}

NodeTypeSpec makeBlur() {
    NodeTypeSpec spec;
    spec.name = "effects.blur";
    spec.inputs.push_back(PortSpec{"image", PortPatternKind::kAnyImage, TypeKind::kImageRgba8, true});
    ParamSpec radius;  // separable Gaussian; kernel statically bounded (13 taps),
    radius.name = "radius";  // radius clamped to 16 px by the backend
    radius.kind = ParamKind::kF32;
    radius.default_value = JsonValue::makeFloat(4.0);
    spec.params.push_back(std::move(radius));
    spec.outputs.push_back(OutputSpec{"image", TypeKind::kImageRgba8});
    return spec;
}

NodeTypeSpec makeColorTransform() {
    NodeTypeSpec spec;
    spec.name = "effects.color_transform";
    spec.inputs.push_back(PortSpec{"image", PortPatternKind::kAnyImage, TypeKind::kImageRgba8, true});
    ParamSpec brightness;
    brightness.name = "brightness";
    brightness.kind = ParamKind::kF32;
    brightness.default_value = JsonValue::makeFloat(0.0);
    spec.params.push_back(std::move(brightness));
    ParamSpec contrast;
    contrast.name = "contrast";
    contrast.kind = ParamKind::kF32;
    contrast.default_value = JsonValue::makeFloat(1.0);
    spec.params.push_back(std::move(contrast));
    ParamSpec saturation;
    saturation.name = "saturation";
    saturation.kind = ParamKind::kF32;
    saturation.default_value = JsonValue::makeFloat(1.0);
    spec.params.push_back(std::move(saturation));
    ParamSpec gamma;
    gamma.name = "gamma";
    gamma.kind = ParamKind::kF32;
    gamma.default_value = JsonValue::makeFloat(1.0);
    spec.params.push_back(std::move(gamma));
    spec.outputs.push_back(OutputSpec{"image", TypeKind::kImageRgba8});
    return spec;
}

NodeTypeSpec makeCompositeLayers() {
    NodeTypeSpec spec;
    spec.name = "composite.layers";
    spec.inputs.push_back(PortSpec{"layers", PortPatternKind::kLayerList, TypeKind::kLayer, true});
    ParamSpec color_space;
    color_space.name = "color_space";
    color_space.kind = ParamKind::kEnum;
    color_space.enum_values = {"srgb", "linear"};
    color_space.default_value = JsonValue::makeString("srgb");
    spec.params.push_back(std::move(color_space));
    spec.outputs.push_back(OutputSpec{"image", TypeKind::kImageRgba8});
    return spec;
}

}  // namespace

NodeRegistry NodeRegistry::builtinMvp() {
    NodeRegistry registry;
    registry.add(makeImage2d());
    registry.add(makeBoxes2d());
    registry.add(makeCircles2d());
    registry.add(makePolyline2d());
    registry.add(makeBlur());
    registry.add(makeColorTransform());
    registry.add(makeTextDynamic());
    registry.add(makeCompositeLayers());
    return registry;
}

}  // namespace fluent_scene
