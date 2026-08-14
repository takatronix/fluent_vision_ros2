// compiler.cpp — SceneDoc → Stage, through the public Layer API only.
//
// Every layer is created by the same drawing call a C++ author writes
// (stage.rect(...), stage.image(...)), and only explicitly-declared fields
// are applied afterwards — so every default lives once, in the C++ API, and
// the §2 "same picture from YAML and C++" contract holds by construction.

#include <algorithm>

#include "fluent_stage/scene/compiler.hpp"
#include "fluent_stage/transaction.hpp"

namespace fluent_stage::scene {

namespace {

Color placeholderInk(float alpha) { return {1, 1, 1, alpha}; }

}  // namespace

/// Grants compile() access to CompiledScene's private members without
/// widening its public API.
struct CompileAccess {
    static std::unique_ptr<CompiledScene> create() {
        return std::unique_ptr<CompiledScene>(new CompiledScene());
    }
    static SceneDoc& doc(CompiledScene& s) { return s.doc_; }
    static std::string& digestRef(CompiledScene& s) { return s.digest_; }
    static std::unique_ptr<Stage>& stage(CompiledScene& s) { return s.stage_; }
    static auto& nodes(CompiledScene& s) { return s.nodes_; }
    static auto& paramBindings(CompiledScene& s) { return s.param_bindings_; }
    static auto& inputBindings(CompiledScene& s) { return s.input_bindings_; }
    static auto& controlBindings(CompiledScene& s) { return s.control_bindings_; }
    static auto& layerTransitions(CompiledScene& s) { return s.layer_transitions_; }
    static auto& buttons(CompiledScene& s) { return s.buttons_; }
    static auto& switches(CompiledScene& s) { return s.switches_; }
    static auto& sliders(CompiledScene& s) { return s.sliders_; }
    static auto& gauges(CompiledScene& s) { return s.gauges_; }
    static void fire(CompiledScene& s, const UiEvent& event) {
        if (s.ui_handler_) {
            s.ui_handler_(event);
        }
    }
};

namespace {

/// The value an attribute starts at when bound to `$params.<name>`: the
/// param's declared default, typed like the attribute expects.
Value paramInitialValue(const ParamDecl& p, ValueKind kind) {
    Value v;
    v.kind = kind;
    v.num[0] = p.def[0];
    v.num[1] = p.def[1];
    v.num[2] = p.def[2];
    v.num[3] = p.def[3];
    v.flag = p.def_flag;
    return v;
}

/// Applies one attribute value through the Layer setters. Shared by the
/// build pass and setParam so both spell the mapping exactly once.
void applyAttr(Layer& layer, AttrId id, const Value& v) {
    switch (id) {
        case AttrId::Bounds: layer.bounds(v.rect()); break;
        case AttrId::Position: layer.position(v.vec2()); break;
        case AttrId::Anchor: layer.anchor(v.num[0], v.num[1]); break;
        case AttrId::Frame: layer.frame(v.rect()); break;
        case AttrId::Rotation: layer.rotation(v.num[0]); break;
        case AttrId::Scale: layer.scale(v.num[0], v.num[1]); break;
        case AttrId::Transform:
            layer.transform(Mat23{v.mat[0], v.mat[1], v.mat[2], v.mat[3], v.mat[4], v.mat[5]});
            break;
        case AttrId::Opacity: layer.opacity(v.num[0]); break;
        case AttrId::Hidden: layer.hidden(v.flag); break;
        case AttrId::MasksToBounds: layer.masksToBounds(v.flag); break;
        case AttrId::CornerRadius: layer.cornerRadius(v.num[0]); break;
        case AttrId::Background: layer.background(v.color()); break;
        case AttrId::Blend: layer.blend(static_cast<Blend>(v.enum_value)); break;
        case AttrId::Shadow:
        case AttrId::Border: break;  // map attributes, applied by applyMapAttr
    }
}

void applyMapAttr(Layer& layer, const AttrDecl& attr) {
    if (attr.spec->id == AttrId::Shadow) {
        Shadow s{};
        for (const MapField& f : attr.map) {
            const std::string name = f.spec->name;
            if (name == "offset") {
                s.offset = f.value.vec2();
            } else if (name == "radius") {
                s.radius = f.value.num[0];
            } else if (name == "color") {
                s.color = f.value.color();
            } else if (name == "opacity") {
                s.opacity = f.value.num[0];
            }
        }
        layer.shadow(s);
    } else if (attr.spec->id == AttrId::Border) {
        Border b{};
        for (const MapField& f : attr.map) {
            const std::string name = f.spec->name;
            if (name == "width") {
                b.width = f.value.num[0];
            } else if (name == "color") {
                b.color = f.value.color();
            }
        }
        layer.border(b);
    }
}

/// Applies a shared style field (color/thickness/…) via its Layer setter.
void applyStyle(Layer& layer, const MapField& f) {
    const std::string name = f.spec->name;
    if (name == "color") {
        layer.color(f.value.color());
    } else if (name == "thickness") {
        layer.thickness(f.value.num[0]);
    } else if (name == "dash") {
        layer.dash(f.value.num[0]);
    } else if (name == "cap") {
        layer.cap(static_cast<Cap>(f.value.enum_value));
    } else if (name == "corner_radius") {
        layer.cornerRadius(f.value.num[0]);
    } else if (name == "smoothing") {
        layer.smoothing(f.value.num[0]);
    } else if (name == "show_label") {
        layer.showLabel(f.value.flag);
    }
}

/// The statically-resolvable size of a declared layer, for placeholder
/// layout: declared frame/bounds win; image and grid content fill the
/// parent; everything else does not matter for placeholders.
Vec2 staticSize(const LayerDecl& decl, Vec2 parent_size) {
    if (const AttrDecl* a = decl.find(AttrId::Bounds)) {
        return {a->value.num[2], a->value.num[3]};
    }
    if (const AttrDecl* a = decl.find(AttrId::Frame)) {
        return {a->value.num[2], a->value.num[3]};
    }
    return parent_size;
}

class Builder {
public:
    Builder(CompiledScene& scene, const SceneDoc& doc, const CompileOptions& options,
            DiagnosticList& diags)
        : scene_(scene), doc_(doc), options_(options), diags_(diags) {}

    bool build() {
        // Bound resource check, up front (§15-4: never throw mid-build).
        // Each unfed image input may add up to 2 placeholder layers.
        uint32_t placeholder_budget = 0;
        for (const InputDecl& in : doc_.inputs) {
            if (in.type == InputType::ImageRgba8 &&
                in.fallback == InputFallback::Placeholder) {
                placeholder_budget += 2;
            }
        }
        if (doc_.layer_count + placeholder_budget + 1 > options_.limits.max_layers) {
            error("compile.budget",
                  "the document declares " + std::to_string(doc_.layer_count) +
                      " layers (+" + std::to_string(placeholder_budget) +
                      " placeholders), over the limit of " +
                      std::to_string(options_.limits.max_layers));
            return false;
        }
        if (doc_.max_depth + 2 > options_.limits.max_depth) {
            error("compile.budget",
                  "the document nests " + std::to_string(doc_.max_depth) +
                      " layers deep, over the limit of " +
                      std::to_string(options_.limits.max_depth));
            return false;
        }
        for (const InputDecl& in : doc_.inputs) {
            const uint32_t cap = in.type == InputType::SeqVec2 ? options_.limits.max_points
                                 : in.type == InputType::SeqDetection2D
                                     ? options_.limits.max_boxes
                                     : 0;
            if (cap != 0 && in.capacity > cap) {
                error("compile.budget",
                      "input '" + in.name + "' declares capacity " +
                          std::to_string(in.capacity) + ", over the limit of " +
                          std::to_string(cap));
                return false;
            }
        }

        auto stage = std::make_unique<Stage>(doc_.width, doc_.height, options_.limits);
        stage->fit(doc_.fit);
        CompileAccess::stage(scene_) = std::move(stage);
        for (const LayerDecl& decl : doc_.layers) {
            buildLayer(CompileAccess::stage(scene_)->root(), decl,
                       {doc_.width, doc_.height});
        }
        return !diags_.hasErrors();
    }

private:
    void error(std::string code, std::string message) {
        diags_.add(std::move(code), Severity::kError, Phase::kCompile, Span{},
                   std::move(message));
    }

    void buildLayer(Layer& parent, const LayerDecl& decl, Vec2 parent_size) {
        Layer* layer = createLayer(parent, decl);
        if (layer == nullptr) {
            return;
        }
        CompileAccess::nodes(scene_).emplace_back(&decl, layer);
        if (!decl.id.empty()) {
            layer->id(decl.id);
        }

        // Content style fields (explicit only — creation defaults stand).
        if (decl.content) {
            for (const MapField& f : decl.content->fields) {
                if (f.spec->is_style) {
                    applyStyle(*layer, f);
                }
            }
        }

        // Attributes (table order; $params refs start at the param default).
        for (const AttrDecl& attr : decl.attrs) {
            const ValueKind kind = attr.spec->kind;
            if (kind == ValueKind::MapShadow || kind == ValueKind::MapBorder) {
                applyMapAttr(*layer, attr);
            } else if (!attr.value.param.empty()) {
                const ParamDecl* p = doc_.findParam(attr.value.param);
                applyAttr(*layer, attr.spec->id, paramInitialValue(*p, kind));
                CompileAccess::paramBindings(scene_)[p->name].push_back(
                    {layer, attr.spec->id});
            } else {
                applyAttr(*layer, attr.spec->id, attr.value);
            }
        }

        for (const FilterDecl& f : decl.filters) {
            layer->filter(f.value);
        }
        if (decl.transition) {
            CompileAccess::layerTransitions(scene_)[layer] = *decl.transition;
        }

        const Vec2 own_size = staticSize(decl, parent_size);
        for (const LayerDecl& sub : decl.sublayers) {
            buildLayer(*layer, sub, own_size);
        }
    }

    /// Creates the layer through the same drawing call a C++ author uses.
    Layer* createLayer(Layer& parent, const LayerDecl& decl) {
        if (!decl.content) {
            return &parent.group();
        }
        const ContentDecl& c = *decl.content;
        const std::string name = c.spec->name;
        auto num = [&](const char* field, float fallback) {
            const MapField* f = c.find(field);
            return f != nullptr ? f->value.num[0] : fallback;
        };
        auto vec = [&](const char* field) {
            const MapField* f = c.find(field);
            return f != nullptr ? f->value.vec2() : Vec2{};
        };

        // UI controls (§10, L4): the compiler instantiates the same ui::
        // prefabs a C++ author uses. The layer's declared frame places them;
        // gestures leave through CompiledScene::onUiEvent.
        if (name == "button" || name == "switch" || name == "slider" || name == "gauge") {
            return createControl(parent, decl, c);
        }

        Layer* layer = nullptr;
        if (name == "image") {
            layer = &parent.image(ImageView{});
            if (const MapField* f = c.find("fit")) {
                layer->fit(static_cast<Fit>(f->value.enum_value));
            }
            if (const MapField* f = c.find("source_rect")) {
                layer->sourceRect(f->value.rect());
            }
        } else if (name == "text") {
            const MapField* literal = c.find("text");
            layer = &parent.text(literal != nullptr ? literal->value.str : std::string{},
                                 vec("position"));
            if (const MapField* f = c.find("size")) {
                layer->size(f->value.num[0]);
            }
            if (const MapField* f = c.find("align")) {
                layer->align(static_cast<Align>(f->value.enum_value));
            }
        } else if (name == "line") {
            layer = &parent.line(vec("from"), vec("to"));
        } else if (name == "polyline" || name == "polygon" || name == "circles") {
            const MapField* pts = c.find("points");
            static const std::vector<Vec2> kEmpty;
            const std::vector<Vec2>& points = pts != nullptr ? pts->value.points : kEmpty;
            if (name == "polyline") {
                layer = &parent.polyline(points);
            } else if (name == "polygon") {
                layer = &parent.polygon(points);
            } else if (const MapField* r = c.find("radius")) {
                layer = &parent.circles(points, r->value.num[0]);
            } else {
                layer = &parent.circles(points);
            }
        } else if (name == "rect") {
            Rect r{};
            if (const MapField* f = c.find("rect")) {
                r = f->value.rect();
            } else if (const MapField* f2 = c.find("size")) {
                r = {0, 0, f2->value.num[0], f2->value.num[1]};
            }
            layer = &parent.rect(r);
        } else if (name == "circle") {
            layer = &parent.circle(vec("center"), num("radius", 0));
        } else if (name == "arc") {
            layer = &parent.arc(vec("center"), num("radius", 0), num("start_deg", 0),
                                num("end_deg", 0));
        } else if (name == "arrow") {
            layer = &parent.arrow(vec("from"), vec("to"));
            if (const MapField* f = c.find("head_size")) {
                layer->headSize(f->value.num[0]);
            }
        } else if (name == "crosshair") {
            if (const MapField* f = c.find("size")) {
                layer = &parent.crosshair(vec("center"), f->value.num[0]);
            } else {
                layer = &parent.crosshair(vec("center"));
            }
        } else if (name == "grid") {
            if (const MapField* f = c.find("spacing")) {
                layer = &parent.grid(f->value.num[0]);
            } else {
                layer = &parent.grid();
            }
        } else if (name == "boxes") {
            layer = &parent.boxes({});
        } else {
            error("compile.internal", std::string("unhandled content type '") + name + "'");
            return nullptr;
        }

        // Input binding + fallback presentation.
        if (const MapField* src = c.find("source")) {
            registerInput(*layer, decl, src->value.input);
        }
        return layer;
    }

    /// Builds one UI prefab. Bindable fields (`on`, `value`) may carry a
    /// $params reference: the control starts at the param's default and
    /// setParam drives it from then on (programmatic changes fire no event,
    /// same as the C++ setters).
    Layer* createControl(Layer& parent, const LayerDecl& decl, const ContentDecl& c) {
        CompiledScene* scene = &scene_;
        const std::string name = c.spec->name;
        const AttrDecl* frame_attr = decl.find(AttrId::Frame);
        const Rect frame = frame_attr != nullptr ? frame_attr->value.rect() : Rect{};

        // The event id: the declared id, or the auto id assigned later —
        // resolved lazily at fire time from the control's layer.
        if (name == "button") {
            const MapField* label = c.find("label");
            auto& control = CompileAccess::buttons(scene_).emplace_back(
                std::make_unique<ui::Button>(parent, frame,
                                             label != nullptr ? label->value.str : ""));
            ui::Button* raw = control.get();
            raw->onTap([scene, raw] {
                CompileAccess::fire(*scene, {raw->layer().id(), "button", 1, true});
            });
            return &raw->layer();
        }
        if (name == "switch") {
            auto& control = CompileAccess::switches(scene_).emplace_back(
                std::make_unique<ui::Switch>(parent, frame));
            ui::Switch* raw = control.get();
            if (const MapField* on = c.find("on")) {
                if (!on->value.param.empty()) {
                    const ParamDecl* p = doc_.findParam(on->value.param);
                    raw->setOn(p->def_flag, false);
                    CompileAccess::controlBindings(scene_)[p->name].push_back(
                        {CompiledScene::ControlBinding::Kind::SwitchOn, raw});
                } else {
                    raw->setOn(on->value.flag, false);
                }
            }
            raw->onChange([scene, raw](bool on) {
                CompileAccess::fire(*scene,
                                    {raw->layer().id(), "switch", on ? 1.0f : 0.0f, on});
            });
            return &raw->layer();
        }
        if (name == "slider") {
            float initial = 0;
            const ParamDecl* bound = nullptr;
            if (const MapField* value = c.find("value")) {
                if (!value->value.param.empty()) {
                    bound = doc_.findParam(value->value.param);
                    initial = bound->def[0];
                } else {
                    initial = value->value.num[0];
                }
            }
            auto& control = CompileAccess::sliders(scene_).emplace_back(
                std::make_unique<ui::Slider>(parent, frame, initial));
            ui::Slider* raw = control.get();
            if (bound != nullptr) {
                CompileAccess::controlBindings(scene_)[bound->name].push_back(
                    {CompiledScene::ControlBinding::Kind::SliderValue, raw});
            }
            raw->onChange([scene, raw](float v) {
                CompileAccess::fire(*scene, {raw->layer().id(), "slider", v, false});
            });
            return &raw->layer();
        }
        // gauge — display only, no events.
        auto num_of = [&](const char* field, float fallback) {
            const MapField* f = c.find(field);
            return f != nullptr && f->value.param.empty() ? f->value.num[0] : fallback;
        };
        const MapField* center = c.find("center");
        auto& control = CompileAccess::gauges(scene_).emplace_back(std::make_unique<ui::Gauge>(
            parent, center != nullptr ? center->value.vec2() : Vec2{},
            num_of("radius", 0)));
        ui::Gauge* raw = control.get();
        if (const MapField* value = c.find("value")) {
            if (!value->value.param.empty()) {
                const ParamDecl* p = doc_.findParam(value->value.param);
                raw->setValue(p->def[0]);
                CompileAccess::controlBindings(scene_)[p->name].push_back(
                    {CompiledScene::ControlBinding::Kind::GaugeValue, raw});
            } else {
                raw->setValue(value->value.num[0]);
            }
        }
        return &raw->layer();
    }

    void registerInput(Layer& layer, const LayerDecl& decl, const std::string& input_name) {
        const InputDecl* input = doc_.findInput(input_name);
        CompiledScene::InputBinding binding;
        binding.layer = &layer;
        binding.fallback = input->fallback;

        if (input->fallback == InputFallback::Hide) {
            layer.hidden(true);
        } else if (input->fallback == InputFallback::Placeholder &&
                   input->type == InputType::ImageRgba8) {
            // A deterministic "no signal" panel: dark plate, faint grid,
            // and the input's name — so a missing feed diagnoses itself.
            const Vec2 size = staticSizeOf(decl);
            layer.background({0.10f, 0.11f, 0.13f, 1.0f});
            binding.background_was_placeholder = true;
            Layer& grid = layer.grid(48).color(placeholderInk(0.07f)).thickness(1);
            Layer& label = layer.text("$inputs." + input_name, {size.x * 0.5f, size.y * 0.5f})
                               .size(24)
                               .align(Align::Center)
                               .color(placeholderInk(0.45f));
            binding.placeholder_parts = {&grid, &label};
        }
        CompileAccess::inputBindings(scene_)[input_name].push_back(std::move(binding));
    }

    /// Static size of a declared layer against its statically-known parent
    /// chain (placeholder layout only).
    Vec2 staticSizeOf(const LayerDecl& target) {
        Vec2 size = {doc_.width, doc_.height};
        findStaticSize(doc_.layers, target, size, &size);
        return size;
    }

    static bool findStaticSize(const std::vector<LayerDecl>& layers, const LayerDecl& target,
                               Vec2 parent_size, Vec2* out) {
        for (const LayerDecl& decl : layers) {
            const Vec2 own = staticSize(decl, parent_size);
            if (&decl == &target) {
                *out = own;
                return true;
            }
            if (findStaticSize(decl.sublayers, target, own, out)) {
                return true;
            }
        }
        return false;
    }

    CompiledScene& scene_;
    const SceneDoc& doc_;
    const CompileOptions& options_;
    DiagnosticList& diags_;
};

}  // namespace

// ---------------------------------------------------------------------------
// CompiledScene runtime surface
// ---------------------------------------------------------------------------

void CompiledScene::diagnose(std::string message) {
    if (diagnostics_.size() < 256) {
        diagnostics_.push_back(std::move(message));
    }
}

std::vector<std::string> CompiledScene::drainDiagnostics() {
    std::vector<std::string> out = std::move(diagnostics_);
    diagnostics_.clear();
    for (std::string& s : stage_->drainDiagnostics()) {
        out.push_back(std::move(s));
    }
    return out;
}

Layer* CompiledScene::layerFor(const LayerDecl* decl) const {
    for (const auto& [d, layer] : nodes_) {
        if (d == decl) {
            return layer;
        }
    }
    return nullptr;
}

namespace {

/// Clears a binding's fallback presentation on first data.
void clearFallback(CompiledScene::InputBinding& binding) {
    if (binding.fed) {
        return;
    }
    binding.fed = true;
    if (binding.fallback == InputFallback::Hide) {
        binding.layer->hidden(false);
    }
    for (Layer* part : binding.placeholder_parts) {
        part->remove();
    }
    binding.placeholder_parts.clear();
    if (binding.background_was_placeholder) {
        binding.layer->background(Color::Transparent);
    }
}

}  // namespace

bool CompiledScene::setImage(const std::string& input, const ImageView& view) {
    const InputDecl* decl = doc_.findInput(input);
    if (decl == nullptr || decl->type != InputType::ImageRgba8) {
        diagnose("setImage('" + input + "'): " +
                 (decl == nullptr ? "undeclared input" : "input is not image.rgba8"));
        return false;
    }
    for (InputBinding& b : input_bindings_[input]) {
        clearFallback(b);
        b.layer->setImage(view);
    }
    return true;
}

bool CompiledScene::setText(const std::string& input, const std::string& utf8) {
    const InputDecl* decl = doc_.findInput(input);
    if (decl == nullptr || decl->type != InputType::TextUtf8) {
        diagnose("setText('" + input + "'): " +
                 (decl == nullptr ? "undeclared input" : "input is not text.utf8"));
        return false;
    }
    for (InputBinding& b : input_bindings_[input]) {
        clearFallback(b);
        b.layer->setText(utf8);
    }
    return true;
}

bool CompiledScene::setPoints(const std::string& input, const std::vector<Vec2>& points) {
    const InputDecl* decl = doc_.findInput(input);
    if (decl == nullptr || decl->type != InputType::SeqVec2) {
        diagnose("setPoints('" + input + "'): " +
                 (decl == nullptr ? "undeclared input" : "input is not sequence<vec2, N>"));
        return false;
    }
    const std::vector<Vec2>* data = &points;
    std::vector<Vec2> clamped;
    if (points.size() > decl->capacity) {
        diagnose("setPoints('" + input + "'): " + std::to_string(points.size()) +
                 " points clamped to declared capacity " + std::to_string(decl->capacity));
        clamped.assign(points.begin(), points.begin() + decl->capacity);
        data = &clamped;
    }
    for (InputBinding& b : input_bindings_[input]) {
        clearFallback(b);
        b.layer->setPoints(*data);
    }
    return true;
}

bool CompiledScene::setBoxes(const std::string& input, const std::vector<Box>& boxes) {
    const InputDecl* decl = doc_.findInput(input);
    if (decl == nullptr || decl->type != InputType::SeqDetection2D) {
        diagnose("setBoxes('" + input + "'): " +
                 (decl == nullptr ? "undeclared input"
                                  : "input is not sequence<detection2d, N>"));
        return false;
    }
    const std::vector<Box>* data = &boxes;
    std::vector<Box> clamped;
    if (boxes.size() > decl->capacity) {
        diagnose("setBoxes('" + input + "'): " + std::to_string(boxes.size()) +
                 " boxes clamped to declared capacity " + std::to_string(decl->capacity));
        clamped.assign(boxes.begin(), boxes.begin() + decl->capacity);
        data = &clamped;
    }
    for (InputBinding& b : input_bindings_[input]) {
        clearFallback(b);
        b.layer->setBoxes(*data);
    }
    return true;
}

void CompiledScene::applyParam(const ParamDecl& decl, const Value& value) {
    // Bound UI controls first: their setters carry their own animation and
    // never fire user-gesture events.
    for (const ControlBinding& b : control_bindings_[decl.name]) {
        switch (b.kind) {
            case ControlBinding::Kind::SwitchOn:
                static_cast<ui::Switch*>(b.control)->setOn(value.flag);
                break;
            case ControlBinding::Kind::SliderValue:
                static_cast<ui::Slider*>(b.control)->setValue(value.num[0]);
                break;
            case ControlBinding::Kind::GaugeValue:
                static_cast<ui::Gauge*>(b.control)->setValue(value.num[0]);
                break;
        }
    }
    for (const ParamBinding& b : param_bindings_[decl.name]) {
        // Animation priority (§9): the param's animate declaration, else the
        // layer's transition, else snap.
        if (decl.has_animate) {
            Transaction t(decl.animate_duration, decl.animate_ease);
            applyAttr(*b.layer, b.attr, value);
        } else if (auto it = layer_transitions_.find(b.layer);
                   it != layer_transitions_.end()) {
            Transaction t(it->second.duration, it->second.ease);
            applyAttr(*b.layer, b.attr, value);
        } else {
            applyAttr(*b.layer, b.attr, value);
        }
    }
}

namespace {

bool paramGate(const SceneDoc& doc, const std::string& name, ParamType expected,
               const ParamDecl*& out, std::string& message) {
    out = doc.findParam(name);
    if (out == nullptr) {
        message = "undeclared param";
        return false;
    }
    if (!out->runtime_mutable) {
        message = "param is not runtime_mutable";
        return false;
    }
    if (out->type != expected) {
        message = std::string("param has type ") + paramTypeName(out->type);
        return false;
    }
    return true;
}

}  // namespace

bool CompiledScene::setParam(const std::string& name, float value) {
    const ParamDecl* decl = nullptr;
    std::string why;
    if (!paramGate(doc_, name, ParamType::F32, decl, why)) {
        diagnose("setParam('" + name + "'): " + why);
        return false;
    }
    Value v;
    v.kind = ValueKind::F32;
    v.num[0] = value;
    applyParam(*decl, v);
    return true;
}

bool CompiledScene::setParam(const std::string& name, Vec2 value) {
    const ParamDecl* decl = nullptr;
    std::string why;
    if (!paramGate(doc_, name, ParamType::Vec2, decl, why)) {
        diagnose("setParam('" + name + "'): " + why);
        return false;
    }
    Value v;
    v.kind = ValueKind::Vec2;
    v.num[0] = value.x;
    v.num[1] = value.y;
    applyParam(*decl, v);
    return true;
}

bool CompiledScene::setParam(const std::string& name, Color value) {
    const ParamDecl* decl = nullptr;
    std::string why;
    if (!paramGate(doc_, name, ParamType::Color, decl, why)) {
        diagnose("setParam('" + name + "'): " + why);
        return false;
    }
    Value v;
    v.kind = ValueKind::Color;
    v.num[0] = value.r;
    v.num[1] = value.g;
    v.num[2] = value.b;
    v.num[3] = value.a;
    applyParam(*decl, v);
    return true;
}

bool CompiledScene::setParam(const std::string& name, bool value) {
    const ParamDecl* decl = nullptr;
    std::string why;
    if (!paramGate(doc_, name, ParamType::Bool, decl, why)) {
        diagnose("setParam('" + name + "'): " + why);
        return false;
    }
    Value v;
    v.kind = ValueKind::Bool;
    v.flag = value;
    applyParam(*decl, v);
    return true;
}

// ---------------------------------------------------------------------------
// compile()
// ---------------------------------------------------------------------------

CompileResult compile(const SceneDoc& doc, const CompileOptions& options) {
    CompileResult result;
    if (doc.schema != kSchemaId) {
        result.diagnostics.add("compile.rejected", Severity::kError, Phase::kCompile, Span{},
                               "document was not validated (schema is empty or unsupported); "
                               "compile only parseScene results with ok()");
        return result;
    }
    auto scene = CompileAccess::create();
    CompileAccess::doc(*scene) = doc;  // own copy: nodes_ points into it
    CompileAccess::digestRef(*scene) = digest(doc);
    Builder builder(*scene, CompileAccess::doc(*scene), options, result.diagnostics);
    if (!builder.build()) {
        return result;
    }
    result.scene = std::move(scene);
    return result;
}

}  // namespace fluent_stage::scene
