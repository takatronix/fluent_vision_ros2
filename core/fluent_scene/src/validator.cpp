#include "fluent_scene/validator.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "fluent_scene/sha256.hpp"

namespace fluent_scene {

const char kSupportedSchemaIdentity[] = "fluent.scene/v1alpha1";

namespace {

bool isIdentifier(const std::string& text) {
    if (text.empty()) {
        return false;
    }
    const char first = text[0];
    if (!((first >= 'a' && first <= 'z') || (first >= 'A' && first <= 'Z') || first == '_')) {
        return false;
    }
    for (char c : text) {
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_')) {
            return false;
        }
    }
    return true;
}

struct Ref {
    enum class Space { kInputs, kParams, kResources, kNodes, kRuntime };
    Space space = Space::kInputs;
    std::string name;
    std::string port;  // kNodes only
};

bool parseRefText(const std::string& text, Ref& out) {
    if (text.size() < 2 || text[0] != '$') {
        return false;
    }
    std::vector<std::string> parts;
    size_t start = 0;
    while (start <= text.size()) {
        const size_t dot = text.find('.', start);
        if (dot == std::string::npos) {
            parts.push_back(text.substr(start));
            break;
        }
        parts.push_back(text.substr(start, dot - start));
        start = dot + 1;
    }
    for (const std::string& part : parts) {
        if (part.empty()) {
            return false;
        }
    }
    if (parts[0] == "$inputs" && parts.size() == 2) {
        out.space = Ref::Space::kInputs;
        out.name = parts[1];
        return true;
    }
    if (parts[0] == "$params" && parts.size() == 2) {
        out.space = Ref::Space::kParams;
        out.name = parts[1];
        return true;
    }
    if (parts[0] == "$resources" && parts.size() == 2) {
        out.space = Ref::Space::kResources;
        out.name = parts[1];
        return true;
    }
    if (parts[0] == "$runtime" && parts.size() == 2) {
        out.space = Ref::Space::kRuntime;
        out.name = parts[1];
        return true;
    }
    if (parts[0] == "$nodes" && parts.size() == 3) {
        out.space = Ref::Space::kNodes;
        out.name = parts[1];
        out.port = parts[2];
        return true;
    }
    return false;
}

enum class RefContext { kPort, kParam, kOutput };

const char* allowedNamespaces(RefContext context) {
    switch (context) {
        case RefContext::kPort: return "$inputs and $nodes";
        case RefContext::kParam: return "$params and $resources";
        case RefContext::kOutput: return "$nodes and $runtime";
    }
    return "";
}

class SceneValidator {
public:
    SceneValidator(const YamlNode& root, const NodeRegistry& registry, DiagnosticList& diagnostics)
        : root_(root), registry_(registry), diags_(diagnostics) {}

    ValidationResult run() {
        ValidationResult result;
        if (!root_.isMapping()) {
            err("validate.root_not_mapping", root_.span, "a Fluent Scene document must be a mapping");
            return result;
        }
        if (!validateHeader()) {
            return result;
        }
        validateTopLevelKeys();
        validateMetadata();
        validateTypes();
        validateParams();
        validateFallbacks();
        validateInputs();
        validateResources();
        collectNodes();
        resolveNodes();
        validateOutputs();
        validateBudgets();
        validateExtensions();
        detectCyclesAndOrder();
        checkUnused();
        if (!diags_.hasErrors()) {
            result.ir = buildIr();
            result.ir_text = result.ir.serialize();
            result.digest = "sha256:" + sha256Hex(result.ir_text);
            result.ok = true;
        }
        return result;
    }

private:
    // --- declaration records -------------------------------------------------

    struct ParamDecl {
        TypePtr type;
        JsonValue default_value;
        bool has_default = false;
        bool runtime_mutable = false;
        JsonValue constraints;
        bool has_constraints = false;
        uint64_t max_utf8_bytes = 0;
        Span span;
        bool used = false;
    };

    struct InputDecl {
        TypePtr type;
        bool required = true;
        std::string update;
        std::vector<std::pair<std::string, std::string>> metadata;
        JsonValue constraints;
        bool has_constraints = false;
        uint64_t max_utf8_bytes = 0;
        std::string fallback;
        Span fallback_span;
        Span span;
        bool used = false;
    };

    struct ResourceDecl {
        std::string type;
        std::string uri;
        uint64_t glyph_capacity = 0;
        Span span;
        bool used = false;
    };

    struct FallbackDecl {
        bool is_behavior = false;
        std::string behavior;
        const YamlNode* value = nullptr;
        JsonValue normalized;
        bool has_normalized = false;
        Span span;
        bool used = false;
    };

    struct PortBinding {
        std::string port;
        bool is_list = false;
        std::vector<std::string> refs;
    };

    struct NodeDecl {
        std::string id;
        const NodeTypeSpec* spec = nullptr;
        const YamlNode* yaml = nullptr;
        std::vector<PortBinding> inputs;
        std::vector<std::pair<std::string, JsonValue>> params;
        bool has_bounds = false;
        uint64_t bound_count = 0;
        std::string overflow_rule;
        Span span;
    };

    struct OutputDecl {
        TypePtr type;
        std::string source;
        Span span;
    };

    // --- diagnostics helpers --------------------------------------------------

    void err(std::string code, Span span, std::string message,
             std::vector<std::pair<std::string, std::string>> context = {}) {
        diags_.add(std::move(code), Severity::kError, Phase::kValidate, span, std::move(message),
                   std::move(context));
    }

    void info(std::string code, Span span, std::string message,
              std::vector<std::pair<std::string, std::string>> context = {}) {
        diags_.add(std::move(code), Severity::kInfo, Phase::kValidate, span, std::move(message),
                   std::move(context));
    }

    bool checkKnownKeys(const YamlNode& mapping, const std::vector<std::string>& allowed,
                        const std::string& section) {
        bool ok = true;
        for (const auto& entry : mapping.entries) {
            if (std::find(allowed.begin(), allowed.end(), entry.key) == allowed.end()) {
                err("validate.unknown_field", entry.key_span,
                    "unknown field \"" + entry.key + "\" in " + section, {{"field", entry.key}});
                ok = false;
            }
        }
        return ok;
    }

    // --- scalar interpretation (schema-driven; plain scalars only) -----------

    static bool isPlainNull(const YamlNode& node) {
        return node.isNull() || (node.isPlainScalar() && node.scalar == "null");
    }

    bool parseBoolNode(const YamlNode& node, bool& out) {
        if (!node.isPlainScalar()) {
            return false;
        }
        if (node.scalar == "true") {
            out = true;
            return true;
        }
        if (node.scalar == "false") {
            out = false;
            return true;
        }
        return false;
    }

    static bool parseU64Text(const std::string& text, uint64_t& out) {
        if (text.empty()) {
            return false;
        }
        auto result = std::from_chars(text.data(), text.data() + text.size(), out);
        return result.ec == std::errc() && result.ptr == text.data() + text.size();
    }

    bool parseU64Node(const YamlNode& node, uint64_t& out) {
        return node.isPlainScalar() && parseU64Text(node.scalar, out);
    }

    static bool parseI64Text(const std::string& text, int64_t& out) {
        if (text.empty()) {
            return false;
        }
        auto result = std::from_chars(text.data(), text.data() + text.size(), out);
        return result.ec == std::errc() && result.ptr == text.data() + text.size();
    }

    static bool parseF64Text(const std::string& text, double& out) {
        if (text.empty()) {
            return false;
        }
        auto result = std::from_chars(text.data(), text.data() + text.size(), out);
        return result.ec == std::errc() && result.ptr == text.data() + text.size() && std::isfinite(out);
    }

    bool parseF64Node(const YamlNode& node, double& out) {
        return node.isPlainScalar() && parseF64Text(node.scalar, out);
    }

    // --- typed literal checking ----------------------------------------------

    bool checkVector(const YamlNode& node, size_t count, const std::string& type_name, JsonValue& out) {
        if (!node.isSequence() || node.elements.size() != count) {
            err("validate.type_mismatch", node.span,
                type_name + " requires a sequence of exactly " + std::to_string(count) + " numbers");
            return false;
        }
        JsonValue array = JsonValue::makeArray();
        for (const YamlNode& element : node.elements) {
            double value = 0.0;
            if (!parseF64Node(element, value)) {
                err("validate.type_mismatch", element.span, "expected a number in " + type_name + " literal");
                return false;
            }
            array.append(JsonValue::makeFloat(value));
        }
        out = std::move(array);
        return true;
    }

    bool checkLiteral(const YamlNode& node, const Type& type, JsonValue& out) {
        const std::string type_name = TypeTable::canonicalName(type);
        switch (type.kind) {
            case TypeKind::kBool: {
                bool value = false;
                if (!parseBoolNode(node, value)) {
                    err("validate.type_mismatch", node.span, "expected bool (true or false)");
                    return false;
                }
                out = JsonValue::makeBool(value);
                return true;
            }
            case TypeKind::kI32: {
                int64_t value = 0;
                if (!node.isPlainScalar() || !parseI64Text(node.scalar, value) || value < INT32_MIN ||
                    value > INT32_MAX) {
                    err("validate.type_mismatch", node.span, "expected an i32 integer");
                    return false;
                }
                out = JsonValue::makeInt(value);
                return true;
            }
            case TypeKind::kU32: {
                uint64_t value = 0;
                if (!parseU64Node(node, value) || value > UINT32_MAX) {
                    err("validate.type_mismatch", node.span, "expected a u32 integer");
                    return false;
                }
                out = JsonValue::makeUInt(value);
                return true;
            }
            case TypeKind::kF32: {
                double value = 0.0;
                if (!parseF64Node(node, value)) {
                    err("validate.type_mismatch", node.span, "expected an f32 number");
                    return false;
                }
                out = JsonValue::makeFloat(value);
                return true;
            }
            case TypeKind::kString: {
                if (!node.isScalar() || (node.isPlainScalar() && node.scalar == "null")) {
                    err("validate.type_mismatch", node.span,
                        "expected a string (quote the value if it could be read as null)");
                    return false;
                }
                out = JsonValue::makeString(node.scalar);
                return true;
            }
            case TypeKind::kVec2f: return checkVector(node, 2, type_name, out);
            case TypeKind::kVec3f: return checkVector(node, 3, type_name, out);
            case TypeKind::kVec4f: return checkVector(node, 4, type_name, out);
            case TypeKind::kQuatf: return checkVector(node, 4, type_name, out);
            case TypeKind::kMat3f: return checkVector(node, 9, type_name, out);
            case TypeKind::kMat4f: return checkVector(node, 16, type_name, out);
            case TypeKind::kTransform3d: return checkVector(node, 16, type_name, out);
            case TypeKind::kSequence: {
                if (!node.isSequence()) {
                    err("validate.type_mismatch", node.span, "expected a sequence literal for " + type_name);
                    return false;
                }
                if (node.elements.size() > type.capacity) {
                    err("validate.capacity_overflow", node.span,
                        "sequence literal has " + std::to_string(node.elements.size()) +
                            " elements but the declared capacity is " + std::to_string(type.capacity),
                        {{"capacity", std::to_string(type.capacity)},
                         {"count", std::to_string(node.elements.size())}});
                    return false;
                }
                JsonValue array = JsonValue::makeArray();
                for (const YamlNode& element : node.elements) {
                    JsonValue value;
                    if (!checkLiteral(element, *type.element, value)) {
                        return false;
                    }
                    array.append(std::move(value));
                }
                out = std::move(array);
                return true;
            }
            case TypeKind::kOptional: {
                if (isPlainNull(node)) {
                    out = JsonValue::makeNull();
                    return true;
                }
                return checkLiteral(node, *type.element, out);
            }
            case TypeKind::kStruct: {
                const StructDecl* decl = types_.findStruct(type.struct_name);
                if (decl == nullptr) {
                    err("validate.unknown_type", node.span, "unknown struct type \"" + type.struct_name + "\"");
                    return false;
                }
                if (!node.isMapping()) {
                    err("validate.type_mismatch", node.span,
                        "expected a mapping literal for struct " + type.struct_name);
                    return false;
                }
                bool ok = true;
                for (const auto& entry : node.entries) {
                    bool known = false;
                    for (const StructField& field : decl->fields) {
                        if (field.name == entry.key) {
                            known = true;
                            break;
                        }
                    }
                    if (!known) {
                        err("validate.unknown_field", entry.key_span,
                            "struct " + type.struct_name + " has no field \"" + entry.key +
                                "\" (struct types are closed)");
                        ok = false;
                    }
                }
                JsonValue object = JsonValue::makeObject();
                for (const StructField& field : decl->fields) {
                    const YamlNode* value = node.find(field.name);
                    if (value == nullptr) {
                        err("validate.missing_field", node.span,
                            "struct " + type.struct_name + " literal is missing field \"" + field.name + "\"");
                        ok = false;
                        continue;
                    }
                    JsonValue field_value;
                    if (field.type != nullptr && checkLiteral(*value, *field.type, field_value)) {
                        object.set(field.name, std::move(field_value));
                    } else {
                        ok = false;
                    }
                }
                if (ok) {
                    out = std::move(object);
                }
                return ok;
            }
            default:
                err("validate.type_mismatch", node.span,
                    "type \"" + type_name + "\" has no literal representation");
                return false;
        }
    }

    // --- constraints -----------------------------------------------------------

    bool validateConstraints(const TypePtr& type, const YamlNode& node, JsonValue& out, uint64_t& max_utf8) {
        if (!node.isMapping()) {
            err("validate.wrong_node_kind", node.span, "constraints must be a mapping");
            return false;
        }
        TypeKind kind = type->kind;
        if (kind == TypeKind::kOptional) {
            kind = type->element->kind;
        }
        std::vector<std::pair<std::string, JsonValue>> items;
        bool ok = true;
        for (const auto& entry : node.entries) {
            if (entry.key == "max_utf8_bytes" && kind == TypeKind::kString) {
                uint64_t value = 0;
                if (!parseU64Node(entry.value, value) || value == 0 || value > (1u << 20)) {
                    err("validate.value_out_of_range", entry.value.span,
                        "max_utf8_bytes must be an integer in 1..1048576");
                    ok = false;
                    continue;
                }
                max_utf8 = value;
                items.emplace_back(entry.key, JsonValue::makeUInt(value));
                continue;
            }
            if ((entry.key == "min" || entry.key == "max") &&
                (kind == TypeKind::kI32 || kind == TypeKind::kU32 || kind == TypeKind::kF32)) {
                double value = 0.0;
                if (!parseF64Node(entry.value, value)) {
                    err("validate.value_out_of_range", entry.value.span,
                        "constraint \"" + entry.key + "\" must be a finite number");
                    ok = false;
                    continue;
                }
                items.emplace_back(entry.key, JsonValue::makeFloat(value));
                continue;
            }
            err("validate.unknown_constraint", entry.key_span,
                "constraint \"" + entry.key + "\" is not valid for type " + TypeTable::canonicalName(*type));
            ok = false;
        }
        std::sort(items.begin(), items.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        JsonValue object = JsonValue::makeObject();
        for (auto& item : items) {
            object.set(item.first, std::move(item.second));
        }
        out = std::move(object);
        return ok;
    }

    void enforceStringBytes(const JsonValue& value, uint64_t max_utf8, Span span, const std::string& what) {
        if (max_utf8 > 0 && value.kind() == JsonValue::Kind::kString &&
            value.stringValue().size() > max_utf8) {
            err("validate.constraint_violation", span,
                what + " exceeds max_utf8_bytes=" + std::to_string(max_utf8),
                {{"max_utf8_bytes", std::to_string(max_utf8)},
                 {"actual_bytes", std::to_string(value.stringValue().size())}});
        }
    }

    // --- header and top level ---------------------------------------------------

    bool validateHeader() {
        const YamlNode* schema = root_.find("schema");
        if (schema == nullptr) {
            err("validate.missing_field", root_.span, "top-level \"schema\" is required");
            return false;
        }
        if (!schema->isPlainScalar()) {
            err("validate.wrong_node_kind", schema->span, "\"schema\" must be a plain scalar identity");
            return false;
        }
        if (schema->scalar != kSupportedSchemaIdentity) {
            err("validate.unsupported_schema", schema->span,
                "unsupported schema identity \"" + schema->scalar + "\"; this validator supports exactly: " +
                    kSupportedSchemaIdentity,
                {{"found", schema->scalar}, {"supported", kSupportedSchemaIdentity}});
            return false;
        }
        const YamlNode* kind = root_.find("kind");
        if (kind == nullptr) {
            err("validate.missing_field", root_.span, "top-level \"kind\" is required");
            return false;
        }
        if (!kind->isPlainScalar() || kind->scalar != "Scene") {
            err("validate.unsupported_kind", kind->span, "\"kind\" must be \"Scene\" for a .fvs document");
            return false;
        }
        return true;
    }

    void validateTopLevelKeys() {
        checkKnownKeys(root_,
                       {"schema", "kind", "metadata", "params", "types", "inputs", "resources", "nodes",
                        "outputs", "budgets", "fallbacks", "extensions"},
                       "the top-level document");
        for (const char* required : {"metadata", "nodes", "outputs", "budgets"}) {
            if (root_.find(required) == nullptr) {
                err("validate.missing_field", root_.span,
                    std::string("top-level \"") + required + "\" section is required");
            }
        }
    }

    void validateMetadata() {
        const YamlNode* metadata = root_.find("metadata");
        if (metadata == nullptr) {
            return;  // reported by validateTopLevelKeys
        }
        if (!metadata->isMapping()) {
            err("validate.wrong_node_kind", metadata->span, "\"metadata\" must be a mapping");
            return;
        }
        const YamlNode* name = metadata->find("name");
        if (name == nullptr || !name->isScalar()) {
            err("validate.missing_field", metadata->span, "metadata.name is required and must be a scalar");
        } else {
            scene_name_ = name->scalar;
            if (!isIdentifier(scene_name_)) {
                err("validate.bad_identifier", name->span,
                    "metadata.name must be an identifier ([A-Za-z_][A-Za-z0-9_]*)");
            }
        }
        for (const auto& entry : metadata->entries) {
            if (entry.key == "name") {
                continue;
            }
            if (!entry.value.isScalar()) {
                err("validate.wrong_node_kind", entry.key_span,
                    "metadata values must be scalar descriptive text");
                continue;
            }
            metadata_extra_.emplace_back(entry.key, entry.value.scalar);
        }
        std::sort(metadata_extra_.begin(), metadata_extra_.end());
    }

    // --- types -------------------------------------------------------------------

    void validateTypes() {
        const YamlNode* types = root_.find("types");
        if (types == nullptr) {
            return;
        }
        if (!types->isMapping()) {
            err("validate.wrong_node_kind", types->span, "\"types\" must be a mapping");
            return;
        }
        for (const auto& entry : types->entries) {
            if (!isIdentifier(entry.key)) {
                err("validate.bad_identifier", entry.key_span,
                    "type name \"" + entry.key + "\" must be an identifier");
                continue;
            }
            if (!types_.declareStruct(entry.key, entry.key_span)) {
                err("validate.duplicate_id", entry.key_span,
                    "type name \"" + entry.key + "\" collides with a built-in or existing type");
            }
        }
        for (const auto& entry : types->entries) {
            StructDecl* decl = types_.findStruct(entry.key);
            if (decl == nullptr) {
                continue;
            }
            if (!entry.value.isMapping()) {
                err("validate.wrong_node_kind", entry.value.span,
                    "type declaration must be a mapping containing \"struct\"");
                continue;
            }
            checkKnownKeys(entry.value, {"struct"}, "type \"" + entry.key + "\"");
            const YamlNode* fields = entry.value.find("struct");
            if (fields == nullptr || !fields->isMapping()) {
                err("validate.missing_field", entry.value.span,
                    "type \"" + entry.key + "\" must declare a \"struct\" mapping of fields");
                continue;
            }
            for (const auto& field : fields->entries) {
                if (!isIdentifier(field.key)) {
                    err("validate.bad_identifier", field.key_span,
                        "field name \"" + field.key + "\" must be an identifier");
                    continue;
                }
                if (!field.value.isPlainScalar()) {
                    err("validate.wrong_node_kind", field.value.span, "field type must be a type expression");
                    continue;
                }
                TypePtr type = types_.parse(field.value.scalar, field.value.span, diags_);
                if (type != nullptr) {
                    decl->fields.push_back(StructField{field.key, type, field.key_span});
                }
            }
        }
        detectTypeCycles();
    }

    static void collectStructRefs(const Type& type, std::vector<std::string>& out) {
        if (type.kind == TypeKind::kStruct) {
            out.push_back(type.struct_name);
        } else if (type.element != nullptr) {
            collectStructRefs(*type.element, out);
        }
    }

    void detectTypeCycles() {
        std::map<std::string, int> color;  // 0 unvisited, 1 visiting, 2 done
        std::vector<std::string> stack;
        bool reported = false;
        std::function<void(const std::string&)> visit = [&](const std::string& name) {
            if (reported) {
                return;
            }
            color[name] = 1;
            stack.push_back(name);
            const StructDecl* decl = types_.findStruct(name);
            if (decl != nullptr) {
                for (const StructField& field : decl->fields) {
                    if (field.type == nullptr) {
                        continue;
                    }
                    std::vector<std::string> refs;
                    collectStructRefs(*field.type, refs);
                    for (const std::string& ref : refs) {
                        if (types_.findStruct(ref) == nullptr) {
                            continue;
                        }
                        if (color[ref] == 1) {
                            std::string cycle;
                            auto it = std::find(stack.begin(), stack.end(), ref);
                            for (; it != stack.end(); ++it) {
                                cycle += *it + " -> ";
                            }
                            cycle += ref;
                            err("validate.cyclic_type", decl->span,
                                "cyclic value type detected: " + cycle, {{"cycle", cycle}});
                            reported = true;
                            return;
                        }
                        if (color[ref] == 0) {
                            visit(ref);
                        }
                    }
                }
            }
            stack.pop_back();
            color[name] = 2;
        };
        for (const auto& [name, decl] : types_.structs()) {
            if (color[name] == 0) {
                visit(name);
            }
        }
    }

    // --- params --------------------------------------------------------------------

    void validateParams() {
        const YamlNode* params = root_.find("params");
        if (params == nullptr) {
            return;
        }
        if (!params->isMapping()) {
            err("validate.wrong_node_kind", params->span, "\"params\" must be a mapping");
            return;
        }
        for (const auto& entry : params->entries) {
            if (!isIdentifier(entry.key)) {
                err("validate.bad_identifier", entry.key_span,
                    "parameter name \"" + entry.key + "\" must be an identifier");
                continue;
            }
            if (!entry.value.isMapping()) {
                err("validate.wrong_node_kind", entry.value.span, "parameter declaration must be a mapping");
                continue;
            }
            checkKnownKeys(entry.value, {"type", "default", "runtime_mutable", "constraints"},
                           "parameter \"" + entry.key + "\"");
            ParamDecl decl;
            decl.span = entry.key_span;
            const YamlNode* type_node = entry.value.find("type");
            if (type_node == nullptr || !type_node->isPlainScalar()) {
                err("validate.missing_field", entry.value.span,
                    "parameter \"" + entry.key + "\" requires a \"type\"");
                continue;
            }
            decl.type = types_.parse(type_node->scalar, type_node->span, diags_);
            if (decl.type == nullptr) {
                continue;
            }
            if (const YamlNode* mutable_node = entry.value.find("runtime_mutable")) {
                if (!parseBoolNode(*mutable_node, decl.runtime_mutable)) {
                    err("validate.type_mismatch", mutable_node->span, "runtime_mutable must be true or false");
                }
            }
            if (const YamlNode* constraints_node = entry.value.find("constraints")) {
                decl.has_constraints =
                    validateConstraints(decl.type, *constraints_node, decl.constraints, decl.max_utf8_bytes);
            }
            if (const YamlNode* default_node = entry.value.find("default")) {
                if (checkLiteral(*default_node, *decl.type, decl.default_value)) {
                    decl.has_default = true;
                    enforceStringBytes(decl.default_value, decl.max_utf8_bytes, default_node->span,
                                       "default of parameter \"" + entry.key + "\"");
                }
            }
            params_.emplace(entry.key, std::move(decl));
        }
    }

    // --- fallbacks -------------------------------------------------------------------

    void validateFallbacks() {
        const YamlNode* fallbacks = root_.find("fallbacks");
        if (fallbacks == nullptr) {
            return;
        }
        if (!fallbacks->isMapping()) {
            err("validate.wrong_node_kind", fallbacks->span, "\"fallbacks\" must be a mapping");
            return;
        }
        for (const auto& entry : fallbacks->entries) {
            if (!isIdentifier(entry.key)) {
                err("validate.bad_identifier", entry.key_span,
                    "fallback name \"" + entry.key + "\" must be an identifier");
                continue;
            }
            if (!entry.value.isMapping()) {
                err("validate.wrong_node_kind", entry.value.span, "fallback declaration must be a mapping");
                continue;
            }
            checkKnownKeys(entry.value, {"behavior", "value"}, "fallback \"" + entry.key + "\"");
            const YamlNode* behavior = entry.value.find("behavior");
            const YamlNode* value = entry.value.find("value");
            FallbackDecl decl;
            decl.span = entry.key_span;
            if ((behavior != nullptr) == (value != nullptr)) {
                err("validate.missing_field", entry.value.span,
                    "fallback \"" + entry.key + "\" must declare exactly one of \"behavior\" or \"value\"");
                continue;
            }
            if (behavior != nullptr) {
                if (!behavior->isPlainScalar() || behavior->scalar != "output_unavailable") {
                    err("validate.bad_enum_value", behavior->span,
                        "fallback behavior must be one of: output_unavailable");
                    continue;
                }
                decl.is_behavior = true;
                decl.behavior = behavior->scalar;
            } else {
                decl.value = value;
            }
            fallbacks_.emplace(entry.key, std::move(decl));
        }
    }

    // --- inputs ---------------------------------------------------------------------

    void validateInputs() {
        const YamlNode* inputs = root_.find("inputs");
        if (inputs == nullptr) {
            return;
        }
        if (!inputs->isMapping()) {
            err("validate.wrong_node_kind", inputs->span, "\"inputs\" must be a mapping");
            return;
        }
        static const std::vector<std::string> kMetadataKeys = {
            "frame", "timestamp", "clock", "calibration", "depth", "coordinate", "synchronization_group"};
        for (const auto& entry : inputs->entries) {
            if (!isIdentifier(entry.key)) {
                err("validate.bad_identifier", entry.key_span,
                    "input name \"" + entry.key + "\" must be an identifier");
                continue;
            }
            if (!entry.value.isMapping()) {
                err("validate.wrong_node_kind", entry.value.span, "input declaration must be a mapping");
                continue;
            }
            checkKnownKeys(entry.value, {"type", "required", "update", "metadata", "constraints", "fallback"},
                           "input \"" + entry.key + "\"");
            InputDecl decl;
            decl.span = entry.key_span;
            const YamlNode* type_node = entry.value.find("type");
            if (type_node == nullptr || !type_node->isPlainScalar()) {
                err("validate.missing_field", entry.value.span,
                    "input \"" + entry.key + "\" requires a \"type\"");
                continue;
            }
            decl.type = types_.parse(type_node->scalar, type_node->span, diags_);
            if (decl.type == nullptr) {
                continue;
            }
            const YamlNode* required_node = entry.value.find("required");
            if (required_node == nullptr || !parseBoolNode(*required_node, decl.required)) {
                err("validate.missing_field", entry.value.span,
                    "input \"" + entry.key + "\" requires an explicit boolean \"required\"");
            }
            const YamlNode* update_node = entry.value.find("update");
            if (update_node == nullptr || !update_node->isPlainScalar() ||
                (update_node->scalar != "per_frame" && update_node->scalar != "on_change" &&
                 update_node->scalar != "static")) {
                err("validate.bad_enum_value", update_node ? update_node->span : entry.value.span,
                    "input \"" + entry.key + "\" requires update: per_frame, on_change, or static");
            } else {
                decl.update = update_node->scalar;
            }
            if (const YamlNode* metadata_node = entry.value.find("metadata")) {
                if (!metadata_node->isMapping()) {
                    err("validate.wrong_node_kind", metadata_node->span, "input metadata must be a mapping");
                } else {
                    for (const auto& meta : metadata_node->entries) {
                        if (std::find(kMetadataKeys.begin(), kMetadataKeys.end(), meta.key) ==
                            kMetadataKeys.end()) {
                            err("validate.unknown_metadata_key", meta.key_span,
                                "unknown metadata key \"" + meta.key + "\"", {{"key", meta.key}});
                            continue;
                        }
                        if (!meta.value.isPlainScalar()) {
                            err("validate.bad_metadata_value", meta.value.span,
                                "metadata value must be a plain scalar");
                            continue;
                        }
                        const std::string& value = meta.value.scalar;
                        if (meta.key == "calibration" || meta.key == "synchronization_group") {
                            if (!isIdentifier(value)) {
                                err("validate.bad_metadata_value", meta.value.span,
                                    "metadata \"" + meta.key + "\" must name an identifier");
                                continue;
                            }
                        } else if (value != "required") {
                            err("validate.bad_metadata_value", meta.value.span,
                                "metadata \"" + meta.key + "\" must be the keyword \"required\"");
                            continue;
                        }
                        decl.metadata.emplace_back(meta.key, value);
                    }
                    std::sort(decl.metadata.begin(), decl.metadata.end());
                }
            }
            if (const YamlNode* constraints_node = entry.value.find("constraints")) {
                decl.has_constraints =
                    validateConstraints(decl.type, *constraints_node, decl.constraints, decl.max_utf8_bytes);
            }
            if (const YamlNode* fallback_node = entry.value.find("fallback")) {
                if (!fallback_node->isPlainScalar() || !isIdentifier(fallback_node->scalar)) {
                    err("validate.bad_reference", fallback_node->span,
                        "input fallback must name a declared fallback");
                } else {
                    decl.fallback = fallback_node->scalar;
                    decl.fallback_span = fallback_node->span;
                }
            }
            inputs_.emplace(entry.key, std::move(decl));
        }

        // Cross-references that need every input declared first.
        for (auto& [name, decl] : inputs_) {
            for (const auto& [key, value] : decl.metadata) {
                if (key != "calibration") {
                    continue;
                }
                auto target = inputs_.find(value);
                if (target == inputs_.end()) {
                    err("validate.unknown_reference", decl.span,
                        "input \"" + name + "\" metadata calibration references unknown input \"" + value +
                            "\"",
                        {{"reference", value}});
                    continue;
                }
                if (target->second.type == nullptr || target->second.type->kind != TypeKind::kCalibration) {
                    err("validate.type_mismatch", decl.span,
                        "metadata calibration must reference an input of type calibration");
                    continue;
                }
                target->second.used = true;
            }
            if (decl.fallback.empty()) {
                continue;
            }
            auto fallback_it = fallbacks_.find(decl.fallback);
            if (fallback_it == fallbacks_.end()) {
                err("validate.unknown_reference", decl.fallback_span,
                    "fallback \"" + decl.fallback + "\" is not declared", {{"reference", decl.fallback}});
                continue;
            }
            FallbackDecl& fallback = fallback_it->second;
            fallback.used = true;
            if (fallback.is_behavior || fallback.value == nullptr) {
                continue;
            }
            if (isPlainNull(*fallback.value)) {
                if (decl.required) {
                    err("validate.type_mismatch", fallback.value->span,
                        "null fallback value is only valid for optional inputs (input \"" + name +
                            "\" is required)");
                } else if (!fallback.has_normalized) {
                    fallback.normalized = JsonValue::makeNull();
                    fallback.has_normalized = true;
                }
                continue;
            }
            JsonValue normalized;
            if (checkLiteral(*fallback.value, *decl.type, normalized)) {
                enforceStringBytes(normalized, decl.max_utf8_bytes, fallback.value->span,
                                   "fallback \"" + decl.fallback + "\" for input \"" + name + "\"");
                if (!fallback.has_normalized) {
                    fallback.normalized = std::move(normalized);
                    fallback.has_normalized = true;
                }
            }
        }
    }

    // --- resources -------------------------------------------------------------------

    void validateResources() {
        const YamlNode* resources = root_.find("resources");
        if (resources == nullptr) {
            return;
        }
        if (!resources->isMapping()) {
            err("validate.wrong_node_kind", resources->span, "\"resources\" must be a mapping");
            return;
        }
        for (const auto& entry : resources->entries) {
            if (!isIdentifier(entry.key)) {
                err("validate.bad_identifier", entry.key_span,
                    "resource name \"" + entry.key + "\" must be an identifier");
                continue;
            }
            if (!entry.value.isMapping()) {
                err("validate.wrong_node_kind", entry.value.span, "resource declaration must be a mapping");
                continue;
            }
            const YamlNode* type_node = entry.value.find("type");
            if (type_node == nullptr || !type_node->isPlainScalar()) {
                err("validate.missing_field", entry.value.span,
                    "resource \"" + entry.key + "\" requires a \"type\"");
                continue;
            }
            ResourceDecl decl;
            decl.span = entry.key_span;
            decl.type = type_node->scalar;
            if (decl.type == "font") {
                checkKnownKeys(entry.value, {"type", "uri", "glyph_capacity"},
                               "resource \"" + entry.key + "\"");
                const YamlNode* capacity_node = entry.value.find("glyph_capacity");
                if (capacity_node == nullptr || !parseU64Node(*capacity_node, decl.glyph_capacity) ||
                    decl.glyph_capacity == 0) {
                    err("validate.missing_field", entry.value.span,
                        "font resource \"" + entry.key + "\" requires a positive glyph_capacity");
                } else if (decl.glyph_capacity > 65536) {
                    err("validate.capacity_overflow", capacity_node->span,
                        "glyph_capacity " + std::to_string(decl.glyph_capacity) + " exceeds the limit of 65536",
                        {{"capacity", std::to_string(decl.glyph_capacity)}, {"limit", "65536"}});
                }
            } else if (decl.type == "texture") {
                checkKnownKeys(entry.value, {"type", "uri"}, "resource \"" + entry.key + "\"");
            } else {
                err("validate.unknown_type", type_node->span,
                    "unsupported resource type \"" + decl.type + "\" (supported: font, texture)");
                continue;
            }
            const YamlNode* uri_node = entry.value.find("uri");
            if (uri_node == nullptr || !uri_node->isScalar()) {
                err("validate.missing_field", entry.value.span,
                    "resource \"" + entry.key + "\" requires a \"uri\"");
            } else {
                decl.uri = uri_node->scalar;
                if (decl.uri.rfind("builtin://", 0) != 0) {
                    err("validate.uri_scheme_forbidden", uri_node->span,
                        "asset URI scheme is not on the allowlist (allowed: builtin://)",
                        {{"uri", decl.uri}});
                }
            }
            resources_.emplace(entry.key, std::move(decl));
        }
    }

    // --- nodes -----------------------------------------------------------------------

    void collectNodes() {
        const YamlNode* nodes = root_.find("nodes");
        if (nodes == nullptr) {
            return;
        }
        if (!nodes->isSequence()) {
            err("validate.wrong_node_kind", nodes->span, "\"nodes\" must be a sequence of node declarations");
            return;
        }
        for (const YamlNode& element : nodes->elements) {
            if (!element.isMapping()) {
                err("validate.wrong_node_kind", element.span, "each node declaration must be a mapping");
                continue;
            }
            NodeDecl node;
            node.yaml = &element;
            node.span = element.span;
            const YamlNode* id_node = element.find("id");
            if (id_node == nullptr || !id_node->isPlainScalar() || !isIdentifier(id_node->scalar)) {
                err("validate.bad_identifier", element.span, "node requires an identifier \"id\"");
                continue;
            }
            node.id = id_node->scalar;
            node.span = id_node->span;
            const YamlNode* type_node = element.find("type");
            if (type_node == nullptr || !type_node->isPlainScalar()) {
                err("validate.missing_field", element.span,
                    "node \"" + node.id + "\" requires a \"type\"");
            } else {
                node.spec = registry_.find(type_node->scalar);
                if (node.spec == nullptr) {
                    err("validate.unknown_node_type", type_node->span,
                        "unknown node type \"" + type_node->scalar + "\"", {{"type", type_node->scalar}});
                }
            }
            if (node_index_.count(node.id) != 0) {
                err("validate.duplicate_id", node.span, "duplicate node id \"" + node.id + "\"",
                    {{"id", node.id}});
                continue;
            }
            node_index_.emplace(node.id, nodes_.size());
            nodes_.push_back(std::move(node));
        }
    }

    TypePtr resolveRef(const std::string& text, Span span, const std::string& consumer_id, RefContext context) {
        Ref ref;
        if (!parseRefText(text, ref)) {
            err("validate.bad_reference", span,
                "malformed reference \"" + text + "\" (expected $inputs.name, $params.name, $resources.name, "
                "$nodes.id.port, or $runtime.name)",
                {{"reference", text}});
            return nullptr;
        }
        const auto reject = [&]() -> TypePtr {
            err("validate.bad_reference", span,
                "reference \"" + text + "\" is not allowed here (allowed namespaces: " +
                    allowedNamespaces(context) + ")",
                {{"reference", text}});
            return nullptr;
        };
        switch (ref.space) {
            case Ref::Space::kInputs: {
                if (context != RefContext::kPort) {
                    return reject();
                }
                auto it = inputs_.find(ref.name);
                if (it == inputs_.end()) {
                    err("validate.unknown_reference", span, "unknown input \"" + ref.name + "\"",
                        {{"reference", text}});
                    return nullptr;
                }
                it->second.used = true;
                return it->second.type;
            }
            case Ref::Space::kParams: {
                if (context != RefContext::kParam) {
                    return reject();
                }
                auto it = params_.find(ref.name);
                if (it == params_.end()) {
                    err("validate.unknown_reference", span, "unknown parameter \"" + ref.name + "\"",
                        {{"reference", text}});
                    return nullptr;
                }
                it->second.used = true;
                return it->second.type;
            }
            case Ref::Space::kResources: {
                if (context != RefContext::kParam) {
                    return reject();
                }
                auto it = resources_.find(ref.name);
                if (it == resources_.end()) {
                    err("validate.unknown_reference", span, "unknown resource \"" + ref.name + "\"",
                        {{"reference", text}});
                    return nullptr;
                }
                it->second.used = true;
                return TypeTable::builtin(it->second.type == "font" ? TypeKind::kFont : TypeKind::kTexture);
            }
            case Ref::Space::kNodes: {
                if (context != RefContext::kPort && context != RefContext::kOutput) {
                    return reject();
                }
                auto it = node_index_.find(ref.name);
                if (it == node_index_.end()) {
                    err("validate.unknown_reference", span, "unknown node \"" + ref.name + "\"",
                        {{"reference", text}});
                    return nullptr;
                }
                const NodeDecl& target = nodes_[it->second];
                if (target.spec == nullptr) {
                    return nullptr;  // unknown node type already reported
                }
                const OutputSpec* output = target.spec->findOutput(ref.port);
                if (output == nullptr) {
                    err("validate.unknown_reference", span,
                        "node \"" + ref.name + "\" (" + target.spec->name + ") has no output \"" + ref.port +
                            "\"",
                        {{"reference", text}});
                    return nullptr;
                }
                if (context == RefContext::kPort && !consumer_id.empty()) {
                    edges_[consumer_id].insert(ref.name);
                }
                return TypeTable::builtin(output->type);
            }
            case Ref::Space::kRuntime: {
                if (context != RefContext::kOutput) {
                    return reject();
                }
                if (ref.name != "diagnostics") {
                    err("validate.unknown_reference", span,
                        "unknown runtime value \"" + ref.name + "\" (known: diagnostics)",
                        {{"reference", text}});
                    return nullptr;
                }
                return TypeTable::builtin(TypeKind::kDiagnosticSet);
            }
        }
        return nullptr;
    }

    bool matchesPort(const PortSpec& port, const Type& type, std::string& why) {
        switch (port.pattern) {
            case PortPatternKind::kExact:
                if (type.kind == port.exact_kind) {
                    return true;
                }
                why = "expected " + TypeTable::canonicalName(*TypeTable::builtin(port.exact_kind));
                return false;
            case PortPatternKind::kAnyImage:
                if (isImageKind(type.kind)) {
                    return true;
                }
                why = "expected an image.* type";
                return false;
            case PortPatternKind::kDetectionSequence: {
                if (type.kind != TypeKind::kSequence || type.element == nullptr ||
                    type.element->kind != TypeKind::kStruct) {
                    why = "expected a bounded sequence of a detection struct";
                    return false;
                }
                const StructDecl* decl = types_.findStruct(type.element->struct_name);
                if (decl == nullptr) {
                    why = "unknown struct element type";
                    return false;
                }
                static const std::vector<std::pair<std::string, TypeKind>> kRequired = {
                    {"bbox", TypeKind::kVec4f}, {"score", TypeKind::kF32}, {"label", TypeKind::kString}};
                for (const auto& [field_name, field_kind] : kRequired) {
                    bool found = false;
                    for (const StructField& field : decl->fields) {
                        if (field.name == field_name && field.type != nullptr &&
                            field.type->kind == field_kind) {
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        why = "detection struct requires field \"" + field_name + "\" of type " +
                              TypeTable::canonicalName(*TypeTable::builtin(field_kind));
                        return false;
                    }
                }
                return true;
            }
            case PortPatternKind::kLayerList:
                if (type.kind == TypeKind::kLayer) {
                    return true;
                }
                why = "expected layer";
                return false;
        }
        return false;
    }

    void checkPortBinding(NodeDecl& node, const PortSpec& port, const YamlNode& value) {
        PortBinding binding;
        binding.port = port.name;
        if (port.pattern == PortPatternKind::kLayerList) {
            binding.is_list = true;
            if (!value.isSequence() || value.elements.empty()) {
                err("validate.type_mismatch", value.span,
                    "port \"" + port.name + "\" expects a non-empty list of layer references");
                return;
            }
            if (value.elements.size() > kMaxCompositeLayers) {
                err("validate.capacity_overflow", value.span,
                    "port \"" + port.name + "\" accepts at most " + std::to_string(kMaxCompositeLayers) +
                        " layers",
                    {{"count", std::to_string(value.elements.size())},
                     {"limit", std::to_string(kMaxCompositeLayers)}});
                return;
            }
            for (const YamlNode& element : value.elements) {
                if (!element.isPlainScalar()) {
                    err("validate.type_mismatch", element.span,
                        "port \"" + port.name + "\" expects layer references");
                    continue;
                }
                TypePtr type = resolveRef(element.scalar, element.span, node.id, RefContext::kPort);
                if (type == nullptr) {
                    continue;
                }
                std::string why;
                if (!matchesPort(port, *type, why)) {
                    err("validate.type_mismatch", element.span,
                        "port \"" + port.name + "\": " + why + ", got " + TypeTable::canonicalName(*type));
                    continue;
                }
                binding.refs.push_back(element.scalar);
            }
            node.inputs.push_back(std::move(binding));
            return;
        }
        if (!value.isPlainScalar() || value.scalar.empty() || value.scalar[0] != '$') {
            err("validate.type_mismatch", value.span,
                "port \"" + port.name + "\" expects a typed reference such as $inputs.name");
            return;
        }
        TypePtr type = resolveRef(value.scalar, value.span, node.id, RefContext::kPort);
        if (type == nullptr) {
            return;
        }
        std::string why;
        if (!matchesPort(port, *type, why)) {
            err("validate.type_mismatch", value.span,
                "port \"" + port.name + "\": " + why + ", got " + TypeTable::canonicalName(*type),
                {{"actual", TypeTable::canonicalName(*type)}});
            return;
        }
        binding.refs.push_back(value.scalar);
        node.inputs.push_back(std::move(binding));
    }

    bool checkParamValue(const NodeDecl& node, const ParamSpec& spec, const YamlNode& value, JsonValue& out) {
        if (value.isPlainScalar() && !value.scalar.empty() && value.scalar[0] == '$') {
            TypePtr type = resolveRef(value.scalar, value.span, node.id, RefContext::kParam);
            if (type == nullptr) {
                return false;
            }
            bool matches = false;
            switch (spec.kind) {
                case ParamKind::kBool: matches = type->kind == TypeKind::kBool; break;
                case ParamKind::kU32: matches = type->kind == TypeKind::kU32; break;
                case ParamKind::kF32: matches = type->kind == TypeKind::kF32; break;
                case ParamKind::kVec2f: matches = type->kind == TypeKind::kVec2f; break;
                case ParamKind::kVec4f: matches = type->kind == TypeKind::kVec4f; break;
                case ParamKind::kString: matches = type->kind == TypeKind::kString; break;
                case ParamKind::kFontResource: matches = type->kind == TypeKind::kFont; break;
                case ParamKind::kEnum:
                    err("validate.type_mismatch", value.span,
                        "parameter \"" + spec.name + "\" is an enum and requires a literal value");
                    return false;
            }
            if (!matches) {
                err("validate.type_mismatch", value.span,
                    "parameter \"" + spec.name + "\" reference has type " + TypeTable::canonicalName(*type) +
                        ", which does not match the declared parameter kind");
                return false;
            }
            JsonValue ref_object = JsonValue::makeObject();
            ref_object.set("ref", JsonValue::makeString(value.scalar));
            out = std::move(ref_object);
            return true;
        }
        switch (spec.kind) {
            case ParamKind::kBool:
                return checkLiteral(value, *TypeTable::builtin(TypeKind::kBool), out);
            case ParamKind::kU32:
                return checkLiteral(value, *TypeTable::builtin(TypeKind::kU32), out);
            case ParamKind::kF32:
                return checkLiteral(value, *TypeTable::builtin(TypeKind::kF32), out);
            case ParamKind::kVec2f:
                return checkLiteral(value, *TypeTable::builtin(TypeKind::kVec2f), out);
            case ParamKind::kVec4f:
                return checkLiteral(value, *TypeTable::builtin(TypeKind::kVec4f), out);
            case ParamKind::kString:
                return checkLiteral(value, *TypeTable::builtin(TypeKind::kString), out);
            case ParamKind::kEnum: {
                if (!value.isPlainScalar() ||
                    std::find(spec.enum_values.begin(), spec.enum_values.end(), value.scalar) ==
                        spec.enum_values.end()) {
                    std::string valid;
                    for (const std::string& candidate : spec.enum_values) {
                        if (!valid.empty()) {
                            valid += ", ";
                        }
                        valid += candidate;
                    }
                    err("validate.bad_enum_value", value.span,
                        "parameter \"" + spec.name + "\" must be one of: " + valid);
                    return false;
                }
                out = JsonValue::makeString(value.scalar);
                return true;
            }
            case ParamKind::kFontResource:
                err("validate.type_mismatch", value.span,
                    "parameter \"" + spec.name + "\" requires a $resources reference to a font");
                return false;
        }
        return false;
    }

    void resolveNodes() {
        for (NodeDecl& node : nodes_) {
            if (node.spec == nullptr || node.yaml == nullptr) {
                continue;
            }
            const YamlNode& body = *node.yaml;
            checkKnownKeys(body, {"id", "type", "inputs", "params", "bounds"},
                           "node \"" + node.id + "\"");
            const YamlNode* inputs = body.find("inputs");
            if (inputs != nullptr && !inputs->isMapping()) {
                err("validate.wrong_node_kind", inputs->span, "node inputs must be a mapping");
                inputs = nullptr;
            }
            for (const PortSpec& port : node.spec->inputs) {
                const YamlNode* value = inputs != nullptr ? inputs->find(port.name) : nullptr;
                if (value == nullptr) {
                    if (port.required) {
                        err("validate.missing_port", node.span,
                            "node \"" + node.id + "\" is missing required port \"" + port.name + "\"",
                            {{"port", port.name}});
                    }
                    continue;
                }
                checkPortBinding(node, port, *value);
            }
            if (inputs != nullptr) {
                for (const auto& entry : inputs->entries) {
                    if (node.spec->findInput(entry.key) == nullptr) {
                        err("validate.unknown_port", entry.key_span,
                            "node type \"" + node.spec->name + "\" has no port \"" + entry.key + "\"",
                            {{"port", entry.key}});
                    }
                }
            }
            const YamlNode* params = body.find("params");
            if (params != nullptr && !params->isMapping()) {
                err("validate.wrong_node_kind", params->span, "node params must be a mapping");
                params = nullptr;
            }
            if (params != nullptr) {
                for (const auto& entry : params->entries) {
                    if (node.spec->findParam(entry.key) == nullptr) {
                        err("validate.unknown_param", entry.key_span,
                            "node type \"" + node.spec->name + "\" has no parameter \"" + entry.key + "\"",
                            {{"param", entry.key}});
                    }
                }
            }
            for (const ParamSpec& spec : node.spec->params) {
                const YamlNode* value = params != nullptr ? params->find(spec.name) : nullptr;
                if (value == nullptr) {
                    if (spec.required) {
                        err("validate.missing_param", node.span,
                            "node \"" + node.id + "\" is missing required parameter \"" + spec.name + "\"",
                            {{"param", spec.name}});
                    } else {
                        node.params.emplace_back(spec.name, spec.default_value);
                    }
                    continue;
                }
                JsonValue resolved;
                if (checkParamValue(node, spec, *value, resolved)) {
                    node.params.emplace_back(spec.name, std::move(resolved));
                }
            }
            resolveNodeBounds(node, body.find("bounds"));
        }
    }

    void resolveNodeBounds(NodeDecl& node, const YamlNode* bounds) {
        const BoundsSpec& spec = node.spec->bounds;
        if (bounds == nullptr) {
            if (spec.required) {
                err("validate.missing_bounds", node.span,
                    "node \"" + node.id + "\" (" + node.spec->name +
                        ") renders dynamic instance counts and must declare bounds {" + spec.count_key +
                        ", overflow}",
                    {{"required_key", spec.count_key}});
            }
            return;
        }
        if (!bounds->isMapping()) {
            err("validate.wrong_node_kind", bounds->span, "node bounds must be a mapping");
            return;
        }
        if (spec.count_key.empty()) {
            err("validate.unknown_field", bounds->span,
                "node type \"" + node.spec->name + "\" does not accept bounds");
            return;
        }
        for (const auto& entry : bounds->entries) {
            if (entry.key != spec.count_key && entry.key != "overflow") {
                err("validate.unknown_bounds_key", entry.key_span,
                    "unknown bounds key \"" + entry.key + "\" (expected " + spec.count_key + ", overflow)",
                    {{"key", entry.key}});
            }
        }
        const YamlNode* count = bounds->find(spec.count_key);
        if (count == nullptr || !parseU64Node(*count, node.bound_count) || node.bound_count == 0) {
            err("validate.missing_field", bounds->span,
                "bounds require a positive integer \"" + spec.count_key + "\"");
        } else if (node.bound_count > spec.max_count) {
            err("validate.capacity_overflow", count->span,
                spec.count_key + " " + std::to_string(node.bound_count) +
                    " exceeds the registry limit of " + std::to_string(spec.max_count) + " for node type " +
                    node.spec->name,
                {{"capacity", std::to_string(node.bound_count)},
                 {"limit", std::to_string(spec.max_count)}});
        }
        const YamlNode* overflow = bounds->find("overflow");
        if (overflow == nullptr || !overflow->isPlainScalar() ||
            std::find(spec.overflow_rules.begin(), spec.overflow_rules.end(), overflow->scalar) ==
                spec.overflow_rules.end()) {
            std::string valid;
            for (const std::string& rule : spec.overflow_rules) {
                if (!valid.empty()) {
                    valid += ", ";
                }
                valid += rule;
            }
            err("validate.bad_overflow_rule", overflow != nullptr ? overflow->span : bounds->span,
                "bounds require a deterministic overflow rule (one of: " + valid + ")");
            return;
        }
        node.overflow_rule = overflow->scalar;
        node.has_bounds = true;
    }

    // --- outputs ---------------------------------------------------------------------

    void validateOutputs() {
        const YamlNode* outputs = root_.find("outputs");
        if (outputs == nullptr) {
            return;
        }
        if (!outputs->isMapping()) {
            err("validate.wrong_node_kind", outputs->span, "\"outputs\" must be a mapping");
            return;
        }
        for (const auto& entry : outputs->entries) {
            if (!isIdentifier(entry.key)) {
                err("validate.bad_identifier", entry.key_span,
                    "output name \"" + entry.key + "\" must be an identifier");
                continue;
            }
            if (!entry.value.isMapping()) {
                err("validate.wrong_node_kind", entry.value.span, "output declaration must be a mapping");
                continue;
            }
            checkKnownKeys(entry.value, {"type", "source"}, "output \"" + entry.key + "\"");
            OutputDecl decl;
            decl.span = entry.key_span;
            const YamlNode* type_node = entry.value.find("type");
            if (type_node == nullptr || !type_node->isPlainScalar()) {
                err("validate.missing_field", entry.value.span,
                    "output \"" + entry.key + "\" requires a \"type\"");
                continue;
            }
            decl.type = types_.parse(type_node->scalar, type_node->span, diags_);
            const YamlNode* source_node = entry.value.find("source");
            if (source_node == nullptr || !source_node->isPlainScalar()) {
                err("validate.missing_field", entry.value.span,
                    "output \"" + entry.key + "\" requires a \"source\" reference");
                continue;
            }
            decl.source = source_node->scalar;
            TypePtr produced = resolveRef(decl.source, source_node->span, "", RefContext::kOutput);
            if (produced != nullptr && decl.type != nullptr &&
                !TypeTable::equals(*produced, *decl.type)) {
                err("validate.type_mismatch", source_node->span,
                    "output \"" + entry.key + "\" declares type " + TypeTable::canonicalName(*decl.type) +
                        " but the source produces " + TypeTable::canonicalName(*produced),
                    {{"declared", TypeTable::canonicalName(*decl.type)},
                     {"produced", TypeTable::canonicalName(*produced)}});
            }
            outputs_.emplace(entry.key, std::move(decl));
        }
        if (outputs_.empty() && outputs->isMapping() && outputs->entries.empty()) {
            err("validate.missing_field", outputs->span, "a scene must expose at least one output");
        }
    }

    // --- budgets ---------------------------------------------------------------------

    void validateBudgets() {
        const YamlNode* budgets = root_.find("budgets");
        if (budgets == nullptr) {
            return;
        }
        if (!budgets->isMapping()) {
            err("validate.wrong_node_kind", budgets->span, "\"budgets\" must be a mapping");
            return;
        }
        struct BudgetSpec {
            const char* key;
            uint64_t min;
            uint64_t max;
        };
        static const BudgetSpec kBudgetSpecs[] = {
            {"max_width", 1, 16384},
            {"max_height", 1, 16384},
            {"max_gpu_bytes", 1, 1ull << 40},
            {"max_upload_bytes_per_frame", 1, 1ull << 32},
            {"max_frames_in_flight", 1, 8},
        };
        std::vector<std::string> allowed;
        for (const BudgetSpec& spec : kBudgetSpecs) {
            allowed.push_back(spec.key);
        }
        checkKnownKeys(*budgets, allowed, "budgets");
        for (const BudgetSpec& spec : kBudgetSpecs) {
            const YamlNode* value = budgets->find(spec.key);
            if (value == nullptr) {
                err("validate.missing_field", budgets->span,
                    std::string("budgets.") + spec.key + " is required");
                continue;
            }
            uint64_t parsed = 0;
            if (!parseU64Node(*value, parsed) || parsed < spec.min || parsed > spec.max) {
                err("validate.value_out_of_range", value->span,
                    std::string("budgets.") + spec.key + " must be an integer in " +
                        std::to_string(spec.min) + ".." + std::to_string(spec.max));
                continue;
            }
            budgets_[spec.key] = parsed;
        }
    }

    // --- extensions ------------------------------------------------------------------

    void validateExtensions() {
        extensions_ = root_.find("extensions");
        if (extensions_ == nullptr) {
            return;
        }
        if (!extensions_->isMapping()) {
            err("validate.wrong_node_kind", extensions_->span, "\"extensions\" must be a mapping");
            extensions_ = nullptr;
            return;
        }
        for (const auto& entry : extensions_->entries) {
            if (entry.key.find('.') == std::string::npos) {
                err("validate.extension_namespace", entry.key_span,
                    "extension keys must be namespaced (e.g. \"vendor.feature\")", {{"key", entry.key}});
            }
        }
    }

    // --- graph ----------------------------------------------------------------------

    void detectCyclesAndOrder() {
        std::map<std::string, size_t> indegree;
        std::map<std::string, std::vector<std::string>> consumers;
        for (const auto& [id, index] : node_index_) {
            indegree[id] = 0;
        }
        for (const auto& [consumer, deps] : edges_) {
            for (const std::string& dep : deps) {
                if (indegree.count(consumer) == 0 || indegree.count(dep) == 0) {
                    continue;
                }
                ++indegree[consumer];
                consumers[dep].push_back(consumer);
            }
        }
        for (auto& [id, list] : consumers) {
            std::sort(list.begin(), list.end());
        }
        std::set<std::string> ready;
        for (const auto& [id, degree] : indegree) {
            if (degree == 0) {
                ready.insert(id);
            }
        }
        while (!ready.empty()) {
            const std::string id = *ready.begin();
            ready.erase(ready.begin());
            topo_order_.push_back(id);
            for (const std::string& consumer : consumers[id]) {
                if (--indegree[consumer] == 0) {
                    ready.insert(consumer);
                }
            }
        }
        if (topo_order_.size() == indegree.size()) {
            return;
        }
        // At least one combinational cycle exists; walk it for the message.
        std::set<std::string> resolved(topo_order_.begin(), topo_order_.end());
        std::string start;
        for (const auto& [id, degree] : indegree) {
            if (resolved.count(id) == 0) {
                start = id;
                break;
            }
        }
        std::vector<std::string> path;
        std::set<std::string> seen;
        std::string current = start;
        while (seen.count(current) == 0) {
            seen.insert(current);
            path.push_back(current);
            std::string next;
            for (const std::string& dep : edges_[current]) {
                if (resolved.count(dep) == 0) {
                    next = dep;
                    break;
                }
            }
            if (next.empty()) {
                break;
            }
            current = next;
        }
        std::string cycle;
        bool in_cycle = false;
        for (const std::string& id : path) {
            if (id == current) {
                in_cycle = true;
            }
            if (in_cycle) {
                cycle += id + " -> ";
            }
        }
        cycle += current;
        Span span = root_.span;
        auto it = node_index_.find(current);
        if (it != node_index_.end()) {
            span = nodes_[it->second].span;
        }
        err("validate.graph_cycle", span, "combinational cycle detected: " + cycle, {{"cycle", cycle}});
    }

    // --- unused declarations -----------------------------------------------------------

    void checkUnused() {
        for (const auto& [name, decl] : inputs_) {
            if (!decl.used) {
                info("validate.unused_input", decl.span,
                     "input \"" + name +
                         "\" is declared but not consumed by any node; the public binding contract is kept",
                     {{"input", name}});
            }
        }
        for (const auto& [name, decl] : params_) {
            if (!decl.used) {
                info("validate.unused_param", decl.span, "parameter \"" + name + "\" is never referenced",
                     {{"param", name}});
            }
        }
        for (const auto& [name, decl] : resources_) {
            if (!decl.used) {
                info("validate.unused_resource", decl.span, "resource \"" + name + "\" is never referenced",
                     {{"resource", name}});
            }
        }
        for (const auto& [name, decl] : fallbacks_) {
            if (!decl.used) {
                info("validate.unused_fallback", decl.span, "fallback \"" + name + "\" is never referenced",
                     {{"fallback", name}});
            }
        }
        for (const auto& [name, decl] : types_.structs()) {
            if (!decl.used) {
                info("validate.unused_type", decl.span, "type \"" + name + "\" is never referenced",
                     {{"type", name}});
            }
        }
    }

    // --- IR emission ---------------------------------------------------------------

    static JsonValue descriptiveJson(const YamlNode& node) {
        switch (node.kind) {
            case YamlNode::Kind::kNull:
                return JsonValue::makeNull();
            case YamlNode::Kind::kScalar:
                if (node.isPlainScalar() && node.scalar == "null") {
                    return JsonValue::makeNull();
                }
                return JsonValue::makeString(node.scalar);
            case YamlNode::Kind::kSequence: {
                JsonValue array = JsonValue::makeArray();
                for (const YamlNode& element : node.elements) {
                    array.append(descriptiveJson(element));
                }
                return array;
            }
            case YamlNode::Kind::kMapping: {
                std::vector<std::pair<std::string, const YamlNode*>> sorted;
                for (const auto& entry : node.entries) {
                    sorted.emplace_back(entry.key, &entry.value);
                }
                std::sort(sorted.begin(), sorted.end(),
                          [](const auto& a, const auto& b) { return a.first < b.first; });
                JsonValue object = JsonValue::makeObject();
                for (const auto& [key, value] : sorted) {
                    object.set(key, descriptiveJson(*value));
                }
                return object;
            }
        }
        return JsonValue::makeNull();
    }

    JsonValue buildIr() {
        JsonValue ir = JsonValue::makeObject();
        ir.set("schema", JsonValue::makeString(kSupportedSchemaIdentity));
        ir.set("kind", JsonValue::makeString("Scene"));

        JsonValue metadata = JsonValue::makeObject();
        metadata.set("name", JsonValue::makeString(scene_name_));
        for (const auto& [key, value] : metadata_extra_) {
            metadata.set(key, JsonValue::makeString(value));
        }
        ir.set("metadata", std::move(metadata));

        JsonValue params = JsonValue::makeArray();
        for (const auto& [name, decl] : params_) {
            JsonValue param = JsonValue::makeObject();
            param.set("name", JsonValue::makeString(name));
            param.set("type", JsonValue::makeString(TypeTable::canonicalName(*decl.type)));
            param.set("runtime_mutable", JsonValue::makeBool(decl.runtime_mutable));
            if (decl.has_default) {
                param.set("default", decl.default_value);
            }
            if (decl.has_constraints) {
                param.set("constraints", decl.constraints);
            }
            params.append(std::move(param));
        }
        ir.set("params", std::move(params));

        JsonValue types = JsonValue::makeArray();
        for (const auto& [name, decl] : types_.structs()) {
            JsonValue type = JsonValue::makeObject();
            type.set("name", JsonValue::makeString(name));
            JsonValue fields = JsonValue::makeArray();
            for (const StructField& field : decl.fields) {
                JsonValue field_value = JsonValue::makeObject();
                field_value.set("name", JsonValue::makeString(field.name));
                field_value.set("type",
                                JsonValue::makeString(field.type ? TypeTable::canonicalName(*field.type)
                                                                 : "invalid"));
                fields.append(std::move(field_value));
            }
            type.set("fields", std::move(fields));
            types.append(std::move(type));
        }
        ir.set("types", std::move(types));

        JsonValue inputs = JsonValue::makeArray();
        for (const auto& [name, decl] : inputs_) {
            JsonValue input = JsonValue::makeObject();
            input.set("name", JsonValue::makeString(name));
            input.set("type", JsonValue::makeString(TypeTable::canonicalName(*decl.type)));
            input.set("required", JsonValue::makeBool(decl.required));
            input.set("update", JsonValue::makeString(decl.update));
            if (!decl.metadata.empty()) {
                JsonValue metadata_value = JsonValue::makeObject();
                for (const auto& [key, value] : decl.metadata) {
                    metadata_value.set(key, JsonValue::makeString(value));
                }
                input.set("metadata", std::move(metadata_value));
            }
            if (decl.has_constraints) {
                input.set("constraints", decl.constraints);
            }
            if (!decl.fallback.empty()) {
                input.set("fallback", JsonValue::makeString(decl.fallback));
            }
            inputs.append(std::move(input));
        }
        ir.set("inputs", std::move(inputs));

        JsonValue resources = JsonValue::makeArray();
        for (const auto& [name, decl] : resources_) {
            JsonValue resource = JsonValue::makeObject();
            resource.set("name", JsonValue::makeString(name));
            resource.set("type", JsonValue::makeString(decl.type));
            resource.set("uri", JsonValue::makeString(decl.uri));
            if (decl.type == "font") {
                resource.set("glyph_capacity", JsonValue::makeUInt(decl.glyph_capacity));
            }
            resources.append(std::move(resource));
        }
        ir.set("resources", std::move(resources));

        JsonValue nodes = JsonValue::makeArray();
        for (const std::string& id : topo_order_) {
            const NodeDecl& node = nodes_[node_index_.at(id)];
            JsonValue node_value = JsonValue::makeObject();
            node_value.set("id", JsonValue::makeString(node.id));
            node_value.set("type", JsonValue::makeString(node.spec->name));
            std::vector<const PortBinding*> bindings;
            for (const PortBinding& binding : node.inputs) {
                bindings.push_back(&binding);
            }
            std::sort(bindings.begin(), bindings.end(),
                      [](const PortBinding* a, const PortBinding* b) { return a->port < b->port; });
            JsonValue inputs_value = JsonValue::makeArray();
            for (const PortBinding* binding : bindings) {
                JsonValue binding_value = JsonValue::makeObject();
                binding_value.set("port", JsonValue::makeString(binding->port));
                if (binding->is_list) {
                    JsonValue refs = JsonValue::makeArray();
                    for (const std::string& ref : binding->refs) {
                        JsonValue ref_object = JsonValue::makeObject();
                        ref_object.set("ref", JsonValue::makeString(ref));
                        refs.append(std::move(ref_object));
                    }
                    binding_value.set("source", std::move(refs));
                } else {
                    JsonValue ref_object = JsonValue::makeObject();
                    ref_object.set("ref", JsonValue::makeString(binding->refs.front()));
                    binding_value.set("source", std::move(ref_object));
                }
                inputs_value.append(std::move(binding_value));
            }
            node_value.set("inputs", std::move(inputs_value));
            std::vector<std::pair<std::string, const JsonValue*>> sorted_params;
            for (const auto& [param_name, param_value] : node.params) {
                sorted_params.emplace_back(param_name, &param_value);
            }
            std::sort(sorted_params.begin(), sorted_params.end(),
                      [](const auto& a, const auto& b) { return a.first < b.first; });
            JsonValue params_value = JsonValue::makeArray();
            for (const auto& [param_name, param_value] : sorted_params) {
                JsonValue param_object = JsonValue::makeObject();
                param_object.set("name", JsonValue::makeString(param_name));
                param_object.set("value", *param_value);
                params_value.append(std::move(param_object));
            }
            node_value.set("params", std::move(params_value));
            if (node.has_bounds) {
                JsonValue bounds_value = JsonValue::makeObject();
                bounds_value.set(node.spec->bounds.count_key, JsonValue::makeUInt(node.bound_count));
                bounds_value.set("overflow", JsonValue::makeString(node.overflow_rule));
                node_value.set("bounds", std::move(bounds_value));
            }
            nodes.append(std::move(node_value));
        }
        ir.set("nodes", std::move(nodes));

        JsonValue outputs = JsonValue::makeArray();
        for (const auto& [name, decl] : outputs_) {
            JsonValue output = JsonValue::makeObject();
            output.set("name", JsonValue::makeString(name));
            output.set("type",
                       JsonValue::makeString(decl.type ? TypeTable::canonicalName(*decl.type) : "invalid"));
            JsonValue ref_object = JsonValue::makeObject();
            ref_object.set("ref", JsonValue::makeString(decl.source));
            output.set("source", std::move(ref_object));
            outputs.append(std::move(output));
        }
        ir.set("outputs", std::move(outputs));

        JsonValue budgets = JsonValue::makeObject();
        for (const auto& [key, value] : budgets_) {
            budgets.set(key, JsonValue::makeUInt(value));
        }
        ir.set("budgets", std::move(budgets));

        JsonValue fallbacks = JsonValue::makeArray();
        for (const auto& [name, decl] : fallbacks_) {
            JsonValue fallback = JsonValue::makeObject();
            fallback.set("name", JsonValue::makeString(name));
            if (decl.is_behavior) {
                fallback.set("behavior", JsonValue::makeString(decl.behavior));
            } else if (decl.has_normalized) {
                fallback.set("value", decl.normalized);
            } else if (decl.value != nullptr) {
                fallback.set("value", descriptiveJson(*decl.value));
            }
            fallbacks.append(std::move(fallback));
        }
        ir.set("fallbacks", std::move(fallbacks));

        if (extensions_ != nullptr) {
            ir.set("extensions", descriptiveJson(*extensions_));
        }
        return ir;
    }

    // --- state ----------------------------------------------------------------------

    const YamlNode& root_;
    const NodeRegistry& registry_;
    DiagnosticList& diags_;
    TypeTable types_;
    std::string scene_name_;
    std::vector<std::pair<std::string, std::string>> metadata_extra_;
    std::map<std::string, ParamDecl> params_;
    std::map<std::string, InputDecl> inputs_;
    std::map<std::string, ResourceDecl> resources_;
    std::map<std::string, FallbackDecl> fallbacks_;
    std::vector<NodeDecl> nodes_;
    std::map<std::string, size_t> node_index_;
    std::map<std::string, std::set<std::string>> edges_;
    std::vector<std::string> topo_order_;
    std::map<std::string, OutputDecl> outputs_;
    std::map<std::string, uint64_t> budgets_;
    const YamlNode* extensions_ = nullptr;
};

}  // namespace

ValidationResult validateScene(const YamlNode& root, const NodeRegistry& registry,
                               DiagnosticList& diagnostics) {
    SceneValidator validator(root, registry, diagnostics);
    return validator.run();
}

}  // namespace fluent_scene
