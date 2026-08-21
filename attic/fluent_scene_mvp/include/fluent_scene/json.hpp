#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace fluent_scene {

// Minimal JSON document model used for the canonical typed IR and structured
// diagnostic output. Object member order is preserved exactly as inserted, so
// callers own the canonical ordering. Serialization is deterministic:
// floats use shortest round-trip formatting (std::to_chars), and strings are
// escaped the same way on every run and platform.
class JsonValue {
public:
    enum class Kind { kNull, kBool, kInt, kUInt, kFloat, kString, kArray, kObject };

    JsonValue() = default;

    static JsonValue makeNull() { return JsonValue(); }
    static JsonValue makeBool(bool value);
    static JsonValue makeInt(int64_t value);
    static JsonValue makeUInt(uint64_t value);
    static JsonValue makeFloat(double value);  // must be finite
    static JsonValue makeString(std::string value);
    static JsonValue makeArray();
    static JsonValue makeObject();

    Kind kind() const { return kind_; }
    bool isNull() const { return kind_ == Kind::kNull; }
    bool isObject() const { return kind_ == Kind::kObject; }
    bool isArray() const { return kind_ == Kind::kArray; }

    bool boolValue() const { return bool_value_; }
    int64_t intValue() const { return int_value_; }
    uint64_t uintValue() const { return uint_value_; }
    double floatValue() const { return float_value_; }
    const std::string& stringValue() const { return string_value_; }

    std::vector<JsonValue>& elements() { return elements_; }
    const std::vector<JsonValue>& elements() const { return elements_; }
    std::vector<std::pair<std::string, JsonValue>>& members() { return members_; }
    const std::vector<std::pair<std::string, JsonValue>>& members() const { return members_; }

    void append(JsonValue value);                       // array
    void set(std::string key, JsonValue value);         // object (append; caller keeps order canonical)
    const JsonValue* find(const std::string& key) const;

    // Serializes with 2-space indentation and a trailing newline at top level.
    std::string serialize() const;

private:
    void serializeTo(std::string& out, int depth) const;

    Kind kind_ = Kind::kNull;
    bool bool_value_ = false;
    int64_t int_value_ = 0;
    uint64_t uint_value_ = 0;
    double float_value_ = 0.0;
    std::string string_value_;
    std::vector<JsonValue> elements_;
    std::vector<std::pair<std::string, JsonValue>> members_;
};

// Escapes `text` as a JSON string body (no surrounding quotes added).
void appendJsonEscaped(std::string& out, const std::string& text);

// Deterministic shortest round-trip formatting for a finite double.
std::string formatJsonNumber(double value);

}  // namespace fluent_scene
