#include "fluent_scene/json.hpp"

#include <cassert>
#include <charconv>
#include <cmath>
#include <cstdio>

namespace fluent_scene {

JsonValue JsonValue::makeBool(bool value) {
    JsonValue v;
    v.kind_ = Kind::kBool;
    v.bool_value_ = value;
    return v;
}

JsonValue JsonValue::makeInt(int64_t value) {
    JsonValue v;
    v.kind_ = Kind::kInt;
    v.int_value_ = value;
    return v;
}

JsonValue JsonValue::makeUInt(uint64_t value) {
    JsonValue v;
    v.kind_ = Kind::kUInt;
    v.uint_value_ = value;
    return v;
}

JsonValue JsonValue::makeFloat(double value) {
    assert(std::isfinite(value));
    JsonValue v;
    v.kind_ = Kind::kFloat;
    v.float_value_ = value;
    return v;
}

JsonValue JsonValue::makeString(std::string value) {
    JsonValue v;
    v.kind_ = Kind::kString;
    v.string_value_ = std::move(value);
    return v;
}

JsonValue JsonValue::makeArray() {
    JsonValue v;
    v.kind_ = Kind::kArray;
    return v;
}

JsonValue JsonValue::makeObject() {
    JsonValue v;
    v.kind_ = Kind::kObject;
    return v;
}

void JsonValue::append(JsonValue value) {
    assert(kind_ == Kind::kArray);
    elements_.push_back(std::move(value));
}

void JsonValue::set(std::string key, JsonValue value) {
    assert(kind_ == Kind::kObject);
    members_.emplace_back(std::move(key), std::move(value));
}

const JsonValue* JsonValue::find(const std::string& key) const {
    for (const auto& member : members_) {
        if (member.first == key) {
            return &member.second;
        }
    }
    return nullptr;
}

void appendJsonEscaped(std::string& out, const std::string& text) {
    for (unsigned char c : text) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out.push_back(static_cast<char>(c));
                }
        }
    }
}

std::string formatJsonNumber(double value) {
    char buf[64];
    auto result = std::to_chars(buf, buf + sizeof(buf), value);
    std::string text(buf, result.ptr);
    // Keep integral floats distinguishable from integers is unnecessary for the
    // canonical form; shortest round-trip output is used verbatim.
    return text;
}

void JsonValue::serializeTo(std::string& out, int depth) const {
    const auto indent = [&out](int levels) {
        for (int i = 0; i < levels; ++i) {
            out += "  ";
        }
    };
    switch (kind_) {
        case Kind::kNull:
            out += "null";
            break;
        case Kind::kBool:
            out += bool_value_ ? "true" : "false";
            break;
        case Kind::kInt: {
            char buf[32];
            auto result = std::to_chars(buf, buf + sizeof(buf), int_value_);
            out.append(buf, result.ptr);
            break;
        }
        case Kind::kUInt: {
            char buf[32];
            auto result = std::to_chars(buf, buf + sizeof(buf), uint_value_);
            out.append(buf, result.ptr);
            break;
        }
        case Kind::kFloat:
            out += formatJsonNumber(float_value_);
            break;
        case Kind::kString:
            out.push_back('"');
            appendJsonEscaped(out, string_value_);
            out.push_back('"');
            break;
        case Kind::kArray: {
            if (elements_.empty()) {
                out += "[]";
                break;
            }
            out += "[\n";
            for (size_t i = 0; i < elements_.size(); ++i) {
                indent(depth + 1);
                elements_[i].serializeTo(out, depth + 1);
                if (i + 1 < elements_.size()) {
                    out.push_back(',');
                }
                out.push_back('\n');
            }
            indent(depth);
            out.push_back(']');
            break;
        }
        case Kind::kObject: {
            if (members_.empty()) {
                out += "{}";
                break;
            }
            out += "{\n";
            for (size_t i = 0; i < members_.size(); ++i) {
                indent(depth + 1);
                out.push_back('"');
                appendJsonEscaped(out, members_[i].first);
                out += "\": ";
                members_[i].second.serializeTo(out, depth + 1);
                if (i + 1 < members_.size()) {
                    out.push_back(',');
                }
                out.push_back('\n');
            }
            indent(depth);
            out.push_back('}');
            break;
        }
    }
}

std::string JsonValue::serialize() const {
    std::string out;
    serializeTo(out, 0);
    out.push_back('\n');
    return out;
}

}  // namespace fluent_scene
