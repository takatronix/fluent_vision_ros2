#include "fluent_stage/scene/yaml.hpp"

#include <algorithm>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace fluent_stage::scene {

const YamlNode* YamlNode::find(const std::string& key) const {
    for (const auto& entry : entries) {
        if (entry.key == key) {
            return &entry.value;
        }
    }
    return nullptr;
}

namespace {

constexpr size_t kNpos = std::string::npos;

struct Line {
    uint32_t number = 0;       // 1-based
    uint32_t indent = 0;       // number of leading spaces
    uint32_t content_col = 0;  // 1-based column of first content character
    std::string text;          // content without indent, comments, or trailing spaces
};

bool isSeqItem(const std::string& text) {
    return text == "-" || (text.size() >= 2 && text[0] == '-' && text[1] == ' ');
}

std::string rtrim(std::string s) {
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) {
        s.pop_back();
    }
    return s;
}

// Finds the ':' that terminates a mapping key (followed by space or EOL,
// outside quotes). Returns kNpos when the line cannot be a mapping entry.
size_t findKeyColon(const std::string& text) {
    char quote = '\0';
    for (size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        if (quote != '\0') {
            if (quote == '\'' && c == '\'') {
                if (i + 1 < text.size() && text[i + 1] == '\'') {
                    ++i;  // escaped quote
                } else {
                    quote = '\0';
                }
            } else if (quote == '"') {
                if (c == '\\') {
                    ++i;
                } else if (c == '"') {
                    quote = '\0';
                }
            }
            continue;
        }
        if (c == '"' || c == '\'') {
            quote = c;
            continue;
        }
        if (c == ':' && (i + 1 == text.size() || text[i + 1] == ' ')) {
            return i;
        }
    }
    return kNpos;
}

void encodeUtf8(uint32_t code_point, std::string& out) {
    if (code_point < 0x80u) {
        out.push_back(static_cast<char>(code_point));
    } else if (code_point < 0x800u) {
        out.push_back(static_cast<char>(0xc0u | (code_point >> 6)));
        out.push_back(static_cast<char>(0x80u | (code_point & 0x3fu)));
    } else if (code_point < 0x10000u) {
        out.push_back(static_cast<char>(0xe0u | (code_point >> 12)));
        out.push_back(static_cast<char>(0x80u | ((code_point >> 6) & 0x3fu)));
        out.push_back(static_cast<char>(0x80u | (code_point & 0x3fu)));
    } else {
        out.push_back(static_cast<char>(0xf0u | (code_point >> 18)));
        out.push_back(static_cast<char>(0x80u | ((code_point >> 12) & 0x3fu)));
        out.push_back(static_cast<char>(0x80u | ((code_point >> 6) & 0x3fu)));
        out.push_back(static_cast<char>(0x80u | (code_point & 0x3fu)));
    }
}

class Parser {
public:
    Parser(const std::string& source, DiagnosticList& diagnostics, const ParseLimits& limits)
        : diagnostics_(diagnostics), limits_(limits) {
        splitLines(source);
        joinFlowContinuations();
    }

    YamlNode parse() {
        if (fatal_) {
            return YamlNode{};
        }
        if (lines_.empty()) {
            error("parse.empty_document", Span{1, 1, 1, 1}, "document contains no content");
            return YamlNode{};
        }
        if (lines_.front().indent != 0) {
            error("parse.bad_indent", lineSpan(lines_.front()), "top-level content must start at column 1");
        }
        YamlNode root = parseBlockNode(lines_.front().indent, 0);
        while (cursor_ < lines_.size()) {
            error("parse.bad_indent", lineSpan(lines_[cursor_]),
                  "content does not align with any open block");
            ++cursor_;
        }
        return root;
    }

private:
    // --- input preparation -------------------------------------------------

    void splitLines(const std::string& source) {
        if (source.size() > limits_.max_input_bytes) {
            error("parse.input_too_large", Span{1, 1, 1, 1},
                  "input exceeds the configured maximum of " + std::to_string(limits_.max_input_bytes) + " bytes");
            fatal_ = true;
            return;
        }
        size_t pos = 0;
        // Strip a UTF-8 byte order mark if present.
        if (source.size() >= 3 && static_cast<unsigned char>(source[0]) == 0xef &&
            static_cast<unsigned char>(source[1]) == 0xbb && static_cast<unsigned char>(source[2]) == 0xbf) {
            pos = 3;
        }
        uint32_t line_no = 0;
        while (pos <= source.size()) {
            if (pos == source.size() && line_no > 0) {
                break;
            }
            size_t eol = source.find('\n', pos);
            std::string raw = source.substr(pos, eol == kNpos ? kNpos : eol - pos);
            pos = (eol == kNpos) ? source.size() + 1 : eol + 1;
            ++line_no;
            if (!raw.empty() && raw.back() == '\r') {
                raw.pop_back();
            }
            processRawLine(line_no, raw);
            if (eol == kNpos) {
                break;
            }
        }
    }

    void processRawLine(uint32_t number, const std::string& raw) {
        uint32_t indent = 0;
        size_t i = 0;
        while (i < raw.size() && (raw[i] == ' ' || raw[i] == '\t')) {
            if (raw[i] == '\t') {
                error("parse.tab_forbidden", Span{number, static_cast<uint32_t>(i + 1), number,
                                                 static_cast<uint32_t>(i + 1)},
                      "tab characters are forbidden in indentation");
                return;
            }
            ++indent;
            ++i;
        }
        std::string content = stripComment(raw.substr(i));
        content = rtrim(std::move(content));
        if (content.empty()) {
            return;
        }
        const Span span{number, indent + 1, number, static_cast<uint32_t>(indent + content.size())};
        if (indent == 0 && content[0] == '%') {
            error("parse.directive_forbidden", span, "YAML directives are forbidden");
            return;
        }
        if (content == "---" || content.rfind("--- ", 0) == 0 || content == "...") {
            error("parse.document_marker_forbidden", span,
                  "multi-document markers are forbidden; a file holds exactly one document");
            return;
        }
        Line line;
        line.number = number;
        line.indent = indent;
        line.content_col = indent + 1;
        line.text = std::move(content);
        lines_.push_back(std::move(line));
    }

    /// Net flow-collection depth of a line: '['/'{' minus ']'/'}' outside
    /// quoted scalars. Quoted scalars never span lines in this subset, so
    /// the count is per-line exact.
    static int netFlowDepth(const std::string& text) {
        int depth = 0;
        char quote = '\0';
        for (size_t i = 0; i < text.size(); ++i) {
            const char c = text[i];
            if (quote != '\0') {
                if (quote == '"' && c == '\\') {
                    ++i;
                } else if (c == quote) {
                    quote = '\0';
                }
                continue;
            }
            if (c == '"' || c == '\'') {
                quote = c;
            } else if (c == '[' || c == '{') {
                ++depth;
            } else if (c == ']' || c == '}') {
                --depth;
            }
        }
        return depth;
    }

    /// A flow collection may wrap across lines (`content: { boxes: { ...,\n
    /// color: [...] } }` — the §2 canonical layout). Lines whose flow
    /// brackets stay open absorb the following lines until balanced, so the
    /// rest of the parser only ever sees single-line flow values.
    void joinFlowContinuations() {
        std::vector<Line> joined;
        joined.reserve(lines_.size());
        for (size_t i = 0; i < lines_.size(); ++i) {
            Line line = std::move(lines_[i]);
            int depth = netFlowDepth(line.text);
            while (depth > 0 && i + 1 < lines_.size()) {
                const Line& next = lines_[i + 1];
                line.text += " " + next.text;
                depth += netFlowDepth(next.text);
                ++i;
            }
            joined.push_back(std::move(line));
        }
        lines_ = std::move(joined);
    }

    static std::string stripComment(const std::string& text) {
        char quote = '\0';
        for (size_t i = 0; i < text.size(); ++i) {
            const char c = text[i];
            if (quote != '\0') {
                if (quote == '\'' && c == '\'') {
                    if (i + 1 < text.size() && text[i + 1] == '\'') {
                        ++i;
                    } else {
                        quote = '\0';
                    }
                } else if (quote == '"') {
                    if (c == '\\') {
                        ++i;
                    } else if (c == '"') {
                        quote = '\0';
                    }
                }
                continue;
            }
            if (c == '"' || c == '\'') {
                quote = c;
                continue;
            }
            if (c == '#' && (i == 0 || text[i - 1] == ' ' || text[i - 1] == '\t')) {
                return text.substr(0, i);
            }
        }
        return text;
    }

    // --- helpers -----------------------------------------------------------

    void error(std::string code, Span span, std::string message,
               std::vector<std::pair<std::string, std::string>> context = {}) {
        diagnostics_.add(std::move(code), Severity::kError, Phase::kParse, span, std::move(message),
                         std::move(context));
    }

    static Span lineSpan(const Line& line) {
        return Span{line.number, line.content_col, line.number,
                    static_cast<uint32_t>(line.content_col + line.text.size() - 1)};
    }

    bool nodeBudgetOk(const Span& span) {
        if (fatal_) {
            return false;
        }
        if (++nodes_created_ > limits_.max_nodes) {
            error("parse.node_limit_exceeded", span,
                  "document exceeds the configured maximum of " + std::to_string(limits_.max_nodes) + " nodes");
            fatal_ = true;
            cursor_ = lines_.size();
            return false;
        }
        return true;
    }

    void skipDeeperLines(uint32_t indent) {
        while (cursor_ < lines_.size() && lines_[cursor_].indent > indent) {
            ++cursor_;
        }
    }

    std::string boundedScalar(std::string text, const Span& span) {
        if (text.size() > limits_.max_scalar_bytes) {
            error("parse.scalar_too_long", span,
                  "scalar exceeds the configured maximum of " + std::to_string(limits_.max_scalar_bytes) +
                      " bytes");
            text.resize(limits_.max_scalar_bytes);
        }
        return text;
    }

    // --- block structure ---------------------------------------------------

    YamlNode parseBlockNode(uint32_t indent, size_t depth) {
        YamlNode null_node;
        if (cursor_ >= lines_.size() || fatal_) {
            return null_node;
        }
        const Line& line = lines_[cursor_];
        null_node.span = lineSpan(line);
        if (depth > limits_.max_depth) {
            error("parse.depth_exceeded", lineSpan(line),
                  "nesting exceeds the configured maximum depth of " + std::to_string(limits_.max_depth));
            ++cursor_;
            skipDeeperLines(indent);
            return null_node;
        }
        if (isSeqItem(line.text)) {
            return parseBlockSequence(indent, depth);
        }
        if (findKeyColon(line.text) != kNpos) {
            return parseBlockMapping(indent, depth);
        }
        // Single-line scalar node.
        YamlNode node = parseInlineValue(line.text, line.number, line.content_col, depth);
        ++cursor_;
        if (cursor_ < lines_.size() && lines_[cursor_].indent > indent) {
            error("parse.bad_indent", lineSpan(lines_[cursor_]),
                  "unexpected indented content after a scalar value (multi-line plain scalars are forbidden)");
            skipDeeperLines(indent);
        }
        return node;
    }

    YamlNode parseBlockMapping(uint32_t indent, size_t depth) {
        YamlNode node;
        node.kind = YamlNode::Kind::kMapping;
        node.span = lineSpan(lines_[cursor_]);
        if (!nodeBudgetOk(node.span)) {
            return node;
        }
        std::unordered_set<std::string> seen_keys;
        while (cursor_ < lines_.size() && !fatal_) {
            Line& line = lines_[cursor_];
            if (line.indent != indent) {
                if (line.indent > indent) {
                    error("parse.bad_indent", lineSpan(line),
                          "line is indented more deeply than its enclosing block allows");
                    ++cursor_;
                    continue;
                }
                break;
            }
            if (isSeqItem(line.text)) {
                break;
            }
            std::string key;
            YamlNode::ScalarStyle key_style = YamlNode::ScalarStyle::kPlain;
            size_t colon = 0;
            Span key_span{line.number, line.content_col, line.number, line.content_col};
            if (!splitKey(line, key, key_style, colon, key_span)) {
                ++cursor_;
                continue;
            }
            const bool duplicate = !seen_keys.insert(key).second;
            if (duplicate) {
                error("parse.duplicate_key", key_span, "duplicate mapping key \"" + key + "\"",
                      {{"key", key}});
            }
            // Parse the value (also for duplicates, to keep the cursor consistent).
            std::string rest = line.text.substr(colon + 1);
            size_t rest_offset = colon + 1;
            while (rest_offset < line.text.size() && line.text[rest_offset] == ' ') {
                ++rest_offset;
            }
            rest = line.text.substr(rest_offset);
            YamlNode value;
            if (!rest.empty()) {
                value = parseInlineValue(rest, line.number,
                                         static_cast<uint32_t>(line.content_col + rest_offset), depth + 1);
                ++cursor_;
                if (cursor_ < lines_.size() && lines_[cursor_].indent > indent) {
                    error("parse.bad_indent", lineSpan(lines_[cursor_]),
                          "unexpected indented block after an inline value");
                    skipDeeperLines(indent);
                }
            } else {
                ++cursor_;
                if (cursor_ < lines_.size() && lines_[cursor_].indent > indent) {
                    value = parseBlockNode(lines_[cursor_].indent, depth + 1);
                } else if (cursor_ < lines_.size() && lines_[cursor_].indent == indent &&
                           isSeqItem(lines_[cursor_].text)) {
                    // Sequence items may sit at the same indentation as their key.
                    value = parseBlockSequence(indent, depth + 1);
                } else {
                    value.kind = YamlNode::Kind::kNull;
                    value.span = key_span;
                }
            }
            if (!duplicate) {
                YamlMapEntry entry;
                entry.key = std::move(key);
                entry.key_span = key_span;
                entry.value = std::move(value);
                node.entries.push_back(std::move(entry));
            }
        }
        if (!node.entries.empty()) {
            node.span.end_line = node.entries.back().value.span.end_line;
            node.span.end_col = node.entries.back().value.span.end_col;
        }
        return node;
    }

    YamlNode parseBlockSequence(uint32_t indent, size_t depth) {
        YamlNode node;
        node.kind = YamlNode::Kind::kSequence;
        node.span = lineSpan(lines_[cursor_]);
        if (!nodeBudgetOk(node.span)) {
            return node;
        }
        while (cursor_ < lines_.size() && !fatal_) {
            Line& line = lines_[cursor_];
            if (line.indent != indent || !isSeqItem(line.text)) {
                break;
            }
            size_t rest_offset = 1;
            while (rest_offset < line.text.size() && line.text[rest_offset] == ' ') {
                ++rest_offset;
            }
            const std::string rest = line.text.substr(rest_offset);
            const uint32_t rest_col = static_cast<uint32_t>(line.content_col + rest_offset);
            YamlNode item;
            if (rest.empty()) {
                ++cursor_;
                if (cursor_ < lines_.size() && lines_[cursor_].indent > indent) {
                    item = parseBlockNode(lines_[cursor_].indent, depth + 1);
                } else {
                    item.kind = YamlNode::Kind::kNull;
                    item.span = lineSpan(line);
                }
            } else if (isSeqItem(rest) || findKeyColon(rest) != kNpos) {
                // Compact nested collection ("- key: value" or "- - x"): rewrite the
                // current line so the nested block starts at the content column.
                line.text = rest;
                line.indent = rest_col - 1;
                line.content_col = rest_col;
                item = parseBlockNode(line.indent, depth + 1);
            } else {
                item = parseInlineValue(rest, line.number, rest_col, depth + 1);
                ++cursor_;
                if (cursor_ < lines_.size() && lines_[cursor_].indent > indent) {
                    error("parse.bad_indent", lineSpan(lines_[cursor_]),
                          "unexpected indented block after an inline sequence item");
                    skipDeeperLines(indent);
                }
            }
            node.elements.push_back(std::move(item));
        }
        if (!node.elements.empty()) {
            node.span.end_line = node.elements.back().span.end_line;
            node.span.end_col = node.elements.back().span.end_col;
        }
        return node;
    }

    bool splitKey(const Line& line, std::string& key, YamlNode::ScalarStyle& style, size_t& colon_pos,
                  Span& key_span) {
        const std::string& text = line.text;
        if (text[0] == '?') {
            error("parse.complex_key_forbidden", lineSpan(line), "complex mapping keys are forbidden");
            return false;
        }
        if (text[0] == '"' || text[0] == '\'') {
            size_t i = 0;
            std::string value;
            if (!parseQuotedScalarText(text, i, line.number, line.content_col, value)) {
                return false;
            }
            style = (text[0] == '"') ? YamlNode::ScalarStyle::kDoubleQuoted : YamlNode::ScalarStyle::kSingleQuoted;
            size_t j = i;
            while (j < text.size() && text[j] == ' ') {
                ++j;
            }
            if (j >= text.size() || text[j] != ':') {
                error("parse.bad_indent", lineSpan(line), "expected ':' after quoted mapping key");
                return false;
            }
            key = std::move(value);
            colon_pos = j;
            key_span = Span{line.number, line.content_col, line.number,
                            static_cast<uint32_t>(line.content_col + i - 1)};
            return true;
        }
        const size_t colon = findKeyColon(text);
        std::string raw_key = rtrim(text.substr(0, colon));
        if (raw_key.empty()) {
            error("parse.bad_indent", lineSpan(line), "mapping key must not be empty");
            return false;
        }
        const char first = raw_key[0];
        if (first == '&' || first == '*' || first == '!' || first == '|' || first == '>' || first == '@' ||
            first == '`') {
            error(forbiddenIndicatorCode(first), lineSpan(line),
                  std::string("mapping key must not start with '") + first + "'");
            return false;
        }
        style = YamlNode::ScalarStyle::kPlain;
        key = boundedScalar(std::move(raw_key), lineSpan(line));
        colon_pos = colon;
        key_span = Span{line.number, line.content_col, line.number,
                        static_cast<uint32_t>(line.content_col + key.size() - 1)};
        return true;
    }

    static const char* forbiddenIndicatorCode(char c) {
        switch (c) {
            case '&': return "parse.anchor_forbidden";
            case '*': return "parse.alias_forbidden";
            case '!': return "parse.tag_forbidden";
            case '|':
            case '>': return "parse.block_scalar_forbidden";
            default: return "parse.flow_syntax";
        }
    }

    // --- inline and flow values ---------------------------------------------

    YamlNode parseInlineValue(const std::string& text, uint32_t line, uint32_t col, size_t depth) {
        YamlNode node;
        node.span = Span{line, col, line, static_cast<uint32_t>(col + text.size() - 1)};
        if (!nodeBudgetOk(node.span)) {
            return node;
        }
        if (depth > limits_.max_depth) {
            error("parse.depth_exceeded", node.span,
                  "nesting exceeds the configured maximum depth of " + std::to_string(limits_.max_depth));
            return node;
        }
        const char first = text[0];
        if (first == '&' || first == '*' || first == '!' || first == '|' || first == '>') {
            error(forbiddenIndicatorCode(first), node.span,
                  std::string("value must not start with the forbidden indicator '") + first + "'");
            node.kind = YamlNode::Kind::kScalar;
            node.scalar = text;
            return node;
        }
        if (first == '[' || first == '{') {
            size_t i = 0;
            node = parseFlowValue(text, i, line, col, depth);
            while (i < text.size() && text[i] == ' ') {
                ++i;
            }
            if (i < text.size()) {
                error("parse.flow_syntax",
                      Span{line, static_cast<uint32_t>(col + i), line,
                           static_cast<uint32_t>(col + text.size() - 1)},
                      "trailing content after flow value");
            }
            return node;
        }
        if (first == '"' || first == '\'') {
            size_t i = 0;
            std::string value;
            const bool ok = parseQuotedScalarText(text, i, line, col, value);
            node.kind = YamlNode::Kind::kScalar;
            node.style = (first == '"') ? YamlNode::ScalarStyle::kDoubleQuoted : YamlNode::ScalarStyle::kSingleQuoted;
            node.scalar = boundedScalar(std::move(value), node.span);
            if (ok) {
                while (i < text.size() && text[i] == ' ') {
                    ++i;
                }
                if (i < text.size()) {
                    error("parse.trailing_content",
                          Span{line, static_cast<uint32_t>(col + i), line,
                               static_cast<uint32_t>(col + text.size() - 1)},
                          "trailing content after quoted scalar");
                }
            }
            return node;
        }
        if (first == '?') {
            error("parse.complex_key_forbidden", node.span, "complex keys are forbidden");
            return node;
        }
        if (findKeyColon(text) != kNpos) {
            error("parse.ambiguous_scalar", node.span,
                  "plain scalar must not contain ': '; quote the value to disambiguate");
        }
        node.kind = YamlNode::Kind::kScalar;
        node.style = YamlNode::ScalarStyle::kPlain;
        node.scalar = boundedScalar(text, node.span);
        return node;
    }

    YamlNode parseFlowValue(const std::string& text, size_t& i, uint32_t line, uint32_t base_col, size_t depth) {
        YamlNode node;
        auto colAt = [&](size_t pos) { return static_cast<uint32_t>(base_col + pos); };
        auto spanAt = [&](size_t begin, size_t end) {
            return Span{line, colAt(begin), line, colAt(end > begin ? end - 1 : begin)};
        };
        while (i < text.size() && text[i] == ' ') {
            ++i;
        }
        const size_t start = i;
        node.span = spanAt(start, start + 1);
        if (!nodeBudgetOk(node.span)) {
            i = text.size();
            return node;
        }
        if (depth > limits_.max_depth) {
            error("parse.depth_exceeded", node.span,
                  "nesting exceeds the configured maximum depth of " + std::to_string(limits_.max_depth));
            i = text.size();
            return node;
        }
        if (i >= text.size()) {
            error("parse.unterminated_flow", spanAt(start, start + 1), "unexpected end of flow value");
            return node;
        }
        const char c = text[i];
        if (c == '[') {
            node.kind = YamlNode::Kind::kSequence;
            ++i;
            bool first_element = true;
            while (true) {
                while (i < text.size() && text[i] == ' ') {
                    ++i;
                }
                if (i >= text.size()) {
                    error("parse.unterminated_flow", spanAt(start, text.size()),
                          "flow sequence is not terminated by ']' on the same line");
                    break;
                }
                if (text[i] == ']') {
                    ++i;
                    break;
                }
                if (!first_element) {
                    if (text[i] != ',') {
                        error("parse.flow_syntax", spanAt(i, i + 1), "expected ',' or ']' in flow sequence");
                        i = text.size();
                        break;
                    }
                    ++i;
                    while (i < text.size() && text[i] == ' ') {
                        ++i;
                    }
                    if (i < text.size() && text[i] == ']') {
                        error("parse.flow_syntax", spanAt(i, i + 1),
                              "trailing comma is forbidden in flow sequence");
                        ++i;
                        break;
                    }
                }
                node.elements.push_back(parseFlowValue(text, i, line, base_col, depth + 1));
                first_element = false;
            }
            node.span = spanAt(start, i);
            return node;
        }
        if (c == '{') {
            node.kind = YamlNode::Kind::kMapping;
            ++i;
            std::unordered_set<std::string> seen_keys;
            bool first_entry = true;
            while (true) {
                while (i < text.size() && text[i] == ' ') {
                    ++i;
                }
                if (i >= text.size()) {
                    error("parse.unterminated_flow", spanAt(start, text.size()),
                          "flow mapping is not terminated by '}' on the same line");
                    break;
                }
                if (text[i] == '}') {
                    ++i;
                    break;
                }
                if (!first_entry) {
                    if (text[i] != ',') {
                        error("parse.flow_syntax", spanAt(i, i + 1), "expected ',' or '}' in flow mapping");
                        i = text.size();
                        break;
                    }
                    ++i;
                    while (i < text.size() && text[i] == ' ') {
                        ++i;
                    }
                }
                const size_t key_start = i;
                YamlNode key_node = parseFlowValue(text, i, line, base_col, depth + 1);
                if (!key_node.isScalar()) {
                    error("parse.complex_key_forbidden", spanAt(key_start, i),
                          "flow mapping keys must be scalars");
                    i = text.size();
                    break;
                }
                while (i < text.size() && text[i] == ' ') {
                    ++i;
                }
                if (i >= text.size() || text[i] != ':') {
                    error("parse.flow_syntax", spanAt(i > 0 ? i - 1 : 0, i), "expected ':' in flow mapping entry");
                    i = text.size();
                    break;
                }
                ++i;
                YamlNode value = parseFlowValue(text, i, line, base_col, depth + 1);
                if (!seen_keys.insert(key_node.scalar).second) {
                    error("parse.duplicate_key", key_node.span,
                          "duplicate mapping key \"" + key_node.scalar + "\"", {{"key", key_node.scalar}});
                } else {
                    YamlMapEntry entry;
                    entry.key = key_node.scalar;
                    entry.key_span = key_node.span;
                    entry.value = std::move(value);
                    node.entries.push_back(std::move(entry));
                }
                first_entry = false;
            }
            node.span = spanAt(start, i);
            return node;
        }
        if (c == '"' || c == '\'') {
            std::string value;
            parseQuotedScalarText(text, i, line, base_col, value);
            node.kind = YamlNode::Kind::kScalar;
            node.style = (c == '"') ? YamlNode::ScalarStyle::kDoubleQuoted : YamlNode::ScalarStyle::kSingleQuoted;
            node.scalar = boundedScalar(std::move(value), spanAt(start, i));
            node.span = spanAt(start, i);
            return node;
        }
        if (c == '&' || c == '*' || c == '!' || c == '|' || c == '>') {
            error(forbiddenIndicatorCode(c), spanAt(i, i + 1),
                  std::string("flow value must not start with the forbidden indicator '") + c + "'");
            i = text.size();
            return node;
        }
        // Plain flow scalar: runs until a flow delimiter, or a mapping colon
        // (':' followed by a space or delimiter — "builtin://x" stays intact).
        size_t end = i;
        while (end < text.size() && text[end] != ',' && text[end] != ']' && text[end] != '}') {
            if (text[end] == ':' &&
                (end + 1 >= text.size() || text[end + 1] == ' ' || text[end + 1] == ',' ||
                 text[end + 1] == ']' || text[end + 1] == '}')) {
                break;
            }
            ++end;
        }
        std::string value = rtrim(text.substr(i, end - i));
        if (value.empty()) {
            error("parse.flow_syntax", spanAt(i, end + 1), "empty plain scalar in flow value");
            i = end;
            return node;
        }
        node.kind = YamlNode::Kind::kScalar;
        node.style = YamlNode::ScalarStyle::kPlain;
        node.scalar = boundedScalar(std::move(value), spanAt(i, end));
        node.span = spanAt(i, i + node.scalar.size());
        i = end;
        return node;
    }

    // Parses a quoted scalar starting at text[i] (which must be ' or ").
    // Advances i past the closing quote. Returns false on error.
    bool parseQuotedScalarText(const std::string& text, size_t& i, uint32_t line, uint32_t base_col,
                               std::string& out) {
        const char quote = text[i];
        const size_t start = i;
        ++i;
        auto errSpan = [&](size_t begin, size_t end) {
            return Span{line, static_cast<uint32_t>(base_col + begin), line,
                        static_cast<uint32_t>(base_col + (end > begin ? end - 1 : begin))};
        };
        if (quote == '\'') {
            while (i < text.size()) {
                if (text[i] == '\'') {
                    if (i + 1 < text.size() && text[i + 1] == '\'') {
                        out.push_back('\'');
                        i += 2;
                        continue;
                    }
                    ++i;
                    return true;
                }
                out.push_back(text[i]);
                ++i;
            }
            error("parse.unterminated_quote", errSpan(start, text.size()),
                  "single-quoted scalar is not terminated on the same line");
            return false;
        }
        while (i < text.size()) {
            const char c = text[i];
            if (c == '"') {
                ++i;
                return true;
            }
            if (c != '\\') {
                out.push_back(c);
                ++i;
                continue;
            }
            if (i + 1 >= text.size()) {
                error("parse.bad_escape", errSpan(i, i + 1), "dangling escape at end of line");
                ++i;
                break;
            }
            const char e = text[i + 1];
            i += 2;
            switch (e) {
                case '"': out.push_back('"'); break;
                case '\\': out.push_back('\\'); break;
                case '/': out.push_back('/'); break;
                case 'n': out.push_back('\n'); break;
                case 't': out.push_back('\t'); break;
                case 'r': out.push_back('\r'); break;
                case 'b': out.push_back('\b'); break;
                case 'f': out.push_back('\f'); break;
                case '0': out.push_back('\0'); break;
                case 'u': {
                    uint32_t code_point = 0;
                    if (!readHex4(text, i, code_point)) {
                        error("parse.bad_escape", errSpan(i - 2, i), "\\u requires four hex digits");
                        break;
                    }
                    if (code_point >= 0xd800u && code_point <= 0xdbffu) {
                        uint32_t low = 0;
                        if (i + 1 < text.size() && text[i] == '\\' && text[i + 1] == 'u') {
                            size_t j = i + 2;
                            if (readHex4(text, j, low) && low >= 0xdc00u && low <= 0xdfffu) {
                                code_point = 0x10000u + ((code_point - 0xd800u) << 10) + (low - 0xdc00u);
                                i = j;
                                encodeUtf8(code_point, out);
                                break;
                            }
                        }
                        error("parse.bad_escape", errSpan(i - 6, i), "unpaired UTF-16 surrogate escape");
                        break;
                    }
                    if (code_point >= 0xdc00u && code_point <= 0xdfffu) {
                        error("parse.bad_escape", errSpan(i - 6, i), "unpaired UTF-16 surrogate escape");
                        break;
                    }
                    encodeUtf8(code_point, out);
                    break;
                }
                default:
                    error("parse.bad_escape", errSpan(i - 2, i),
                          std::string("unsupported escape sequence '\\") + e + "'");
                    out.push_back(e);
            }
        }
        error("parse.unterminated_quote", errSpan(start, text.size()),
              "double-quoted scalar is not terminated on the same line");
        return false;
    }

    static bool readHex4(const std::string& text, size_t& i, uint32_t& out) {
        if (i + 4 > text.size()) {
            return false;
        }
        uint32_t value = 0;
        for (size_t k = 0; k < 4; ++k) {
            const char c = text[i + k];
            value <<= 4;
            if (c >= '0' && c <= '9') {
                value |= static_cast<uint32_t>(c - '0');
            } else if (c >= 'a' && c <= 'f') {
                value |= static_cast<uint32_t>(c - 'a' + 10);
            } else if (c >= 'A' && c <= 'F') {
                value |= static_cast<uint32_t>(c - 'A' + 10);
            } else {
                return false;
            }
        }
        i += 4;
        out = value;
        return true;
    }

    DiagnosticList& diagnostics_;
    const ParseLimits& limits_;
    std::vector<Line> lines_;
    size_t cursor_ = 0;
    size_t nodes_created_ = 0;
    bool fatal_ = false;
};

}  // namespace

YamlNode parseYaml(const std::string& source, DiagnosticList& diagnostics, const ParseLimits& limits) {
    Parser parser(source, diagnostics, limits);
    return parser.parse();
}

}  // namespace fluent_stage::scene
