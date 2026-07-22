#include "fv_tts/contracts.hpp"

#include <algorithm>
#include <charconv>
#include <limits>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

namespace fv_tts {
namespace {

using Json = nlohmann::ordered_json;

std::pair<std::uint32_t, std::size_t> decode_utf8(
    const std::string &value, std::size_t offset) {
  const auto first = static_cast<unsigned char>(value[offset]);
  if (first < 0x80U) {
    return {first, 1};
  }
  std::size_t width = 0;
  std::uint32_t codepoint = 0;
  if ((first & 0xE0U) == 0xC0U) {
    width = 2;
    codepoint = first & 0x1FU;
  } else if ((first & 0xF0U) == 0xE0U) {
    width = 3;
    codepoint = first & 0x0FU;
  } else if ((first & 0xF8U) == 0xF0U) {
    width = 4;
    codepoint = first & 0x07U;
  } else {
    return {first, 1};
  }
  if (width > value.size() - offset) {
    return {first, 1};
  }
  for (std::size_t index = 1; index < width; ++index) {
    const auto byte = static_cast<unsigned char>(value[offset + index]);
    if ((byte & 0xC0U) != 0x80U) {
      return {first, 1};
    }
    codepoint = (codepoint << 6U) | (byte & 0x3FU);
  }
  return {codepoint, width};
}

bool is_python_whitespace(std::uint32_t codepoint) {
  return (codepoint >= 0x09U && codepoint <= 0x0DU) ||
         (codepoint >= 0x1CU && codepoint <= 0x20U) ||
         codepoint == 0x85U || codepoint == 0xA0U ||
         codepoint == 0x1680U ||
         (codepoint >= 0x2000U && codepoint <= 0x200AU) ||
         codepoint == 0x2028U || codepoint == 0x2029U ||
         codepoint == 0x202FU || codepoint == 0x205FU ||
         codepoint == 0x3000U;
}

Json parse_object(const std::string &payload, const char *description) {
  try {
    auto value = Json::parse(payload);
    if (!value.is_object()) {
      throw std::invalid_argument(std::string(description) + " must be an object");
    }
    return value;
  } catch (const nlohmann::json::parse_error &) {
    throw std::invalid_argument(std::string(description) + " must be valid JSON");
  }
}

bool has_exact_fields(const Json &value,
                      std::initializer_list<const char *> fields) {
  if (value.size() != fields.size()) {
    return false;
  }
  return std::all_of(fields.begin(), fields.end(),
                     [&value](const char *field) { return value.contains(field); });
}

}  // namespace

std::string trim_unicode_whitespace(const std::string &value) {
  std::size_t first_non_whitespace = std::string::npos;
  std::size_t last_non_whitespace_end = 0;
  for (std::size_t offset = 0; offset < value.size();) {
    const auto [codepoint, width] = decode_utf8(value, offset);
    if (!is_python_whitespace(codepoint)) {
      if (first_non_whitespace == std::string::npos) {
        first_non_whitespace = offset;
      }
      last_non_whitespace_end = offset + width;
    }
    offset += width;
  }
  if (first_non_whitespace == std::string::npos) {
    return {};
  }
  return value.substr(first_non_whitespace,
                      last_non_whitespace_end - first_non_whitespace);
}

SayRequest parse_say_request(const std::string &payload) {
  const auto value = parse_object(payload, "TTS request");
  if (!has_exact_fields(value, {"kind", "utterance_id", "text"})) {
    throw std::invalid_argument(
        "TTS request fields must be kind, utterance_id, and text");
  }
  if (!value["kind"].is_string()) {
    throw std::invalid_argument("TTS kind must be agent or system");
  }
  const auto kind_value = value["kind"].get<std::string>();
  SpeechKind kind;
  if (kind_value == "agent") {
    kind = SpeechKind::kAgent;
  } else if (kind_value == "system") {
    kind = SpeechKind::kSystem;
  } else {
    throw std::invalid_argument("TTS kind must be agent or system");
  }
  for (const char *field : {"utterance_id", "text"}) {
    if (!value[field].is_string() ||
        trim_unicode_whitespace(value[field].get_ref<const std::string &>()).empty()) {
      throw std::invalid_argument(std::string("TTS ") + field +
                                  " must not be empty");
    }
  }
  return SayRequest{kind, value["utterance_id"].get<std::string>(),
                    trim_unicode_whitespace(value["text"].get<std::string>())};
}

TtsSettings parse_tts_settings(const std::string &payload) {
  const auto value = parse_object(payload, "TTS settings");
  if (!has_exact_fields(value, {"version", "style_id"})) {
    throw std::invalid_argument(
        "TTS settings fields must be version and style_id");
  }
  if (!value["version"].is_number_integer() ||
      value["version"].get<std::int64_t>() != 1) {
    throw std::invalid_argument("TTS settings version must be 1");
  }
  if (!value["style_id"].is_number_integer()) {
    throw std::invalid_argument(
        "TTS settings style_id must be a non-negative integer");
  }
  const auto style_id = value["style_id"].get<std::int64_t>();
  if (style_id < 0 ||
      static_cast<std::uint64_t>(style_id) >
          std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument(
        "TTS settings style_id must be a non-negative integer");
  }
  return TtsSettings{static_cast<std::uint32_t>(style_id)};
}

std::string voice_catalog_json(std::uint32_t current_style_id,
                               const std::vector<Voice> &voices) {
  Json serialized_voices = Json::array();
  for (const auto &voice : voices) {
    serialized_voices.push_back(Json{{"id", voice.id},
                                     {"speaker", voice.speaker},
                                     {"style", voice.style},
                                     {"label", voice.label}});
  }
  return Json{{"version", 1},
              {"current_style_id", current_style_id},
              {"voices", std::move(serialized_voices)}}
      .dump();
}

std::string tts_result_json(SpeechKind kind, const std::string &utterance_id,
                            ResultStatus status, const std::string &error) {
  if (trim_unicode_whitespace(utterance_id).empty()) {
    throw std::invalid_argument("TTS result utterance_id must not be empty");
  }
  const char *status_name = nullptr;
  switch (status) {
    case ResultStatus::kCompleted:
      status_name = "completed";
      break;
    case ResultStatus::kFailed:
      status_name = "failed";
      if (trim_unicode_whitespace(error).empty()) {
        throw std::invalid_argument("failed TTS result requires error");
      }
      break;
    case ResultStatus::kCancelled:
      status_name = "cancelled";
      break;
  }
  if (status != ResultStatus::kFailed && !error.empty()) {
    throw std::invalid_argument("only failed TTS result accepts error");
  }
  Json value{{"kind", speech_kind_name(kind)},
             {"utterance_id", utterance_id},
             {"status", status_name}};
  if (status == ResultStatus::kFailed) {
    value["error"] = error;
  }
  return value.dump();
}

std::uint64_t agent_floor_epoch(const std::string &utterance_id) {
  constexpr char kPrefix[] = "agent-";
  if (utterance_id.compare(0, sizeof(kPrefix) - 1, kPrefix) != 0) {
    throw std::invalid_argument(
        "agent utterance_id must be agent-<floor_epoch>-<id>");
  }
  const auto epoch_start = sizeof(kPrefix) - 1;
  const auto separator = utterance_id.find('-', epoch_start);
  if (separator == std::string::npos || separator == epoch_start) {
    throw std::invalid_argument(
        "agent utterance_id must be agent-<floor_epoch>-<id>");
  }
  const auto epoch_text = utterance_id.substr(epoch_start, separator - epoch_start);
  if (!epoch_text.empty() && epoch_text.front() == '-') {
    throw std::invalid_argument("agent utterance_id floor epoch is negative");
  }
  std::uint64_t epoch = 0;
  const auto result = std::from_chars(epoch_text.data(),
                                      epoch_text.data() + epoch_text.size(), epoch);
  if (result.ec != std::errc{} || result.ptr != epoch_text.data() + epoch_text.size()) {
    throw std::invalid_argument("agent utterance_id floor epoch is invalid");
  }
  return epoch;
}

const char *speech_kind_name(SpeechKind kind) {
  return kind == SpeechKind::kAgent ? "agent" : "system";
}

}  // namespace fv_tts
