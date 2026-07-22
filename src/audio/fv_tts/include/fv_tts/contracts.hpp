#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace fv_tts {

enum class SpeechKind { kAgent, kSystem };

struct SayRequest {
  SpeechKind kind;
  std::string utterance_id;
  std::string text;
};

struct TtsSettings {
  std::uint32_t style_id;
};

struct Voice {
  std::uint32_t id;
  std::string speaker;
  std::string style;
  std::string label;

  bool operator==(const Voice &other) const {
    return id == other.id && speaker == other.speaker && style == other.style &&
           label == other.label;
  }
};

enum class ResultStatus { kCompleted, kFailed, kCancelled };

std::string trim_unicode_whitespace(const std::string &value);
SayRequest parse_say_request(const std::string &payload);
TtsSettings parse_tts_settings(const std::string &payload);
std::string voice_catalog_json(std::uint32_t current_style_id,
                               const std::vector<Voice> &voices);
std::string tts_result_json(SpeechKind kind, const std::string &utterance_id,
                            ResultStatus status,
                            const std::string &error = {});
std::uint64_t agent_floor_epoch(const std::string &utterance_id);
const char *speech_kind_name(SpeechKind kind);

}  // namespace fv_tts
