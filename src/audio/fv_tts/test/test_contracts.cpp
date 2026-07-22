#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "fv_tts/contracts.hpp"
#include "fv_tts/voicevox_backend.hpp"

namespace fv_tts {
namespace {

void append_u16(std::vector<std::uint8_t> &bytes, std::uint16_t value) {
  bytes.push_back(static_cast<std::uint8_t>(value));
  bytes.push_back(static_cast<std::uint8_t>(value >> 8U));
}

void append_u32(std::vector<std::uint8_t> &bytes, std::uint32_t value) {
  bytes.push_back(static_cast<std::uint8_t>(value));
  bytes.push_back(static_cast<std::uint8_t>(value >> 8U));
  bytes.push_back(static_cast<std::uint8_t>(value >> 16U));
  bytes.push_back(static_cast<std::uint8_t>(value >> 24U));
}

std::vector<std::uint8_t> pcm16_wav() {
  std::vector<std::uint8_t> wav{'R', 'I', 'F', 'F'};
  append_u32(wav, 40);
  wav.insert(wav.end(), {'W', 'A', 'V', 'E', 'f', 'm', 't', ' '});
  append_u32(wav, 16);
  append_u16(wav, 1);
  append_u16(wav, 1);
  append_u32(wav, 24000);
  append_u32(wav, 48000);
  append_u16(wav, 2);
  append_u16(wav, 16);
  wav.insert(wav.end(), {'d', 'a', 't', 'a'});
  append_u32(wav, 4);
  wav.insert(wav.end(), {1, 0, 2, 0});
  return wav;
}

TEST(Contracts, SayRequestRequiresExactlyThreeFields) {
  const auto request = parse_say_request(
      R"({"kind":"system","utterance_id":"system-ready-1","text":"  準備完了  "})");
  EXPECT_EQ(request.kind, SpeechKind::kSystem);
  EXPECT_EQ(request.utterance_id, "system-ready-1");
  EXPECT_EQ(request.text, "準備完了");
  EXPECT_THROW(parse_say_request(
                   R"({"kind":"system","utterance_id":"system-ready-1","text":"準備完了","voice":3})"),
               std::invalid_argument);
  EXPECT_THROW(parse_say_request("not-json"), std::invalid_argument);
  const auto unicode_trimmed = parse_say_request(
      R"({"kind":"system","utterance_id":"system-ready-2","text":"　起動確認　"})");
  EXPECT_EQ(unicode_trimmed.text, "起動確認");
  EXPECT_THROW(parse_say_request(
                   R"({"kind":"system","utterance_id":"system-ready-2","text":"　"})"),
               std::invalid_argument);
  EXPECT_EQ(trim_unicode_whitespace("\u00a0\t native error 　"),
            "native error");
}

TEST(Contracts, SettingsRejectBooleanVersionAndExtraFields) {
  EXPECT_EQ(parse_tts_settings(R"({"version":1,"style_id":30})").style_id,
            30U);
  EXPECT_THROW(parse_tts_settings(R"({"version":true,"style_id":30})"),
               std::invalid_argument);
  EXPECT_THROW(
      parse_tts_settings(R"({"version":1,"style_id":30,"speaker":"x"})"),
      std::invalid_argument);
  EXPECT_THROW(parse_tts_settings(R"({"version":1,"style_id":-1})"),
               std::invalid_argument);
}

TEST(Contracts, CatalogAndResultKeepTheExistingWireShape) {
  const std::vector<Voice> voices{
      Voice{30, "No.7", "アナウンス", "No.7 / アナウンス"},
      Voice{31, "No.7", "読み聞かせ", "No.7 / 読み聞かせ"}};
  EXPECT_EQ(
      voice_catalog_json(30, voices),
      R"({"version":1,"current_style_id":30,"voices":[{"id":30,"speaker":"No.7","style":"アナウンス","label":"No.7 / アナウンス"},{"id":31,"speaker":"No.7","style":"読み聞かせ","label":"No.7 / 読み聞かせ"}]})");
  EXPECT_EQ(tts_result_json(SpeechKind::kAgent, "agent-1-a",
                            ResultStatus::kCompleted),
            R"({"kind":"agent","utterance_id":"agent-1-a","status":"completed"})");
  EXPECT_EQ(tts_result_json(SpeechKind::kSystem, "system-a",
                            ResultStatus::kFailed, "native error"),
            R"({"kind":"system","utterance_id":"system-a","status":"failed","error":"native error"})");
  EXPECT_THROW(tts_result_json(SpeechKind::kAgent, "agent-1-b",
                               ResultStatus::kFailed),
               std::invalid_argument);
}

TEST(Contracts, AgentFloorEpochIsStrict) {
  EXPECT_EQ(agent_floor_epoch("agent-12-id"), 12U);
  EXPECT_THROW(agent_floor_epoch("agent-x-id"), std::invalid_argument);
  EXPECT_THROW(agent_floor_epoch("agent--1-id"), std::invalid_argument);
  EXPECT_THROW(agent_floor_epoch("other-0-id"), std::invalid_argument);
}

TEST(Wav, DecodesPcm16WithoutSampleConversion) {
  const auto audio = decode_pcm16_wav(pcm16_wav());
  EXPECT_EQ(audio.pcm, (std::vector<std::uint8_t>{1, 0, 2, 0}));
  EXPECT_EQ(audio.sample_rate_hz, 24000U);
  EXPECT_EQ(audio.channels, 1U);
  EXPECT_EQ(audio.bit_depth, 16U);
}

TEST(Wav, RejectsCompressedOrTruncatedPayloads) {
  auto compressed = pcm16_wav();
  compressed[20] = 3;
  EXPECT_THROW(decode_pcm16_wav(compressed), std::invalid_argument);
  auto truncated = pcm16_wav();
  truncated.pop_back();
  EXPECT_THROW(decode_pcm16_wav(truncated), std::invalid_argument);
}

}  // namespace
}  // namespace fv_tts
