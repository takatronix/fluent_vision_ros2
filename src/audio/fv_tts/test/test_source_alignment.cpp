#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "fv_tts/source_alignment.hpp"

namespace fv_tts {
namespace {

nlohmann::json audio_query() {
  return {
      {"accent_phrases",
       {{{"moras",
          {{{"text", "ア"},
            {"consonant", nullptr},
            {"consonant_length", nullptr},
            {"vowel", "a"},
            {"vowel_length", 0.1},
            {"pitch", 1.0}}}},
         {"accent", 1},
         {"pause_mora",
          {{"text", "、"},
           {"consonant", nullptr},
           {"consonant_length", nullptr},
           {"vowel", "pau"},
           {"vowel_length", 0.1},
           {"pitch", 0.0}}},
         {"is_interrogative", false}}}},
      {"speedScale", 1.0},
      {"pitchScale", 0.0},
      {"intonationScale", 1.0},
      {"volumeScale", 1.0},
      {"prePhonemeLength", 0.1},
      {"postPhonemeLength", 0.1},
      {"outputSamplingRate", 24000},
      {"outputStereo", false},
      {"kana", nullptr},
  };
}

TEST(SourceAlignment, UsesVoicevoxMixedCaseAudioQueryContract) {
  const nlohmann::json source_spans = {
      {{"source_start", 0},
       {"source_end", 1},
       {"surface", "あ"},
       {"pronunciation", "ア"},
       {"mora_start", 0},
       {"mora_end", 1}},
      {{"source_start", 1},
       {"source_end", 2},
       {"surface", "。"},
       {"pronunciation", "、"},
       {"mora_start", 1},
       {"mora_end", 1}},
  };

  const auto marks =
      build_synthesis_marks("あ。", source_spans, audio_query(), 9216);

  ASSERT_EQ(marks.size(), 2U);
  EXPECT_EQ(marks[0].start_frame, 2304U);
  EXPECT_EQ(marks[0].end_frame, 4608U);
  EXPECT_EQ(marks[1].start_frame, 4608U);
  EXPECT_EQ(marks[1].end_frame, 6912U);
}

TEST(SourceAlignment, RejectsCamelCaseAccentPhrasesField) {
  auto query = audio_query();
  query["accentPhrases"] = query["accent_phrases"];
  query.erase("accent_phrases");
  const nlohmann::json source_spans = {
      {{"source_start", 0},
       {"source_end", 1},
       {"surface", "あ"},
       {"pronunciation", "ア"},
       {"mora_start", 0},
       {"mora_end", 1}},
  };

  EXPECT_THROW(
      build_synthesis_marks("あ", source_spans, query, 9216),
      nlohmann::json::out_of_range);
}

}  // namespace
}  // namespace fv_tts
