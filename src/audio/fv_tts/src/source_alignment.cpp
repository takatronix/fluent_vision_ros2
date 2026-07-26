#include "fv_tts/source_alignment.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

namespace fv_tts {
namespace {

float round_ties_even(float value) {
  if (!std::isfinite(value) || value < 0.0F) {
    throw std::invalid_argument(
        "VOICEVOX query contains an invalid phoneme duration");
  }
  const auto lower = std::floor(value);
  const auto fraction = value - lower;
  if (fraction < 0.5F) {
    return lower;
  }
  if (fraction > 0.5F) {
    return lower + 1.0F;
  }
  return std::fmod(lower, 2.0F) == 0.0F ? lower : lower + 1.0F;
}

std::uint64_t decoder_frames(float seconds, float speed_scale) {
  constexpr float kDecoderFramesPerSecond = 24000.0F / 256.0F;
  if (!std::isfinite(speed_scale) || speed_scale <= 0.0F) {
    throw std::invalid_argument(
        "VOICEVOX query contains an invalid speedScale");
  }
  const auto frames =
      round_ties_even(round_ties_even(seconds * kDecoderFramesPerSecond) /
                      speed_scale);
  if (frames >
      static_cast<float>(std::numeric_limits<std::uint64_t>::max())) {
    throw std::overflow_error("VOICEVOX query duration is too large");
  }
  return static_cast<std::uint64_t>(frames);
}

std::uint64_t checked_add(std::uint64_t left, std::uint64_t right,
                          const char *context) {
  if (right > std::numeric_limits<std::uint64_t>::max() - left) {
    throw std::overflow_error(context);
  }
  return left + right;
}

std::uint64_t output_frames(std::uint64_t decoder_frame_count,
                            std::uint32_t output_sampling_rate) {
  constexpr std::uint64_t kDecoderHop = 256;
  constexpr std::uint32_t kNativeSamplingRate = 24000;
  if (output_sampling_rate == 0 ||
      output_sampling_rate % kNativeSamplingRate != 0) {
    throw std::invalid_argument(
        "VOICEVOX outputSamplingRate must be a positive multiple of 24000");
  }
  const auto multiplier = output_sampling_rate / kNativeSamplingRate;
  if (decoder_frame_count >
      std::numeric_limits<std::uint64_t>::max() / kDecoderHop /
          multiplier) {
    throw std::overflow_error("VOICEVOX output duration is too large");
  }
  return decoder_frame_count * kDecoderHop * multiplier;
}

float required_float(const nlohmann::json &value, const char *field) {
  try {
    const auto result = value.at(field).get<float>();
    if (!std::isfinite(result) || result < 0.0F) {
      throw std::invalid_argument(std::string(field) +
                                  " must be finite and non-negative");
    }
    return result;
  } catch (const nlohmann::json::exception &error) {
    throw std::invalid_argument(std::string("VOICEVOX query has invalid ") +
                                field + ": " + error.what());
  }
}

std::uint64_t required_u64(const nlohmann::json &value, const char *field) {
  try {
    return value.at(field).get<std::uint64_t>();
  } catch (const nlohmann::json::exception &error) {
    throw std::invalid_argument(std::string("VOICEVOX alignment has invalid ") +
                                field + ": " + error.what());
  }
}

std::string required_string(const nlohmann::json &value, const char *field) {
  try {
    return value.at(field).get<std::string>();
  } catch (const nlohmann::json::exception &error) {
    throw std::invalid_argument(std::string("VOICEVOX alignment has invalid ") +
                                field + ": " + error.what());
  }
}

struct QueryTimeline {
  std::vector<std::uint64_t> before_pause;
  std::vector<std::uint64_t> after_pause;
  std::uint64_t total_frames;
};

QueryTimeline query_timeline(const nlohmann::json &query) {
  const auto speed_scale = required_float(query, "speedScale");
  if (speed_scale <= 0.0F) {
    throw std::invalid_argument(
        "VOICEVOX query speedScale must be greater than zero");
  }
  const auto output_sampling_rate =
      query.at("outputSamplingRate").get<std::uint32_t>();
  auto decoder_cursor =
      decoder_frames(required_float(query, "prePhonemeLength"), speed_scale);
  std::vector<std::uint64_t> before_pause{
      output_frames(decoder_cursor, output_sampling_rate)};
  std::vector<std::uint64_t> after_pause = before_pause;

  const auto &phrases = query.at("accent_phrases");
  if (!phrases.is_array()) {
    throw std::invalid_argument(
        "VOICEVOX query accent_phrases must be an array");
  }
  for (const auto &phrase : phrases) {
    const auto &moras = phrase.at("moras");
    if (!moras.is_array()) {
      throw std::invalid_argument("VOICEVOX query moras must be an array");
    }
    for (const auto &mora : moras) {
      const auto &consonant_length = mora.at("consonant_length");
      if (!consonant_length.is_null()) {
        decoder_cursor =
            checked_add(decoder_cursor,
                        decoder_frames(consonant_length.get<float>(),
                                       speed_scale),
                        "VOICEVOX query duration is too large");
      }
      decoder_cursor =
          checked_add(decoder_cursor,
                      decoder_frames(required_float(mora, "vowel_length"),
                                     speed_scale),
                      "VOICEVOX query duration is too large");
      const auto boundary =
          output_frames(decoder_cursor, output_sampling_rate);
      before_pause.push_back(boundary);
      after_pause.push_back(boundary);
    }

    const bool interrogative = phrase.at("is_interrogative").get<bool>();
    if (interrogative && !moras.empty() &&
        required_float(moras.back(), "pitch") != 0.0F) {
      decoder_cursor =
          checked_add(decoder_cursor, decoder_frames(0.15F, speed_scale),
                      "VOICEVOX query duration is too large");
      const auto boundary =
          output_frames(decoder_cursor, output_sampling_rate);
      before_pause.back() = boundary;
      after_pause.back() = boundary;
    }

    const auto &pause = phrase.at("pause_mora");
    if (!pause.is_null()) {
      const auto &consonant_length = pause.at("consonant_length");
      if (!consonant_length.is_null()) {
        decoder_cursor =
            checked_add(decoder_cursor,
                        decoder_frames(consonant_length.get<float>(),
                                       speed_scale),
                        "VOICEVOX query duration is too large");
      }
      decoder_cursor =
          checked_add(decoder_cursor,
                      decoder_frames(required_float(pause, "vowel_length"),
                                     speed_scale),
                      "VOICEVOX query duration is too large");
      after_pause.back() =
          output_frames(decoder_cursor, output_sampling_rate);
    }
  }
  decoder_cursor =
      checked_add(decoder_cursor,
                  decoder_frames(required_float(query, "postPhonemeLength"),
                                 speed_scale),
                  "VOICEVOX query duration is too large");
  return QueryTimeline{
      std::move(before_pause),
      std::move(after_pause),
      output_frames(decoder_cursor, output_sampling_rate)};
}

}  // namespace

std::vector<SynthesisMark> build_synthesis_marks(
    const std::string &text, const nlohmann::json &source_spans,
    const nlohmann::json &audio_query, std::uint64_t actual_total_frames) {
  if (!source_spans.is_array() || source_spans.empty()) {
    throw std::invalid_argument(
        "VOICEVOX source alignment must be a non-empty array");
  }
  const auto timeline = query_timeline(audio_query);
  if (timeline.total_frames != actual_total_frames) {
    throw std::runtime_error(
        "VOICEVOX alignment duration does not match synthesized PCM");
  }

  std::vector<SynthesisMark> marks;
  marks.reserve(source_spans.size());
  std::string reconstructed_text;
  std::uint64_t source_cursor = 0;
  std::uint64_t mora_cursor = 0;
  for (const auto &span : source_spans) {
    SynthesisMark mark{
        required_u64(span, "source_start"),
        required_u64(span, "source_end"),
        required_string(span, "surface"),
        required_string(span, "pronunciation"),
        required_u64(span, "mora_start"),
        required_u64(span, "mora_end"),
        0,
        0};
    if (mark.source_start != source_cursor ||
        mark.source_end <= mark.source_start ||
        mark.mora_start != mora_cursor || mark.mora_end < mark.mora_start ||
        mark.mora_end >= timeline.before_pause.size() ||
        mark.surface.empty() || mark.pronunciation.empty()) {
      throw std::invalid_argument(
          "VOICEVOX source alignment is not contiguous");
    }
    const auto start = static_cast<std::size_t>(mark.mora_start);
    const auto end = static_cast<std::size_t>(mark.mora_end);
    if (start == end) {
      mark.start_frame = timeline.before_pause[start];
      mark.end_frame = timeline.after_pause[end];
    } else {
      mark.start_frame = timeline.after_pause[start];
      mark.end_frame = timeline.before_pause[end];
    }
    reconstructed_text += mark.surface;
    source_cursor = mark.source_end;
    mora_cursor = mark.mora_end;
    marks.push_back(std::move(mark));
  }
  if (reconstructed_text != text ||
      mora_cursor + 1 != timeline.before_pause.size() ||
      timeline.before_pause.size() != timeline.after_pause.size()) {
    throw std::invalid_argument(
        "VOICEVOX source alignment does not cover the synthesis request");
  }
  return marks;
}

}  // namespace fv_tts
