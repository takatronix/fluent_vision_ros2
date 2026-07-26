#include "fv_tts/voicevox_backend.hpp"

#include <voicevox_core.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <glob.h>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

#include "fv_tts/source_alignment.hpp"

namespace fv_tts {
namespace {

namespace fs = std::filesystem;

std::string result_message(VoicevoxResultCode code) {
  const char *message = voicevox_error_result_to_message(code);
  return message == nullptr || *message == '\0' ? "unknown voicevox_core error"
                                                : std::string(message);
}

void check_result(VoicevoxResultCode code, const std::string &context) {
  if (code != VOICEVOX_RESULT_OK) {
    throw std::runtime_error(context + ": " + result_message(code));
  }
}

std::string lower_ascii(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](char byte) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(byte)));
  });
  return value;
}

std::string expand_user(const std::string &path) {
  if (path == "~" || path.rfind("~/", 0) == 0) {
    const char *home = std::getenv("HOME");
    if (home == nullptr || *home == '\0') {
      throw std::invalid_argument("HOME is unset; cannot expand VOICEVOX path");
    }
    return std::string(home) + path.substr(1);
  }
  return path;
}

std::vector<fs::path> expand_model_paths(const std::string &pattern) {
  glob_t matches{};
  const auto expanded = expand_user(pattern);
  const int status = ::glob(expanded.c_str(), 0, nullptr, &matches);
  if (status == GLOB_NOMATCH) {
    globfree(&matches);
    return {};
  }
  if (status != 0) {
    globfree(&matches);
    throw std::runtime_error("failed to expand VOICEVOX voice model pattern: " +
                             expanded);
  }
  std::vector<fs::path> paths;
  paths.reserve(matches.gl_pathc);
  for (std::size_t index = 0; index < matches.gl_pathc; ++index) {
    paths.push_back(fs::absolute(matches.gl_pathv[index]));
  }
  globfree(&matches);
  return paths;
}

struct OpenJtalkDeleter {
  void operator()(OpenJtalkRc *value) const {
    voicevox_open_jtalk_rc_delete(value);
  }
};

struct SynthesizerDeleter {
  void operator()(VoicevoxSynthesizer *value) const {
    voicevox_synthesizer_delete(value);
  }
};

struct VoiceModelDeleter {
  void operator()(VoicevoxVoiceModelFile *value) const {
    voicevox_voice_model_file_delete(value);
  }
};

struct JsonDeleter {
  void operator()(char *value) const { voicevox_json_free(value); }
};

struct WavDeleter {
  void operator()(std::uint8_t *value) const { voicevox_wav_free(value); }
};

using OpenJtalkPtr = std::unique_ptr<OpenJtalkRc, OpenJtalkDeleter>;
using SynthesizerPtr =
    std::unique_ptr<VoicevoxSynthesizer, SynthesizerDeleter>;
using VoiceModelPtr =
    std::unique_ptr<VoicevoxVoiceModelFile, VoiceModelDeleter>;
using JsonPtr = std::unique_ptr<char, JsonDeleter>;
using WavPtr = std::unique_ptr<std::uint8_t, WavDeleter>;

VoiceModelPtr open_model(const fs::path &path) {
  VoicevoxVoiceModelFile *raw = nullptr;
  check_result(voicevox_voice_model_file_open(path.c_str(), &raw),
               "failed to open VOICEVOX model " + path.string());
  return VoiceModelPtr(raw);
}

std::array<std::uint8_t, 16> model_id(
    const VoicevoxVoiceModelFile *model) {
  std::uint8_t raw[16]{};
  voicevox_voice_model_file_id(model, &raw);
  std::array<std::uint8_t, 16> id{};
  std::copy(std::begin(raw), std::end(raw), id.begin());
  return id;
}

VoicevoxVoiceModelId c_model_id(
    const std::array<std::uint8_t, 16> &id,
    std::uint8_t (&storage)[16]) {
  std::copy(id.begin(), id.end(), std::begin(storage));
  return &storage;
}

std::uint16_t read_u16(const std::vector<std::uint8_t> &bytes,
                       std::size_t offset) {
  if (offset > bytes.size() || bytes.size() - offset < 2) {
    throw std::invalid_argument("VOICEVOX returned an invalid WAV payload");
  }
  return static_cast<std::uint16_t>(bytes[offset]) |
         static_cast<std::uint16_t>(bytes[offset + 1]) << 8U;
}

std::uint32_t read_u32(const std::vector<std::uint8_t> &bytes,
                       std::size_t offset) {
  if (offset > bytes.size() || bytes.size() - offset < 4) {
    throw std::invalid_argument("VOICEVOX returned an invalid WAV payload");
  }
  return static_cast<std::uint32_t>(bytes[offset]) |
         static_cast<std::uint32_t>(bytes[offset + 1]) << 8U |
         static_cast<std::uint32_t>(bytes[offset + 2]) << 16U |
         static_cast<std::uint32_t>(bytes[offset + 3]) << 24U;
}

bool tag_at(const std::vector<std::uint8_t> &bytes, std::size_t offset,
            const char (&tag)[5]) {
  return offset <= bytes.size() && bytes.size() - offset >= 4 &&
         std::memcmp(bytes.data() + offset, tag, 4) == 0;
}

}  // namespace

struct VoicevoxCoreBackend::Impl {
  struct ModelRef {
    fs::path path;
    std::array<std::uint8_t, 16> id;
  };

  explicit Impl(const VoicevoxConfig &config) {
    if (config.onnxruntime_filename.empty() ||
        config.open_jtalk_dict_dir.empty() || config.voice_model_path.empty()) {
      throw std::invalid_argument(
          "onnxruntime_filename, open_jtalk_dict_dir, and voice_model_path are required");
    }

    const fs::path runtime = fs::absolute(expand_user(config.onnxruntime_filename));
    const fs::path dictionary =
        fs::absolute(expand_user(config.open_jtalk_dict_dir));
    const auto model_paths = expand_model_paths(config.voice_model_path);
    if (!fs::is_regular_file(runtime)) {
      throw std::runtime_error("VOICEVOX ONNX Runtime was not found: " +
                               runtime.string());
    }
    if (!fs::is_directory(dictionary)) {
      throw std::runtime_error("Open JTalk dictionary was not found: " +
                               dictionary.string());
    }
    if (model_paths.empty()) {
      throw std::runtime_error(
          "VOICEVOX voice model pattern matched no files: " +
          config.voice_model_path);
    }
    for (const auto &path : model_paths) {
      if (!fs::is_regular_file(path)) {
        throw std::runtime_error("VOICEVOX voice model was not found: " +
                                 path.string());
      }
    }

    // 合成結果の同一性。エンジン版数・ONNX Runtime・モデル群のどれかが
    // 変われば同じ文字列でも別の音になりうる。モデルはバージョン付きの
    // ディレクトリに入るので、パスの一覧を identity に含めれば足りる。
    identity_ = std::string(voicevox_get_version()) +
                "|aspa-source-map-v1|" + runtime.string();
    for (const auto &path : model_paths) {
      identity_ += "|" + path.string();
    }

    const auto acceleration = lower_ascii(config.acceleration_mode);
    VoicevoxAccelerationMode acceleration_mode;
    if (acceleration == "auto") {
      acceleration_mode = VOICEVOX_ACCELERATION_MODE_AUTO;
    } else if (acceleration == "cpu") {
      acceleration_mode = VOICEVOX_ACCELERATION_MODE_CPU;
    } else if (acceleration == "gpu") {
      acceleration_mode = VOICEVOX_ACCELERATION_MODE_GPU;
    } else {
      throw std::invalid_argument(
          "acceleration_mode must be auto, cpu, or gpu");
    }

    auto load_options = voicevox_make_default_load_onnxruntime_options();
    const auto runtime_string = runtime.string();
    load_options.filename = runtime_string.c_str();
    check_result(voicevox_onnxruntime_load_once(load_options, &onnxruntime_),
                 "failed to load VOICEVOX ONNX Runtime");

    OpenJtalkRc *raw_open_jtalk = nullptr;
    const auto dictionary_string = dictionary.string();
    check_result(
        voicevox_open_jtalk_rc_new(dictionary_string.c_str(), &raw_open_jtalk),
        "failed to load Open JTalk dictionary");
    open_jtalk_.reset(raw_open_jtalk);

    auto initialize_options = voicevox_make_default_initialize_options();
    initialize_options.acceleration_mode = acceleration_mode;
    initialize_options.cpu_num_threads = config.cpu_num_threads;
    VoicevoxSynthesizer *raw_synthesizer = nullptr;
    check_result(voicevox_synthesizer_new(onnxruntime_, open_jtalk_.get(),
                                          initialize_options,
                                          &raw_synthesizer),
                 "failed to initialize VOICEVOX synthesizer");
    synthesizer_.reset(raw_synthesizer);

    for (const auto &path : model_paths) {
      auto model = open_model(path);
      const ModelRef reference{path, model_id(model.get())};
      JsonPtr metadata(voicevox_voice_model_file_create_metas_json(model.get()));
      if (!metadata) {
        throw std::runtime_error("VOICEVOX model returned no metadata: " +
                                 path.string());
      }
      nlohmann::json characters;
      try {
        characters = nlohmann::json::parse(metadata.get());
      } catch (const nlohmann::json::exception &error) {
        throw std::runtime_error("VOICEVOX model returned invalid metadata: " +
                                 path.string() + ": " + error.what());
      }
      if (!characters.is_array()) {
        throw std::runtime_error("VOICEVOX model metadata is not an array: " +
                                 path.string());
      }
      for (const auto &character : characters) {
        const auto speaker = character.at("name").get<std::string>();
        for (const auto &style : character.at("styles")) {
          const auto type = style.at("type").get<std::string>();
          if (type != "talk") {
            continue;
          }
          const auto id = style.at("id").get<std::uint32_t>();
          const auto style_name = style.at("name").get<std::string>();
          if (model_by_style_.count(id) != 0U) {
            throw std::runtime_error(
                "VOICEVOX model contains duplicate style id " +
                std::to_string(id));
          }
          voices_.push_back(
              Voice{id, speaker, style_name, speaker + " / " + style_name});
          model_by_style_.emplace(id, reference);
        }
      }
    }
    if (voices_.empty()) {
      throw std::runtime_error("VOICEVOX models contain no talk styles");
    }
    std::sort(voices_.begin(), voices_.end(),
              [](const Voice &left, const Voice &right) {
                return left.speaker < right.speaker ||
                       (left.speaker == right.speaker && left.id < right.id);
              });
    set_style_id(config.style_id);
  }

  std::uint32_t style_id() const {
    std::lock_guard<std::mutex> lock(style_mutex_);
    return style_id_;
  }

  bool set_style_id(std::uint32_t style_id) {
    const auto entry = model_by_style_.find(style_id);
    if (entry == model_by_style_.end()) {
      std::string available;
      for (const auto &[candidate, unused] : model_by_style_) {
        (void)unused;
        if (!available.empty()) {
          available += ", ";
        }
        available += std::to_string(candidate);
      }
      throw std::invalid_argument("VOICEVOX style_id " +
                                  std::to_string(style_id) +
                                  " is not available; available: " + available);
    }

    std::lock_guard<std::mutex> lock(style_mutex_);
    if (loaded_model_.has_value() && style_id_ == style_id) {
      return false;
    }
    const auto &target = entry->second;
    if (!loaded_model_.has_value() || loaded_model_->id != target.id) {
      auto model = open_model(target.path);
      if (model_id(model.get()) != target.id) {
        throw std::runtime_error(
            "VOICEVOX model changed after catalog scan: " +
            target.path.string());
      }
      check_result(
          voicevox_synthesizer_load_voice_model(synthesizer_.get(), model.get()),
          "failed to load VOICEVOX style " + std::to_string(style_id));

      if (loaded_model_.has_value()) {
        std::uint8_t previous_storage[16]{};
        const auto previous_id = c_model_id(loaded_model_->id, previous_storage);
        const auto unload_result = voicevox_synthesizer_unload_voice_model(
            synthesizer_.get(), previous_id);
        if (unload_result != VOICEVOX_RESULT_OK) {
          std::uint8_t target_storage[16]{};
          const auto target_id = c_model_id(target.id, target_storage);
          (void)voicevox_synthesizer_unload_voice_model(synthesizer_.get(),
                                                        target_id);
          throw std::runtime_error(
              "failed to unload previous VOICEVOX model: " +
              result_message(unload_result));
        }
      }
      loaded_model_ = target;
    }
    style_id_ = style_id;
    return true;
  }

  SynthesizedAudio synthesize(const std::string &text) {
    std::lock_guard<std::mutex> lock(style_mutex_);
    char *raw_analysis = nullptr;
    check_result(voicevox_open_jtalk_rc_analyze_with_source_map(
                     open_jtalk_.get(), text.c_str(), &raw_analysis),
                 "failed to analyze VOICEVOX source alignment");
    JsonPtr analysis_json(raw_analysis);

    nlohmann::json analysis;
    try {
      analysis = nlohmann::json::parse(analysis_json.get());
    } catch (const nlohmann::json::exception &error) {
      throw std::runtime_error(
          "VOICEVOX returned invalid source alignment JSON: " +
          std::string(error.what()));
    }
    if (!analysis.is_object() || !analysis.contains("accent_phrases") ||
        !analysis.contains("source_spans")) {
      throw std::runtime_error(
          "VOICEVOX source alignment JSON is missing required fields");
    }

    const auto accent_phrases = analysis.at("accent_phrases").dump();
    char *raw_predicted_phrases = nullptr;
    check_result(voicevox_synthesizer_replace_mora_data(
                     synthesizer_.get(), accent_phrases.c_str(), style_id_,
                     &raw_predicted_phrases),
                 "failed to predict VOICEVOX mora data");
    JsonPtr predicted_phrases(raw_predicted_phrases);

    char *raw_query = nullptr;
    check_result(voicevox_audio_query_create_from_accent_phrases(
                     predicted_phrases.get(), &raw_query),
                 "failed to create VOICEVOX audio query");
    JsonPtr query(raw_query);

    std::uintptr_t wav_length = 0;
    std::uint8_t *raw_wav = nullptr;
    check_result(voicevox_synthesizer_synthesis(
                     synthesizer_.get(), query.get(), style_id_,
                     voicevox_make_default_synthesis_options(), &wav_length,
                     &raw_wav),
                 "VOICEVOX synthesis failed");
    WavPtr wav(raw_wav);
    if (!wav || wav_length == 0 ||
        wav_length > std::numeric_limits<std::size_t>::max()) {
      throw std::runtime_error("VOICEVOX returned empty or malformed audio");
    }
    auto audio = decode_pcm16_wav(std::vector<std::uint8_t>(
        wav.get(), wav.get() + static_cast<std::size_t>(wav_length)));
    const auto frame_bytes = static_cast<std::uint64_t>(audio.channels) * 2ULL;
    const auto total_frames = audio.pcm.size() / frame_bytes;
    nlohmann::json query_value;
    try {
      query_value = nlohmann::json::parse(query.get());
    } catch (const nlohmann::json::exception &error) {
      throw std::runtime_error("VOICEVOX returned invalid AudioQuery JSON: " +
                               std::string(error.what()));
    }
    audio.marks = build_synthesis_marks(
        text, analysis.at("source_spans"), query_value,
        static_cast<std::uint64_t>(total_frames));
    return audio;
  }

  std::string identity_;
  const VoicevoxOnnxruntime *onnxruntime_{nullptr};
  OpenJtalkPtr open_jtalk_;
  SynthesizerPtr synthesizer_;
  std::vector<Voice> voices_;
  std::map<std::uint32_t, ModelRef> model_by_style_;
  mutable std::mutex style_mutex_;
  std::uint32_t style_id_{std::numeric_limits<std::uint32_t>::max()};
  std::optional<ModelRef> loaded_model_;
};

VoicevoxCoreBackend::VoicevoxCoreBackend(const VoicevoxConfig &config)
    : impl_(std::make_unique<Impl>(config)) {}

VoicevoxCoreBackend::~VoicevoxCoreBackend() = default;

const std::vector<Voice> &VoicevoxCoreBackend::voices() const {
  return impl_->voices_;
}

std::uint32_t VoicevoxCoreBackend::style_id() const {
  return impl_->style_id();
}

bool VoicevoxCoreBackend::set_style_id(std::uint32_t style_id) {
  return impl_->set_style_id(style_id);
}

SynthesizedAudio VoicevoxCoreBackend::synthesize(const std::string &text) {
  return impl_->synthesize(text);
}

const std::string &VoicevoxCoreBackend::identity() const {
  return impl_->identity_;
}

SynthesizedAudio decode_pcm16_wav(const std::vector<std::uint8_t> &wav) {
  if (wav.size() < 12 || !tag_at(wav, 0, "RIFF") ||
      !tag_at(wav, 8, "WAVE")) {
    throw std::invalid_argument("VOICEVOX returned an invalid WAV payload");
  }
  const std::size_t riff_end = static_cast<std::size_t>(read_u32(wav, 4)) + 8U;
  if (riff_end < 12 || riff_end > wav.size()) {
    throw std::invalid_argument("VOICEVOX returned an invalid WAV payload");
  }

  bool found_format = false;
  std::uint16_t channels = 0;
  std::uint32_t sample_rate = 0;
  std::uint16_t bit_depth = 0;
  std::size_t data_offset = 0;
  std::size_t data_size = 0;
  for (std::size_t offset = 12; offset + 8 <= riff_end;) {
    const auto chunk_size = static_cast<std::size_t>(read_u32(wav, offset + 4));
    const auto chunk_data = offset + 8;
    if (chunk_data > riff_end || chunk_size > riff_end - chunk_data) {
      throw std::invalid_argument("VOICEVOX returned an invalid WAV payload");
    }
    if (tag_at(wav, offset, "fmt ")) {
      if (chunk_size < 16 || read_u16(wav, chunk_data) != 1) {
        throw std::invalid_argument(
            "VOICEVOX output must be uncompressed PCM16 WAV");
      }
      channels = read_u16(wav, chunk_data + 2);
      sample_rate = read_u32(wav, chunk_data + 4);
      bit_depth = read_u16(wav, chunk_data + 14);
      found_format = true;
    } else if (tag_at(wav, offset, "data") && data_size == 0) {
      data_offset = chunk_data;
      data_size = chunk_size;
    }
    const auto padded_size = chunk_size + (chunk_size & 1U);
    if (padded_size > riff_end - chunk_data) {
      throw std::invalid_argument("VOICEVOX returned an invalid WAV payload");
    }
    offset = chunk_data + padded_size;
  }

  if (!found_format || channels == 0 || sample_rate == 0 || bit_depth != 16 ||
      data_size == 0) {
    throw std::invalid_argument("VOICEVOX returned empty or malformed audio");
  }
  const auto frame_bytes = static_cast<std::size_t>(channels) * 2U;
  if (data_size % frame_bytes != 0) {
    throw std::invalid_argument("VOICEVOX returned empty or malformed audio");
  }
  return SynthesizedAudio{
      std::vector<std::uint8_t>(wav.begin() + static_cast<std::ptrdiff_t>(data_offset),
                                wav.begin() + static_cast<std::ptrdiff_t>(data_offset + data_size)),
      sample_rate, channels, bit_depth, {}};
}

}  // namespace fv_tts
