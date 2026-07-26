#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "fv_tts/contracts.hpp"
#include "fv_tts/synthesis_scheduler.hpp"

namespace fv_tts {

struct VoicevoxConfig {
  std::string onnxruntime_filename;
  std::string open_jtalk_dict_dir;
  std::string voice_model_path;
  std::string acceleration_mode;
  std::uint16_t cpu_num_threads;
  std::uint32_t style_id;
};

class VoicevoxCoreBackend {
 public:
  explicit VoicevoxCoreBackend(const VoicevoxConfig &config);
  ~VoicevoxCoreBackend();

  VoicevoxCoreBackend(const VoicevoxCoreBackend &) = delete;
  VoicevoxCoreBackend &operator=(const VoicevoxCoreBackend &) = delete;

  const std::vector<Voice> &voices() const;
  std::uint32_t style_id() const;
  bool set_style_id(std::uint32_t style_id);
  SynthesizedAudio synthesize(const std::string &text);

  // 合成結果を再利用してよい範囲を表す文字列 (voicevox_core版数 + ONNX
  // Runtime + モデル群)。どれかが変われば別の音になりうるので、
  // キャッシュのキーに混ぜる。
  const std::string &identity() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

SynthesizedAudio decode_pcm16_wav(const std::vector<std::uint8_t> &wav);

}  // namespace fv_tts
