#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <list>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "fv_tts/synthesis_scheduler.hpp"

namespace fv_tts {

struct SynthesisCacheConfig {
  // 空なら永続層なし (プロセス内だけのキャッシュになる)
  std::filesystem::path directory;
  // 0 なら memory 層なし
  std::size_t memory_entries{64};
  // 永続層の上限。超えたら古いものから捨てる
  std::uintmax_t disk_budget_bytes{256ULL * 1024ULL * 1024ULL};
};

// 合成済みPCMの使い回し。
//
// 音声対話では同じ文字列が何度も来る — フィラー(「えっと。」「うん。」)、
// 定型の結論(「アスパだよ。」)、システム通知。VOICEVOXはCPU実行で1文
// 300ms〜1s かかるので、2回目以降を合成し直す理由がない。
//
// 「事前合成バンク」を手で用意する方式は採らない。話者(style_id)や
// エンジン版数を変えるたびに作り直しが要るし、実際に使われる文と
// バンクの中身がずれる。キャッシュならキーに identity と style_id が
// 入るので、変わったら当たらなくなるだけで、無効化処理も要らない。
//
// キーは (identity, style_id, text) のハッシュだが、ハッシュ衝突と
// 他人のファイルを掴まないよう、**読み出し時に中身の identity/style/text を
// 必ず突き合わせる**。一致しなければ miss として扱う (例外は投げない —
// キャッシュの不調で発話が止まってはいけない)。
class SynthesisCache {
 public:
  using ErrorLogger = std::function<void(const std::string &)>;

  // identity: エンジンの同一性 (voicevox版数 + ONNX Runtime + モデル群)。
  // これが変われば同じ文字列でも別キーになる。
  SynthesisCache(SynthesisCacheConfig config, std::string identity,
                 ErrorLogger on_error = {});

  SynthesisCache(const SynthesisCache &) = delete;
  SynthesisCache &operator=(const SynthesisCache &) = delete;

  std::optional<SynthesizedAudio> lookup(std::uint32_t style_id,
                                         const std::string &text);
  void store(std::uint32_t style_id, const std::string &text,
             const SynthesizedAudio &audio);
  bool contains(std::uint32_t style_id, const std::string &text);

  std::size_t hits() const;
  std::size_t misses() const;
  std::uintmax_t disk_bytes() const;

  // ---- 単体テスト用に公開している純粋関数 ----
  static std::string entry_name(const std::string &identity,
                                std::uint32_t style_id,
                                const std::string &text);
  static std::vector<std::uint8_t> encode(const std::string &identity,
                                          std::uint32_t style_id,
                                          const std::string &text,
                                          const SynthesizedAudio &audio);
  static std::optional<SynthesizedAudio> decode(
      const std::vector<std::uint8_t> &blob, const std::string &identity,
      std::uint32_t style_id, const std::string &text);

 private:
  struct MemoryEntry {
    std::list<std::string>::iterator order;
    SynthesizedAudio audio;
  };

  std::optional<SynthesizedAudio> lookup_memory_locked(const std::string &name);
  void store_memory_locked(const std::string &name,
                           const SynthesizedAudio &audio);
  std::optional<SynthesizedAudio> read_disk(const std::string &name,
                                            std::uint32_t style_id,
                                            const std::string &text);
  void write_disk(const std::string &name, std::uint32_t style_id,
                  const std::string &text, const SynthesizedAudio &audio);
  void enforce_budget_locked();
  void report(const std::string &message) const;

  SynthesisCacheConfig config_;
  std::string identity_;
  ErrorLogger on_error_;
  std::list<std::string> order_;
  std::unordered_map<std::string, MemoryEntry> memory_;
  std::uintmax_t disk_bytes_{0};
  std::uint64_t write_counter_{0};
  std::size_t hits_{0};
  std::size_t misses_{0};
  mutable std::mutex mutex_;
};

// 先頭の "~" をHOMEへ展開する。空文字はそのまま空を返す。
std::filesystem::path expand_user_path(const std::string &value);

}  // namespace fv_tts
