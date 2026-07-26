#include "fv_tts/synthesis_cache.hpp"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <system_error>
#include <utility>

namespace fv_tts {
namespace {

constexpr std::array<char, 4> kMagic{'F', 'V', 'T', 'C'};
// 形式を変えたら上げる。古いファイルは decode が弾いて miss になるだけで、
// 消さなくても害はない (予算超過で古い順に落ちる)。
constexpr std::uint32_t kFormatVersion = 1;
constexpr const char *kSuffix = ".fvtc";

std::uint64_t fnv1a64(const std::string &value, std::uint64_t seed) {
  std::uint64_t hash = seed;
  for (const unsigned char byte : value) {
    hash ^= static_cast<std::uint64_t>(byte);
    hash *= 0x00000100000001B3ULL;
  }
  return hash;
}

std::string to_hex(std::uint64_t value) {
  static constexpr char kDigits[] = "0123456789abcdef";
  std::string out(16, '0');
  for (int index = 15; index >= 0; --index) {
    out[static_cast<std::size_t>(index)] = kDigits[value & 0xFULL];
    value >>= 4;
  }
  return out;
}

void append_u32(std::vector<std::uint8_t> &out, std::uint32_t value) {
  out.push_back(static_cast<std::uint8_t>(value & 0xFFU));
  out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFFU));
  out.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFFU));
  out.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFFU));
}

void append_u64(std::vector<std::uint8_t> &out, std::uint64_t value) {
  for (int index = 0; index < 8; ++index) {
    out.push_back(static_cast<std::uint8_t>((value >> (8 * index)) & 0xFFULL));
  }
}

bool read_u32(const std::vector<std::uint8_t> &blob, std::size_t &offset,
              std::uint32_t &value) {
  if (blob.size() < 4 || offset > blob.size() - 4) {
    return false;
  }
  value = static_cast<std::uint32_t>(blob[offset]) |
          (static_cast<std::uint32_t>(blob[offset + 1]) << 8) |
          (static_cast<std::uint32_t>(blob[offset + 2]) << 16) |
          (static_cast<std::uint32_t>(blob[offset + 3]) << 24);
  offset += 4;
  return true;
}

bool read_u64(const std::vector<std::uint8_t> &blob, std::size_t &offset,
              std::uint64_t &value) {
  if (blob.size() < 8 || offset > blob.size() - 8) {
    return false;
  }
  value = 0;
  for (int index = 0; index < 8; ++index) {
    value |= static_cast<std::uint64_t>(blob[offset + static_cast<std::size_t>(index)])
             << (8 * index);
  }
  offset += 8;
  return true;
}

bool read_bytes(const std::vector<std::uint8_t> &blob, std::size_t &offset,
                std::uint64_t length, std::string &value) {
  if (length > blob.size() || offset > blob.size() - length) {
    return false;
  }
  value.assign(blob.begin() + static_cast<std::ptrdiff_t>(offset),
               blob.begin() + static_cast<std::ptrdiff_t>(offset + length));
  offset += static_cast<std::size_t>(length);
  return true;
}

}  // namespace

std::filesystem::path expand_user_path(const std::string &value) {
  if (value.empty()) {
    return {};
  }
  if (value == "~" || value.rfind("~/", 0) == 0) {
    const char *home = std::getenv("HOME");
    if (home != nullptr && *home != '\0') {
      return std::filesystem::path(home) / value.substr(value == "~" ? 1 : 2);
    }
  }
  return std::filesystem::path(value);
}

SynthesisCache::SynthesisCache(SynthesisCacheConfig config,
                               std::string identity, ErrorLogger on_error)
    : config_(std::move(config)),
      identity_(std::move(identity)),
      on_error_(std::move(on_error)) {
  if (config_.directory.empty()) {
    return;
  }
  std::error_code error;
  std::filesystem::create_directories(config_.directory, error);
  if (error) {
    report("TTS cache directory is unusable (" + config_.directory.string() +
           "): " + error.message());
    config_.directory.clear();
    return;
  }
  // 起動時に一度だけ実測する。以降は書き込み分を足し込むだけにして、
  // 合成のたびにディレクトリを舐めない。
  for (std::filesystem::directory_iterator iterator(config_.directory, error),
       end;
       !error && iterator != end; iterator.increment(error)) {
    if (iterator->path().extension() != kSuffix) {
      continue;
    }
    std::error_code size_error;
    const auto size = std::filesystem::file_size(iterator->path(), size_error);
    if (!size_error) {
      disk_bytes_ += size;
    }
  }
  if (error) {
    report("TTS cache directory scan failed: " + error.message());
  }
}

std::string SynthesisCache::entry_name(const std::string &identity,
                                       std::uint32_t style_id,
                                       const std::string &text) {
  // identity と style を混ぜてから text を流す。話者やエンジンが変われば
  // 同じ文字列でも別のキーになる。
  std::uint64_t hash = fnv1a64(identity, 0xCBF29CE484222325ULL);
  hash = fnv1a64("|" + std::to_string(style_id) + "|", hash);
  hash = fnv1a64(text, hash);
  return to_hex(hash) + kSuffix;
}

std::vector<std::uint8_t> SynthesisCache::encode(const std::string &identity,
                                                 std::uint32_t style_id,
                                                 const std::string &text,
                                                 const SynthesizedAudio &audio) {
  std::vector<std::uint8_t> blob;
  blob.reserve(64 + identity.size() + text.size() + audio.pcm.size());
  blob.insert(blob.end(), kMagic.begin(), kMagic.end());
  append_u32(blob, kFormatVersion);
  append_u32(blob, style_id);
  append_u32(blob, audio.sample_rate_hz);
  append_u32(blob, audio.channels);
  append_u32(blob, audio.bit_depth);
  append_u64(blob, static_cast<std::uint64_t>(identity.size()));
  append_u64(blob, static_cast<std::uint64_t>(text.size()));
  append_u64(blob, static_cast<std::uint64_t>(audio.pcm.size()));
  blob.insert(blob.end(), identity.begin(), identity.end());
  blob.insert(blob.end(), text.begin(), text.end());
  blob.insert(blob.end(), audio.pcm.begin(), audio.pcm.end());
  return blob;
}

std::optional<SynthesizedAudio> SynthesisCache::decode(
    const std::vector<std::uint8_t> &blob, const std::string &identity,
    std::uint32_t style_id, const std::string &text) {
  if (blob.size() < kMagic.size() ||
      !std::equal(kMagic.begin(), kMagic.end(), blob.begin())) {
    return std::nullopt;
  }
  std::size_t offset = kMagic.size();
  std::uint32_t format = 0;
  std::uint32_t stored_style = 0;
  SynthesizedAudio audio{};
  std::uint64_t identity_length = 0;
  std::uint64_t text_length = 0;
  std::uint64_t pcm_length = 0;
  if (!read_u32(blob, offset, format) || format != kFormatVersion ||
      !read_u32(blob, offset, stored_style) ||
      !read_u32(blob, offset, audio.sample_rate_hz) ||
      !read_u32(blob, offset, audio.channels) ||
      !read_u32(blob, offset, audio.bit_depth) ||
      !read_u64(blob, offset, identity_length) ||
      !read_u64(blob, offset, text_length) ||
      !read_u64(blob, offset, pcm_length)) {
    return std::nullopt;
  }
  std::string stored_identity;
  std::string stored_text;
  if (!read_bytes(blob, offset, identity_length, stored_identity) ||
      !read_bytes(blob, offset, text_length, stored_text)) {
    return std::nullopt;
  }
  // ハッシュ衝突と、他所のファイルを掴む事故はここで落とす。
  if (stored_style != style_id || stored_identity != identity ||
      stored_text != text) {
    return std::nullopt;
  }
  if (pcm_length > blob.size() - offset) {
    return std::nullopt;
  }
  if (audio.channels == 0 || audio.sample_rate_hz == 0 ||
      audio.bit_depth != 16 || pcm_length == 0 ||
      pcm_length % (static_cast<std::uint64_t>(audio.channels) * 2ULL) != 0) {
    return std::nullopt;
  }
  audio.pcm.assign(blob.begin() + static_cast<std::ptrdiff_t>(offset),
                   blob.begin() + static_cast<std::ptrdiff_t>(offset + pcm_length));
  return audio;
}

std::optional<SynthesizedAudio> SynthesisCache::lookup(std::uint32_t style_id,
                                                       const std::string &text) {
  const auto name = entry_name(identity_, style_id, text);
  std::unique_lock<std::mutex> lock(mutex_);
  if (auto hit = lookup_memory_locked(name)) {
    ++hits_;
    return hit;
  }
  const bool has_disk = !config_.directory.empty();
  lock.unlock();
  if (!has_disk) {
    std::lock_guard<std::mutex> miss_lock(mutex_);
    ++misses_;
    return std::nullopt;
  }
  auto disk = read_disk(name, style_id, text);
  std::lock_guard<std::mutex> commit_lock(mutex_);
  if (!disk) {
    ++misses_;
    return std::nullopt;
  }
  ++hits_;
  store_memory_locked(name, *disk);
  return disk;
}

bool SynthesisCache::contains(std::uint32_t style_id, const std::string &text) {
  const auto name = entry_name(identity_, style_id, text);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (memory_.count(name) != 0U) {
      return true;
    }
    if (config_.directory.empty()) {
      return false;
    }
  }
  return read_disk(name, style_id, text).has_value();
}

void SynthesisCache::store(std::uint32_t style_id, const std::string &text,
                           const SynthesizedAudio &audio) {
  if (audio.pcm.empty()) {
    return;
  }
  const auto name = entry_name(identity_, style_id, text);
  bool has_disk = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    store_memory_locked(name, audio);
    has_disk = !config_.directory.empty();
  }
  if (has_disk) {
    write_disk(name, style_id, text, audio);
  }
}

std::optional<SynthesizedAudio> SynthesisCache::lookup_memory_locked(
    const std::string &name) {
  const auto entry = memory_.find(name);
  if (entry == memory_.end()) {
    return std::nullopt;
  }
  order_.splice(order_.begin(), order_, entry->second.order);
  return entry->second.audio;
}

void SynthesisCache::store_memory_locked(const std::string &name,
                                         const SynthesizedAudio &audio) {
  if (config_.memory_entries == 0) {
    return;
  }
  const auto existing = memory_.find(name);
  if (existing != memory_.end()) {
    order_.splice(order_.begin(), order_, existing->second.order);
    existing->second.audio = audio;
    return;
  }
  order_.push_front(name);
  memory_.emplace(name, MemoryEntry{order_.begin(), audio});
  while (memory_.size() > config_.memory_entries) {
    memory_.erase(order_.back());
    order_.pop_back();
  }
}

std::optional<SynthesizedAudio> SynthesisCache::read_disk(
    const std::string &name, std::uint32_t style_id, const std::string &text) {
  const auto path = config_.directory / name;
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return std::nullopt;
  }
  std::vector<std::uint8_t> blob((std::istreambuf_iterator<char>(input)),
                                 std::istreambuf_iterator<char>());
  if (!input.eof() && input.fail()) {
    report("TTS cache read failed: " + path.string());
    return std::nullopt;
  }
  return decode(blob, identity_, style_id, text);
}

void SynthesisCache::write_disk(const std::string &name, std::uint32_t style_id,
                                const std::string &text,
                                const SynthesizedAudio &audio) {
  const auto blob = encode(identity_, style_id, text, audio);
  std::filesystem::path temporary;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    temporary = config_.directory /
                (name + ".tmp." + std::to_string(++write_counter_));
  }
  {
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    if (!output) {
      report("TTS cache write failed: " + temporary.string());
      return;
    }
    output.write(reinterpret_cast<const char *>(blob.data()),
                 static_cast<std::streamsize>(blob.size()));
    if (!output) {
      output.close();
      std::error_code ignored;
      std::filesystem::remove(temporary, ignored);
      report("TTS cache write failed: " + temporary.string());
      return;
    }
  }
  // 中途半端なファイルを読ませないため、書き終えてから名前を付ける。
  std::error_code error;
  std::filesystem::rename(temporary, config_.directory / name, error);
  if (error) {
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    report("TTS cache rename failed: " + error.message());
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  disk_bytes_ += blob.size();
  enforce_budget_locked();
}

void SynthesisCache::enforce_budget_locked() {
  if (config_.disk_budget_bytes == 0 ||
      disk_bytes_ <= config_.disk_budget_bytes) {
    return;
  }
  struct Victim {
    std::filesystem::path path;
    std::filesystem::file_time_type modified;
    std::uintmax_t size;
  };
  std::vector<Victim> victims;
  std::error_code error;
  for (std::filesystem::directory_iterator iterator(config_.directory, error),
       end;
       !error && iterator != end; iterator.increment(error)) {
    if (iterator->path().extension() != kSuffix) {
      continue;
    }
    std::error_code entry_error;
    const auto size = std::filesystem::file_size(iterator->path(), entry_error);
    const auto modified =
        std::filesystem::last_write_time(iterator->path(), entry_error);
    if (entry_error) {
      continue;
    }
    victims.push_back(Victim{iterator->path(), modified, size});
  }
  if (error) {
    report("TTS cache eviction scan failed: " + error.message());
    return;
  }
  std::sort(victims.begin(), victims.end(),
            [](const Victim &left, const Victim &right) {
              return left.modified < right.modified;
            });
  // 予算ぎりぎりで消し続けないよう、9割まで落としてから止める。
  const auto target = config_.disk_budget_bytes -
                      (config_.disk_budget_bytes / 10U);
  std::uintmax_t total = 0;
  for (const auto &victim : victims) {
    total += victim.size;
  }
  for (const auto &victim : victims) {
    if (total <= target) {
      break;
    }
    std::error_code remove_error;
    if (std::filesystem::remove(victim.path, remove_error) && !remove_error) {
      // memory 層はそのまま残す。ディスクから落ちただけで、プロセス内の
      // PCM は有効なままであり、捨てる理由がない。
      total -= victim.size;
    }
  }
  disk_bytes_ = total;
}

std::size_t SynthesisCache::hits() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return hits_;
}

std::size_t SynthesisCache::misses() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return misses_;
}

std::uintmax_t SynthesisCache::disk_bytes() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return disk_bytes_;
}

void SynthesisCache::report(const std::string &message) const {
  if (on_error_) {
    on_error_(message);
  }
}

}  // namespace fv_tts
