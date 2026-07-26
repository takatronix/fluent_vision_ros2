#include <gtest/gtest.h>

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "fv_tts/synthesis_cache.hpp"

namespace fv_tts {
namespace {

namespace fs = std::filesystem;

SynthesizedAudio make_audio(std::uint8_t seed, std::size_t frames = 8) {
  SynthesizedAudio audio;
  audio.pcm.assign(frames * 2U, seed);
  audio.sample_rate_hz = 24000;
  audio.channels = 1;
  audio.bit_depth = 16;
  audio.marks = {
      SynthesisMark{0, 1, "あ", "ア", 0, 1, 0,
                    static_cast<std::uint64_t>(frames)}};
  return audio;
}

class TemporaryDirectory {
 public:
  TemporaryDirectory() {
    static std::atomic<unsigned> counter{0};
    path_ = fs::temp_directory_path() /
            ("fv_tts_cache_test_" + std::to_string(::getpid()) + "_" +
             std::to_string(counter++));
    fs::create_directories(path_);
  }
  ~TemporaryDirectory() {
    std::error_code ignored;
    fs::remove_all(path_, ignored);
  }
  const fs::path &path() const { return path_; }

 private:
  fs::path path_;
};

SynthesisCacheConfig disk_config(const fs::path &directory) {
  SynthesisCacheConfig config;
  config.directory = directory;
  config.memory_entries = 4;
  config.disk_budget_bytes = 1024ULL * 1024ULL;
  return config;
}

}  // namespace

TEST(SynthesisCache, StoresAndReturnsTheSameAudio) {
  TemporaryDirectory directory;
  SynthesisCache cache(disk_config(directory.path()), "engine-1");
  const auto audio = make_audio(7);

  EXPECT_FALSE(cache.lookup(30, "えっと。").has_value());
  cache.store(30, "えっと。", audio);

  const auto hit = cache.lookup(30, "えっと。");
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->pcm, audio.pcm);
  EXPECT_EQ(hit->sample_rate_hz, audio.sample_rate_hz);
  EXPECT_EQ(hit->channels, audio.channels);
  EXPECT_EQ(hit->marks, audio.marks);
  EXPECT_EQ(cache.hits(), 1U);
  EXPECT_EQ(cache.misses(), 1U);
}

TEST(SynthesisCache, SurvivesAcrossInstances) {
  TemporaryDirectory directory;
  {
    SynthesisCache writer(disk_config(directory.path()), "engine-1");
    writer.store(30, "うん。", make_audio(3));
  }
  SynthesisCache reader(disk_config(directory.path()), "engine-1");

  const auto hit = reader.lookup(30, "うん。");
  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->pcm, make_audio(3).pcm);
}

TEST(SynthesisCache, DoesNotCrossStyleOrEngine) {
  TemporaryDirectory directory;
  {
    SynthesisCache writer(disk_config(directory.path()), "engine-1");
    writer.store(30, "うん。", make_audio(3));
  }

  SynthesisCache other_style(disk_config(directory.path()), "engine-1");
  EXPECT_FALSE(other_style.lookup(31, "うん。").has_value());

  SynthesisCache other_engine(disk_config(directory.path()), "engine-2");
  EXPECT_FALSE(other_engine.lookup(30, "うん。").has_value());
}

TEST(SynthesisCache, VerifiesTheStoredKeyInsteadOfTrustingTheHash) {
  const auto blob = SynthesisCache::encode("engine-1", 30, "うん。",
                                           make_audio(3));

  EXPECT_TRUE(SynthesisCache::decode(blob, "engine-1", 30, "うん。").has_value());
  // 名前が衝突しても中身が違えば拾わない
  EXPECT_FALSE(SynthesisCache::decode(blob, "engine-1", 30, "ちがう。").has_value());
  EXPECT_FALSE(SynthesisCache::decode(blob, "engine-2", 30, "うん。").has_value());
  EXPECT_FALSE(SynthesisCache::decode(blob, "engine-1", 31, "うん。").has_value());
}

TEST(SynthesisCache, TreatsCorruptFilesAsMisses) {
  TemporaryDirectory directory;
  SynthesisCache cache(disk_config(directory.path()), "engine-1");
  const auto name = SynthesisCache::entry_name("engine-1", 30, "壊れた。");
  {
    std::ofstream output(directory.path() / name, std::ios::binary);
    output << "not a cache entry at all";
  }

  EXPECT_NO_THROW({ EXPECT_FALSE(cache.lookup(30, "壊れた。").has_value()); });

  // 0 バイトでも同じ (書き込み中に落ちた場合)
  { std::ofstream truncate(directory.path() / name, std::ios::trunc); }
  EXPECT_FALSE(cache.lookup(30, "壊れた。").has_value());
}

TEST(SynthesisCache, RejectsTruncatedPayloads) {
  auto blob = SynthesisCache::encode("engine-1", 30, "うん。", make_audio(3));
  blob.resize(blob.size() - 4);

  EXPECT_FALSE(SynthesisCache::decode(blob, "engine-1", 30, "うん。").has_value());
}

TEST(SynthesisCache, WorksWithoutADiskLayer) {
  SynthesisCacheConfig config;
  config.memory_entries = 2;
  SynthesisCache cache(config, "engine-1");

  cache.store(30, "うん。", make_audio(3));
  EXPECT_TRUE(cache.lookup(30, "うん。").has_value());
  EXPECT_EQ(cache.disk_bytes(), 0U);
}

TEST(SynthesisCache, EvictsTheLeastRecentlyUsedMemoryEntry) {
  SynthesisCacheConfig config;
  config.memory_entries = 2;
  SynthesisCache cache(config, "engine-1");

  cache.store(30, "一。", make_audio(1));
  cache.store(30, "二。", make_audio(2));
  ASSERT_TRUE(cache.lookup(30, "一。").has_value());  // 一 を最近使用にする
  cache.store(30, "三。", make_audio(3));

  EXPECT_TRUE(cache.lookup(30, "一。").has_value());
  EXPECT_TRUE(cache.lookup(30, "三。").has_value());
  EXPECT_FALSE(cache.lookup(30, "二。").has_value());
}

TEST(SynthesisCache, KeepsTheDiskLayerUnderBudget) {
  TemporaryDirectory directory;
  SynthesisCacheConfig config;
  config.directory = directory.path();
  config.memory_entries = 0;  // memory に逃がさず、必ずディスクを見る
  config.disk_budget_bytes = 2048;
  SynthesisCache cache(config, "engine-1");

  for (int index = 0; index < 12; ++index) {
    cache.store(30, "文" + std::to_string(index),
                make_audio(static_cast<std::uint8_t>(index), 256));
    // last_write_time の分解能で順序が潰れないよう間隔を空ける
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_LE(cache.disk_bytes(), config.disk_budget_bytes);
  EXPECT_TRUE(cache.lookup(30, "文11").has_value());
  EXPECT_FALSE(cache.lookup(30, "文0").has_value());
}

TEST(SynthesisCache, ReportsButSurvivesAnUnusableDirectory) {
  TemporaryDirectory directory;
  const auto blocked = directory.path() / "not-a-directory";
  { std::ofstream output(blocked); output << "x"; }

  std::vector<std::string> errors;
  SynthesisCacheConfig config;
  config.directory = blocked;
  config.memory_entries = 2;
  SynthesisCache cache(config, "engine-1",
                       [&errors](const std::string &message) {
                         errors.push_back(message);
                       });

  EXPECT_FALSE(errors.empty());
  // ディスクが使えなくても memory 層として動き続ける
  cache.store(30, "うん。", make_audio(3));
  EXPECT_TRUE(cache.lookup(30, "うん。").has_value());
}

TEST(SynthesisCache, ContainsDoesNotCountAsAHitOrMiss) {
  TemporaryDirectory directory;
  SynthesisCache cache(disk_config(directory.path()), "engine-1");
  cache.store(30, "うん。", make_audio(3));

  EXPECT_TRUE(cache.contains(30, "うん。"));
  EXPECT_FALSE(cache.contains(30, "まだ。"));
  EXPECT_EQ(cache.hits(), 0U);
  EXPECT_EQ(cache.misses(), 0U);
}

TEST(ExpandUserPath, ExpandsOnlyALeadingTilde) {
  const std::string home = std::getenv("HOME") != nullptr
                               ? std::getenv("HOME")
                               : std::string{};
  if (!home.empty()) {
    EXPECT_EQ(expand_user_path("~/x/y"), fs::path(home) / "x/y");
  }
  EXPECT_EQ(expand_user_path("/tmp/~/x"), fs::path("/tmp/~/x"));
  EXPECT_EQ(expand_user_path("relative/~x"), fs::path("relative/~x"));
  EXPECT_TRUE(expand_user_path("").empty());
}

}  // namespace fv_tts
