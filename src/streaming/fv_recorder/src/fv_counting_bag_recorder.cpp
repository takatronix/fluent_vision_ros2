#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <rosbag2_transport/recorder.hpp>

namespace
{

std::atomic_bool g_stop_requested{false};

void handle_signal(int)
{
  g_stop_requested.store(true);
}

struct Args
{
  std::string output;
  std::string storage = "sqlite3";
  uint64_t max_bag_size = 0;
  std::string ready_file;
  std::vector<std::string> topics;
  std::unordered_set<std::string> ready_topics;
};

std::string take_value(int & index, int argc, char ** argv)
{
  if (index + 1 >= argc) {
    throw std::runtime_error(std::string("missing value for ") + argv[index]);
  }
  ++index;
  return argv[index];
}

Args parse_args(int argc, char ** argv)
{
  Args args;
  for (int index = 1; index < argc; ++index) {
    const std::string key = argv[index];
    if (key == "--output") {
      args.output = take_value(index, argc, argv);
    } else if (key == "--storage") {
      args.storage = take_value(index, argc, argv);
    } else if (key == "--max-bag-size") {
      args.max_bag_size = std::stoull(take_value(index, argc, argv));
    } else if (key == "--ready-file") {
      args.ready_file = take_value(index, argc, argv);
    } else if (key == "--topic") {
      args.topics.push_back(take_value(index, argc, argv));
    } else if (key == "--ready-topic") {
      args.ready_topics.insert(take_value(index, argc, argv));
    } else {
      throw std::runtime_error("unknown argument: " + key);
    }
  }
  if (args.output.empty()) {
    throw std::runtime_error("--output is required");
  }
  if (args.ready_file.empty()) {
    throw std::runtime_error("--ready-file is required");
  }
  return args;
}

uint64_t unix_time_ns()
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

class ReadyState
{
public:
  ReadyState(std::filesystem::path ready_file, std::unordered_set<std::string> required_topics)
  : ready_file_(std::move(ready_file)), required_topics_(std::move(required_topics))
  {
    for (const auto & topic : required_topics_) {
      counts_[topic] = 0;
    }
    ready_ = required_topics_.empty();
    ready_at_unix_ns_ = ready_ ? unix_time_ns() : 0;
    write_status(ready_, ready_at_unix_ns_);
  }

  void mark_written(const std::string & topic)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto found = counts_.find(topic);
    if (found == counts_.end()) {
      return;
    }
    found->second += 1;
    const bool became_ready = !ready_ && all_required_seen();
    if (became_ready) {
      ready_ = true;
      ready_at_unix_ns_ = unix_time_ns();
    }
    if (found->second == 1 || became_ready) {
      write_status(ready_, ready_at_unix_ns_);
    }
  }

private:
  bool all_required_seen() const
  {
    for (const auto & topic : required_topics_) {
      const auto count = counts_.find(topic);
      if (count == counts_.end() || count->second <= 0) {
        return false;
      }
    }
    return true;
  }

  void write_status(bool ready, uint64_t ready_at_unix_ns) const
  {
    std::filesystem::create_directories(ready_file_.parent_path());
    nlohmann::json body;
    body["ready"] = ready;
    body["ready_at_unix_ns"] = ready_at_unix_ns;
    body["bag_topics"] = counts_;
    const auto tmp = ready_file_.string() + ".tmp";
    {
      std::ofstream out(tmp, std::ios::out | std::ios::trunc);
      if (!out.is_open()) {
        throw std::runtime_error("failed to open ready status file: " + tmp);
      }
      out << body.dump();
      out << "\n";
    }
    std::filesystem::rename(tmp, ready_file_);
  }

  std::filesystem::path ready_file_;
  std::unordered_set<std::string> required_topics_;
  std::unordered_map<std::string, int64_t> counts_;
  uint64_t ready_at_unix_ns_ = 0;
  bool ready_ = false;
  mutable std::mutex mutex_;
};

class CountingSequentialWriter : public rosbag2_cpp::writers::SequentialWriter
{
public:
  explicit CountingSequentialWriter(std::shared_ptr<ReadyState> ready_state)
  : ready_state_(std::move(ready_state))
  {}

  void write(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message) override
  {
    rosbag2_cpp::writers::SequentialWriter::write(message);
    ready_state_->mark_written(message->topic_name);
  }

private:
  std::shared_ptr<ReadyState> ready_state_;
};

}  // namespace

int main(int argc, char ** argv)
{
  Args args;
  try {
    args = parse_args(argc, argv);
  } catch (const std::exception & exc) {
    std::cerr << "fv_counting_bag_recorder: " << exc.what() << "\n";
    return 2;
  }

  rclcpp::init(argc, argv);
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
  try {
    auto ready_state = std::make_shared<ReadyState>(
      std::filesystem::path(args.ready_file),
      args.ready_topics);
    auto writer = std::make_shared<rosbag2_cpp::Writer>(
      std::make_unique<CountingSequentialWriter>(ready_state));

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = args.output;
    storage_options.storage_id = args.storage;
    storage_options.max_bagfile_size = args.max_bag_size;

    rosbag2_transport::RecordOptions record_options;
    record_options.all = false;
    record_options.is_discovery_disabled = false;
    record_options.topics = args.topics;
    record_options.rmw_serialization_format = rmw_get_serialization_format();
    record_options.topic_polling_interval = std::chrono::milliseconds(50);

    auto recorder = std::make_shared<rosbag2_transport::Recorder>(
      writer,
      storage_options,
      record_options,
      "fv_counting_bag_recorder");
    recorder->record();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(recorder);
    while (rclcpp::ok() && !g_stop_requested.load()) {
      executor.spin_some(std::chrono::milliseconds(50));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    recorder->stop();
    rclcpp::shutdown();
  } catch (const std::exception & exc) {
    std::cerr << "fv_counting_bag_recorder: " << exc.what() << "\n";
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  return g_stop_requested.load() ? 130 : 0;
}
