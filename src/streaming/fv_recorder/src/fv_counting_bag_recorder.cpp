#include <algorithm>
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
#include <rcutils/time.h>
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

class ReadyState
{
public:
  ReadyState(std::filesystem::path ready_file, std::unordered_set<std::string> required_topics)
  : ready_file_(std::move(ready_file)), required_topics_(std::move(required_topics))
  {
    for (const auto & topic : required_topics_) {
      counts_[topic] = 0;
      first_bag_timestamp_ns_[topic] = 0;
      latest_bag_timestamp_ns_[topic] = 0;
    }
    ready_ = required_topics_.empty();
    write_status_locked();
  }

  void mark_written(const std::string & topic, rcutils_time_point_value_t bag_timestamp_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto found = counts_.find(topic);
    if (found == counts_.end()) {
      return;
    }
    if (bag_timestamp_ns <= 0) {
      throw std::runtime_error(
              "rosbag2 accepted a required message without a positive receive timestamp: " + topic);
    }
    if (found->second == 0) {
      first_bag_timestamp_ns_[topic] = bag_timestamp_ns;
    }
    latest_bag_timestamp_ns_[topic] = std::max(
      latest_bag_timestamp_ns_[topic], bag_timestamp_ns);
    found->second += 1;
    if (!ready_ && all_required_seen()) {
      ready_ = true;
      ready_at_ros_ns_ = latest_first_bag_timestamp_ns();
    }
    dirty_ = true;
  }

  void flush_status(bool force = false)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (!dirty_ || (!force && now - last_status_write_ < std::chrono::milliseconds(50))) {
      return;
    }
    write_status_locked();
    dirty_ = false;
    last_status_write_ = now;
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

  rcutils_time_point_value_t latest_first_bag_timestamp_ns() const
  {
    rcutils_time_point_value_t latest = 0;
    for (const auto & entry : first_bag_timestamp_ns_) {
      latest = std::max(latest, entry.second);
    }
    return latest;
  }

  void write_status_locked() const
  {
    std::filesystem::create_directories(ready_file_.parent_path());
    nlohmann::json body;
    body["ready"] = ready_;
    body["ready_at_ros_ns"] = ready_at_ros_ns_;
    body["bag_topics"] = counts_;
    body["first_bag_timestamp_ns"] = first_bag_timestamp_ns_;
    body["latest_bag_timestamp_ns"] = latest_bag_timestamp_ns_;
    body["timestamp_source"] = "rosbag2_serialized_message_time_stamp";
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
  std::unordered_map<std::string, rcutils_time_point_value_t> first_bag_timestamp_ns_;
  std::unordered_map<std::string, rcutils_time_point_value_t> latest_bag_timestamp_ns_;
  rcutils_time_point_value_t ready_at_ros_ns_ = 0;
  bool ready_ = false;
  bool dirty_ = false;
  std::chrono::steady_clock::time_point last_status_write_ =
    std::chrono::steady_clock::now();
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
    // `time_stamp` is the receive timestamp rosbag2 stores for this serialized
    // message. It is not a deserialized message-header timestamp.
    ready_state_->mark_written(message->topic_name, message->time_stamp);
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
      ready_state->flush_status();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    recorder->stop();
    ready_state->flush_status(true);
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
