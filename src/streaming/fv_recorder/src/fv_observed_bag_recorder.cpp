#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
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

constexpr char kTickTopicPrefix[] = "/fv_episode_recorder/write_tick/bag";
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
  std::vector<std::string> topics;
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
    } else if (key == "--topic") {
      args.topics.push_back(take_value(index, argc, argv));
    } else {
      throw std::runtime_error("unknown argument: " + key);
    }
  }
  if (args.output.empty()) {
    throw std::runtime_error("--output is required");
  }
  if (args.topics.empty()) {
    throw std::runtime_error("at least one --topic is required");
  }
  return args;
}

class ObservedSequentialWriter : public rosbag2_cpp::writers::SequentialWriter
{
public:
  ObservedSequentialWriter(
    const rclcpp::Node::SharedPtr & tick_node,
    const std::vector<std::string> & topics)
  : tick_node_(tick_node)
  {
    for (const auto & topic : topics) {
      publishers_.emplace(
        topic,
        tick_node_->create_publisher<builtin_interfaces::msg::Time>(
          std::string(kTickTopicPrefix) + topic,
          rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile()));
    }
  }

  void write(std::shared_ptr<rosbag2_storage::SerializedBagMessage> message) override
  {
    rosbag2_cpp::writers::SequentialWriter::write(message);
    const auto publisher = publishers_.find(message->topic_name);
    if (publisher == publishers_.end()) {
      throw std::runtime_error("bag writer received an unconfigured topic: " + message->topic_name);
    }
    publisher->second->publish(
      static_cast<builtin_interfaces::msg::Time>(tick_node_->get_clock()->now()));
  }

private:
  rclcpp::Node::SharedPtr tick_node_;
  std::unordered_map<
    std::string,
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr> publishers_;
};

}  // namespace

int main(int argc, char ** argv)
{
  Args args;
  try {
    args = parse_args(argc, argv);
  } catch (const std::exception & exc) {
    std::cerr << "fv_observed_bag_recorder: " << exc.what() << "\n";
    return 2;
  }

  rclcpp::init(argc, argv);
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
  try {
    auto tick_node = std::make_shared<rclcpp::Node>("fv_bag_write_tick_publisher");
    auto writer = std::make_shared<rosbag2_cpp::Writer>(
      std::make_unique<ObservedSequentialWriter>(tick_node, args.topics));

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
      "fv_observed_bag_recorder");
    recorder->record();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(recorder);
    executor.add_node(tick_node);
    while (rclcpp::ok() && !g_stop_requested.load()) {
      executor.spin_some(std::chrono::milliseconds(50));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    recorder->stop();
    rclcpp::shutdown();
  } catch (const std::exception & exc) {
    std::cerr << "fv_observed_bag_recorder: " << exc.what() << "\n";
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  return g_stop_requested.load() ? 130 : 0;
}
