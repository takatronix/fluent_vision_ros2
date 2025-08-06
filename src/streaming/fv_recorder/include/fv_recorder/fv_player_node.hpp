#ifndef FV_PLAYER_NODE_HPP
#define FV_PLAYER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>

#include "fv_recorder/srv/start_playback.hpp"
#include "fv_recorder/srv/stop_playback.hpp"
#include "fv_recorder/msg/recording_status.hpp"

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

struct PlayerConfig {
    std::string recording_directory;
    double playback_speed;
    std::vector<std::string> output_topics;
};

class FVPlayerNode : public rclcpp::Node
{
public:
    FVPlayerNode();
    void publishStatus();

private:
    // 設定
    PlayerConfig config_;
    
    // 状態管理
    std::atomic<bool> running_;
    std::thread playback_thread_;
    
    // Publishers
    rclcpp::Publisher<fv_recorder::msg::RecordingStatus>::SharedPtr status_publisher_;
    
    // Services
    rclcpp::Service<fv_recorder::srv::StartPlayback>::SharedPtr start_playback_service_;
    rclcpp::Service<fv_recorder::srv::StopPlayback>::SharedPtr stop_playback_service_;
    
    // メソッド
    void loadParameters();
    void initializeServices();
    void initializePublishers();
    
    void startPlayback(const std::shared_ptr<fv_recorder::srv::StartPlayback::Request> request,
                      std::shared_ptr<fv_recorder::srv::StartPlayback::Response> response);
    void stopPlayback(const std::shared_ptr<fv_recorder::srv::StopPlayback::Request> request,
                     std::shared_ptr<fv_recorder::srv::StopPlayback::Response> response);
    
    void runPlaybackLoop(const std::vector<std::string>& recording_files);
    void publishMessage(const rclcpp::SerializedMessage& serialized_msg, const std::string& topic_name);
    
    std::vector<std::string> findRecordingFiles(const std::string& directory, const std::string& date_format);
    std::string mapTopicName(const std::string& input_topic);
    std::string getCurrentTimestamp();
    void sleepForDuration(int64_t nanoseconds);
};

#endif // FV_PLAYER_NODE_HPP 