#include "fv_recorder/fv_player_node.hpp"
#include <filesystem>
#include <chrono>
#include <thread>

FVPlayerNode::FVPlayerNode() : Node("fv_player_node"), running_(false)
{
    loadParameters();
    initializeServices();
    initializePublishers();
    
    RCLCPP_INFO(this->get_logger(), "🎬 FV Player Node initialized");
}

void FVPlayerNode::loadParameters()
{
    // パラメータの読み込み
    config_.recording_directory = this->declare_parameter("recording_directory", "/tmp/recordings");
    config_.playback_speed = this->declare_parameter("playback_speed", 1.0);
    config_.output_topics = this->declare_parameter("output_topics", std::vector<std::string>{});
    
    RCLCPP_INFO(this->get_logger(), "📁 Recording directory: %s", config_.recording_directory.c_str());
    RCLCPP_INFO(this->get_logger(), "⏩ Playback speed: %f", config_.playback_speed);
}

void FVPlayerNode::initializeServices()
{
    start_playback_service_ = this->create_service<fv_recorder::srv::StartPlayback>(
        "start_playback",
        std::bind(&FVPlayerNode::startPlayback, this, std::placeholders::_1, std::placeholders::_2));
    
    stop_playback_service_ = this->create_service<fv_recorder::srv::StopPlayback>(
        "stop_playback",
        std::bind(&FVPlayerNode::stopPlayback, this, std::placeholders::_1, std::placeholders::_2));
}

void FVPlayerNode::initializePublishers()
{
    status_publisher_ = this->create_publisher<fv_recorder::msg::RecordingStatus>("status", 10);
}

void FVPlayerNode::startPlayback(const std::shared_ptr<fv_recorder::srv::StartPlayback::Request> request,
                                std::shared_ptr<fv_recorder::srv::StartPlayback::Response> response)
{
    if (running_) {
        response->success = false;
        response->message = "Playback is already running";
        return;
    }
    
    try {
        // 録画ファイルを検索
        std::vector<std::string> recording_files = findRecordingFiles(request->recording_directory, request->date_format);
        
        if (recording_files.empty()) {
            response->success = false;
            response->message = "No recording files found";
            return;
        }
        
        // 再生ループを開始
        running_ = true;
        playback_thread_ = std::thread(&FVPlayerNode::runPlaybackLoop, this, recording_files);
        
        response->success = true;
        response->message = "Playback started";
        response->playback_id = "playback_" + getCurrentTimestamp();
        
        RCLCPP_INFO(this->get_logger(), "▶️ Playback started with %zu files", recording_files.size());
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to start playback: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to start playback: %s", e.what());
    }
}

void FVPlayerNode::stopPlayback(const std::shared_ptr<fv_recorder::srv::StopPlayback::Request> request,
                               std::shared_ptr<fv_recorder::srv::StopPlayback::Response> response)
{
    if (!running_) {
        response->success = false;
        response->message = "No playback is running";
        return;
    }
    
    running_ = false;
    
    if (playback_thread_.joinable()) {
        playback_thread_.join();
    }
    
    response->success = true;
    response->message = "Playback stopped";
    
    RCLCPP_INFO(this->get_logger(), "⏹️ Playback stopped");
}

void FVPlayerNode::publishMessage(const rclcpp::SerializedMessage& serialized_msg, const std::string& topic_name)
{
    // 基本的なメッセージタイプのみ対応
    try {
        // Imageメッセージとしてパブリッシュを試行
        auto publisher = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
        
        // シリアライゼーションを使用してメッセージを復元
        rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
        sensor_msgs::msg::Image msg;
        serialization.deserialize_message(&serialized_msg, &msg);
        
        publisher->publish(msg);
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Failed to publish message to %s: %s", topic_name.c_str(), e.what());
    }
}

void FVPlayerNode::runPlaybackLoop(const std::vector<std::string>& recording_files)
{
    try {
        for (const auto& file : recording_files) {
            if (!running_) break;
            
            RCLCPP_INFO(this->get_logger(), "📂 Playing file: %s", file.c_str());
            
            // ROSBag2ファイルを読み込み
            rosbag2_storage::StorageOptions storage_options;
            storage_options.uri = file;
            storage_options.storage_id = "sqlite3";
            
            rosbag2_cpp::ConverterOptions converter_options;
            converter_options.input_serialization_format = "cdr";
            converter_options.output_serialization_format = "cdr";
            
            rosbag2_cpp::readers::SequentialReader reader;
            reader.open(storage_options, converter_options);
            
            while (reader.has_next() && running_) {
                auto serialized_message = reader.read_next();
                
                if (!serialized_message || serialized_message->topic_name.empty()) {
                    continue;
                }
                
                std::string output_topic = mapTopicName(serialized_message->topic_name);
                
                // SerializedBagMessageをrclcpp::SerializedMessageに変換
                rclcpp::SerializedMessage rclcpp_msg;
                rclcpp_msg.reserve(serialized_message->serialized_data->buffer_length);
                std::memcpy(rclcpp_msg.get_rcl_serialized_message().buffer, 
                           serialized_message->serialized_data->buffer, 
                           serialized_message->serialized_data->buffer_length);
                rclcpp_msg.get_rcl_serialized_message().buffer_length = serialized_message->serialized_data->buffer_length;
                
                // メッセージをパブリッシュ
                publishMessage(rclcpp_msg, output_topic);
                
                // 再生速度に応じて待機
                sleepForDuration(
                    static_cast<int64_t>(serialized_message->time_stamp * 1e9 / config_.playback_speed));
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Playback error: %s", e.what());
    }
}

void FVPlayerNode::publishStatus()
{
    auto status_msg = fv_recorder::msg::RecordingStatus();
    status_msg.is_recording = false;
    status_msg.is_playing = running_;
    status_msg.header.stamp = this->now();
    
    status_publisher_->publish(status_msg);
}

std::vector<std::string> FVPlayerNode::findRecordingFiles(const std::string& directory, const std::string& date_format)
{
    std::vector<std::string> files;
    
    try {
        std::filesystem::path dir_path(directory);
        if (!std::filesystem::exists(dir_path)) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Directory does not exist: %s", directory.c_str());
            return files;
        }
        
        for (const auto& entry : std::filesystem::directory_iterator(dir_path)) {
            if (entry.is_regular_file() && entry.path().extension() == ".db3") {
                std::string filename = entry.path().filename().string();
                
                // 日付フォーマットでフィルタリング
                if (date_format.empty() || filename.find(date_format) != std::string::npos) {
                    files.push_back(entry.path().string());
                }
            }
        }
        
        std::sort(files.begin(), files.end());
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Error finding recording files: %s", e.what());
    }
    
    return files;
}

std::string FVPlayerNode::mapTopicName(const std::string& input_topic)
{
    // トピック名のマッピング（必要に応じて実装）
    return input_topic;
}

std::string FVPlayerNode::getCurrentTimestamp()
{
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
    ss << "_" << std::setfill('0') << std::setw(3) << ms.count();
    
    return ss.str();
}

void FVPlayerNode::sleepForDuration(int64_t nanoseconds)
{
    auto duration = std::chrono::nanoseconds(nanoseconds);
    std::this_thread::sleep_for(duration);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<FVPlayerNode>();
    
    // ステータスパブリッシュのタイマー
    auto status_timer = node->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&FVPlayerNode::publishStatus, node));
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
} 