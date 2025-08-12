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
    // overlays (global defaults can be overridden per-node)
    config_.overlay_play_indicator = this->declare_parameter("preview.overlay_play_indicator", true);
    config_.overlay_topic = this->declare_parameter("preview.output_topic", std::string("/fv_player/preview"));
    
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
    if (config_.overlay_play_indicator) {
        preview_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(config_.overlay_topic, rclcpp::QoS(2).best_effort());
        RCLCPP_INFO(this->get_logger(), "🖼️ Player preview on: %s", config_.overlay_topic.c_str());
    }
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
    try {
        // 型情報を問い合わせて適切なgeneric publisherを作成
        auto names_and_types = this->get_topic_names_and_types();
        std::string type_name;
        auto it = names_and_types.find(topic_name);
        if (it != names_and_types.end() && !it->second.empty()) {
            type_name = it->second.front();
        } else {
            // フォールバック（よくある型）
            type_name = "sensor_msgs/msg/Image";
        }
        auto gen_pub = this->create_generic_publisher(topic_name, type_name, rclcpp::QoS(10));
        gen_pub->publish(serialized_msg);
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
            publishPlayOverlay();
            
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
                
                // SerializedBagMessageをrclcpp::SerializedMessageに安全にコピー
                rclcpp::SerializedMessage rclcpp_msg(serialized_message->serialized_data->buffer_length);
                auto& rcl_buf = rclcpp_msg.get_rcl_serialized_message();
                std::memcpy(rcl_buf.buffer, serialized_message->serialized_data->buffer, serialized_message->serialized_data->buffer_length);
                rcl_buf.buffer_length = serialized_message->serialized_data->buffer_length;
                
                // メッセージをパブリッシュ
                publishMessage(rclcpp_msg, output_topic);
                
                // 再生速度に応じて待機（Bagのタイムスタンプはナノ秒、差分を使う）
                static uint64_t prev_ts = 0;
                uint64_t ts = serialized_message->time_stamp; // nanoseconds
                if (prev_ts != 0 && ts > prev_ts) {
                    uint64_t delta = ts - prev_ts;
                    delta = static_cast<uint64_t>(static_cast<double>(delta) / std::max(0.0001, config_.playback_speed));
                    sleepForDuration(static_cast<int64_t>(delta));
                }
                prev_ts = ts;
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

void FVPlayerNode::publishPlayOverlay()
{
    if (!preview_publisher_) return;
    // Create a small banner image with "PLAY" and timestamp
    cv::Mat banner(60, 240, CV_8UC3, cv::Scalar(40, 40, 40));
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    char timebuf[64];
    std::strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
    cv::putText(banner, "PLAY", {12, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0,255,0}, 2, cv::LINE_AA);
    cv::putText(banner, timebuf, {12, 50}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255,255,255}, 1, cv::LINE_AA);
    cv_bridge::CvImage out;
    out.header.stamp = this->now();
    out.encoding = sensor_msgs::image_encodings::BGR8;
    out.image = banner;
    preview_publisher_->publish(*out.toImageMsg());
}