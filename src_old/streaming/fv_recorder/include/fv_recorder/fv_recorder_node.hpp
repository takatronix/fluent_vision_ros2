#ifndef FV_RECORDER_NODE_HPP
#define FV_RECORDER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/empty.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/record_options.hpp>
#include <rclcpp/serialization.hpp>

#include "fv_recorder/srv/start_recording.hpp"
#include "fv_recorder/srv/stop_recording.hpp"
#include "fv_recorder/msg/recording_status.hpp"

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <unordered_map>

// フォーマット対応クラス
class FormatWriter {
public:
    virtual ~FormatWriter() = default;
    virtual bool open(const std::string& filepath) = 0;
    
    // 基本メッセージタイプ
    virtual bool write(const sensor_msgs::msg::Image::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) = 0;
    virtual bool write(const std_msgs::msg::String::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) = 0;
    virtual bool write(const std_msgs::msg::Bool::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) = 0;
    
    // 汎用メッセージ書き込み（任意のROS2メッセージに対応）
    virtual bool writeGeneric(const rclcpp::SerializedMessage& serialized_msg, const std::string& topic_name, const std::string& message_type, const rclcpp::Time& timestamp) = 0;
    
    virtual void close() = 0;
    virtual std::string getFileExtension() const = 0;
};

// ROSBag2Writer
class ROSBag2Writer : public FormatWriter {
private:
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    std::vector<std::pair<std::string, std::string>> topics_with_types_;

public:
    bool open(const std::string& filepath) override;
    bool write(const sensor_msgs::msg::Image::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) override;
    bool write(const std_msgs::msg::String::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) override;
    bool write(const std_msgs::msg::Bool::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) override;
    bool writeGeneric(const rclcpp::SerializedMessage& serialized_msg, const std::string& topic_name, const std::string& message_type, const rclcpp::Time& timestamp) override;
    void close() override;
    std::string getFileExtension() const override { return ".db3"; }
    void setTopicsWithTypes(const std::vector<std::pair<std::string, std::string>>& topics_with_types) { topics_with_types_ = topics_with_types; }
};

// JSONWriter
class JSONWriter : public FormatWriter {
private:
    std::string filepath_;
    nlohmann::json data_;

public:
    bool open(const std::string& filepath) override;
    bool write(const sensor_msgs::msg::Image::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) override;
    bool write(const std_msgs::msg::String::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) override;
    bool write(const std_msgs::msg::Bool::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) override;
    bool writeGeneric(const rclcpp::SerializedMessage& serialized_msg, const std::string& topic_name, const std::string& message_type, const rclcpp::Time& timestamp) override;
    void close() override;
    std::string getFileExtension() const override { return ".json"; }
};

// YAMLフォーマット
class YAMLWriter : public FormatWriter {
private:
    std::ofstream file_;
    std::string filepath_;

public:
    bool open(const std::string& filepath) override;
    bool write(const sensor_msgs::msg::Image::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) override;
    bool write(const std_msgs::msg::String::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) override;
    bool write(const std_msgs::msg::Bool::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) override;
    bool writeGeneric(const rclcpp::SerializedMessage& serialized_msg, const std::string& topic_name, const std::string& message_type, const rclcpp::Time& timestamp) override;
    void close() override;
    std::string getFileExtension() const override { return ".yaml"; }
};

// CSVフォーマット
class CSVWriter : public FormatWriter {
private:
    std::ofstream file_;
    std::string filepath_;

public:
    bool open(const std::string& filepath) override;
    bool write(const sensor_msgs::msg::Image::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) override;
    bool write(const std_msgs::msg::String::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) override;
    bool write(const std_msgs::msg::Bool::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) override;
    bool writeGeneric(const rclcpp::SerializedMessage& serialized_msg, const std::string& topic_name, const std::string& message_type, const rclcpp::Time& timestamp) override;
    void close() override;
    std::string getFileExtension() const override { return ".csv"; }
};

// 動画フォーマット（MP4/AVI）
class VideoWriter : public FormatWriter {
private:
    cv::VideoWriter video_writer_;
    std::string filepath_;
    std::string format_;
    bool initialized_ {false};
    bool overlay_time_ {false};
    std::string overlay_time_format_ {"%Y-%m-%d %H:%M:%S"};

public:
    VideoWriter(const std::string& format) : format_(format) {}
    VideoWriter(const std::string& format, bool overlay_time, const std::string& overlay_format)
        : format_(format), overlay_time_(overlay_time), overlay_time_format_(overlay_format) {}
    bool open(const std::string& filepath) override;
    bool write(const sensor_msgs::msg::Image::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) override;
    bool write(const std_msgs::msg::String::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) override;
    bool write(const std_msgs::msg::Bool::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) override;
    bool writeGeneric(const rclcpp::SerializedMessage& serialized_msg, const std::string& topic_name, const std::string& message_type, const rclcpp::Time& timestamp) override;
    void close() override;
    std::string getFileExtension() const override { return format_ == "mp4" ? ".mp4" : ".avi"; }
    static cv::Mat drawTimeOverlay(const cv::Mat& src, const std::string& time_format, const rclcpp::Time& timestamp);
};

class FVRecorderNode : public rclcpp::Node
{
public:
    FVRecorderNode();
    ~FVRecorderNode();
    void publishStatus();

private:
    // パラメータ
    struct RecordingConfig {
        std::vector<std::string> input_topics;
        std::string output_directory;
        int segment_duration;  // 秒
        int retention_days;
        std::string date_format;
        bool auto_recording;   // 自動録画フラグ
        std::string default_format; // デフォルト出力フォーマット
    } config_;

    // 録画状態
    struct RecordingSession {
        std::string id;
        std::string directory;
        std::chrono::steady_clock::time_point start_time;
        std::chrono::steady_clock::time_point segment_start_time;
        std::unique_ptr<FormatWriter> writer;
        std::filesystem::path current_file_path;
        int segment_count;
        bool is_active;
        std::string format;
    };

    // メンバー変数
    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscriptions_;
    std::map<std::string, rclcpp::GenericSubscription::SharedPtr> generic_subscriptions_;
    std::map<std::string, std::string> topic_to_filename_map_;
    std::map<std::string, std::string> topic_message_types_;
    std::unique_ptr<RecordingSession> current_session_;
    std::mutex session_mutex_;
    std::thread cleanup_thread_;
    bool running_;

    // Publishers
    rclcpp::Publisher<fv_recorder::msg::RecordingStatus>::SharedPtr status_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr preview_image_publisher_;

    // Services
    rclcpp::Service<fv_recorder::srv::StartRecording>::SharedPtr start_recording_service_;
    rclcpp::Service<fv_recorder::srv::StopRecording>::SharedPtr stop_recording_service_;

    // 外部制御用サブスクリプション
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recording_control_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recording_command_sub_;

    // メソッド
    void loadParameters();
    void initializeServices();
    void initializePublishers();
    void initializeSubscriptions();
    void initializeControlSubscriptions();
    void maybeStartAutoRecording();
    
    // 録画関連
    void startRecording(const std::shared_ptr<fv_recorder::srv::StartRecording::Request> request,
                       std::shared_ptr<fv_recorder::srv::StartRecording::Response> response);
    void stopRecording(const std::shared_ptr<fv_recorder::srv::StopRecording::Request> request,
                      std::shared_ptr<fv_recorder::srv::StopRecording::Response> response);
    
    // 外部制御コールバック
    void recordingControlCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void recordingCommandCallback(const std_msgs::msg::String::SharedPtr msg);
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg, const std::string& topic_name);
    void genericMessageCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string& topic_name);
    void createNewSegment(const std::vector<std::string>& topics_to_record, const std::string& format);
    
    // フォーマット関連
    std::unique_ptr<FormatWriter> createFormatWriter(const std::string& format);
    std::string getFileExtension(const std::string& format);
    
    // ファイル管理
    void cleanupOldFiles();
    std::string generateFilename(const std::string& topic_name, const std::string& format);
    std::string getCurrentTimestamp();
    void runCleanupLoop();
    
    // ユーティリティ
    std::string sanitizeTopicName(const std::string& topic_name);
    bool createDirectoryIfNotExists(const std::string& path);
    void startRecordingInternal(const std::string& directory = "", const std::string& date_format = "", 
                               const std::vector<std::string>& custom_topics = {}, const std::string& format = "");
    void stopRecordingInternal();
    std::string getMessageType(const std::string& topic_name);
    void discoverAndCacheTopicTypes();

    // Overlay/preview settings
    bool preview_enabled_ {true};
    bool time_overlay_enabled_ {false};
    std::string time_overlay_format_ {"%Y-%m-%d %H:%M:%S"};
    std::string preview_output_topic_ {"/fv_recorder/preview"};
    cv::Mat drawTimeOverlay(const cv::Mat& src);

    // Video overlay settings
    bool video_time_overlay_enabled_ {false};
    std::string video_time_overlay_format_ {"%Y-%m-%d %H:%M:%S"};
};

#endif // FV_RECORDER_NODE_HPP 