#include "fv_recorder/fv_recorder_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>

FVRecorderNode::FVRecorderNode()
    : Node("fv_recorder"), running_(true)
{
    RCLCPP_INFO(this->get_logger(), "🚀 FV Recorder Node starting...");
    
    try {
        // パラメータ読み込み
        loadParameters();
        
        // サービス初期化
        initializeServices();
        
        // パブリッシャー初期化
        initializePublishers();
        
        // サブスクリプション初期化（エラーが発生しても継続）
        try {
            initializeSubscriptions();
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Warning during subscription initialization: %s", e.what());
            RCLCPP_INFO(this->get_logger(), "📝 Node will continue without subscriptions");
        }
        
        // 外部制御サブスクリプション初期化
        try {
            initializeControlSubscriptions();
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Warning during control subscription initialization: %s", e.what());
        }
        
        // クリーンアップスレッド開始
        cleanup_thread_ = std::thread(&FVRecorderNode::runCleanupLoop, this);
        
        // 自動録画が有効なら起動時に開始
        maybeStartAutoRecording();

        RCLCPP_INFO(this->get_logger(), "✅ FV Recorder Node started successfully");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Exception during initialization: %s", e.what());
        throw; // 致命的なエラーの場合は再スロー
    }
}

void FVRecorderNode::maybeStartAutoRecording()
{
    if (!config_.auto_recording) {
        return;
    }
    RCLCPP_INFO(this->get_logger(), "🎬 Auto recording is enabled → starting now (format=%s, topics=%zu)",
                config_.default_format.c_str(), config_.input_topics.size());
    try {
        startRecordingInternal(
            config_.output_directory,
            config_.date_format,
            config_.input_topics,
            config_.default_format);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to start auto recording: %s", e.what());
    }
}

FVRecorderNode::~FVRecorderNode()
{
    RCLCPP_INFO(this->get_logger(), "🛑 Shutting down FV Recorder Node...");
    
    running_ = false;
    
    // 録画セッション停止
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        if (current_session_ && current_session_->is_active) {
            current_session_->is_active = false;
            if (current_session_->writer) {
                current_session_->writer->close();
            }
        }
    }
    
    // スレッド終了待機
    if (cleanup_thread_.joinable()) {
        cleanup_thread_.join();
    }
}

void FVRecorderNode::loadParameters()
{
    RCLCPP_INFO(this->get_logger(), "📋 Loading parameters...");
    
    // 録画設定
    config_.input_topics = this->declare_parameter("recording.input_topics",
        std::vector<std::string>{"~/color/image_raw", "~/depth/image_rect_raw"});
    config_.output_directory = this->declare_parameter("recording.output_directory", 
        "/home/takatronix/recordings");
    config_.segment_duration = this->declare_parameter("recording.segment_duration", 300);
    config_.retention_days = this->declare_parameter("recording.retention_days", 7);
    config_.date_format = this->declare_parameter("recording.date_format", "YYYYMMDD");
    config_.auto_recording = this->declare_parameter("recording.auto_recording", false);
    config_.default_format = this->declare_parameter("recording.default_format", "rosbag");

    // プレビュー/オーバーレイ設定（グローバル→ノード個別で上書き）
    preview_enabled_ = this->declare_parameter("preview.enabled", true);
    time_overlay_enabled_ = this->declare_parameter("preview.time_overlay", false);
    time_overlay_format_ = this->declare_parameter("preview.time_format", std::string("%Y-%m-%d %H:%M:%S"));
    preview_output_topic_ = this->declare_parameter("preview.output_topic", std::string("~/preview"));
    // Video overlay (on saved video)
    video_time_overlay_enabled_ = this->declare_parameter("recording.video_time_overlay", false);
    video_time_overlay_format_ = this->declare_parameter("recording.video_time_overlay_format", std::string("%Y-%m-%d %H:%M:%S"));
    
    RCLCPP_INFO(this->get_logger(), "📁 Output directory: %s", config_.output_directory.c_str());
    RCLCPP_INFO(this->get_logger(), "⏱️ Segment duration: %d seconds", config_.segment_duration);
    RCLCPP_INFO(this->get_logger(), "🗓️ Retention days: %d", config_.retention_days);
    RCLCPP_INFO(this->get_logger(), "🔄 Auto recording: %s", config_.auto_recording ? "enabled" : "disabled");
    RCLCPP_INFO(this->get_logger(), "📄 Default format: %s", config_.default_format.c_str());
}

void FVRecorderNode::initializeServices()
{
    start_recording_service_ = this->create_service<fv_recorder::srv::StartRecording>(
        "start_recording",
        std::bind(&FVRecorderNode::startRecording, this, std::placeholders::_1, std::placeholders::_2));
    
    stop_recording_service_ = this->create_service<fv_recorder::srv::StopRecording>(
        "stop_recording",
        std::bind(&FVRecorderNode::stopRecording, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "🔧 Services initialized");
}

void FVRecorderNode::initializePublishers()
{
    status_publisher_ = this->create_publisher<fv_recorder::msg::RecordingStatus>("status", 10);
    
    if (preview_enabled_) {
        preview_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            preview_output_topic_, rclcpp::QoS(2).best_effort());
        RCLCPP_INFO(this->get_logger(), "🖼️ Preview enabled on: %s", preview_output_topic_.c_str());
    }
    
    RCLCPP_INFO(this->get_logger(), "📤 Publishers initialized");
}

void FVRecorderNode::initializeSubscriptions()
{
    RCLCPP_INFO(this->get_logger(), "📥 Initializing subscriptions for %zu topics...", config_.input_topics.size());
    
    // トピックタイプの自動検出
    discoverAndCacheTopicTypes();
    
    for (const auto& topic : config_.input_topics) {
        try {
            // 既知のメッセージタイプの場合は型付きサブスクリプション
            if (topic_message_types_.find(topic) != topic_message_types_.end()) {
                auto it = subscriptions_.find(topic);
                if (it == subscriptions_.end()) {
                    auto sub = this->create_subscription<sensor_msgs::msg::Image>(
                        topic, rclcpp::QoS(10).best_effort(),
                        [this, topic](const sensor_msgs::msg::Image::SharedPtr msg) {
                            imageCallback(msg, topic);
                        });
                    subscriptions_[topic] = sub;
                    RCLCPP_INFO(this->get_logger(), "📹 Subscribed to %s (Image)", topic.c_str());
                }
            } else {
                // 汎用サブスクリプション
                auto generic_sub = this->create_generic_subscription(
                    topic, "std_msgs/msg/String", rclcpp::QoS(10).best_effort(),
                    [this, topic](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
                        genericMessageCallback(serialized_msg, topic);
                    });
                generic_subscriptions_[topic] = generic_sub;
                RCLCPP_INFO(this->get_logger(), "🔌 Generic subscription to %s", topic.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Failed to subscribe to %s: %s", topic.c_str(), e.what());
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ Subscriptions initialized");
}

void FVRecorderNode::initializeControlSubscriptions()
{
    recording_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/fv_recorder/recording_control", rclcpp::QoS(10),
        std::bind(&FVRecorderNode::recordingControlCallback, this, std::placeholders::_1));
    
    recording_command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/fv_recorder/recording_command", rclcpp::QoS(10),
        std::bind(&FVRecorderNode::recordingCommandCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "🎮 Control subscriptions initialized");
}

void FVRecorderNode::recordingControlCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "🎬 Recording control: START");
        startRecordingInternal();
    } else {
        RCLCPP_INFO(this->get_logger(), "⏹️ Recording control: STOP");
        stopRecordingInternal();
    }
}

void FVRecorderNode::recordingCommandCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string command = msg->data;
    RCLCPP_INFO(this->get_logger(), "📝 Recording command: %s", command.c_str());
    
    if (command == "start") {
        startRecordingInternal();
    } else if (command == "stop") {
        stopRecordingInternal();
    } else if (command == "toggle") {
        std::lock_guard<std::mutex> lock(session_mutex_);
        if (current_session_ && current_session_->is_active) {
            stopRecordingInternal();
        } else {
            startRecordingInternal();
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ Unknown command: %s", command.c_str());
    }
}

void FVRecorderNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg, 
                                   const std::string& topic_name)
{
    std::lock_guard<std::mutex> lock(session_mutex_);
    
    if (!current_session_ || !current_session_->is_active) {
        return;
    }
    
    try {
        // 録画
        if (current_session_->writer) {
            const bool write_ok = current_session_->writer->write(msg, topic_name, this->now());
            if (!write_ok) {
                RCLCPP_WARN(this->get_logger(), "⚠️ write() returned false (topic=%s, file=%s)",
                            topic_name.c_str(), current_session_->current_file_path.string().c_str());
            }
        }
        
        // プレビュー画像の生成とパブリッシュ
        if (preview_enabled_ && preview_image_publisher_) {
            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_WARN(this->get_logger(), "⚠️ cv_bridge exception: %s", e.what());
                return;
            }
            
            cv::Mat preview_img = cv_ptr->image.clone();
            
            // タイムオーバーレイ
            if (time_overlay_enabled_) {
                preview_img = drawTimeOverlay(preview_img);
            }
            
            // 録画状態オーバーレイ
            cv::putText(preview_img, "REC", cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            
            // プレビュー画像をパブリッシュ
            cv_bridge::CvImage preview_msg;
            preview_msg.header = msg->header;
            preview_msg.encoding = sensor_msgs::image_encodings::BGR8;
            preview_msg.image = preview_img;
            
            preview_image_publisher_->publish(*preview_msg.toImageMsg());
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Error in imageCallback: %s", e.what());
    }
}

void FVRecorderNode::genericMessageCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg, 
                                           const std::string& topic_name)
{
    std::lock_guard<std::mutex> lock(session_mutex_);
    
    if (!current_session_ || !current_session_->is_active) {
        return;
    }
    
    try {
        // メッセージタイプの取得
        std::string message_type = getMessageType(topic_name);
        
        // 汎用録画
        if (current_session_->writer) {
            current_session_->writer->writeGeneric(*msg, topic_name, message_type, this->now());
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Error in genericMessageCallback: %s", e.what());
    }
}

void FVRecorderNode::startRecording(const std::shared_ptr<fv_recorder::srv::StartRecording::Request> request,
                                   std::shared_ptr<fv_recorder::srv::StartRecording::Response> response)
{
    try {
        std::string directory = request->recording_directory.empty() ? config_.output_directory : request->recording_directory;
        std::string date_format = request->date_format.empty() ? config_.date_format : request->date_format;
        std::vector<std::string> custom_topics = request->input_topics.empty() ? config_.input_topics : request->input_topics;
        std::string format = request->output_format.empty() ? config_.default_format : request->output_format;
        
        startRecordingInternal(directory, date_format, custom_topics, format);
        
        response->success = true;
        response->recording_id = current_session_ ? current_session_->id : "unknown";
        response->message = "Recording started successfully";
        
        RCLCPP_INFO(this->get_logger(), "🎬 Recording started with ID: %s", response->recording_id.c_str());
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to start recording: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to start recording: %s", e.what());
    }
}

void FVRecorderNode::stopRecording(const std::shared_ptr<fv_recorder::srv::StopRecording::Request> request,
                                  std::shared_ptr<fv_recorder::srv::StopRecording::Response> response)
{
    (void)request; // 未使用パラメータ警告を抑制
    
    try {
        std::lock_guard<std::mutex> lock(session_mutex_);
        
        if (!current_session_ || !current_session_->is_active) {
            response->success = false;
            response->message = "No active recording session";
            return;
        }
        
        stopRecordingInternal();
        
        response->success = true;
        response->message = "Recording stopped successfully";
        
        RCLCPP_INFO(this->get_logger(), "⏹️ Recording stopped");
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to stop recording: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to stop recording: %s", e.what());
    }
}

void FVRecorderNode::startRecordingInternal(const std::string& directory, const std::string& date_format, 
                                           const std::vector<std::string>& custom_topics, const std::string& format)
{
    (void)date_format; // 未使用パラメータ警告を抑制
    
    std::lock_guard<std::mutex> lock(session_mutex_);
    
    // 既存セッションの停止
    if (current_session_ && current_session_->is_active) {
        stopRecordingInternal();
    }
    
    // ディレクトリ作成
    if (!createDirectoryIfNotExists(directory)) {
        throw std::runtime_error("Failed to create output directory: " + directory);
    }
    
    // 新しいセッション作成
    current_session_ = std::make_unique<RecordingSession>();
    current_session_->id = getCurrentTimestamp();
    current_session_->directory = directory;
    current_session_->start_time = std::chrono::steady_clock::now();
    current_session_->segment_start_time = current_session_->start_time;
    current_session_->segment_count = 0;
    current_session_->is_active = true;
    current_session_->format = format;
    
    // フォーマットライター作成
    current_session_->writer = createFormatWriter(format);
    if (!current_session_->writer) {
        throw std::runtime_error("Failed to create format writer for: " + format);
    }
    
    // セグメント作成
    createNewSegment(custom_topics, format);
    
    RCLCPP_INFO(this->get_logger(), "🎬 Recording session started: %s", current_session_->id.c_str());
    RCLCPP_INFO(this->get_logger(), "📁 Directory: %s", directory.c_str());
    RCLCPP_INFO(this->get_logger(), "📄 Format: %s", format.c_str());
}

void FVRecorderNode::stopRecordingInternal()
{
    if (current_session_ && current_session_->is_active) {
        if (current_session_->writer) {
            current_session_->writer->close();
        }
        current_session_->is_active = false;
        RCLCPP_INFO(this->get_logger(), "⏹️ Recording session stopped: %s", current_session_->id.c_str());
    }
}

void FVRecorderNode::createNewSegment(const std::vector<std::string>& topics_to_record, const std::string& format)
{
    if (!current_session_) return;
    
    // 既存ライターをクローズ
    if (current_session_->writer) {
        current_session_->writer->close();
    }
    
    // 新しいセグメントファイル名生成
    std::string filename = generateFilename("segment_" + std::to_string(current_session_->segment_count), format);
    current_session_->current_file_path = std::filesystem::path(current_session_->directory) / filename;
    
    // 新しいライター作成
    current_session_->writer = createFormatWriter(format);
    if (!current_session_->writer) {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to create format writer for new segment");
        return;
    }
    
    // ファイルオープン
    if (!current_session_->writer->open(current_session_->current_file_path.string())) {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to open new segment file: %s", current_session_->current_file_path.c_str());
        return;
    }
    
    // ROSBag2の場合はトピック情報を設定
    if (auto* rosbag_writer = dynamic_cast<ROSBag2Writer*>(current_session_->writer.get())) {
        std::vector<std::pair<std::string, std::string>> topics_with_types;
        for (const auto& topic : topics_to_record) {
            std::string message_type = getMessageType(topic);
            topics_with_types.emplace_back(topic, message_type);
        }
        rosbag_writer->setTopicsWithTypes(topics_with_types);
    }
    
    current_session_->segment_count++;
    current_session_->segment_start_time = std::chrono::steady_clock::now();
    
    RCLCPP_INFO(this->get_logger(), "📁 New segment created: %s", current_session_->current_file_path.c_str());
}

std::unique_ptr<FormatWriter> FVRecorderNode::createFormatWriter(const std::string& format)
{
    if (format == "rosbag" || format == "db3") {
        return std::make_unique<ROSBag2Writer>();
    } else if (format == "json") {
        return std::make_unique<JSONWriter>();
    } else if (format == "yaml") {
        return std::make_unique<YAMLWriter>();
    } else if (format == "csv") {
        return std::make_unique<CSVWriter>();
    } else if (format == "mp4" || format == "avi") {
        return std::make_unique<VideoWriter>(format, video_time_overlay_enabled_, video_time_overlay_format_);
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ Unknown format: %s, falling back to rosbag", format.c_str());
        return std::make_unique<ROSBag2Writer>();
    }
}

std::string FVRecorderNode::getFileExtension(const std::string& format)
{
    if (format == "rosbag" || format == "db3") return ".db3";
    if (format == "json") return ".json";
    if (format == "yaml") return ".yaml";
    if (format == "csv") return ".csv";
    if (format == "mp4") return ".mp4";
    if (format == "avi") return ".avi";
    return ".db3"; // デフォルト
}

void FVRecorderNode::cleanupOldFiles()
{
    try {
        std::filesystem::path dir_path(config_.output_directory);
        if (!std::filesystem::exists(dir_path)) return;
        
        auto now = std::chrono::system_clock::now();
        auto cutoff_time = now - std::chrono::hours(24 * config_.retention_days);
        
        int deleted_count = 0;
        for (const auto& entry : std::filesystem::directory_iterator(dir_path)) {
            if (entry.is_regular_file()) {
                auto file_time = std::filesystem::last_write_time(entry);
                // clock_castの代わりにtime_pointを直接使用
                auto file_time_point = std::chrono::system_clock::from_time_t(
                    std::chrono::duration_cast<std::chrono::seconds>(file_time.time_since_epoch()).count());
                
                if (file_time_point < cutoff_time) {
                    std::filesystem::remove(entry);
                    deleted_count++;
                }
            }
        }
        
        if (deleted_count > 0) {
            RCLCPP_INFO(this->get_logger(), "🗑️ Cleaned up %d old files", deleted_count);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Error during cleanup: %s", e.what());
    }
}

std::string FVRecorderNode::generateFilename(const std::string& topic_name, const std::string& format)
{
    std::string timestamp = getCurrentTimestamp();
    std::string sanitized_topic = sanitizeTopicName(topic_name);
    std::string extension = getFileExtension(format);
    
    return timestamp + "_" + sanitized_topic + extension;
}

void FVRecorderNode::discoverAndCacheTopicTypes()
{
    try {
        auto names_and_types = this->get_topic_names_and_types();
        for (const auto& [topic_name, types] : names_and_types) {
            if (!types.empty()) {
                topic_message_types_[topic_name] = types.front();
            }
        }
        RCLCPP_INFO(this->get_logger(), "🔍 Discovered %zu topic types", topic_message_types_.size());
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Failed to discover topic types: %s", e.what());
    }
}

std::string FVRecorderNode::getMessageType(const std::string& topic_name)
{
    auto it = topic_message_types_.find(topic_name);
    if (it != topic_message_types_.end()) {
        return it->second;
    }
    
    // フォールバック: トピック名から推測
    if (topic_name.find("image") != std::string::npos) {
        return "sensor_msgs/msg/Image";
    } else if (topic_name.find("pointcloud") != std::string::npos) {
        return "sensor_msgs/msg/PointCloud2";
    } else if (topic_name.find("imu") != std::string::npos) {
        return "sensor_msgs/msg/Imu";
    } else {
        return "std_msgs/msg/String"; // デフォルト
    }
}

void FVRecorderNode::publishStatus()
{
    auto status_msg = fv_recorder::msg::RecordingStatus();
    status_msg.header.stamp = this->now();
    
    std::lock_guard<std::mutex> lock(session_mutex_);
    
    if (current_session_ && current_session_->is_active) {
        status_msg.is_recording = true;
        status_msg.recording_id = current_session_->id;
        status_msg.current_file_path = current_session_->current_file_path.string();
        
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(
            now - current_session_->start_time).count();
        status_msg.recording_duration = static_cast<double>(duration);
        
        // ファイルサイズ取得
        try {
            if (std::filesystem::exists(current_session_->current_file_path)) {
                status_msg.file_size_bytes = std::filesystem::file_size(current_session_->current_file_path);
            }
        } catch (...) {
            status_msg.file_size_bytes = 0;
        }
    } else {
        status_msg.is_recording = false;
        status_msg.recording_id = "";
        status_msg.current_file_path = "";
        status_msg.recording_duration = 0.0;
        status_msg.file_size_bytes = 0;
    }
    
    status_msg.is_playing = false;
    status_msg.playback_id = "";
    status_msg.total_frames_recorded = 0;
    status_msg.total_frames_played = 0;
    status_msg.fps = 0.0;
    
    status_publisher_->publish(status_msg);
}

std::string FVRecorderNode::getCurrentTimestamp()
{
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);
    
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

void FVRecorderNode::runCleanupLoop()
{
    while (running_) {
        try {
            std::this_thread::sleep_for(std::chrono::hours(1)); // 1時間ごとにクリーンアップ
            cleanupOldFiles();
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Error in cleanup loop: %s", e.what());
        }
    }
}

std::string FVRecorderNode::sanitizeTopicName(const std::string& topic_name)
{
    std::string sanitized = topic_name;
    std::replace(sanitized.begin(), sanitized.end(), '/', '_');
    if (sanitized.front() == '_') {
        sanitized = sanitized.substr(1);
    }
    return sanitized;
}

bool FVRecorderNode::createDirectoryIfNotExists(const std::string& path)
{
    try {
        if (!std::filesystem::exists(path)) {
            std::filesystem::create_directories(path);
            RCLCPP_INFO(this->get_logger(), "📁 Created directory: %s", path.c_str());
        }
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to create directory %s: %s", path.c_str(), e.what());
        return false;
    }
}

cv::Mat FVRecorderNode::drawTimeOverlay(const cv::Mat& src)
{
    cv::Mat img = src.clone();
    try {
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&t);
        char buf[128];
        std::strftime(buf, sizeof(buf), time_overlay_format_.c_str(), &tm);
        std::string text(buf);
        int font = cv::FONT_HERSHEY_SIMPLEX;
        double scale = 0.6;
        int thickness = 2;
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(text, font, scale, thickness, &baseline);
        cv::Point org(12, 12 + text_size.height);
        // shadow
        cv::putText(img, text, org + cv::Point(2, 2), font, scale, cv::Scalar(0,0,0), thickness + 2, cv::LINE_AA);
        // main
        cv::putText(img, text, org, font, scale, cv::Scalar(255,255,255), thickness, cv::LINE_AA);
    } catch (...) {
    }
    return img;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<FVRecorderNode>();
        
        // 状態公開タイマー
        auto status_timer = node->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&FVRecorderNode::publishStatus, node));
        
        RCLCPP_INFO(node->get_logger(), "🔄 Starting main loop...");
        
        // メインループ
        rclcpp::spin(node);
        
        RCLCPP_INFO(node->get_logger(), "🛑 Main loop ended");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "❌ Fatal error in main: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
} 