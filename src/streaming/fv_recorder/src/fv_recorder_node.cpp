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
        
        RCLCPP_INFO(this->get_logger(), "✅ FV Recorder Node started successfully");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Exception during initialization: %s", e.what());
        throw; // 致命的なエラーの場合は再スロー
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
        std::vector<std::string>{"/fv/d415/color/image_raw", "/fv/d415/depth/image_rect_raw", 
                                "/fv/d405/color/image_raw", "/fv/d405/depth/image_rect_raw"});
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
    preview_output_topic_ = this->declare_parameter("preview.output_topic", std::string("/fv_recorder/preview"));
    
    RCLCPP_INFO(this->get_logger(), "📁 Output directory: %s", config_.output_directory.c_str());
    RCLCPP_INFO(this->get_logger(), "⏱️ Segment duration: %d seconds", config_.segment_duration);
    RCLCPP_INFO(this->get_logger(), "🗓️ Retention days: %d", config_.retention_days);
    RCLCPP_INFO(this->get_logger(), "🔄 Auto recording: %s", config_.auto_recording ? "enabled" : "disabled");
    RCLCPP_INFO(this->get_logger(), "📄 Default format: %s", config_.default_format.c_str());
}

void FVRecorderNode::initializeServices()
{
    RCLCPP_INFO(this->get_logger(), "🔧 Initializing services...");
    
    // 録画開始サービス
    start_recording_service_ = this->create_service<fv_recorder::srv::StartRecording>(
        "start_recording",
        std::bind(&FVRecorderNode::startRecording, this, 
            std::placeholders::_1, std::placeholders::_2));
    
    // 録画停止サービス
    stop_recording_service_ = this->create_service<fv_recorder::srv::StopRecording>(
        "stop_recording",
        std::bind(&FVRecorderNode::stopRecording, this, 
            std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "✅ Services initialized");
}

void FVRecorderNode::initializePublishers()
{
    RCLCPP_INFO(this->get_logger(), "📤 Initializing publishers...");
    
    // 状態パブリッシャー
    status_publisher_ = this->create_publisher<fv_recorder::msg::RecordingStatus>(
        "status", 10);

    if (preview_enabled_) {
        preview_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            preview_output_topic_, rclcpp::QoS(2).best_effort());
        RCLCPP_INFO(this->get_logger(), "🖼️ Preview publisher on: %s", preview_output_topic_.c_str());
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ Publishers initialized");
}

void FVRecorderNode::initializeSubscriptions()
{
    RCLCPP_INFO(this->get_logger(), "📥 Initializing subscriptions...");
    
    // 各トピックのサブスクリプションを作成（型付き + フォールバック汎用）
    for (const auto& topic : config_.input_topics) {
        try {
            auto subscription = this->create_subscription<sensor_msgs::msg::Image>(
                topic, rclcpp::SensorDataQoS(),
                [this, topic](const sensor_msgs::msg::Image::SharedPtr msg) {
                    this->imageCallback(msg, topic);
                });
            
            subscriptions_[topic] = subscription;
            topic_to_filename_map_[topic] = sanitizeTopicName(topic);
            
            RCLCPP_INFO(this->get_logger(), "📹 Subscribed to: %s", topic.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to create subscription for %s: %s", topic.c_str(), e.what());
            // フォールバックとして汎用サブスクリプションを試す
            try {
                rclcpp::SubscriptionOptions options;
                auto generic_sub = this->create_generic_subscription(
                    topic, "", rclcpp::QoS(rclcpp::SensorDataQoS()),
                    [this, topic](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
                        this->genericMessageCallback(serialized_msg, topic);
                    }, options);
                generic_subscriptions_[topic] = generic_sub;
                topic_to_filename_map_[topic] = sanitizeTopicName(topic);
                RCLCPP_INFO(this->get_logger(), "🧩 Fallback generic subscription created for: %s", topic.c_str());
            } catch (const std::exception& eg) {
                RCLCPP_ERROR(this->get_logger(), "❌ Failed generic subscription for %s: %s", topic.c_str(), eg.what());
            }
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ Subscriptions initialized");
}

void FVRecorderNode::initializeControlSubscriptions()
{
    RCLCPP_INFO(this->get_logger(), "🎮 Initializing control subscriptions...");
    
    // 録画制御トピック（Bool型）
    recording_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "recording_control", 10,
        std::bind(&FVRecorderNode::recordingControlCallback, this, std::placeholders::_1));
    
    // 録画コマンドトピック（String型）
    recording_command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "recording_command", 10,
        std::bind(&FVRecorderNode::recordingCommandCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "✅ Control subscriptions initialized");
}

void FVRecorderNode::recordingControlCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "🎮 Received recording control: %s", msg->data ? "START" : "STOP");
    
    if (msg->data) {
        // 録画開始
        startRecordingInternal();
    } else {
        // 録画停止
        stopRecordingInternal();
    }
}

void FVRecorderNode::recordingCommandCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "🎮 Received recording command: %s", msg->data.c_str());
    
    std::string command = msg->data;
    std::transform(command.begin(), command.end(), command.begin(), ::tolower);
    
    if (command == "start" || command == "start_recording") {
        startRecordingInternal();
    } else if (command == "stop" || command == "stop_recording") {
        stopRecordingInternal();
    } else if (command == "toggle") {
        // 現在の状態を確認して切り替え
        std::lock_guard<std::mutex> lock(session_mutex_);
        if (current_session_ && current_session_->is_active) {
            stopRecordingInternal();
        } else {
            startRecordingInternal();
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ Unknown recording command: %s", msg->data.c_str());
    }
}

std::unique_ptr<FormatWriter> FVRecorderNode::createFormatWriter(const std::string& format)
{
    if (format == "rosbag") {
        return std::make_unique<ROSBag2Writer>();
    } else if (format == "json") {
        return std::make_unique<JSONWriter>();
    } else if (format == "yaml") {
        return std::make_unique<YAMLWriter>();
    } else if (format == "csv") {
        return std::make_unique<CSVWriter>();
    } else if (format == "mp4" || format == "avi") {
        return std::make_unique<VideoWriter>(format);
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ Unknown format: %s, using rosbag", format.c_str());
        return std::make_unique<ROSBag2Writer>();
    }
}

std::string FVRecorderNode::getFileExtension(const std::string& format)
{
    if (format == "rosbag") {
        return ".db3";
    } else if (format == "json") {
        return ".json";
    } else if (format == "yaml") {
        return ".yaml";
    } else if (format == "csv") {
        return ".csv";
    } else if (format == "mp4") {
        return ".mp4";
    } else if (format == "avi") {
        return ".avi";
    } else {
        return ".db3"; // デフォルト
    }
}

void FVRecorderNode::startRecordingInternal(const std::string& directory, const std::string& date_format, 
                                           const std::vector<std::string>& custom_topics, const std::string& format)
{
    RCLCPP_INFO(this->get_logger(), "🎬 Starting recording (internal)...");
    
    std::lock_guard<std::mutex> lock(session_mutex_);
    
    // 既存の録画セッションがある場合は停止
    if (current_session_ && current_session_->is_active) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Stopping existing recording session");
        current_session_->is_active = false;
        if (current_session_->writer) {
            current_session_->writer->close();
        }
    }
    
    // 使用するトピックリストを決定
    std::vector<std::string> topics_to_record = custom_topics.empty() ? config_.input_topics : custom_topics;
    
    // 使用するフォーマットを決定
    std::string output_format = format.empty() ? config_.default_format : format;
    
    RCLCPP_INFO(this->get_logger(), "📹 Recording topics:");
    for (const auto& topic : topics_to_record) {
        RCLCPP_INFO(this->get_logger(), "  - %s", topic.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "📄 Output format: %s", output_format.c_str());
    
    // 新しい録画セッション作成
    current_session_ = std::make_unique<RecordingSession>();
    current_session_->id = getCurrentTimestamp();
    current_session_->directory = directory.empty() ? config_.output_directory : directory;
    current_session_->start_time = std::chrono::steady_clock::now();
    current_session_->segment_start_time = current_session_->start_time;
    current_session_->segment_count = 0;
    current_session_->is_active = true;
    current_session_->format = output_format;
    
    // ディレクトリ作成
    if (!createDirectoryIfNotExists(current_session_->directory)) {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to create recording directory");
        return;
    }
    
    // 新しいセグメント作成（カスタムトピックリストとフォーマットを使用）
    createNewSegment(topics_to_record, output_format);
    
    RCLCPP_INFO(this->get_logger(), "✅ Recording started with ID: %s", current_session_->id.c_str());
}

void FVRecorderNode::stopRecordingInternal()
{
    RCLCPP_INFO(this->get_logger(), "⏹️ Stopping recording (internal)...");
    
    std::lock_guard<std::mutex> lock(session_mutex_);
    
    if (!current_session_ || !current_session_->is_active) {
        RCLCPP_WARN(this->get_logger(), "⚠️ No active recording session to stop");
        return;
    }
    
    // 録画セッション停止
    current_session_->is_active = false;
    if (current_session_->writer) {
        current_session_->writer->close();
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ Recording stopped. Final file: %s", 
        current_session_->current_file_path.c_str());
}

void FVRecorderNode::startRecording(
    const std::shared_ptr<fv_recorder::srv::StartRecording::Request> request,
    std::shared_ptr<fv_recorder::srv::StartRecording::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "🎬 Starting recording (service)...");
    
    // カスタムトピックリストを取得
    std::vector<std::string> custom_topics;
    if (!request->input_topics.empty()) {
        custom_topics = request->input_topics;
        RCLCPP_INFO(this->get_logger(), "📹 Using custom topics from service request");
    } else {
        RCLCPP_INFO(this->get_logger(), "📹 Using default topics from config");
    }
    
    // 出力フォーマットを取得
    std::string output_format = request->output_format.empty() ? config_.default_format : request->output_format;
    RCLCPP_INFO(this->get_logger(), "📄 Using output format: %s", output_format.c_str());

    // サービスからセグメント時間/保持日数のオーバーライド（0以下は無視）
    if (request->segment_duration > 0) {
        config_.segment_duration = request->segment_duration;
        RCLCPP_INFO(this->get_logger(), "⏱️ Overridden segment_duration: %d", config_.segment_duration);
    }
    if (request->retention_days >= 0) {
        config_.retention_days = request->retention_days;
        RCLCPP_INFO(this->get_logger(), "🗓️ Overridden retention_days: %d", config_.retention_days);
    }
    
    startRecordingInternal(request->recording_directory, request->date_format, custom_topics, output_format);
    
    response->success = true;
    response->message = "Recording started successfully";
    response->recording_id = current_session_ ? current_session_->id : "";
}

void FVRecorderNode::stopRecording(
    const std::shared_ptr<fv_recorder::srv::StopRecording::Request> request,
    std::shared_ptr<fv_recorder::srv::StopRecording::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "⏹️ Stopping recording (service)...");
    
    std::lock_guard<std::mutex> lock(session_mutex_);
    
    if (!current_session_ || !current_session_->is_active) {
        response->success = false;
        response->message = "No active recording session";
        return;
    }
    
    // 録画セッション停止
    current_session_->is_active = false;
    if (current_session_->writer) {
        current_session_->writer->close();
    }
    
    response->success = true;
    response->message = "Recording stopped successfully";
    response->final_file_path = current_session_->current_file_path;
    
    RCLCPP_INFO(this->get_logger(), "✅ Recording stopped. Final file: %s", 
        current_session_->current_file_path.c_str());
}

void FVRecorderNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg, 
                                  const std::string& topic_name)
{
    std::lock_guard<std::mutex> lock(session_mutex_);
    
    if (!current_session_ || !current_session_->is_active) {
        return;
    }
    
    // セグメント時間チェック
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - current_session_->segment_start_time).count();
    
    if (elapsed >= config_.segment_duration) {
        // 現在のトピックリストを使用してセグメント作成
        std::vector<std::string> current_topics;
        for (const auto& sub : subscriptions_) {
            current_topics.push_back(sub.first);
        }
        createNewSegment(current_topics, current_session_->format);
    }
    
    // メッセージをwriterに記録
    if (current_session_->writer) {
        current_session_->writer->write(msg, topic_name, this->now());
    }

    // プレビュー配信（時刻オーバーレイ対応）
    if (preview_enabled_ && preview_image_publisher_ && preview_image_publisher_->get_subscription_count() > 0) {
        try {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat preview = time_overlay_enabled_ ? drawTimeOverlay(cv_ptr->image) : cv_ptr->image;
            cv_bridge::CvImage out;
            out.header = msg->header;
            out.encoding = sensor_msgs::image_encodings::BGR8;
            out.image = preview;
            preview_image_publisher_->publish(*out.toImageMsg());
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Preview publish failed: %s", e.what());
        }
    }
}

void FVRecorderNode::genericMessageCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg, 
                                           const std::string& topic_name)
{
    std::lock_guard<std::mutex> lock(session_mutex_);
    
    if (!current_session_ || !current_session_->is_active) {
        return;
    }
    
    // セグメント時間チェック
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - current_session_->segment_start_time).count();
    
    if (elapsed >= config_.segment_duration) {
        // 現在のトピックリストを使用してセグメント作成
        std::vector<std::string> current_topics;
        for (const auto& sub : subscriptions_) {
            current_topics.push_back(sub.first);
        }
        for (const auto& sub : generic_subscriptions_) {
            current_topics.push_back(sub.first);
        }
        createNewSegment(current_topics, current_session_->format);
    }
    
    // メッセージタイプを取得
    std::string message_type = getMessageType(topic_name);
    
    // メッセージをwriterに記録
    if (current_session_->writer) {
        current_session_->writer->writeGeneric(*msg, topic_name, message_type, this->now());
    }
}

std::string FVRecorderNode::getMessageType(const std::string& topic_name)
{
    // 既知の型キャッシュを優先
    auto it = topic_message_types_.find(topic_name);
    if (it != topic_message_types_.end()) {
        return it->second;
    }
    
    // デフォルトの推測ロジック
    if (topic_name.find("image") != std::string::npos) {
        return "sensor_msgs/msg/Image";
    } else if (topic_name.find("pointcloud") != std::string::npos || topic_name.find("points") != std::string::npos) {
        return "sensor_msgs/msg/PointCloud2";
    } else if (topic_name.find("imu") != std::string::npos) {
        return "sensor_msgs/msg/Imu";
    } else if (topic_name.find("gps") != std::string::npos || topic_name.find("navsat") != std::string::npos) {
        return "sensor_msgs/msg/NavSatFix";
    } else if (topic_name.find("scan") != std::string::npos || topic_name.find("laser") != std::string::npos) {
        return "sensor_msgs/msg/LaserScan";
    } else if (topic_name.find("pose") != std::string::npos) {
        return "geometry_msgs/msg/PoseStamped";
    } else if (topic_name.find("twist") != std::string::npos) {
        return "geometry_msgs/msg/Twist";
    } else if (topic_name.find("odom") != std::string::npos) {
        return "nav_msgs/msg/Odometry";
    } else if (topic_name.find("string") != std::string::npos) {
        return "std_msgs/msg/String";
    } else if (topic_name.find("bool") != std::string::npos) {
        return "std_msgs/msg/Bool";
    } else if (topic_name.find("float") != std::string::npos) {
        return "std_msgs/msg/Float32";
    } else if (topic_name.find("int") != std::string::npos) {
        return "std_msgs/msg/Int32";
    } else {
        return "std_msgs/msg/String"; // デフォルト
    }
}

void FVRecorderNode::discoverAndCacheTopicTypes()
{
    try {
        // クライアントAPIでトピック一覧と型を取得
        auto topic_names_and_types = this->get_topic_names_and_types();
        for (const auto& entry : topic_names_and_types) {
            const auto& name = entry.first;
            const auto& types = entry.second;
            if (!types.empty()) {
                // 最初の型を使用
                topic_message_types_[name] = types.front();
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Failed to discover topic types: %s", e.what());
    }
}

void FVRecorderNode::createNewSegment(const std::vector<std::string>& topics_to_record, const std::string& format)
{
    if (!current_session_) {
        return;
    }
    
    // 既存のwriterを閉じる
    if (current_session_->writer) {
        current_session_->writer->close();
    }
    
    // 新しいファイル名生成
    std::string filename = generateFilename("segment_" + std::to_string(current_session_->segment_count), format);
    current_session_->current_file_path = current_session_->directory + "/" + filename;
    
    // 新しいwriter作成
    current_session_->writer = createFormatWriter(format);
    
    // ROSBag2Writerの場合は特別な設定が必要（トピックの型を付与してcreate_topic）
    if (auto* rosbag_writer = dynamic_cast<ROSBag2Writer*>(current_session_->writer.get())) {
        // 最新の型キャッシュを更新
        discoverAndCacheTopicTypes();
        std::vector<std::pair<std::string, std::string>> topics_with_types;
        topics_with_types.reserve(topics_to_record.size());
        for (const auto& t : topics_to_record) {
            topics_with_types.emplace_back(t, getMessageType(t));
        }
        rosbag_writer->setTopicsWithTypes(topics_with_types);
    }
    
    if (!current_session_->writer->open(current_session_->current_file_path)) {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to open writer for format: %s", format.c_str());
        return;
    }
    
    current_session_->segment_start_time = std::chrono::steady_clock::now();
    current_session_->segment_count++;
    
    RCLCPP_INFO(this->get_logger(), "📁 New segment created: %s", filename.c_str());
}

void FVRecorderNode::publishStatus()
{
    try {
        if (!status_publisher_) {
            return;
        }
        
        auto status_msg = fv_recorder::msg::RecordingStatus();
        status_msg.header.stamp = this->now();
        
        std::lock_guard<std::mutex> lock(session_mutex_);
        
        if (current_session_ && current_session_->is_active) {
            status_msg.is_recording = true;
            status_msg.recording_id = current_session_->id;
            status_msg.current_file_path = current_session_->current_file_path;
            
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                now - current_session_->start_time).count();
            status_msg.recording_duration = duration;
        } else {
            status_msg.is_recording = false;
        }
        
        status_publisher_->publish(status_msg);
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Error in publishStatus: %s", e.what());
    }
}

void FVRecorderNode::cleanupOldFiles()
{
    if (config_.retention_days <= 0) {
        return; // 保持期間が0以下の場合は削除しない
    }
    
    try {
        std::filesystem::path recording_dir(config_.output_directory);
        if (!std::filesystem::exists(recording_dir)) {
            return;
        }
        
        auto now = std::chrono::system_clock::now();
        auto cutoff_time = now - std::chrono::hours(24 * config_.retention_days);
        
        for (const auto& entry : std::filesystem::directory_iterator(recording_dir)) {
            if (entry.is_regular_file()) {
                auto file_time = std::filesystem::last_write_time(entry.path());
                auto file_time_point = std::chrono::system_clock::from_time_t(
                    std::chrono::duration_cast<std::chrono::seconds>(file_time.time_since_epoch()).count());
                
                if (file_time_point < cutoff_time) {
                    std::filesystem::remove(entry.path());
                    RCLCPP_INFO(this->get_logger(), "🗑️ Deleted old file: %s", entry.path().c_str());
                }
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Error during cleanup: %s", e.what());
    }
}

std::string FVRecorderNode::generateFilename(const std::string& topic_name, const std::string& format)
{
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);
    
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S") << "_" << topic_name << getFileExtension(format);
    return oss.str();
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