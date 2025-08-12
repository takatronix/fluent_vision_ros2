#include "fv_recorder/fv_recorder_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <sstream>

// ROSBag2Writer実装
bool ROSBag2Writer::open(const std::string& filepath) {
    try {
        writer_ = std::make_unique<rosbag2_cpp::Writer>();
        writer_->open(filepath);
        // Create topics if type information is provided
        for (const auto& [topic_name, type_name] : topics_with_types_) {
            rosbag2_storage::TopicMetadata metadata;
            metadata.name = topic_name;
            metadata.type = type_name;
            metadata.serialization_format = "cdr";
            writer_->create_topic(metadata);
        }
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool ROSBag2Writer::write(const sensor_msgs::msg::Image::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) {
    if (writer_) {
        writer_->write(*msg, topic_name, timestamp);
        return true;
    }
    return false;
}

bool ROSBag2Writer::write(const std_msgs::msg::String::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) {
    if (writer_) {
        writer_->write(*msg, topic_name, timestamp);
        return true;
    }
    return false;
}

bool ROSBag2Writer::write(const std_msgs::msg::Bool::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) {
    if (writer_) {
        writer_->write(*msg, topic_name, timestamp);
        return true;
    }
    return false;
}

bool ROSBag2Writer::writeGeneric(const rclcpp::SerializedMessage& serialized_msg, const std::string& topic_name, const std::string& message_type, const rclcpp::Time& timestamp) {
    if (writer_) {
        // 非推奨のwriteメソッドの代わりに、新しい形式を使用
        auto shared_msg = std::make_shared<rclcpp::SerializedMessage>(serialized_msg);
        writer_->write(shared_msg, topic_name, message_type, timestamp);
        return true;
    }
    return false;
}

void ROSBag2Writer::close() {
    if (writer_) {
        writer_->close();
    }
}

// JSONWriter実装
bool JSONWriter::open(const std::string& filepath) {
    filepath_ = filepath;
    data_ = nlohmann::json::object();
    data_["metadata"] = {
        {"format", "json"},
        {"created_at", std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count()},
        {"version", "1.0"}
    };
    data_["messages"] = nlohmann::json::array();
    return true;
}

bool JSONWriter::write(const sensor_msgs::msg::Image::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) {
    nlohmann::json message = {
        {"timestamp", timestamp.nanoseconds()},
        {"topic", topic_name},
        {"type", "sensor_msgs/msg/Image"},
        {"data", {
            {"width", msg->width},
            {"height", msg->height},
            {"encoding", msg->encoding},
            {"step", msg->step},
            {"data_size", msg->data.size()}
        }}
    };
    data_["messages"].push_back(message);
    return true;
}

bool JSONWriter::write(const std_msgs::msg::String::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) {
    nlohmann::json message = {
        {"timestamp", timestamp.nanoseconds()},
        {"topic", topic_name},
        {"type", "std_msgs/msg/String"},
        {"data", msg->data}
    };
    data_["messages"].push_back(message);
    return true;
}

bool JSONWriter::write(const std_msgs::msg::Bool::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) {
    nlohmann::json message = {
        {"timestamp", timestamp.nanoseconds()},
        {"topic", topic_name},
        {"type", "std_msgs/msg/Bool"},
        {"data", msg->data}
    };
    data_["messages"].push_back(message);
    return true;
}

bool JSONWriter::writeGeneric(const rclcpp::SerializedMessage& serialized_msg, const std::string& topic_name, const std::string& message_type, const rclcpp::Time& timestamp) {
    nlohmann::json message = {
        {"timestamp", timestamp.nanoseconds()},
        {"topic", topic_name},
        {"type", message_type},
        {"data", {
            {"serialized_size", serialized_msg.size()},
            {"message_type", message_type}
        }}
    };
    data_["messages"].push_back(message);
    return true;
}

void JSONWriter::close() {
    if (!filepath_.empty()) {
        std::ofstream file(filepath_);
        if (file.is_open()) {
            file << data_.dump(2);
            file.close();
        }
    }
} 

// YAMLWriter実装
bool YAMLWriter::open(const std::string& filepath) {
    filepath_ = filepath;
    file_.open(filepath_, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
        return false;
    }
    file_ << "metadata:\n";
    file_ << "  format: yaml\n";
    file_ << "  version: 1.0\n";
    file_ << "messages:\n";
    return true;
}

bool YAMLWriter::write(const sensor_msgs::msg::Image::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) {
    if (!file_.is_open()) return false;
    file_ << "  - timestamp: " << timestamp.nanoseconds() << "\n";
    file_ << "    topic: '" << topic_name << "'\n";
    file_ << "    type: sensor_msgs/msg/Image\n";
    file_ << "    data:\n";
    file_ << "      width: " << msg->width << "\n";
    file_ << "      height: " << msg->height << "\n";
    file_ << "      encoding: '" << msg->encoding << "'\n";
    file_ << "      step: " << msg->step << "\n";
    file_ << "      data_size: " << msg->data.size() << "\n";
    return true;
}

bool YAMLWriter::write(const std_msgs::msg::String::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) {
    if (!file_.is_open()) return false;
    file_ << "  - timestamp: " << timestamp.nanoseconds() << "\n";
    file_ << "    topic: '" << topic_name << "'\n";
    file_ << "    type: std_msgs/msg/String\n";
    file_ << "    data: '" << msg->data << "'\n";
    return true;
}

bool YAMLWriter::write(const std_msgs::msg::Bool::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) {
    if (!file_.is_open()) return false;
    file_ << "  - timestamp: " << timestamp.nanoseconds() << "\n";
    file_ << "    topic: '" << topic_name << "'\n";
    file_ << "    type: std_msgs/msg/Bool\n";
    file_ << "    data: " << (msg->data ? "true" : "false") << "\n";
    return true;
}

bool YAMLWriter::writeGeneric(const rclcpp::SerializedMessage& serialized_msg, const std::string& topic_name, const std::string& message_type, const rclcpp::Time& timestamp) {
    if (!file_.is_open()) return false;
    file_ << "  - timestamp: " << timestamp.nanoseconds() << "\n";
    file_ << "    topic: '" << topic_name << "'\n";
    file_ << "    type: '" << message_type << "'\n";
    file_ << "    data:\n";
    file_ << "      serialized_size: " << serialized_msg.size() << "\n";
    return true;
}

void YAMLWriter::close() {
    if (file_.is_open()) {
        file_.close();
    }
}

// CSVWriter実装
bool CSVWriter::open(const std::string& filepath) {
    filepath_ = filepath;
    file_.open(filepath_, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
        return false;
    }
    file_ << "timestamp,topic,type,data\n";
    return true;
}

bool CSVWriter::write(const sensor_msgs::msg::Image::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) {
    if (!file_.is_open()) return false;
    file_ << timestamp.nanoseconds() << "," << topic_name << ",sensor_msgs/msg/Image,"
          << msg->width << "x" << msg->height << "x" << msg->encoding << "\n";
    return true;
}

bool CSVWriter::write(const std_msgs::msg::String::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) {
    if (!file_.is_open()) return false;
    // Escape quotes in CSV
    std::string data = msg->data;
    for (size_t pos = 0; (pos = data.find('"', pos)) != std::string::npos; pos += 2) {
        data.replace(pos, 1, "\"\"");
    }
    file_ << timestamp.nanoseconds() << "," << topic_name << ",std_msgs/msg/String,\"" << data << "\"\n";
    return true;
}

bool CSVWriter::write(const std_msgs::msg::Bool::SharedPtr& msg, const std::string& topic_name, const rclcpp::Time& timestamp) {
    if (!file_.is_open()) return false;
    file_ << timestamp.nanoseconds() << "," << topic_name << ",std_msgs/msg/Bool," << (msg->data ? "true" : "false") << "\n";
    return true;
}

bool CSVWriter::writeGeneric(const rclcpp::SerializedMessage& serialized_msg, const std::string& topic_name, const std::string& message_type, const rclcpp::Time& timestamp) {
    if (!file_.is_open()) return false;
    file_ << timestamp.nanoseconds() << "," << topic_name << "," << message_type << ",size=" << serialized_msg.size() << "\n";
    return true;
}

void CSVWriter::close() {
    if (file_.is_open()) {
        file_.close();
    }
}

// VideoWriter実装（画像トピックのみ対応）
bool VideoWriter::open(const std::string& filepath) {
    filepath_ = filepath;
    // 実際のサイズとFPSは最初のフレームで決定
    initialized_ = false;
    return true;
}

bool VideoWriter::write(const sensor_msgs::msg::Image::SharedPtr& msg, const std::string& /*topic_name*/, const rclcpp::Time& timestamp) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr->image;
        if (overlay_time_) {
            img = drawTimeOverlay(img, overlay_time_format_, timestamp);
        }
        if (!initialized_) {
            double fps = 30.0; // 仮のFPS。厳密なFPS制御はノード側で調整する
            int fourcc = (format_ == "mp4") ? cv::VideoWriter::fourcc('a','v','c','1') : cv::VideoWriter::fourcc('M','J','P','G');
            if (!video_writer_.open(filepath_, fourcc, fps, img.size(), true)) {
                return false;
            }
            initialized_ = true;
        }
        if (video_writer_.isOpened()) {
            video_writer_.write(img);
            return true;
        }
    } catch (...) {
        return false;
    }
    return false;
}

bool VideoWriter::write(const std_msgs::msg::String::SharedPtr& /*msg*/, const std::string& /*topic_name*/, const rclcpp::Time& /*timestamp*/) {
    // 動画では非対応
    return true;
}

bool VideoWriter::write(const std_msgs::msg::Bool::SharedPtr& /*msg*/, const std::string& /*topic_name*/, const rclcpp::Time& /*timestamp*/) {
    // 動画では非対応
    return true;
}

bool VideoWriter::writeGeneric(const rclcpp::SerializedMessage& /*serialized_msg*/, const std::string& /*topic_name*/, const std::string& /*message_type*/, const rclcpp::Time& /*timestamp*/) {
    // 動画では非対応
    return true;
}

void VideoWriter::close() {
    if (video_writer_.isOpened()) {
        video_writer_.release();
    }
}

cv::Mat VideoWriter::drawTimeOverlay(const cv::Mat& src, const std::string& time_format, const rclcpp::Time& timestamp) {
    cv::Mat img = src.clone();
    try {
        // use ROS time
        const int64_t sec = timestamp.seconds();
        std::time_t t = static_cast<std::time_t>(sec);
        std::tm tm = *std::localtime(&t);
        char buf[128];
        std::strftime(buf, sizeof(buf), time_format.c_str(), &tm);
        std::string text(buf);
        int font = cv::FONT_HERSHEY_SIMPLEX;
        double scale = 0.6;
        int thickness = 2;
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(text, font, scale, thickness, &baseline);
        cv::Point org(12, 12 + text_size.height);
        cv::putText(img, text, org + cv::Point(2, 2), font, scale, cv::Scalar(0,0,0), thickness + 2, cv::LINE_AA);
        cv::putText(img, text, org, font, scale, cv::Scalar(255,255,255), thickness, cv::LINE_AA);
    } catch (...) {
    }
    return img;
}