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
        writer_->write(serialized_msg, topic_name, message_type, timestamp);
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