#include "fv_rtmp_server/rtmp_bridge_node.hpp"
#include "fv_rtmp_server/rtmp_server.hpp"
#include "fv_rtmp_server/video_processor.hpp"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fv_rtmp_server
{

RTMPBridgeNode::RTMPBridgeNode()
    : Node("fv_rtmp_server_node"),
      should_stop_(false),
      frames_received_(0),
      frames_published_(0),
      last_status_time_(this->get_clock()->now()),
      client_connected_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing RTMP Bridge Node");

    // Initialize components in order
    initializeParameters();
    initializePublishers();
    initializeServer();

    RCLCPP_INFO(this->get_logger(), "RTMP Bridge Node initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Server will listen on port %d", server_port_);
    RCLCPP_INFO(this->get_logger(), "Endpoint: %s", server_endpoint_.c_str());
    RCLCPP_INFO(this->get_logger(), "Target resolution: %dx%d", target_width_, target_height_);
}

RTMPBridgeNode::~RTMPBridgeNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down RTMP Bridge Node");
    stopServer();
}

void RTMPBridgeNode::initializeParameters()
{
    // Declare parameters with defaults
    this->declare_parameter<int>("server.port", 1935);
    this->declare_parameter<std::string>("server.endpoint", "/live/s");
    this->declare_parameter<std::string>("output.topic_name", "/fv_rtmp_server/camera/image_raw");
    this->declare_parameter<std::string>("output.frame_id", "camera_link");
    this->declare_parameter<int>("output.target_width", 640);
    this->declare_parameter<int>("output.target_height", 480);
    this->declare_parameter<double>("output.publish_rate", 30.0);
    this->declare_parameter<bool>("processing.enable_low_latency", true);
    this->declare_parameter<int>("processing.buffer_size", 1);
    this->declare_parameter<std::string>("processing.quality_mode", "fast");
    this->declare_parameter<std::string>("camera.calibration_file", "config/camera_info_o4.yaml");
    this->declare_parameter<std::string>("camera.info_topic", "/fv_rtmp_server/camera/camera_info");
    this->declare_parameter<bool>("output.compressed", false);

    // Get parameter values
    server_port_ = this->get_parameter("server.port").as_int();
    server_endpoint_ = this->get_parameter("server.endpoint").as_string();
    output_topic_name_ = this->get_parameter("output.topic_name").as_string();
    frame_id_ = this->get_parameter("output.frame_id").as_string();
    target_width_ = this->get_parameter("output.target_width").as_int();
    target_height_ = this->get_parameter("output.target_height").as_int();
    publish_rate_ = this->get_parameter("output.publish_rate").as_double();
    enable_low_latency_ = this->get_parameter("processing.enable_low_latency").as_bool();
    buffer_size_ = this->get_parameter("processing.buffer_size").as_int();
    quality_mode_ = this->get_parameter("processing.quality_mode").as_string();
    calibration_file_ = this->get_parameter("camera.calibration_file").as_string();
    camera_info_topic_ = this->get_parameter("camera.info_topic").as_string();
    compressed_ = this->get_parameter("output.compressed").as_bool();

    RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
    RCLCPP_INFO(this->get_logger(), "  Server port: %d", server_port_);
    RCLCPP_INFO(this->get_logger(), "  Endpoint: %s", server_endpoint_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Target size: %dx%d", target_width_, target_height_);
    RCLCPP_INFO(this->get_logger(), "  Low latency: %s", enable_low_latency_ ? "enabled" : "disabled");
    RCLCPP_INFO(this->get_logger(), "  Compressed: %s", compressed_ ? "enabled" : "disabled");
}

void RTMPBridgeNode::initializePublishers()
{
    // Create QoS profile for low latency
    rclcpp::QoS image_qos(buffer_size_);
    image_qos.best_effort().durability_volatile();

    rclcpp::QoS status_qos(10);
    status_qos.reliable().durability_volatile();

    status_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/fv_rtmp_server/status", status_qos);

    // Camera info publishers for both O4 variants
    camera_info_o4_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/fv_rtmp_server/o4/camera_info", status_qos);
    camera_info_o4_pro_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/fv_rtmp_server/o4_pro/camera_info", status_qos);

    RCLCPP_INFO(this->get_logger(), "Publishers initialized");
}

void RTMPBridgeNode::initializeServer()
{
    // Check if port is already in use
    if (isPortInUse(server_port_)) {
        RCLCPP_ERROR(this->get_logger(), 
                     "Port %d is already in use. Another fv_rtmp_server_node may be running.", 
                     server_port_);
        RCLCPP_ERROR(this->get_logger(), 
                     "Please stop the existing node before starting a new one.");
        throw std::runtime_error("Port already in use");
    }

    // Create video processor
    video_processor_ = std::make_unique<fv_rtmp_server::VideoProcessor>();
    video_processor_->setTargetSize(target_width_, target_height_);
    video_processor_->setFrameId(frame_id_);
    video_processor_->setQualityMode(quality_mode_);

    // Create RTMP server
    rtmp_server_ = std::make_unique<fv_rtmp_server::RTMPServer>();
    rtmp_server_->setPort(server_port_);
    rtmp_server_->setEndpoint(server_endpoint_);
    rtmp_server_->setLowLatencyMode(enable_low_latency_);

    // Set callbacks
    rtmp_server_->setFrameCallback(
        [this](const uint8_t* data, int width, int height, int64_t timestamp) {
            onFrameReceived(data, width, height, timestamp);
        });

    rtmp_server_->setStatusCallback(
        [this](const std::string& status, const std::string& detail) {
            onStatusChanged(status, detail);
        });

    // Load camera calibration info
    if (!loadCameraInfo()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load camera calibration files");
    }

    // Get local IP address
    local_ip_address_ = getLocalIPAddress();
    RCLCPP_INFO(this->get_logger(), "Local IP: %s", local_ip_address_.c_str());
    
    // Start IP info display timer
    startInfoImageTimer();
    
    // Start server
    startServer();
}

void RTMPBridgeNode::startServer()
{
    server_thread_ = std::thread([this]() {
        if (!rtmp_server_->start()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start RTMP server");
            onStatusChanged("ERROR", "Failed to start server on port " + std::to_string(server_port_));
            return;
        }

        RCLCPP_INFO(this->get_logger(), "RTMP server started successfully");
        onStatusChanged("LISTENING", "Server listening on port " + std::to_string(server_port_));

        // Keep the server running
        while (!should_stop_ && rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
}

void RTMPBridgeNode::stopServer()
{
    should_stop_ = true;

    if (rtmp_server_) {
        rtmp_server_->stop();
    }

    if (server_thread_.joinable()) {
        server_thread_.join();
    }

    RCLCPP_INFO(this->get_logger(), "RTMP server stopped");
}

void RTMPBridgeNode::onFrameReceived(const uint8_t* data, int width, int height, int64_t timestamp)
{
    frames_received_++;

    if (!video_processor_) {
        RCLCPP_WARN(this->get_logger(), "Video processor not initialized");
        return;
    }

    // Process frame
    auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
    
    if (video_processor_->processFrame(data, width, height, timestamp, *image_msg)) {
        // Initialize image transport on first use
        if (!image_transport_) {
            image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
            image_transport_publisher_ = image_transport_->advertise(output_topic_name_, buffer_size_);
            
            RCLCPP_INFO(this->get_logger(), "Image topics initialized:");
            RCLCPP_INFO(this->get_logger(), "  Raw: %s", output_topic_name_.c_str());
            RCLCPP_INFO(this->get_logger(), "  Compressed: %s/compressed", output_topic_name_.c_str());
        }
        
        // Publish image (raw + compressed automatically)
        image_transport_publisher_.publish(*image_msg);
        frames_published_++;

        // Periodic status update
        auto current_time = this->get_clock()->now();
        if ((current_time - last_status_time_).seconds() >= 5.0) {
            std::string status_msg = "STREAMING - Frames: " + 
                std::to_string(frames_received_.load()) + " received, " +
                std::to_string(frames_published_.load()) + " published";
            
            onStatusChanged("STREAMING", status_msg);
            last_status_time_ = current_time;
        }
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Failed to process frame");
    }
}

void RTMPBridgeNode::onStatusChanged(const std::string& status, const std::string& detail)
{
    // Update client connection status
    if (status == "CONNECTED" || status == "STREAMING") {
        client_connected_ = true;
    } else if (status == "DISCONNECTED") {
        client_connected_ = false;
    }

    // Create status message
    auto status_msg = std::make_shared<std_msgs::msg::String>();
    status_msg->data = status;
    
    if (!detail.empty()) {
        status_msg->data += ": " + detail;
    }

    // Publish status
    status_publisher_->publish(*status_msg);

    // Log status change
    if (status == "ERROR") {
        RCLCPP_ERROR(this->get_logger(), "Status: %s", status_msg->data.c_str());
    } else if (status == "STREAMING") {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Status: %s", status_msg->data.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Status: %s", status_msg->data.c_str());
    }
}

bool RTMPBridgeNode::isPortInUse(int port)
{
    int test_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (test_socket < 0) {
        return true; // Assume in use if can't create socket
    }

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    int result = bind(test_socket, (struct sockaddr*)&addr, sizeof(addr));
    close(test_socket);
    
    return result != 0; // Port is in use if bind fails
}

std::string RTMPBridgeNode::getLocalIPAddress()
{
    // 環境変数HOST_IPを優先的に使用
    const char* env_host_ip = std::getenv("HOST_IP");
    if (env_host_ip && strlen(env_host_ip) > 0) {
        RCLCPP_INFO(this->get_logger(), "Using HOST_IP from environment: %s", env_host_ip);
        return std::string(env_host_ip);
    }
    
    // 環境変数がない場合は自動検出
    struct ifaddrs *ifaddrs_ptr = nullptr;
    std::string ip_address = "Unknown";
    
    if (getifaddrs(&ifaddrs_ptr) == -1) {
        return ip_address;
    }
    
    for (struct ifaddrs *ifa = ifaddrs_ptr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;
        
        // IPv4アドレスを探す
        if (ifa->ifa_addr->sa_family == AF_INET) {
            struct sockaddr_in* addr_in = (struct sockaddr_in*)ifa->ifa_addr;
            char* ip_str = inet_ntoa(addr_in->sin_addr);
            
            // ローカルホスト以外のIPアドレスを取得
            if (strcmp(ip_str, "127.0.0.1") != 0) {
                ip_address = std::string(ip_str);
                break;
            }
        }
    }
    
    freeifaddrs(ifaddrs_ptr);
    RCLCPP_INFO(this->get_logger(), "Auto-detected local IP: %s", ip_address.c_str());
    return ip_address;
}

void RTMPBridgeNode::publishInfoImage(const std::string& status, const std::string& ip_address)
{
    // 画像サイズ
    int width = target_width_;
    int height = target_height_;
    
    // OpenCVで情報画像を作成
    cv::Mat info_img(height, width, CV_8UC3, cv::Scalar(30, 30, 30)); // 暗いグレー背景
    
    // フォント設定
    int font = cv::FONT_HERSHEY_SIMPLEX;
    cv::Scalar white_color(255, 255, 255);
    cv::Scalar green_color(0, 255, 0);
    cv::Scalar yellow_color(0, 255, 255);
    
    // タイトル
    std::string title = "RTMP Bridge Server";
    double title_scale = 1.5;
    int title_thickness = 3;
    cv::Size title_size = cv::getTextSize(title, font, title_scale, title_thickness, nullptr);
    cv::Point title_pos((width - title_size.width) / 2, 100);
    cv::putText(info_img, title, title_pos, font, title_scale, white_color, title_thickness);
    
    // ステータス
    std::string status_text = "Status: " + status;
    double status_scale = 1.0;
    int status_thickness = 2;
    cv::Size status_size = cv::getTextSize(status_text, font, status_scale, status_thickness, nullptr);
    cv::Point status_pos((width - status_size.width) / 2, 180);
    cv::putText(info_img, status_text, status_pos, font, status_scale, yellow_color, status_thickness);
    
    // メイン接続情報
    std::string rtmp_url = "rtmp://" + ip_address + ":" + std::to_string(server_port_) + server_endpoint_;
    double url_scale = 1.2;
    int url_thickness = 2;
    cv::Size url_size = cv::getTextSize(rtmp_url, font, url_scale, url_thickness, nullptr);
    cv::Point url_pos((width - url_size.width) / 2, height / 2 - 20);
    cv::putText(info_img, rtmp_url, url_pos, font, url_scale, green_color, url_thickness);
    
    // 接続メッセージ
    std::string connect_msg = "Connect to the above URL with DJI Fly";
    double msg_scale = 0.8;
    int msg_thickness = 2;
    cv::Size msg_size = cv::getTextSize(connect_msg, font, msg_scale, msg_thickness, nullptr);
    cv::Point msg_pos((width - msg_size.width) / 2, height / 2 + 40);
    cv::putText(info_img, connect_msg, msg_pos, font, msg_scale, white_color, msg_thickness);
    
    // エンドポイント情報のみ
    std::string endpoint_info = "Endpoint: " + server_endpoint_;
    double endpoint_scale = 0.6;
    int endpoint_thickness = 1;
    cv::Size endpoint_size = cv::getTextSize(endpoint_info, font, endpoint_scale, endpoint_thickness, nullptr);
    cv::Point endpoint_pos((width - endpoint_size.width) / 2, height / 2 + 100);
    cv::putText(info_img, endpoint_info, endpoint_pos, font, endpoint_scale, white_color, endpoint_thickness);
    
    // フレーム統計（接続後）
    if (client_connected_) {
        std::string frame_stats = "Frames: " + std::to_string(frames_received_.load()) + " received, " + 
                                 std::to_string(frames_published_.load()) + " published";
        double stats_scale = 0.6;
        int stats_thickness = 1;
        cv::Size stats_size = cv::getTextSize(frame_stats, font, stats_scale, stats_thickness, nullptr);
        cv::Point stats_pos((width - stats_size.width) / 2, height - 50);
        cv::putText(info_img, frame_stats, stats_pos, font, stats_scale, yellow_color, stats_thickness);
    }
    
    // ROS2 Image メッセージに変換
    auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
    image_msg->header.stamp = this->get_clock()->now();
    image_msg->header.frame_id = frame_id_;
    image_msg->height = height;
    image_msg->width = width;
    image_msg->encoding = "bgr8";
    image_msg->is_bigendian = false;
    image_msg->step = width * 3;
    
    // データをコピー
    size_t data_size = image_msg->step * image_msg->height;
    image_msg->data.resize(data_size);
    std::memcpy(image_msg->data.data(), info_img.data, data_size);
    
    // Initialize image transport on first use if not already done
    if (!image_transport_) {
        image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        image_transport_publisher_ = image_transport_->advertise(output_topic_name_, buffer_size_);
        
        RCLCPP_INFO(this->get_logger(), "Image topics initialized:");
        RCLCPP_INFO(this->get_logger(), "  Raw: %s", output_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Compressed: %s/compressed", output_topic_name_.c_str());
    }
    
    // パブリッシュ (raw + compressed automatically)
    image_transport_publisher_.publish(*image_msg);
}

void RTMPBridgeNode::startInfoImageTimer()
{
    // カメラ情報のみを配信（情報画像は配信しない）
    info_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            // 情報画像の配信を停止
            // if (!client_connected_) {
            //     publishInfoImage("Waiting for connection...", local_ip_address_);
            // }
            // カメラ情報のみ配信
            publishCameraInfo();
        }
    );
}

bool RTMPBridgeNode::loadCameraInfo()
{
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("fv_rtmp_server") + "/";
    
    // Load O4 Air Unit camera info
    std::string o4_config_path = package_share_dir + "config/camera_info_o4.yaml";
    if (!loadSingleCameraInfo(o4_config_path, camera_info_o4_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load O4 camera info from: %s", o4_config_path.c_str());
        return false;
    }
    
    // Load O4 Air Unit Pro camera info
    std::string o4_pro_config_path = package_share_dir + "config/camera_info_o4_pro.yaml";
    if (!loadSingleCameraInfo(o4_pro_config_path, camera_info_o4_pro_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load O4 Pro camera info from: %s", o4_pro_config_path.c_str());
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Successfully loaded camera calibration for both O4 variants");
    return true;
}

bool RTMPBridgeNode::loadSingleCameraInfo(const std::string& file_path, sensor_msgs::msg::CameraInfo& camera_info)
{
    try {
        YAML::Node config = YAML::LoadFile(file_path);
        
        // Basic info
        camera_info.width = config["image_width"].as<uint32_t>();
        camera_info.height = config["image_height"].as<uint32_t>();
        camera_info.distortion_model = config["distortion_model"].as<std::string>();
        
        // Camera matrix
        auto camera_matrix = config["camera_matrix"]["data"];
        for (size_t i = 0; i < 9; ++i) {
            camera_info.k[i] = camera_matrix[i].as<double>();
        }
        
        // Distortion coefficients
        auto distortion = config["distortion_coefficients"]["data"];
        camera_info.d.resize(distortion.size());
        for (size_t i = 0; i < distortion.size(); ++i) {
            camera_info.d[i] = distortion[i].as<double>();
        }
        
        // Rectification matrix
        auto rectification = config["rectification_matrix"]["data"];
        for (size_t i = 0; i < 9; ++i) {
            camera_info.r[i] = rectification[i].as<double>();
        }
        
        // Projection matrix
        auto projection = config["projection_matrix"]["data"];
        for (size_t i = 0; i < 12; ++i) {
            camera_info.p[i] = projection[i].as<double>();
        }
        
        // Header info
        camera_info.header.frame_id = frame_id_;
        
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading camera info: %s", e.what());
        return false;
    }
}

void RTMPBridgeNode::publishCameraInfo()
{
    auto current_time = this->get_clock()->now();
    
    // Update timestamps
    camera_info_o4_.header.stamp = current_time;
    camera_info_o4_pro_.header.stamp = current_time;
    
    // Publish both camera info messages
    camera_info_o4_publisher_->publish(camera_info_o4_);
    camera_info_o4_pro_publisher_->publish(camera_info_o4_pro_);
}

} // namespace fv_rtmp_server

// Main function
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<fv_rtmp_server::RTMPBridgeNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    rclcpp::shutdown();
    return 0;
} 