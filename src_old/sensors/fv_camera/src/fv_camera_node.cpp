#include "fv_camera/fv_camera_node.hpp"
#include "fv_camera/srv/get_camera_info.hpp"
#include "fv_camera/srv/set_camera_settings.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <thread>

FVUSBCameraNode::FVUSBCameraNode()
    : Node("fv_camera"), running_(false)
{
    RCLCPP_INFO(this->get_logger(), "🚀 FV Camera starting...");
    
    try {
        // Step 1: Load parameters
        RCLCPP_INFO(this->get_logger(), "📋 Step 1: Loading parameters...");
        loadParameters();
        
        // Step 2: Initialize camera
        RCLCPP_INFO(this->get_logger(), "📷 Step 2: Initializing camera...");
        if (!initializeCamera()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to initialize camera");
            return;
        }
        
        // Step 3: Initialize publishers
        RCLCPP_INFO(this->get_logger(), "📤 Step 3: Initializing publishers...");
        initializePublishers();
        
        // Step 4: Initialize services
        RCLCPP_INFO(this->get_logger(), "🔧 Step 4: Initializing services...");
        initializeServices();
        
        // Step 5: Initialize TF
        RCLCPP_INFO(this->get_logger(), "🔄 Step 5: Initializing TF...");
        initializeTF();
        

        
        // Step 6: Start processing thread
        RCLCPP_INFO(this->get_logger(), "🔄 Step 6: Starting processing thread...");
        running_ = true;
        processing_thread_ = std::thread(&FVUSBCameraNode::processingLoop, this);
        
        RCLCPP_INFO(this->get_logger(), "✅ FV Camera started successfully");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Exception during initialization: %s", e.what());
    }
}

FVUSBCameraNode::~FVUSBCameraNode()
{
    RCLCPP_INFO(this->get_logger(), "🛑 Shutting down FV Camera...");
    
    running_ = false;
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
    
    if (camera_.isOpened()) {
        camera_.release();
    }
}

void FVUSBCameraNode::loadParameters()
{
    RCLCPP_INFO(this->get_logger(), "📋 Loading parameters...");
    
    // Camera selection
    camera_selection_config_.selection_method = 
        this->declare_parameter("camera_selection.selection_method", "auto");
    camera_selection_config_.device_name = 
        this->declare_parameter("camera_selection.device_name", "");
    camera_selection_config_.device_index = 
        this->declare_parameter("camera_selection.device_index", 0);
    
    // Power management settings
    power_management_config_.startup_delay = 
        this->declare_parameter("power_management.startup_delay", 1.0);
    
    // Camera settings
    camera_config_.width = 
        this->declare_parameter("camera.width", 640);
    camera_config_.height = 
        this->declare_parameter("camera.height", 480);
    camera_config_.fps = 
        this->declare_parameter("camera.fps", 30);
    camera_config_.brightness = 
        this->declare_parameter("camera.brightness", -1);
    camera_config_.contrast = 
        this->declare_parameter("camera.contrast", -1);
    camera_config_.saturation = 
        this->declare_parameter("camera.saturation", -1);
    camera_config_.hue = 
        this->declare_parameter("camera.hue", -1);
    camera_config_.gain = 
        this->declare_parameter("camera.gain", -1);
    camera_config_.exposure = 
        this->declare_parameter("camera.exposure", -1);
    
    // Stream settings
    stream_config_.color_enabled = 
        this->declare_parameter("streams.color_enabled", true);
    stream_config_.compressed_enabled = 
        this->declare_parameter("streams.compressed_enabled", true);
    
    // Camera info and compression
    camera_info_config_.enable_camera_info = 
        this->declare_parameter("camera_info.enable_camera_info", true);
    camera_info_config_.enable_compressed_topics = 
        this->declare_parameter("camera_info.enable_compressed_topics", true);
    camera_info_config_.compressed_quality = 
        this->declare_parameter("camera_info.compressed_quality", 85);
    
    // Services
    services_config_.get_camera_info_enabled = 
        this->declare_parameter("services.get_camera_info_enabled", true);
    services_config_.set_camera_settings_enabled = 
        this->declare_parameter("services.set_camera_settings_enabled", true);
    
    // TF settings
    tf_config_.enabled = 
        this->declare_parameter("tf.enabled", true);
    tf_config_.base_frame = 
        this->declare_parameter("tf.base_frame", "base_link");
    tf_config_.camera_frame = 
        this->declare_parameter("tf.camera_frame", "camera_link");
    tf_config_.optical_frame = 
        this->declare_parameter("tf.optical_frame", "camera_optical_frame");
    
    // Topics
    topic_config_.color = 
        this->declare_parameter("topics.color", "image_raw");
    topic_config_.color_compressed = 
        this->declare_parameter("topics.color_compressed", "image_raw/compressed");
    
    RCLCPP_INFO(this->get_logger(), "📋 Parameters loaded successfully");
}

bool FVUSBCameraNode::initializeCamera()
{
    RCLCPP_INFO(this->get_logger(), "📷 Initializing camera...");
    
    // Startup delay
    if (power_management_config_.startup_delay > 0) {
        RCLCPP_INFO(this->get_logger(), "⏳ Waiting %.1f seconds for camera startup...", 
                   power_management_config_.startup_delay);
        std::this_thread::sleep_for(std::chrono::duration<double>(power_management_config_.startup_delay));
    }
    
    // Select camera
    if (!selectCamera()) {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to select camera");
        return false;
    }
    
    // Set camera properties
    if (camera_config_.width > 0 && camera_config_.height > 0) {
        camera_.set(cv::CAP_PROP_FRAME_WIDTH, camera_config_.width);
        camera_.set(cv::CAP_PROP_FRAME_HEIGHT, camera_config_.height);
    }
    
    if (camera_config_.fps > 0) {
        camera_.set(cv::CAP_PROP_FPS, camera_config_.fps);
    }
    
    // Set additional properties if specified
    if (camera_config_.brightness >= 0) {
        setCameraProperty(cv::CAP_PROP_BRIGHTNESS, camera_config_.brightness);
    }
    if (camera_config_.contrast >= 0) {
        setCameraProperty(cv::CAP_PROP_CONTRAST, camera_config_.contrast);
    }
    if (camera_config_.saturation >= 0) {
        setCameraProperty(cv::CAP_PROP_SATURATION, camera_config_.saturation);
    }
    if (camera_config_.hue >= 0) {
        setCameraProperty(cv::CAP_PROP_HUE, camera_config_.hue);
    }
    if (camera_config_.gain >= 0) {
        setCameraProperty(cv::CAP_PROP_GAIN, camera_config_.gain);
    }
    if (camera_config_.exposure >= 0) {
        setCameraProperty(cv::CAP_PROP_EXPOSURE, camera_config_.exposure);
    }
    
    // Update camera info
    updateCameraInfo();
    
    RCLCPP_INFO(this->get_logger(), "✅ Camera initialized successfully");
    return true;
}

bool FVUSBCameraNode::selectCamera()
{
    RCLCPP_INFO(this->get_logger(), "🔍 Selecting camera...");
    
    if (camera_selection_config_.selection_method == "auto") {
        // Try to open camera 0
        RCLCPP_INFO(this->get_logger(), "🔄 Auto-selecting camera 0...");
        camera_.open(0);
        if (camera_.isOpened()) {
            RCLCPP_INFO(this->get_logger(), "✅ Auto-selected camera 0");
            return true;
        }
        
        // Try other cameras
        for (int i = 1; i < 10; i++) {
            RCLCPP_INFO(this->get_logger(), "🔄 Trying camera %d...", i);
            camera_.open(i);
            if (camera_.isOpened()) {
                RCLCPP_INFO(this->get_logger(), "✅ Auto-selected camera %d", i);
                return true;
            }
        }
        
        RCLCPP_ERROR(this->get_logger(), "❌ No camera found in auto mode");
        return false;
        
    } else if (camera_selection_config_.selection_method == "index") {
        RCLCPP_INFO(this->get_logger(), "🔄 Opening camera index %d...", camera_selection_config_.device_index);
        camera_.open(camera_selection_config_.device_index);
        if (camera_.isOpened()) {
            RCLCPP_INFO(this->get_logger(), "✅ Camera index %d opened successfully", camera_selection_config_.device_index);
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to open camera index %d", camera_selection_config_.device_index);
            return false;
        }
    }
    
    RCLCPP_ERROR(this->get_logger(), "❌ Unknown camera selection method: %s", 
                camera_selection_config_.selection_method.c_str());
    return false;
}

void FVUSBCameraNode::initializePublishers()
{
    RCLCPP_INFO(this->get_logger(), "📤 Initializing publishers...");
    
    // Color image publisher
    if (stream_config_.color_enabled) {
        color_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            topic_config_.color, 10);
        RCLCPP_INFO(this->get_logger(), "📤 Color publisher: %s", topic_config_.color.c_str());
    }
    
    // Camera info publisher
    if (camera_info_config_.enable_camera_info) {
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "camera_info", 10);
        RCLCPP_INFO(this->get_logger(), "📤 Camera info publisher: camera_info");
    }
    
    // Compressed image publisher (will be initialized later)
    if (camera_info_config_.enable_compressed_topics && stream_config_.compressed_enabled) {
        RCLCPP_INFO(this->get_logger(), "📤 Compressed publisher: %s (will be initialized later)", topic_config_.color_compressed.c_str());
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ Publishers initialized successfully");
}

void FVUSBCameraNode::initializeServices()
{
    RCLCPP_INFO(this->get_logger(), "🔧 Initializing services...");
    
    // Get camera info service
    if (services_config_.get_camera_info_enabled) {
        get_camera_info_service_ = this->create_service<fv_camera::srv::GetCameraInfo>(
            "get_camera_info",
            std::bind(&FVUSBCameraNode::handleGetCameraInfo, this, 
                     std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "🔧 Get camera info service: get_camera_info");
    }
    
    // Set camera settings service
    if (services_config_.set_camera_settings_enabled) {
        set_camera_settings_service_ = this->create_service<fv_camera::srv::SetCameraSettings>(
            "set_camera_settings",
            std::bind(&FVUSBCameraNode::handleSetCameraSettings, this, 
                     std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "🔧 Set camera settings service: set_camera_settings");
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ Services initialized successfully");
}

void FVUSBCameraNode::initializeTF()
{
    if (!tf_config_.enabled) {
        RCLCPP_INFO(this->get_logger(), "🔄 TF disabled");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "🔄 Initializing TF...");
    
    tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
    RCLCPP_INFO(this->get_logger(), "✅ TF initialized successfully");
}

void FVUSBCameraNode::processingLoop()
{
    RCLCPP_INFO(this->get_logger(), "🔄 Processing loop started");
    
    cv::Mat frame;
    auto last_time = std::chrono::steady_clock::now();
    int frame_count = 0;
    
    while (running_) {
        if (!camera_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Camera not opened");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }
        
        // Capture frame
        if (!camera_.read(frame)) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Failed to read frame");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Empty frame received");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        // Update frame
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            current_frame_ = frame.clone();
        }
        
        // Publish frame
        publishFrame(frame);
        
        // Publish TF
        if (tf_config_.enabled) {
            publishTF();
        }
        
        // Frame rate logging
        frame_count++;
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_time);
        
        if (elapsed.count() >= 10) {
            double fps = frame_count / elapsed.count();
            RCLCPP_INFO(this->get_logger(), "📊 Frame rate: %.1f FPS", fps);
            frame_count = 0;
            last_time = current_time;
        }
        
        // Sleep to maintain frame rate
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / camera_config_.fps));
    }
    
    RCLCPP_INFO(this->get_logger(), "🛑 Processing loop stopped");
}

void FVUSBCameraNode::publishFrame(const cv::Mat& frame)
{
    auto stamp = this->now();
    
    // Publish color image
    if (stream_config_.color_enabled && color_pub_) {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = stamp;
        msg->header.frame_id = tf_config_.optical_frame;
        color_pub_->publish(*msg);
    }
    
    // Publish compressed image (if initialized)
    if (camera_info_config_.enable_compressed_topics && stream_config_.compressed_enabled && image_transport_) {
        if (color_compressed_pub_.getNumSubscribers() > 0) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            msg->header.stamp = stamp;
            msg->header.frame_id = tf_config_.optical_frame;
            color_compressed_pub_.publish(msg);
        }
    }
    
    // Publish camera info
    if (camera_info_config_.enable_camera_info && camera_info_pub_) {
        camera_info_msg_.header.stamp = stamp;
        camera_info_msg_.header.frame_id = tf_config_.optical_frame;
        camera_info_pub_->publish(camera_info_msg_);
    }
}

void FVUSBCameraNode::publishTF()
{
    auto stamp = this->now();
    
    // Static transform from base_link to camera_link
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = tf_config_.base_frame;
    transform_stamped.child_frame_id = tf_config_.camera_frame;
    
    // Identity transform (camera at origin)
    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;
    
    tf_static_broadcaster_->sendTransform(transform_stamped);
    
    // Transform from camera_link to optical frame
    geometry_msgs::msg::TransformStamped optical_transform;
    optical_transform.header.stamp = stamp;
    optical_transform.header.frame_id = tf_config_.camera_frame;
    optical_transform.child_frame_id = tf_config_.optical_frame;
    
    // Rotate 180 degrees around Z axis (OpenCV to ROS convention)
    optical_transform.transform.translation.x = 0.0;
    optical_transform.transform.translation.y = 0.0;
    optical_transform.transform.translation.z = 0.0;
    optical_transform.transform.rotation.x = 0.0;
    optical_transform.transform.rotation.y = 0.0;
    optical_transform.transform.rotation.z = 1.0;  // 180 degrees around Z
    optical_transform.transform.rotation.w = 0.0;
    
    tf_static_broadcaster_->sendTransform(optical_transform);
}

void FVUSBCameraNode::updateCameraInfo()
{
    camera_info_msg_.header.frame_id = tf_config_.optical_frame;
    camera_info_msg_.width = camera_.get(cv::CAP_PROP_FRAME_WIDTH);
    camera_info_msg_.height = camera_.get(cv::CAP_PROP_FRAME_HEIGHT);
    
    // Set default camera matrix (identity)
    camera_info_msg_.k[0] = 1000.0;  // fx
    camera_info_msg_.k[4] = 1000.0;  // fy
    camera_info_msg_.k[2] = camera_info_msg_.width / 2.0;   // cx
    camera_info_msg_.k[5] = camera_info_msg_.height / 2.0;  // cy
    camera_info_msg_.k[1] = 0.0;
    camera_info_msg_.k[3] = 0.0;
    camera_info_msg_.k[6] = 0.0;
    camera_info_msg_.k[7] = 0.0;
    camera_info_msg_.k[8] = 1.0;
    
    // Set distortion coefficients (no distortion)
    camera_info_msg_.d.resize(5);
    camera_info_msg_.d[0] = 0.0;  // k1
    camera_info_msg_.d[1] = 0.0;  // k2
    camera_info_msg_.d[2] = 0.0;  // p1
    camera_info_msg_.d[3] = 0.0;  // p2
    camera_info_msg_.d[4] = 0.0;  // k3
    
    // Set projection matrix
    camera_info_msg_.p[0] = camera_info_msg_.k[0];
    camera_info_msg_.p[5] = camera_info_msg_.k[4];
    camera_info_msg_.p[2] = camera_info_msg_.k[2];
    camera_info_msg_.p[6] = camera_info_msg_.k[5];
    camera_info_msg_.p[1] = 0.0;
    camera_info_msg_.p[3] = 0.0;
    camera_info_msg_.p[4] = 0.0;
    camera_info_msg_.p[7] = 0.0;
    camera_info_msg_.p[8] = 0.0;
    camera_info_msg_.p[9] = 0.0;
    camera_info_msg_.p[10] = 1.0;
    camera_info_msg_.p[11] = 0.0;
    
    RCLCPP_INFO(this->get_logger(), "📷 Camera info updated: %dx%d", 
               camera_info_msg_.width, camera_info_msg_.height);
}

void FVUSBCameraNode::handleGetCameraInfo(
    const std::shared_ptr<fv_camera::srv::GetCameraInfo::Request> request,
    std::shared_ptr<fv_camera::srv::GetCameraInfo::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "🔧 Get camera info service called");
    
    if (!camera_.isOpened()) {
        response->success = false;
        response->message = "Camera not opened";
        return;
    }
    
    response->success = true;
    response->message = "Camera info retrieved successfully";
    response->width = camera_.get(cv::CAP_PROP_FRAME_WIDTH);
    response->height = camera_.get(cv::CAP_PROP_FRAME_HEIGHT);
    response->fps = camera_.get(cv::CAP_PROP_FPS);
    response->format = "BGR8";
    response->device_name = "USB Camera";
    response->device_index = camera_selection_config_.device_index;
    
    RCLCPP_INFO(this->get_logger(), "📷 Camera info: %dx%d @ %d FPS", 
               response->width, response->height, response->fps);
}

void FVUSBCameraNode::handleSetCameraSettings(
    const std::shared_ptr<fv_camera::srv::SetCameraSettings::Request> request,
    std::shared_ptr<fv_camera::srv::SetCameraSettings::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "🔧 Set camera settings service called");
    
    if (!camera_.isOpened()) {
        response->success = false;
        response->message = "Camera not opened";
        return;
    }
    
    bool success = true;
    std::string message = "Settings applied successfully";
    
    // Apply settings
    if (request->width > 0 && request->height > 0) {
        if (!setCameraProperty(cv::CAP_PROP_FRAME_WIDTH, request->width) ||
            !setCameraProperty(cv::CAP_PROP_FRAME_HEIGHT, request->height)) {
            success = false;
            message = "Failed to set resolution";
        }
    }
    
    if (request->fps > 0) {
        if (!setCameraProperty(cv::CAP_PROP_FPS, request->fps)) {
            success = false;
            message = "Failed to set FPS";
        }
    }
    
    if (request->brightness >= 0) {
        if (!setCameraProperty(cv::CAP_PROP_BRIGHTNESS, request->brightness)) {
            success = false;
            message = "Failed to set brightness";
        }
    }
    
    if (request->contrast >= 0) {
        if (!setCameraProperty(cv::CAP_PROP_CONTRAST, request->contrast)) {
            success = false;
            message = "Failed to set contrast";
        }
    }
    
    if (request->saturation >= 0) {
        if (!setCameraProperty(cv::CAP_PROP_SATURATION, request->saturation)) {
            success = false;
            message = "Failed to set saturation";
        }
    }
    
    if (request->hue >= 0) {
        if (!setCameraProperty(cv::CAP_PROP_HUE, request->hue)) {
            success = false;
            message = "Failed to set hue";
        }
    }
    
    if (request->gain >= 0) {
        if (!setCameraProperty(cv::CAP_PROP_GAIN, request->gain)) {
            success = false;
            message = "Failed to set gain";
        }
    }
    
    if (request->exposure >= 0) {
        if (!setCameraProperty(cv::CAP_PROP_EXPOSURE, request->exposure)) {
            success = false;
            message = "Failed to set exposure";
        }
    }
    
    response->success = success;
    response->message = message;
    
    if (success) {
        updateCameraInfo();
        RCLCPP_INFO(this->get_logger(), "✅ Camera settings applied successfully");
    } else {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to apply camera settings: %s", message.c_str());
    }
}

bool FVUSBCameraNode::setCameraProperty(int property, double value)
{
    if (!camera_.isOpened()) {
        return false;
    }
    
    bool result = camera_.set(property, value);
    if (result) {
        double actual_value = camera_.get(property);
        RCLCPP_DEBUG(this->get_logger(), "📷 Set %s: %.1f (actual: %.1f)", 
                    getCameraPropertyName(property).c_str(), value, actual_value);
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ Failed to set %s to %.1f", 
                   getCameraPropertyName(property).c_str(), value);
    }
    
    return result;
}

double FVUSBCameraNode::getCameraProperty(int property)
{
    if (!camera_.isOpened()) {
        return -1.0;
    }
    
    return camera_.get(property);
}

std::string FVUSBCameraNode::getCameraPropertyName(int property)
{
    switch (property) {
        case cv::CAP_PROP_POS_MSEC: return "POS_MSEC";
        case cv::CAP_PROP_POS_FRAMES: return "POS_FRAMES";
        case cv::CAP_PROP_POS_AVI_RATIO: return "POS_AVI_RATIO";
        case cv::CAP_PROP_FRAME_WIDTH: return "FRAME_WIDTH";
        case cv::CAP_PROP_FRAME_HEIGHT: return "FRAME_HEIGHT";
        case cv::CAP_PROP_FPS: return "FPS";
        case cv::CAP_PROP_FOURCC: return "FOURCC";
        case cv::CAP_PROP_FRAME_COUNT: return "FRAME_COUNT";
        case cv::CAP_PROP_FORMAT: return "FORMAT";
        case cv::CAP_PROP_MODE: return "MODE";
        case cv::CAP_PROP_BRIGHTNESS: return "BRIGHTNESS";
        case cv::CAP_PROP_CONTRAST: return "CONTRAST";
        case cv::CAP_PROP_SATURATION: return "SATURATION";
        case cv::CAP_PROP_HUE: return "HUE";
        case cv::CAP_PROP_GAIN: return "GAIN";
        case cv::CAP_PROP_EXPOSURE: return "EXPOSURE";
        case cv::CAP_PROP_CONVERT_RGB: return "CONVERT_RGB";
        case cv::CAP_PROP_WHITE_BALANCE_BLUE_U: return "WHITE_BALANCE_BLUE_U";
        case cv::CAP_PROP_RECTIFICATION: return "RECTIFICATION";
        default: return "UNKNOWN";
    }
}

std::vector<cv::VideoCapture> FVUSBCameraNode::getAvailableCameras()
{
    std::vector<cv::VideoCapture> cameras;
    
    for (int i = 0; i < 10; i++) {
        cv::VideoCapture cap(i);
        if (cap.isOpened()) {
            cameras.push_back(cap);
            RCLCPP_INFO(this->get_logger(), "📷 Found camera %d", i);
        }
    }
    
    return cameras;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<FVUSBCameraNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
} 