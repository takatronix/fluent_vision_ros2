#include "fv_realsense/fv_realsense_node.hpp"
#include "fv_realsense/srv/get_distance.hpp"
#include "fv_realsense/srv/get_camera_info.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <thread>

FVDepthCameraNode::FVDepthCameraNode()
    : Node("fv_realsense"), running_(false)
{
    RCLCPP_INFO(this->get_logger(), "🚀 FV Depth Camera starting...");
    
    try {
        // Step 1: Load parameters
        RCLCPP_INFO(this->get_logger(), "📋 Step 1: Loading parameters...");
        loadParameters();
        
        // Step 2: Initialize RealSense first (before shared_from_this)
        RCLCPP_INFO(this->get_logger(), "📷 Step 2: Initializing RealSense...");
        if (!initializeRealSense()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to initialize RealSense");
            return;
        }
        
        // Step 3: Initialize publishers (after RealSense)
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
        processing_thread_ = std::thread(&FVDepthCameraNode::processingLoop, this);
        
        RCLCPP_INFO(this->get_logger(), "✅ FV Depth Camera started successfully");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Exception during initialization: %s", e.what());
    }
}

FVDepthCameraNode::~FVDepthCameraNode()
{
    RCLCPP_INFO(this->get_logger(), "🛑 Shutting down FV Depth Camera...");
    
    running_ = false;
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
}

void FVDepthCameraNode::loadParameters()
{
    RCLCPP_INFO(this->get_logger(), "📋 Loading parameters...");
    
    // Camera selection
    camera_selection_config_.selection_method = 
        this->declare_parameter("camera_selection.selection_method", "auto");
    camera_selection_config_.serial_number = 
        this->declare_parameter("camera_selection.serial_number", "");
    camera_selection_config_.device_name = 
        this->declare_parameter("camera_selection.device_name", "");
    camera_selection_config_.device_index = 
        this->declare_parameter("camera_selection.device_index", 0);
    
    // Power management settings
    power_management_config_.startup_delay = 
        this->declare_parameter("power_management.startup_delay", 2.0);
    
    // Camera settings
    camera_config_.color_width = 
        this->declare_parameter("camera.color_width", 640);
    camera_config_.color_height = 
        this->declare_parameter("camera.color_height", 480);
    camera_config_.color_fps = 
        this->declare_parameter("camera.color_fps", 30);
    camera_config_.depth_width = 
        this->declare_parameter("camera.depth_width", 640);
    camera_config_.depth_height = 
        this->declare_parameter("camera.depth_height", 480);
    camera_config_.depth_fps = 
        this->declare_parameter("camera.depth_fps", 30);
    
    // Stream settings
    stream_config_.color_enabled = 
        this->declare_parameter("streams.color_enabled", true);
    stream_config_.depth_enabled = 
        this->declare_parameter("streams.depth_enabled", true);
    stream_config_.infrared_enabled = 
        this->declare_parameter("streams.infrared_enabled", false);
    stream_config_.pointcloud_enabled = 
        this->declare_parameter("streams.pointcloud_enabled", true);
    stream_config_.depth_colormap_enabled = 
        this->declare_parameter("streams.depth_colormap_enabled", true);
    
    // Camera info settings
    camera_info_config_.enable_camera_info = 
        this->declare_parameter("camera_info.enable_camera_info", true);
    camera_info_config_.enable_compressed_topics = 
        this->declare_parameter("camera_info.enable_compressed_topics", true);
    camera_info_config_.compressed_quality = 
        this->declare_parameter("camera_info.compressed_quality", 85);
    camera_info_config_.enable_depth_compressed = 
        this->declare_parameter("camera_info.enable_depth_compressed", false);
    
    // Services settings
    services_config_.get_distance_enabled = 
        this->declare_parameter("services.get_distance_enabled", true);
    services_config_.get_camera_info_enabled = 
        this->declare_parameter("services.get_camera_info_enabled", true);
    
    // TF settings
    tf_config_.enabled = 
        this->declare_parameter("tf.enabled", true);
    tf_config_.base_frame = 
        this->declare_parameter("tf.base_frame", "base_link");
    tf_config_.camera_frame = 
        this->declare_parameter("tf.camera_frame", "camera_link");
    tf_config_.color_optical_frame = 
        this->declare_parameter("tf.color_optical_frame", "color_optical_frame");
    tf_config_.depth_optical_frame = 
        this->declare_parameter("tf.depth_optical_frame", "depth_optical_frame");
    
    // Topics
    topic_config_.color = 
        this->declare_parameter("topics.color", "color/image_raw");
    topic_config_.depth = 
        this->declare_parameter("topics.depth", "depth/image_rect_raw");
    topic_config_.color_compressed = 
        this->declare_parameter("topics.color_compressed", "color/image_raw/compressed");
    topic_config_.depth_colormap = 
        this->declare_parameter("topics.depth_colormap", "depth/colormap");
    topic_config_.pointcloud = 
        this->declare_parameter("topics.pointcloud", "depth/color/points");
    
    RCLCPP_INFO(this->get_logger(), "✅ Parameters loaded successfully");
}

bool FVDepthCameraNode::initializeRealSense()
{
    RCLCPP_INFO(this->get_logger(), "🔧 Initializing RealSense...");
    
    try {
        // Initialize context with error handling
        RCLCPP_INFO(this->get_logger(), "📋 Creating RealSense context...");
        ctx_ = rs2::context();
        
        // Simple device detection without complex querying
        RCLCPP_INFO(this->get_logger(), "🔍 Simple device detection...");
        try {
            auto device_list = ctx_.query_devices();
            if (device_list.size() == 0) {
                RCLCPP_ERROR(this->get_logger(), "❌ No RealSense devices found");
                return false;
            }
            
            // Use first device directly
            device_ = device_list[0];
            std::string name = device_.get_info(RS2_CAMERA_INFO_NAME);
            std::string serial = device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            RCLCPP_INFO(this->get_logger(), "📷 Using device: %s (SN: %s)", name.c_str(), serial.c_str());
            
        } catch (const rs2::error& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Error detecting devices: %s", e.what());
            return false;
        }
        
        // Configure pipeline with minimal settings
        if (stream_config_.color_enabled) {
            cfg_.enable_stream(RS2_STREAM_COLOR, 
                camera_config_.color_width, camera_config_.color_height, 
                RS2_FORMAT_BGR8, camera_config_.color_fps);
            RCLCPP_INFO(this->get_logger(), "📹 Enabled color stream: %dx%d @ %dfps", 
                camera_config_.color_width, camera_config_.color_height, camera_config_.color_fps);
        }
        
        // Enable depth stream if configured
        if (stream_config_.depth_enabled) {
            cfg_.enable_stream(RS2_STREAM_DEPTH,
                camera_config_.depth_width, camera_config_.depth_height,
                RS2_FORMAT_Z16, camera_config_.depth_fps);
            RCLCPP_INFO(this->get_logger(), "📏 Enabled depth stream: %dx%d @ %dfps",
                camera_config_.depth_width, camera_config_.depth_height, camera_config_.depth_fps);
        }
        
        // Apply startup delay for power management
        if (power_management_config_.startup_delay > 0.0) {
            RCLCPP_INFO(this->get_logger(), "⏳ Waiting %.1f seconds for power stabilization...", 
                power_management_config_.startup_delay);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(power_management_config_.startup_delay * 1000)));
        }
        
        // Start pipeline with timeout
        RCLCPP_INFO(this->get_logger(), "🚀 Starting RealSense pipeline...");
        profile_ = pipe_.start(cfg_);
        
        if (!profile_) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to start pipeline");
            return false;
        }
        
        // Wait for first frames to ensure pipeline is ready
        try {
            RCLCPP_INFO(this->get_logger(), "⏳ Waiting for initial frames...");
            auto initial_frames = pipe_.wait_for_frames(5000); // 5 second timeout
            auto color_frame = initial_frames.get_color_frame();
            auto depth_frame = initial_frames.get_depth_frame();
            
            if (color_frame) {
                RCLCPP_INFO(this->get_logger(), "✅ Got initial color frame: %dx%d", 
                    color_frame.get_width(), color_frame.get_height());
            }
            if (depth_frame) {
                RCLCPP_INFO(this->get_logger(), "✅ Got initial depth frame: %dx%d",
                    depth_frame.get_width(), depth_frame.get_height());
            }
        } catch (const rs2::error& e) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Could not get initial frames: %s", e.what());
        }
        
        device_ = profile_.get_device();
        if (!device_) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to get device from profile");
            return false;
        }
        
        // Get depth scale with error handling
        try {
            auto sensors = device_.query_sensors();
            for (auto sensor : sensors) {
                if (sensor.supports(RS2_OPTION_DEPTH_UNITS)) {
                    depth_scale_ = sensor.get_option(RS2_OPTION_DEPTH_UNITS);
                    break;
                }
            }
            RCLCPP_INFO(this->get_logger(), "📏 Depth scale: %f", depth_scale_);
        } catch (const rs2::error& e) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Could not get depth scale: %s", e.what());
            depth_scale_ = 0.001f; // Default value
        }
        
        // Get intrinsics with error handling
        try {
            if (stream_config_.color_enabled) {
                auto color_stream = profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
                color_intrinsics_ = color_stream.get_intrinsics();
                RCLCPP_INFO(this->get_logger(), "📹 Color intrinsics: %dx%d", 
                    color_intrinsics_.width, color_intrinsics_.height);
            }
            
            if (stream_config_.depth_enabled) {
                auto depth_stream = profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
                depth_intrinsics_ = depth_stream.get_intrinsics();
                RCLCPP_INFO(this->get_logger(), "📐 Depth intrinsics: %dx%d", 
                    depth_intrinsics_.width, depth_intrinsics_.height);
            }
        } catch (const rs2::error& e) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Could not get intrinsics: %s", e.what());
        }
        
        RCLCPP_INFO(this->get_logger(), "✅ RealSense initialized successfully");
        return true;
        
    } catch (const rs2::error& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ RealSense error: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Exception during RealSense initialization: %s", e.what());
        return false;
    }
}

bool FVDepthCameraNode::selectCamera()
{
    auto devices = getAvailableDevices();
    
    if (camera_selection_config_.selection_method == "serial" && 
        !camera_selection_config_.serial_number.empty()) {
        
        for (const auto& device : devices) {
            std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            if (serial == camera_selection_config_.serial_number) {
                device_ = device;
                RCLCPP_INFO(this->get_logger(), "📷 Selected camera by serial: %s", serial.c_str());
                return true;
            }
        }
        RCLCPP_ERROR(this->get_logger(), "❌ Camera with serial %s not found", 
            camera_selection_config_.serial_number.c_str());
        return false;
        
    } else if (camera_selection_config_.selection_method == "name" && 
               !camera_selection_config_.device_name.empty()) {
        
        for (const auto& device : devices) {
            std::string name = device.get_info(RS2_CAMERA_INFO_NAME);
            if (name.find(camera_selection_config_.device_name) != std::string::npos) {
                device_ = device;
                RCLCPP_INFO(this->get_logger(), "📷 Selected camera by name: %s", name.c_str());
                return true;
            }
        }
        RCLCPP_ERROR(this->get_logger(), "❌ Camera with name %s not found", 
            camera_selection_config_.device_name.c_str());
        return false;
        
    } else if (camera_selection_config_.selection_method == "index") {
        
        if (camera_selection_config_.device_index < static_cast<int>(devices.size())) {
            device_ = devices[camera_selection_config_.device_index];
            std::string name = device_.get_info(RS2_CAMERA_INFO_NAME);
            RCLCPP_INFO(this->get_logger(), "📷 Selected camera by index %d: %s", 
                camera_selection_config_.device_index, name.c_str());
            return true;
        }
        RCLCPP_ERROR(this->get_logger(), "❌ Camera index %d out of range", 
            camera_selection_config_.device_index);
        return false;
        
    } else {
        // Auto-select first available
        device_ = devices[0];
        std::string name = device_.get_info(RS2_CAMERA_INFO_NAME);
        std::string serial = device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        RCLCPP_INFO(this->get_logger(), "📷 Auto-selected camera: %s (Serial: %s)", 
            name.c_str(), serial.c_str());
        return true;
    }
}

void FVDepthCameraNode::initializePublishers()
{
    RCLCPP_INFO(this->get_logger(), "📤 Initializing publishers...");
    
    // Create image transport - use shared_ptr (delayed initialization)
    // image_transport_ = std::make_unique<image_transport::ImageTransport>(this->shared_from_this());
    
    // Basic publishers
    if (stream_config_.color_enabled) {
        color_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            topic_config_.color, 10);
        RCLCPP_INFO(this->get_logger(), "📷 Color publisher created: %s, ptr: %p", 
            topic_config_.color.c_str(), color_pub_.get());
        
        // if (camera_info_config_.enable_compressed_topics) {
        //     color_compressed_pub_ = image_transport_->advertise(
        //         topic_config_.color_compressed, 10);
        // }
    }
    
    if (stream_config_.depth_enabled) {
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            topic_config_.depth, 10);
    }
    
    if (stream_config_.depth_colormap_enabled) {
        depth_colormap_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            topic_config_.depth_colormap, 10);
    }
    
    if (stream_config_.pointcloud_enabled) {
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            topic_config_.pointcloud, 10);
    }
    
    // Camera info publishers
    if (camera_info_config_.enable_camera_info) {
        color_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "color/camera_info", 10);
        depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "depth/camera_info", 10);
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ Publishers initialized");
}

void FVDepthCameraNode::initializeServices()
{
    RCLCPP_INFO(this->get_logger(), "🔧 Initializing services...");
    
    if (services_config_.get_distance_enabled) {
        get_distance_service_ = this->create_service<fv_realsense::srv::GetDistance>(
            "get_distance",
            std::bind(&FVDepthCameraNode::handleGetDistance, this, 
                std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "📏 GetDistance service initialized");
    }
    
    if (services_config_.get_camera_info_enabled) {
        get_camera_info_service_ = this->create_service<fv_realsense::srv::GetCameraInfo>(
            "get_camera_info",
            std::bind(&FVDepthCameraNode::handleGetCameraInfo, this, 
                std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "📋 GetCameraInfo service initialized");
    }
}

void FVDepthCameraNode::initializeTF()
{
    if (!tf_config_.enabled) {
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "🔄 Initializing TF...");
    
    tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
    publishTF();
    
    RCLCPP_INFO(this->get_logger(), "✅ TF initialized");
}

void FVDepthCameraNode::processingLoop()
{
    RCLCPP_INFO(this->get_logger(), "🔄 Starting processing loop...");
    
    int frame_count = 0;
    auto last_log_time = std::chrono::steady_clock::now();
    
    while (running_ && rclcpp::ok()) {
        try {
            // Wait for frames with timeout
            rs2::frameset frames = pipe_.wait_for_frames(1000);  // 1 second timeout
            
            auto color_frame = frames.get_color_frame();
            auto depth_frame = frames.get_depth_frame();
            
            // Process frames if at least one is available
            if (color_frame || depth_frame) {
                frame_count++;
                publishFrames(color_frame, depth_frame);
                
                // Log every second
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 1) {
                    RCLCPP_INFO(this->get_logger(), "📊 Processing: %d frames, Color: %s, Depth: %s", 
                        frame_count, 
                        color_frame ? "✅" : "❌",
                        depth_frame ? "✅" : "❌");
                    frame_count = 0;
                    last_log_time = now;
                }
                
                if (stream_config_.pointcloud_enabled) {
                    publishPointCloud(color_frame, depth_frame);
                }
            }
            
        } catch (const rs2::error& e) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Frame processing error: %s", e.what());
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "🛑 Processing loop stopped");
}

void FVDepthCameraNode::publishFrames(const rs2::frame& color_frame, const rs2::frame& depth_frame)
{
    auto now = this->now();
    static int publish_count = 0;
    static auto last_publish_log = std::chrono::steady_clock::now();
    
    // Publish color frame
    if (stream_config_.color_enabled && color_frame && color_pub_) {
        cv::Mat color_image(cv::Size(color_intrinsics_.width, color_intrinsics_.height), 
                           CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        
        auto color_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color_image).toImageMsg();
        color_msg->header.stamp = now;
        color_msg->header.frame_id = tf_config_.color_optical_frame;
        color_pub_->publish(*color_msg);
        publish_count++;
        
        // Debug: Check if actually published
        static int debug_count = 0;
        if (++debug_count % 30 == 0) {  // Log every 30 frames (1 second)
            RCLCPP_INFO(this->get_logger(), "🔍 Published color image to topic: %s", 
                topic_config_.color.c_str());
        }
        
        // Publish compressed color (disabled for now)
        // if (camera_info_config_.enable_compressed_topics && color_compressed_pub_.getNumSubscribers() > 0) {
        //     color_compressed_pub_.publish(color_msg);
        // }
    }
    
    // Publish depth frame
    if (stream_config_.depth_enabled && depth_frame && depth_pub_) {
        cv::Mat depth_image(cv::Size(depth_intrinsics_.width, depth_intrinsics_.height), 
                           CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        
        auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", depth_image).toImageMsg();
        depth_msg->header.stamp = now;
        depth_msg->header.frame_id = tf_config_.depth_optical_frame;
        depth_pub_->publish(*depth_msg);
    }
    
    // Publish depth colormap
    if (stream_config_.depth_colormap_enabled && depth_frame && depth_colormap_pub_) {
        cv::Mat colormap = createDepthColormap(depth_frame);
        auto colormap_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", colormap).toImageMsg();
        colormap_msg->header.stamp = now;
        colormap_msg->header.frame_id = tf_config_.depth_optical_frame;
        depth_colormap_pub_->publish(*colormap_msg);
    }
    
    // Log publishing status
    auto current_time = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(current_time - last_publish_log).count() >= 1) {
        RCLCPP_INFO(this->get_logger(), "📤 Published %d frames in last second", publish_count);
        publish_count = 0;
        last_publish_log = current_time;
    }
}

void FVDepthCameraNode::publishPointCloud(const rs2::frame& color_frame, const rs2::frame& depth_frame)
{
    // Point cloud requires both color and depth frames
    if (!color_frame || !depth_frame) {
        return;
    }
    
    // Create point cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    
    // Get color and depth data
    cv::Mat color_image(cv::Size(color_intrinsics_.width, color_intrinsics_.height), 
                       CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat depth_image(cv::Size(depth_intrinsics_.width, depth_intrinsics_.height), 
                       CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    
    // Convert to point cloud
    for (int y = 0; y < depth_intrinsics_.height; y += 2) {
        for (int x = 0; x < depth_intrinsics_.width; x += 2) {
            float depth = depth_image.at<uint16_t>(y, x) * depth_scale_;
            
            if (depth > 0.1f && depth < 10.0f) {
                float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
                float point[3];
                
                rs2_deproject_pixel_to_point(point, &color_intrinsics_, pixel, depth);
                
                pcl::PointXYZRGB pcl_point;
                pcl_point.x = point[0];
                pcl_point.y = point[1];
                pcl_point.z = point[2];
                
                // Get color
                cv::Vec3b color = color_image.at<cv::Vec3b>(y, x);
                pcl_point.r = color[2];
                pcl_point.g = color[1];
                pcl_point.b = color[0];
                
                cloud.points.push_back(pcl_point);
            }
        }
    }
    
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;
    
    // Publish
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = tf_config_.color_optical_frame;
    pointcloud_pub_->publish(cloud_msg);
}

cv::Mat FVDepthCameraNode::createDepthColormap(const rs2::frame& depth_frame)
{
    cv::Mat depth_image(cv::Size(depth_intrinsics_.width, depth_intrinsics_.height), 
                       CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    
    cv::Mat colormap;
    depth_image.convertTo(colormap, CV_8UC1, 255.0 / 65535.0);
    cv::applyColorMap(colormap, colormap, cv::COLORMAP_JET);
    
    return colormap;
}

void FVDepthCameraNode::publishTF()
{
    if (!tf_config_.enabled) {
        return;
    }
    
    auto now = this->now();
    
    // Static transform from base_link to camera_link
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now;
    transform.header.frame_id = tf_config_.base_frame;
    transform.child_frame_id = tf_config_.camera_frame;
    
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    
    tf_static_broadcaster_->sendTransform(transform);
    
    // Static transform from camera_link to color_optical_frame
    geometry_msgs::msg::TransformStamped color_transform;
    color_transform.header.stamp = now;
    color_transform.header.frame_id = tf_config_.camera_frame;
    color_transform.child_frame_id = tf_config_.color_optical_frame;
    
    color_transform.transform.translation.x = 0.0;
    color_transform.transform.translation.y = 0.0;
    color_transform.transform.translation.z = 0.0;
    color_transform.transform.rotation.x = 0.0;
    color_transform.transform.rotation.y = 0.0;
    color_transform.transform.rotation.z = 0.0;
    color_transform.transform.rotation.w = 1.0;
    
    tf_static_broadcaster_->sendTransform(color_transform);
    
    // Static transform from camera_link to depth_optical_frame
    geometry_msgs::msg::TransformStamped depth_transform;
    depth_transform.header.stamp = now;
    depth_transform.header.frame_id = tf_config_.camera_frame;
    depth_transform.child_frame_id = tf_config_.depth_optical_frame;
    
    depth_transform.transform.translation.x = 0.0;
    depth_transform.transform.translation.y = 0.0;
    depth_transform.transform.translation.z = 0.0;
    depth_transform.transform.rotation.x = 0.0;
    depth_transform.transform.rotation.y = 0.0;
    depth_transform.transform.rotation.z = 0.0;
    depth_transform.transform.rotation.w = 1.0;
    
    tf_static_broadcaster_->sendTransform(depth_transform);
}

void FVDepthCameraNode::handleGetDistance(
    const std::shared_ptr<fv_realsense::srv::GetDistance::Request> request,
    std::shared_ptr<fv_realsense::srv::GetDistance::Response> response)
{
    float world_x, world_y, world_z;
    
    if (get3DCoordinate(request->x, request->y, world_x, world_y, world_z)) {
        response->success = true;
        response->x = world_x;
        response->y = world_y;
        response->z = world_z;
        response->frame_id = tf_config_.color_optical_frame;
        response->message = "Success";
    } else {
        response->success = false;
        response->x = 0.0;
        response->y = 0.0;
        response->z = 0.0;
        response->frame_id = tf_config_.color_optical_frame;
        response->message = "Failed to get 3D coordinate";
    }
}

void FVDepthCameraNode::handleGetCameraInfo(
    const std::shared_ptr<fv_realsense::srv::GetCameraInfo::Request> request,
    std::shared_ptr<fv_realsense::srv::GetCameraInfo::Response> response)
{
    try {
        response->success = true;
        response->camera_name = device_.get_info(RS2_CAMERA_INFO_NAME);
        response->serial_number = device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        response->device_name = device_.get_info(RS2_CAMERA_INFO_NAME);
        response->color_width = color_intrinsics_.width;
        response->color_height = color_intrinsics_.height;
        response->color_fps = camera_config_.color_fps;
        response->depth_width = depth_intrinsics_.width;
        response->depth_height = depth_intrinsics_.height;
        response->depth_fps = camera_config_.depth_fps;
        response->depth_scale = depth_scale_;
        response->message = "Success";
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Error: ") + e.what();
    }
}

bool FVDepthCameraNode::get3DCoordinate(int x, int y, float& world_x, float& world_y, float& world_z)
{
    try {
        // Get latest depth frame
        rs2::frameset frames = pipe_.wait_for_frames();
        auto depth_frame = frames.get_depth_frame();
        
        if (!depth_frame) {
            return false;
        }
        
        // Get depth value
        float depth = depth_frame.get_distance(x, y);
        
        if (depth <= 0.0f) {
            return false;
        }
        
        // Convert to 3D coordinates
        float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
        float point[3];
        
        rs2_deproject_pixel_to_point(point, &color_intrinsics_, pixel, depth);
        
        world_x = point[0];
        world_y = point[1];
        world_z = point[2];
        
        return true;
        
    } catch (const rs2::error& e) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Error getting 3D coordinate: %s", e.what());
        return false;
    }
}

std::vector<rs2::device> FVDepthCameraNode::getAvailableDevices()
{
    std::vector<rs2::device> devices;
    
    try {
        RCLCPP_INFO(this->get_logger(), "🔍 Querying RealSense devices...");
        auto device_list = ctx_.query_devices();
        size_t device_count = device_list.size();
        
        RCLCPP_INFO(this->get_logger(), "🔍 Found %zu RealSense device(s)", device_count);
        
        // Limit the number of devices to prevent memory issues
        const size_t max_devices = 5;  // Reduced from 10 to 5
        if (device_count > max_devices) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Limiting devices to %zu (found %zu)", max_devices, device_count);
            device_count = max_devices;
        }
        
        for (size_t i = 0; i < device_count; ++i) {
            try {
                RCLCPP_INFO(this->get_logger(), "📷 Accessing device %zu...", i);
                auto device = device_list[i];
                
                // Get device info with error handling
                std::string name, serial;
                try {
                    name = device.get_info(RS2_CAMERA_INFO_NAME);
                } catch (const rs2::error& e) {
                    name = "Unknown";
                    RCLCPP_WARN(this->get_logger(), "⚠️ Could not get name for device %zu: %s", i, e.what());
                }
                
                try {
                    serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                } catch (const rs2::error& e) {
                    serial = "Unknown";
                    RCLCPP_WARN(this->get_logger(), "⚠️ Could not get serial for device %zu: %s", i, e.what());
                }
                
                RCLCPP_INFO(this->get_logger(), "📷 Device %zu: %s (SN: %s)", i, name.c_str(), serial.c_str());
                devices.push_back(device);
            } catch (const rs2::error& e) {
                RCLCPP_WARN(this->get_logger(), "⚠️ Error accessing device %zu: %s", i, e.what());
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "⚠️ Exception accessing device %zu: %s", i, e.what());
            }
        }
    } catch (const rs2::error& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Error querying devices: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ Exception querying devices: %s", e.what());
    }
    
    return devices;
}

int main(int argc, char** argv)
{
    try {
        rclcpp::init(argc, argv);
        
        RCLCPP_INFO(rclcpp::get_logger("fv_realsense"), "🚀 Starting FV RealSense Node...");
        
        auto node = std::make_shared<FVDepthCameraNode>();
        
        if (node) {
            RCLCPP_INFO(rclcpp::get_logger("fv_realsense"), "✅ Node created successfully");
            rclcpp::spin(node);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("fv_realsense"), "❌ Failed to create node");
            return 1;
        }
        
        rclcpp::shutdown();
        return 0;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("fv_realsense"), "❌ Exception in main: %s", e.what());
        return 1;
    }
} 