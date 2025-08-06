#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class FVRealSenseSimpleNode : public rclcpp::Node
{
public:
    FVRealSenseSimpleNode() : Node("fv_realsense_simple")
    {
        RCLCPP_INFO(this->get_logger(), "🚀 FV RealSense Simple Node starting...");
        
        try {
            // Step 1: Load parameters
            RCLCPP_INFO(this->get_logger(), "📋 Step 1: Loading parameters...");
            loadParameters();
            
            // Step 2: Initialize publishers
            RCLCPP_INFO(this->get_logger(), "📤 Step 2: Initializing publishers...");
            initializePublishers();
            
            // Step 3: Create test image publisher
            RCLCPP_INFO(this->get_logger(), "🖼️ Step 3: Creating test image publisher...");
            createTestImagePublisher();
            
            RCLCPP_INFO(this->get_logger(), "✅ FV RealSense Simple Node started successfully");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Exception during initialization: %s", e.what());
        }
    }
    
    ~FVRealSenseSimpleNode()
    {
        RCLCPP_INFO(this->get_logger(), "🛑 Shutting down FV RealSense Simple Node...");
    }

private:
    void loadParameters()
    {
        // Camera settings
        color_width_ = this->declare_parameter("camera.color_width", 640);
        color_height_ = this->declare_parameter("camera.color_height", 480);
        color_fps_ = this->declare_parameter("camera.color_fps", 30);
        
        RCLCPP_INFO(this->get_logger(), "📷 Camera settings: %dx%d @ %dfps", 
            color_width_, color_height_, color_fps_);
    }
    
    void initializePublishers()
    {
        // Create image publisher
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/fv_realsense/color/image_raw", 10);
        
        RCLCPP_INFO(this->get_logger(), "📤 Image publisher created: /fv_realsense/color/image_raw");
    }
    
    void createTestImagePublisher()
    {
        // Create timer for publishing test images
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / color_fps_),  // Convert fps to milliseconds
            std::bind(&FVRealSenseSimpleNode::publishTestImage, this));
        
        RCLCPP_INFO(this->get_logger(), "⏰ Test image timer created");
    }
    
    void publishTestImage()
    {
        try {
            // Create a colorful test image with gradient background
            cv::Mat test_image = cv::Mat::zeros(color_height_, color_width_, CV_8UC3);
            
            // Create gradient background
            for (int y = 0; y < color_height_; y++) {
                for (int x = 0; x < color_width_; x++) {
                    int blue = (x * 255) / color_width_;
                    int green = (y * 255) / color_height_;
                    int red = 128;
                    test_image.at<cv::Vec3b>(y, x) = cv::Vec3b(blue, green, red);
                }
            }
            
            // Add a colored rectangle
            cv::rectangle(test_image, cv::Point(50, 50), cv::Point(590, 430), cv::Scalar(0, 255, 0), 5);
            
            // Add text
            cv::putText(test_image, "RealSense Test Image", cv::Point(150, 240), 
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 3);
            
            // Add timestamp
            auto now = this->now();
            std::string timestamp = std::to_string(now.seconds());
            cv::putText(test_image, "Time: " + timestamp, cv::Point(50, 50), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
            
            // Add a moving circle
            int circle_x = (static_cast<int>(now.seconds()) % 10) * 60 + 50;
            cv::circle(test_image, cv::Point(circle_x, 100), 20, cv::Scalar(255, 0, 255), -1);
            
            // Convert to ROS message
            auto ros_image = cv_bridge::CvImage();
            ros_image.image = test_image;
            ros_image.encoding = "bgr8";
            ros_image.header.stamp = now;
            ros_image.header.frame_id = "camera_frame";
            
            // Publish
            image_pub_->publish(*ros_image.toImageMsg());
            
            RCLCPP_INFO(this->get_logger(), "📤 Published colorful test image at time: %s", timestamp.c_str());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Error publishing test image: %s", e.what());
        }
    }
    
    // Parameters
    int color_width_;
    int color_height_;
    int color_fps_;
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    try {
        rclcpp::init(argc, argv);
        
        RCLCPP_INFO(rclcpp::get_logger("fv_realsense_simple"), "🚀 Starting FV RealSense Simple Node...");
        
        auto node = std::make_shared<FVRealSenseSimpleNode>();
        
        if (node) {
            RCLCPP_INFO(rclcpp::get_logger("fv_realsense_simple"), "✅ Node created successfully");
            rclcpp::spin(node);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("fv_realsense_simple"), "❌ Failed to create node");
            return 1;
        }
        
        rclcpp::shutdown();
        return 0;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("fv_realsense_simple"), "❌ Exception in main: %s", e.what());
        return 1;
    }
} 