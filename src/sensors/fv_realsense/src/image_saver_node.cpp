#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>

class ImageSaverNode : public rclcpp::Node
{
public:
    ImageSaverNode() : Node("image_saver_node"), image_count_(0)
    {
        // Create subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/fv_realsense/color/image_raw", 10,
            std::bind(&ImageSaverNode::image_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "🚀 Image Saver Node started - listening to /fv_realsense/color/image_raw");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convert ROS image to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            // Save image
            std::string filename = "/tmp/test_image_" + std::to_string(image_count_) + ".jpg";
            cv::imwrite(filename, cv_ptr->image);
            
            RCLCPP_INFO(this->get_logger(), "💾 Saved image: %s (size: %dx%d)", 
                filename.c_str(), cv_ptr->image.cols, cv_ptr->image.rows);
            
            // Only save first 5 images
            image_count_++;
            if (image_count_ >= 5) {
                RCLCPP_INFO(this->get_logger(), "✅ Saved 5 test images. Shutting down...");
                rclcpp::shutdown();
            }
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Error saving image: %s", e.what());
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    int image_count_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSaverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 