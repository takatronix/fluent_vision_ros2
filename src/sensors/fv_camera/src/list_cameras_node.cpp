#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

class ListCamerasNode : public rclcpp::Node
{
public:
    explicit ListCamerasNode()
        : Node("list_cameras")
    {
        RCLCPP_INFO(this->get_logger(), "🔍 Listing available cameras...");
        
        // Find available cameras
        std::vector<int> available_cameras = findAvailableCameras();
        
        if (available_cameras.empty()) {
            RCLCPP_WARN(this->get_logger(), "⚠️ No cameras found");
        } else {
            RCLCPP_INFO(this->get_logger(), "📷 Found %zu camera(s):", available_cameras.size());
            
            for (int camera_index : available_cameras) {
                RCLCPP_INFO(this->get_logger(), "  - Camera %d", camera_index);
                
                // Get camera properties
                cv::VideoCapture cap(camera_index);
                if (cap.isOpened()) {
                    int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
                    int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
                    double fps = cap.get(cv::CAP_PROP_FPS);
                    
                    RCLCPP_INFO(this->get_logger(), "    Resolution: %dx%d", width, height);
                    RCLCPP_INFO(this->get_logger(), "    FPS: %.1f", fps);
                    
                    // Test if camera can actually capture frames
                    cv::Mat test_frame;
                    if (cap.read(test_frame)) {
                        RCLCPP_INFO(this->get_logger(), "    Status: ✅ Working");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "    Status: ⚠️ Connected but not working");
                    }
                    
                    cap.release();
                }
            }
        }
        
        // Shutdown after listing
        rclcpp::shutdown();
    }

private:
    std::vector<int> findAvailableCameras()
    {
        std::vector<int> available_cameras;
        
        // Check cameras 0-9
        for (int i = 0; i < 10; i++) {
            cv::VideoCapture cap(i);
            if (cap.isOpened()) {
                available_cameras.push_back(i);
                cap.release();
            }
        }
        
        return available_cameras;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ListCamerasNode>();
    
    rclcpp::spin(node);
    
    return 0;
} 