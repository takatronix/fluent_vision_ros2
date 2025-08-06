#ifndef FV_OBJECT_MASK_GENERATOR__FV_OBJECT_MASK_GENERATOR_NODE_HPP_
#define FV_OBJECT_MASK_GENERATOR__FV_OBJECT_MASK_GENERATOR_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <nlohmann/json.hpp>

namespace fv_object_mask_generator
{

class FVObjectMaskGeneratorNode : public rclcpp::Node
{
public:
  explicit FVObjectMaskGeneratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~FVObjectMaskGeneratorNode() = default;

private:
  void declareParameters();
  void initializeTopics();
  void initializeUNetModel();
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void processImage();
  
  // UNet segmentation methods (based on vision_ai UnetModel)
  cv::Mat inferUNet(const cv::Mat& image);
  cv::Mat drawSegmentationResult(const cv::Mat& image, const cv::Mat& seg_map, double inference_time_ms);
  std::vector<float> preprocessToCHW(const cv::Mat& src, int out_ch, int out_h, int out_w);
  
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr segmentation_mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr colored_mask_pub_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  
  // Parameters
  std::string input_image_topic_;
  std::string output_segmentation_mask_topic_;
  std::string output_colored_mask_topic_;
  double processing_frequency_;
  bool enable_visualization_;
  
  // UNet model parameters
  std::string model_type_;
  std::string model_path_;
  std::string device_;
  int input_width_;
  int input_height_;
  float confidence_threshold_;
  std::vector<std::string> class_names_;
  std::vector<cv::Scalar> class_colors_;
  
  // OpenVINO UNet model
  ov::CompiledModel compiled_model_;
  bool model_initialized_;
  
  // Data synchronization
  std::mutex data_mutex_;
  cv::Mat latest_image_;
  bool has_new_image_;
  
  // Processing timer
  rclcpp::TimerBase::SharedPtr processing_timer_;
};

}  // namespace fv_object_mask_generator

#endif  // FV_OBJECT_MASK_GENERATOR__FV_OBJECT_MASK_GENERATOR_NODE_HPP_