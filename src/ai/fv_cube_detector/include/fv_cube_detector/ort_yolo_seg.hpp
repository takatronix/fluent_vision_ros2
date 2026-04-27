#pragma once

#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <vector>

namespace fv_cube_detector {

struct SegResult {
  std::vector<cv::Rect> boxes;
  std::vector<int> classes;
  std::vector<float> scores;
  std::vector<cv::Mat> masks;  // per-instance binary mask (CV_8UC1, full image size)
};

class OrtYoloSeg {
 public:
  OrtYoloSeg();
  ~OrtYoloSeg();

  bool load(const std::string& model_path, bool use_gpu = true);
  bool infer(const cv::Mat& bgr, float conf_thres, float iou_thres, SegResult* out);
  std::string device_name() const { return device_name_; }

 private:
  struct LetterboxInfo {
    float scale;
    int pad_w, pad_h;
  };

  LetterboxInfo letterbox(const cv::Mat& src, cv::Mat& dst) const;
  void nms(const std::vector<cv::Rect2f>& boxes,
           const std::vector<float>& scores,
           float iou_th, std::vector<int>& keep) const;

  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::Session> session_;
  Ort::AllocatorWithDefaultOptions allocator_;

  std::vector<std::string> input_names_str_;
  std::vector<std::string> output_names_str_;
  std::vector<const char*> input_names_;
  std::vector<const char*> output_names_;

  int net_w_ = 640, net_h_ = 640;
  int num_classes_ = 6;
  int num_coeff_ = 32;
  int proto_h_ = 160, proto_w_ = 160;
  int max_det_ = 100;
  std::string device_name_ = "CPU";
};

}  // namespace fv_cube_detector
