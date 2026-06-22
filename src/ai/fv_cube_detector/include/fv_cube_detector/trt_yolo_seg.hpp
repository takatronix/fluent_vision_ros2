#pragma once

#include <NvInfer.h>
#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <vector>
#include <cuda_runtime_api.h>

#include "fv_cube_detector/ort_yolo_seg.hpp"  // reuse SegResult

namespace fv_cube_detector {

class TrtLogger : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char* msg) noexcept override;
};

class TrtYoloSeg {
 public:
  TrtYoloSeg();
  ~TrtYoloSeg();

  bool load(const std::string& engine_path);
  bool infer(const cv::Mat& bgr, float conf_thres, float iou_thres, SegResult* out);
  std::string device_name() const { return "TensorRT"; }

 private:
  struct LetterboxInfo { float scale; int pad_w, pad_h; };

  LetterboxInfo letterbox(const cv::Mat& src, cv::Mat& dst) const;
  void nms(const std::vector<cv::Rect2f>& boxes,
           const std::vector<float>& scores,
           float iou_th, std::vector<int>& keep) const;

  TrtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;

  // Device buffers
  void* d_input_ = nullptr;
  void* d_det_ = nullptr;
  void* d_proto_ = nullptr;

  // Host buffers
  std::vector<float> h_det_;
  std::vector<float> h_proto_;

  int net_w_ = 640, net_h_ = 640;
  int num_classes_ = 6;
  int num_coeff_ = 32;
  int proto_h_ = 160, proto_w_ = 160;
  int max_det_ = 100;

  // Output dimensions from engine
  int det_c_ = 0, det_n_ = 0;
  // true = end2end/NMS-free head: det output is [num_det, 4+1+1+coeff]
  // (xyxy,score,class,mask) already NMS'd; false = raw [4+nc+coeff, anchors].
  bool is_end2end_ = false;

  cudaStream_t stream_ = nullptr;
};

}  // namespace fv_cube_detector
