#pragma once

#include "fv_instance_seg/backends/inferencer.hpp"

#include <NvInfer.h>

#include <cuda_runtime_api.h>

#include <memory>
#include <string>
#include <vector>

namespace fv_instance_seg {

class TrtYoloSegInferencer : public Inferencer {
 public:
  TrtYoloSegInferencer();
  ~TrtYoloSegInferencer() override;

  bool load(const std::string& model_path, const std::string& device) override;
  bool infer(const cv::Mat& bgr, float conf_thres, float iou_thres, InferResult* out) override;
  void configure(bool nms_class_agnostic, int max_detections, bool debug_shapes) override;

 private:
  struct LetterboxInfo {
    float scale;
    int pad_w;
    int pad_h;
  };

  struct TensorBinding {
    std::string name;
    bool is_input{false};
    nvinfer1::DataType dtype{nvinfer1::DataType::kFLOAT};
    nvinfer1::Dims dims{};
  };

  class TrtLogger : public nvinfer1::ILogger {
   public:
    explicit TrtLogger(Severity severity) : min_severity_(severity) {}
    void log(Severity severity, const char* msg) noexcept override;

   private:
    Severity min_severity_;
  };

  static std::size_t dtypeBytes(nvinfer1::DataType dtype);
  static std::size_t dimsVolume(const nvinfer1::Dims& dims);

  LetterboxInfo letterbox(const cv::Mat& src, cv::Mat& dst) const;
  void nms(const std::vector<cv::Rect2f>& boxes, const std::vector<float>& scores, float iou_th, std::vector<int>* keep) const;
  bool prepareBindings(int input_h, int input_w);
  bool copyOutputToFloat(int binding_index, std::vector<float>* out, nvinfer1::Dims* dims);
  bool copyInputFromFloat(int binding_index, const std::vector<float>& input_chw);
  void freeBuffers();

  TrtLogger logger_;
  std::shared_ptr<nvinfer1::IRuntime> runtime_;
  std::shared_ptr<nvinfer1::ICudaEngine> engine_;
  std::shared_ptr<nvinfer1::IExecutionContext> context_;

  std::vector<TensorBinding> bindings_;
  std::vector<void*> device_buffers_;
  std::vector<std::size_t> device_buffer_bytes_;
  cudaStream_t stream_{nullptr};

  int input_binding_{-1};
  int det_binding_{-1};
  int proto_binding_{-1};

  int net_w_{0};
  int net_h_{0};
  int proto_c_{0};
  int proto_h_{0};
  int proto_w_{0};
  int num_coeff_{0};
  int num_classes_{1};

  bool nms_agnostic_{true};
  int max_det_{100};
  bool debug_{false};
};

}  // namespace fv_instance_seg

