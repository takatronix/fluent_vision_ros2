#include "fv_instance_seg/backends/inferencer.hpp"

#ifdef FV_INSTANCE_SEG_HAVE_OPENVINO
#include "fv_instance_seg/backends/ov_yolo_seg.hpp"
#endif

#ifdef FV_INSTANCE_SEG_HAVE_TENSORRT
#include "fv_instance_seg/backends/trt_yolo_seg.hpp"
#endif

#include <algorithm>
#include <cctype>
#include <memory>
#include <string>

namespace fv_instance_seg {

namespace {
class NoopInferencer : public Inferencer {
 public:
  bool load(const std::string&, const std::string&) override { return true; }
  bool infer(const cv::Mat&, float, float, InferResult* out) override {
    if (!out) {
      return false;
    }
    out->boxes.clear();
    out->classes.clear();
    out->scores.clear();
    out->masks.clear();
    out->mask_proto_size = cv::Size();
    return true;
  }
  void configure(bool, int, bool) override {}
};
}  // namespace

std::unique_ptr<Inferencer> CreateInferencer(const std::string& backend) {
  std::string key = backend;
  std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (key == "auto") {
#ifdef FV_INSTANCE_SEG_HAVE_TENSORRT
    return std::make_unique<TrtYoloSegInferencer>();
#elif defined(FV_INSTANCE_SEG_HAVE_OPENVINO)
    return std::make_unique<OvYoloSegInferencer>();
#else
    return std::make_unique<NoopInferencer>();
#endif
  }

  if (key == "tensorrt" || key == "trt") {
#ifdef FV_INSTANCE_SEG_HAVE_TENSORRT
    return std::make_unique<TrtYoloSegInferencer>();
#else
    return std::make_unique<NoopInferencer>();
#endif
  }

  if (key.empty() || key == "openvino" || key == "ov") {
#ifdef FV_INSTANCE_SEG_HAVE_OPENVINO
    return std::make_unique<OvYoloSegInferencer>();
#else
    return std::make_unique<NoopInferencer>();
#endif
  }

  return std::make_unique<NoopInferencer>();
}

}  // namespace fv_instance_seg
