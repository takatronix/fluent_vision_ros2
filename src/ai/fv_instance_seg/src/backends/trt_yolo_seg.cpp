#include "fv_instance_seg/backends/trt_yolo_seg.hpp"

#include <cuda_fp16.h>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <fstream>
#include <numeric>
#include <unordered_map>
#include <utility>

namespace fv_instance_seg {

namespace {

template <typename T>
std::shared_ptr<T> makeTrtShared(T* ptr) {
  return std::shared_ptr<T>(ptr, [](T* p) {
    delete p;
  });
}

}  // namespace

TrtYoloSegInferencer::TrtYoloSegInferencer()
    : logger_(nvinfer1::ILogger::Severity::kWARNING) {}

TrtYoloSegInferencer::~TrtYoloSegInferencer() {
  freeBuffers();
  context_.reset();
  engine_.reset();
  runtime_.reset();
}

void TrtYoloSegInferencer::TrtLogger::log(Severity severity, const char* msg) noexcept {
  if (severity > min_severity_) {
    return;
  }
  std::fprintf(stderr, "[TensorRT] %s\n", msg ? msg : "(null)");
}

std::size_t TrtYoloSegInferencer::dtypeBytes(nvinfer1::DataType dtype) {
  switch (dtype) {
    case nvinfer1::DataType::kFLOAT:
      return 4;
    case nvinfer1::DataType::kHALF:
      return 2;
    case nvinfer1::DataType::kINT8:
      return 1;
    case nvinfer1::DataType::kINT32:
      return 4;
#if NV_TENSORRT_MAJOR >= 10
    case nvinfer1::DataType::kINT64:
      return 8;
#endif
    case nvinfer1::DataType::kBOOL:
      return 1;
    default:
      break;
  }
  return 0;
}

std::size_t TrtYoloSegInferencer::dimsVolume(const nvinfer1::Dims& dims) {
  if (dims.nbDims <= 0) {
    return 0;
  }
  std::size_t vol = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) {
      return 0;
    }
    vol *= static_cast<std::size_t>(dims.d[i]);
  }
  return vol;
}

void TrtYoloSegInferencer::configure(bool nms_class_agnostic, int max_detections, bool debug_shapes) {
  nms_agnostic_ = nms_class_agnostic;
  max_det_ = std::max(1, max_detections);
  debug_ = debug_shapes;
}

bool TrtYoloSegInferencer::load(const std::string& model_path, const std::string& /*device*/) {
  freeBuffers();
  bindings_.clear();
  input_binding_ = -1;
  det_binding_ = -1;
  proto_binding_ = -1;
  net_w_ = 0;
  net_h_ = 0;

  std::ifstream ifs(model_path, std::ios::binary | std::ios::ate);
  if (!ifs.is_open()) {
    std::fprintf(stderr, "[fv_instance_seg][trt] failed to open engine: %s\n", model_path.c_str());
    return false;
  }
  const auto file_size = static_cast<std::size_t>(ifs.tellg());
  ifs.seekg(0, std::ios::beg);
  std::vector<char> blob(file_size);
  if (!ifs.read(blob.data(), static_cast<std::streamsize>(file_size))) {
    std::fprintf(stderr, "[fv_instance_seg][trt] failed to read engine: %s\n", model_path.c_str());
    return false;
  }

  runtime_ = makeTrtShared(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) {
    std::fprintf(stderr, "[fv_instance_seg][trt] createInferRuntime failed\n");
    return false;
  }

  engine_ = makeTrtShared(runtime_->deserializeCudaEngine(blob.data(), blob.size()));
  if (!engine_) {
    std::fprintf(stderr, "[fv_instance_seg][trt] deserializeCudaEngine failed: %s\n", model_path.c_str());
    return false;
  }

  context_ = makeTrtShared(engine_->createExecutionContext());
  if (!context_) {
    std::fprintf(stderr, "[fv_instance_seg][trt] createExecutionContext failed\n");
    return false;
  }

  std::vector<int> output_ids;

#if NV_TENSORRT_MAJOR >= 10
  const int nb_tensors = static_cast<int>(engine_->getNbIOTensors());
  bindings_.resize(nb_tensors);
  for (int i = 0; i < nb_tensors; ++i) {
    const char* name = engine_->getIOTensorName(i);
    if (!name) {
      continue;
    }
    TensorBinding tb;
    tb.name = name;
    tb.is_input = engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT;
    tb.dtype = engine_->getTensorDataType(name);
    tb.dims = engine_->getTensorShape(name);
    bindings_[i] = tb;
    if (tb.is_input && input_binding_ < 0) {
      input_binding_ = i;
    } else if (!tb.is_input) {
      output_ids.push_back(i);
    }
  }
#else
  const int nb_bindings = engine_->getNbBindings();
  bindings_.resize(nb_bindings);
  for (int i = 0; i < nb_bindings; ++i) {
    TensorBinding tb;
    tb.name = engine_->getBindingName(i) ? engine_->getBindingName(i) : "";
    tb.is_input = engine_->bindingIsInput(i);
    tb.dtype = engine_->getBindingDataType(i);
    tb.dims = engine_->getBindingDimensions(i);
    bindings_[i] = tb;
    if (tb.is_input && input_binding_ < 0) {
      input_binding_ = i;
    } else if (!tb.is_input) {
      output_ids.push_back(i);
    }
  }
#endif

  if (input_binding_ < 0 || output_ids.size() < 2) {
    std::fprintf(stderr, "[fv_instance_seg][trt] invalid io bindings (input=%d outputs=%zu)\n",
                 input_binding_, output_ids.size());
    return false;
  }

  for (int out_idx : output_ids) {
    const auto& dims = bindings_[out_idx].dims;
    if (dims.nbDims == 3 && det_binding_ < 0) {
      det_binding_ = out_idx;
    } else if (dims.nbDims == 4 && proto_binding_ < 0) {
      proto_binding_ = out_idx;
    }
  }
  if (det_binding_ < 0 || proto_binding_ < 0) {
    det_binding_ = output_ids[0];
    proto_binding_ = output_ids[1];
  }

  if (bindings_[input_binding_].dims.nbDims == 4) {
    net_h_ = bindings_[input_binding_].dims.d[2];
    net_w_ = bindings_[input_binding_].dims.d[3];
  }
  if (net_h_ <= 0) {
    net_h_ = 640;
  }
  if (net_w_ <= 0) {
    net_w_ = 640;
  }

  device_buffers_.assign(bindings_.size(), nullptr);
  device_buffer_bytes_.assign(bindings_.size(), 0);

  if (cudaStreamCreate(&stream_) != cudaSuccess) {
    std::fprintf(stderr, "[fv_instance_seg][trt] cudaStreamCreate failed\n");
    freeBuffers();
    return false;
  }

  return true;
}

bool TrtYoloSegInferencer::prepareBindings(int input_h, int input_w) {
  if (!engine_ || !context_ || input_binding_ < 0) {
    return false;
  }

#if NV_TENSORRT_MAJOR >= 10
  const std::string& in_name = bindings_[input_binding_].name;
  auto in_dims = engine_->getTensorShape(in_name.c_str());
  if (in_dims.nbDims == 4) {
    in_dims.d[0] = 1;
    in_dims.d[1] = 3;
    in_dims.d[2] = input_h;
    in_dims.d[3] = input_w;
    if (!context_->setInputShape(in_name.c_str(), in_dims)) {
      std::fprintf(stderr, "[fv_instance_seg][trt] setInputShape failed for %s\n", in_name.c_str());
      return false;
    }
  }

  for (std::size_t i = 0; i < bindings_.size(); ++i) {
    const auto& name = bindings_[i].name;
    auto runtime_dims = context_->getTensorShape(name.c_str());
    bindings_[i].dims = runtime_dims;
    const auto elem = dtypeBytes(bindings_[i].dtype);
    const auto vol = dimsVolume(runtime_dims);
    const std::size_t bytes = elem * vol;
    if (bytes == 0) {
      return false;
    }
    if (device_buffer_bytes_[i] < bytes) {
      if (device_buffers_[i]) {
        cudaFree(device_buffers_[i]);
      }
      if (cudaMalloc(&device_buffers_[i], bytes) != cudaSuccess) {
        device_buffers_[i] = nullptr;
        device_buffer_bytes_[i] = 0;
        return false;
      }
      device_buffer_bytes_[i] = bytes;
    }
    if (!context_->setTensorAddress(name.c_str(), device_buffers_[i])) {
      std::fprintf(stderr, "[fv_instance_seg][trt] setTensorAddress failed for %s\n", name.c_str());
      return false;
    }
  }
#else
  auto in_dims = context_->getBindingDimensions(input_binding_);
  if (in_dims.nbDims == 4 && (in_dims.d[2] <= 0 || in_dims.d[3] <= 0)) {
    in_dims.d[0] = 1;
    in_dims.d[1] = 3;
    in_dims.d[2] = input_h;
    in_dims.d[3] = input_w;
    if (!context_->setBindingDimensions(input_binding_, in_dims)) {
      std::fprintf(stderr, "[fv_instance_seg][trt] setBindingDimensions failed\n");
      return false;
    }
  }
  if (!context_->allInputDimensionsSpecified()) {
    return false;
  }

  for (std::size_t i = 0; i < bindings_.size(); ++i) {
    auto runtime_dims = context_->getBindingDimensions(static_cast<int>(i));
    bindings_[i].dims = runtime_dims;
    const auto elem = dtypeBytes(bindings_[i].dtype);
    const auto vol = dimsVolume(runtime_dims);
    const std::size_t bytes = elem * vol;
    if (bytes == 0) {
      return false;
    }
    if (device_buffer_bytes_[i] < bytes) {
      if (device_buffers_[i]) {
        cudaFree(device_buffers_[i]);
      }
      if (cudaMalloc(&device_buffers_[i], bytes) != cudaSuccess) {
        device_buffers_[i] = nullptr;
        device_buffer_bytes_[i] = 0;
        return false;
      }
      device_buffer_bytes_[i] = bytes;
    }
  }
#endif

  return true;
}

bool TrtYoloSegInferencer::copyInputFromFloat(int binding_index, const std::vector<float>& input_chw) {
  if (binding_index < 0 || binding_index >= static_cast<int>(bindings_.size())) {
    return false;
  }
  const auto dtype = bindings_[binding_index].dtype;
  const auto vol = dimsVolume(bindings_[binding_index].dims);
  if (vol == 0 || input_chw.size() != vol) {
    return false;
  }

  if (dtype == nvinfer1::DataType::kFLOAT) {
    return cudaMemcpyAsync(
               device_buffers_[binding_index], input_chw.data(), input_chw.size() * sizeof(float),
               cudaMemcpyHostToDevice, stream_) == cudaSuccess;
  }
  if (dtype == nvinfer1::DataType::kHALF) {
    std::vector<__half> fp16(input_chw.size());
    for (std::size_t i = 0; i < input_chw.size(); ++i) {
      fp16[i] = __float2half(input_chw[i]);
    }
    return cudaMemcpyAsync(
               device_buffers_[binding_index], fp16.data(), fp16.size() * sizeof(__half),
               cudaMemcpyHostToDevice, stream_) == cudaSuccess;
  }
  return false;
}

bool TrtYoloSegInferencer::copyOutputToFloat(int binding_index, std::vector<float>* out, nvinfer1::Dims* dims) {
  if (!out || !dims || binding_index < 0 || binding_index >= static_cast<int>(bindings_.size())) {
    return false;
  }

  *dims = bindings_[binding_index].dims;
  const auto vol = dimsVolume(*dims);
  const auto bytes = device_buffer_bytes_[binding_index];
  if (vol == 0 || bytes == 0) {
    return false;
  }

  std::vector<unsigned char> host(bytes);
  if (cudaMemcpyAsync(host.data(), device_buffers_[binding_index], bytes, cudaMemcpyDeviceToHost, stream_) != cudaSuccess) {
    return false;
  }
  if (cudaStreamSynchronize(stream_) != cudaSuccess) {
    return false;
  }

  const auto dtype = bindings_[binding_index].dtype;
  out->resize(vol);
  if (dtype == nvinfer1::DataType::kFLOAT) {
    std::memcpy(out->data(), host.data(), vol * sizeof(float));
    return true;
  }
  if (dtype == nvinfer1::DataType::kHALF) {
    const auto* h = reinterpret_cast<const __half*>(host.data());
    for (std::size_t i = 0; i < vol; ++i) {
      (*out)[i] = __half2float(h[i]);
    }
    return true;
  }
  return false;
}

TrtYoloSegInferencer::LetterboxInfo TrtYoloSegInferencer::letterbox(const cv::Mat& src, cv::Mat& dst) const {
  LetterboxInfo info{1.f, 0, 0};
  if (src.empty() || net_w_ <= 0 || net_h_ <= 0) {
    dst = src.clone();
    return info;
  }
  const float r = std::min(net_w_ / static_cast<float>(src.cols), net_h_ / static_cast<float>(src.rows));
  const int nw = static_cast<int>(std::round(src.cols * r));
  const int nh = static_cast<int>(std::round(src.rows * r));
  cv::Mat rs;
  cv::resize(src, rs, cv::Size(nw, nh));
  const int pw = (net_w_ - nw) / 2;
  const int ph = (net_h_ - nh) / 2;
  cv::copyMakeBorder(rs, dst, ph, net_h_ - nh - ph, pw, net_w_ - nw - pw, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  info.scale = r;
  info.pad_w = pw;
  info.pad_h = ph;
  return info;
}

void TrtYoloSegInferencer::nms(const std::vector<cv::Rect2f>& boxes, const std::vector<float>& scores, float iou_th,
                               std::vector<int>* keep) const {
  if (!keep) {
    return;
  }
  keep->clear();
  std::vector<int> idx(boxes.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::sort(idx.begin(), idx.end(), [&](int a, int b) { return scores[a] > scores[b]; });
  std::vector<char> suppressed(boxes.size(), 0);
  for (std::size_t i = 0; i < idx.size(); ++i) {
    const int bi = idx[i];
    if (suppressed[bi]) {
      continue;
    }
    keep->push_back(bi);
    for (std::size_t j = i + 1; j < idx.size(); ++j) {
      const int bj = idx[j];
      if (suppressed[bj]) {
        continue;
      }
      const float xx1 = std::max(boxes[bi].x, boxes[bj].x);
      const float yy1 = std::max(boxes[bi].y, boxes[bj].y);
      const float xx2 = std::min(boxes[bi].x + boxes[bi].width, boxes[bj].x + boxes[bj].width);
      const float yy2 = std::min(boxes[bi].y + boxes[bi].height, boxes[bj].y + boxes[bj].height);
      const float w = std::max(0.f, xx2 - xx1);
      const float h = std::max(0.f, yy2 - yy1);
      const float inter = w * h;
      const float a = boxes[bi].width * boxes[bi].height;
      const float b = boxes[bj].width * boxes[bj].height;
      const float ovr = inter / (a + b - inter + 1e-6f);
      if (ovr > iou_th) {
        suppressed[bj] = 1;
      }
    }
  }
}

bool TrtYoloSegInferencer::infer(const cv::Mat& bgr, float conf_thres, float iou_thres, InferResult* out) {
  if (!out || bgr.empty() || !engine_ || !context_ || input_binding_ < 0 || det_binding_ < 0 || proto_binding_ < 0) {
    return false;
  }

  out->boxes.clear();
  out->classes.clear();
  out->scores.clear();
  out->masks.clear();
  out->mask_proto_size = cv::Size();

  cv::Mat resized;
  const auto info = letterbox(bgr, resized);
  cv::Mat rgb;
  cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
  cv::Mat f32;
  rgb.convertTo(f32, CV_32F, 1.0 / 255.0);

  std::vector<cv::Mat> channels(3);
  std::vector<float> input_chw(static_cast<std::size_t>(net_w_) * static_cast<std::size_t>(net_h_) * 3);
  for (int c = 0; c < 3; ++c) {
    channels[c] = cv::Mat(net_h_, net_w_, CV_32F, input_chw.data() + static_cast<std::size_t>(c) * net_h_ * net_w_);
  }
  cv::split(f32, channels);

  if (!prepareBindings(net_h_, net_w_)) {
    return false;
  }

  if (!copyInputFromFloat(input_binding_, input_chw)) {
    return false;
  }

#if NV_TENSORRT_MAJOR >= 10
  if (!context_->enqueueV3(stream_)) {
    return false;
  }
#else
  if (!context_->enqueueV2(device_buffers_.data(), stream_, nullptr)) {
    return false;
  }
#endif

  std::vector<float> det_data;
  std::vector<float> proto_data;
  nvinfer1::Dims det_dims{};
  nvinfer1::Dims proto_dims{};
  if (!copyOutputToFloat(det_binding_, &det_data, &det_dims) ||
      !copyOutputToFloat(proto_binding_, &proto_data, &proto_dims)) {
    return false;
  }

  if (det_dims.nbDims != 3 || proto_dims.nbDims != 4) {
    return false;
  }

  const int det_d1 = det_dims.d[1];
  const int det_d2 = det_dims.d[2];
  if (det_d1 <= 0 || det_d2 <= 0) {
    return false;
  }
  const bool layout_cn = det_d1 < det_d2;  // [1,C,N]
  const int det_c = layout_cn ? det_d1 : det_d2;
  const int det_n = layout_cn ? det_d2 : det_d1;

  proto_c_ = proto_dims.d[1];
  proto_h_ = proto_dims.d[2];
  proto_w_ = proto_dims.d[3];
  if (proto_c_ <= 0 || proto_h_ <= 0 || proto_w_ <= 0) {
    return false;
  }
  num_coeff_ = proto_c_;
  num_classes_ = std::max(1, det_c - 4 - num_coeff_);
  if (det_c < 4 + num_classes_ + num_coeff_) {
    return false;
  }

  auto det_at = [&](int c, int i) -> float {
    if (layout_cn) {
      return det_data[static_cast<std::size_t>(c) * det_n + i];
    }
    return det_data[static_cast<std::size_t>(i) * det_c + c];
  };

  std::vector<cv::Rect2f> boxes;
  std::vector<float> scores;
  std::vector<int> classes;
  std::vector<std::vector<float>> coeffs;
  boxes.reserve(128);
  scores.reserve(128);
  classes.reserve(128);
  coeffs.reserve(128);

  for (int i = 0; i < det_n; ++i) {
    const float x = det_at(0, i);
    const float y = det_at(1, i);
    const float w = det_at(2, i);
    const float h = det_at(3, i);

    int cls_best = 0;
    float cls_score = 0.0f;
    for (int c = 0; c < num_classes_; ++c) {
      const float s = det_at(4 + c, i);
      if (s > cls_score) {
        cls_score = s;
        cls_best = c;
      }
    }
    if (cls_score < conf_thres) {
      continue;
    }

    float x1 = x - w * 0.5f;
    float y1 = y - h * 0.5f;
    float x2 = x + w * 0.5f;
    float y2 = y + h * 0.5f;

    x1 -= static_cast<float>(info.pad_w);
    x2 -= static_cast<float>(info.pad_w);
    y1 -= static_cast<float>(info.pad_h);
    y2 -= static_cast<float>(info.pad_h);
    if (info.scale > 0.0f) {
      x1 /= info.scale;
      x2 /= info.scale;
      y1 /= info.scale;
      y2 /= info.scale;
    }

    x1 = std::clamp(x1, 0.0f, static_cast<float>(bgr.cols - 1));
    y1 = std::clamp(y1, 0.0f, static_cast<float>(bgr.rows - 1));
    x2 = std::clamp(x2, 0.0f, static_cast<float>(bgr.cols - 1));
    y2 = std::clamp(y2, 0.0f, static_cast<float>(bgr.rows - 1));

    cv::Rect2f rect(x1, y1, std::max(0.0f, x2 - x1), std::max(0.0f, y2 - y1));
    if (rect.width <= 1.0f || rect.height <= 1.0f) {
      continue;
    }

    boxes.push_back(rect);
    scores.push_back(cls_score);
    classes.push_back(cls_best);
    std::vector<float> coef(num_coeff_);
    for (int k = 0; k < num_coeff_; ++k) {
      coef[k] = det_at(4 + num_classes_ + k, i);
    }
    coeffs.emplace_back(std::move(coef));
  }

  std::vector<int> keep;
  if (nms_agnostic_) {
    nms(boxes, scores, iou_thres, &keep);
  } else {
    std::unordered_map<int, std::vector<int>> by_class;
    for (std::size_t i = 0; i < classes.size(); ++i) {
      by_class[classes[i]].push_back(static_cast<int>(i));
    }
    for (auto& kv : by_class) {
      std::vector<cv::Rect2f> b2;
      std::vector<float> s2;
      b2.reserve(kv.second.size());
      s2.reserve(kv.second.size());
      for (int idx : kv.second) {
        b2.push_back(boxes[idx]);
        s2.push_back(scores[idx]);
      }
      std::vector<int> local_keep;
      nms(b2, s2, iou_thres, &local_keep);
      for (int lk : local_keep) {
        keep.push_back(kv.second[lk]);
      }
    }
  }

  if (static_cast<int>(keep.size()) > max_det_) {
    std::sort(keep.begin(), keep.end(), [&](int a, int b) { return scores[a] > scores[b]; });
    keep.resize(max_det_);
  }

  std::vector<cv::Mat> proto_planes(proto_c_);
  for (int p = 0; p < proto_c_; ++p) {
    float* ptr = proto_data.data() + static_cast<std::size_t>(p) * proto_h_ * proto_w_;
    proto_planes[p] = cv::Mat(proto_h_, proto_w_, CV_32F, ptr);
  }

  for (int idx : keep) {
    const auto& r = boxes[idx];
    const auto& coef = coeffs[idx];

    cv::Mat mask_sum(proto_h_, proto_w_, CV_32F, cv::Scalar(0));
    for (int p = 0; p < proto_c_; ++p) {
      mask_sum += proto_planes[p] * coef[p];
    }
    cv::Mat sig;
    cv::exp(-mask_sum, sig);
    sig = 1.0 / (1.0 + sig);

    cv::Mat mask_net;
    cv::resize(sig, mask_net, cv::Size(net_w_, net_h_), 0, 0, cv::INTER_LINEAR);

    cv::Rect roi(info.pad_w, info.pad_h, net_w_ - 2 * info.pad_w, net_h_ - 2 * info.pad_h);
    roi &= cv::Rect(0, 0, net_w_, net_h_);
    cv::Mat mask_unpad = (roi.width > 0 && roi.height > 0) ? mask_net(roi).clone() : mask_net;

    cv::Mat mask_bin = cv::Mat::zeros(bgr.rows, bgr.cols, CV_8UC1);
    cv::Rect ri(static_cast<int>(r.x), static_cast<int>(r.y), static_cast<int>(r.width), static_cast<int>(r.height));
    ri &= cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (ri.width > 0 && ri.height > 0) {
      const float s = (info.scale > 0.0f) ? info.scale : 1.0f;
      const int x1u = static_cast<int>(std::floor(r.x * s));
      const int y1u = static_cast<int>(std::floor(r.y * s));
      const int x2u = static_cast<int>(std::ceil((r.x + r.width) * s));
      const int y2u = static_cast<int>(std::ceil((r.y + r.height) * s));
      cv::Rect ri_u(x1u, y1u, std::max(0, x2u - x1u), std::max(0, y2u - y1u));
      ri_u &= cv::Rect(0, 0, mask_unpad.cols, mask_unpad.rows);
      if (ri_u.width > 0 && ri_u.height > 0) {
        cv::Mat roi_unpad = mask_unpad(ri_u);
        cv::Mat roi_resized;
        cv::resize(roi_unpad, roi_resized, ri.size(), 0, 0, cv::INTER_LINEAR);
        cv::Mat m8;
        roi_resized.convertTo(m8, CV_8UC1, 255.0);
        cv::threshold(m8, m8, 128, 255, cv::THRESH_BINARY);
        m8.copyTo(mask_bin(ri));
      }
    }

    out->boxes.emplace_back(static_cast<int>(r.x), static_cast<int>(r.y), static_cast<int>(r.width), static_cast<int>(r.height));
    out->classes.emplace_back(classes[idx]);
    out->scores.emplace_back(scores[idx]);
    out->masks.emplace_back(std::move(mask_bin));
  }

  out->mask_proto_size = cv::Size(proto_w_, proto_h_);

  if (debug_) {
    std::fprintf(stderr, "[fv_instance_seg][trt] det=%zu keep=%zu proto=(%d,%d,%d)\n",
                 boxes.size(), out->boxes.size(), proto_c_, proto_h_, proto_w_);
  }

  return true;
}

void TrtYoloSegInferencer::freeBuffers() {
  for (void*& ptr : device_buffers_) {
    if (ptr) {
      cudaFree(ptr);
      ptr = nullptr;
    }
  }
  device_buffers_.clear();
  device_buffer_bytes_.clear();
  if (stream_) {
    cudaStreamDestroy(stream_);
    stream_ = nullptr;
  }
}

}  // namespace fv_instance_seg
