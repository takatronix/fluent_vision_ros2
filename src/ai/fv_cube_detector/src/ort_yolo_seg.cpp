#include "fv_cube_detector/ort_yolo_seg.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <iostream>

namespace fv_cube_detector {

OrtYoloSeg::OrtYoloSeg() {}
OrtYoloSeg::~OrtYoloSeg() {}

bool OrtYoloSeg::load(const std::string& model_path, bool use_gpu) {
  try {
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "fv_cube_detector");
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(4);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    if (use_gpu) {
      try {
        OrtCUDAProviderOptions cuda_opts{};
        cuda_opts.device_id = 0;
        opts.AppendExecutionProvider_CUDA(cuda_opts);
        device_name_ = "CUDA";
      } catch (...) {
        std::cerr << "[fv_cube_detector] CUDA EP not available, falling back to CPU\n";
        device_name_ = "CPU";
      }
    }

    session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), opts);

    // Gather input/output names
    size_t num_in = session_->GetInputCount();
    size_t num_out = session_->GetOutputCount();
    input_names_str_.resize(num_in);
    output_names_str_.resize(num_out);
    input_names_.resize(num_in);
    output_names_.resize(num_out);

    for (size_t i = 0; i < num_in; ++i) {
      auto name = session_->GetInputNameAllocated(i, allocator_);
      input_names_str_[i] = name.get();
      input_names_[i] = input_names_str_[i].c_str();
    }
    for (size_t i = 0; i < num_out; ++i) {
      auto name = session_->GetOutputNameAllocated(i, allocator_);
      output_names_str_[i] = name.get();
      output_names_[i] = output_names_str_[i].c_str();
    }

    // Get input shape
    auto in_shape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (in_shape.size() == 4) {
      net_h_ = static_cast<int>(in_shape[2]);
      net_w_ = static_cast<int>(in_shape[3]);
    }

    // Get det output shape to determine num_classes
    auto det_shape = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (det_shape.size() == 3) {
      int det_c = static_cast<int>(det_shape[1]);
      // det_c = 4 (box) + num_classes + num_coeff(32)
      num_classes_ = std::max(1, det_c - 4 - num_coeff_);
    }

    // Get proto output shape
    auto proto_shape = session_->GetOutputTypeInfo(1).GetTensorTypeAndShapeInfo().GetShape();
    if (proto_shape.size() == 4) {
      num_coeff_ = static_cast<int>(proto_shape[1]);
      proto_h_ = static_cast<int>(proto_shape[2]);
      proto_w_ = static_cast<int>(proto_shape[3]);
    }

    std::cout << "[fv_cube_detector] Model loaded: " << model_path
              << " | device=" << device_name_
              << " | input=" << net_w_ << "x" << net_h_
              << " | classes=" << num_classes_
              << " | proto=" << proto_h_ << "x" << proto_w_
              << std::endl;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "[fv_cube_detector] Model load failed: " << e.what() << std::endl;
    return false;
  }
}

OrtYoloSeg::LetterboxInfo OrtYoloSeg::letterbox(const cv::Mat& src, cv::Mat& dst) const {
  LetterboxInfo info{1.f, 0, 0};
  if (src.empty()) { dst = src.clone(); return info; }
  float r = std::min(static_cast<float>(net_w_) / src.cols,
                     static_cast<float>(net_h_) / src.rows);
  int nw = static_cast<int>(std::round(src.cols * r));
  int nh = static_cast<int>(std::round(src.rows * r));
  cv::Mat rs;
  cv::resize(src, rs, cv::Size(nw, nh));
  int pw = (net_w_ - nw) / 2;
  int ph = (net_h_ - nh) / 2;
  cv::copyMakeBorder(rs, dst, ph, net_h_ - nh - ph, pw, net_w_ - nw - pw,
                     cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  info.scale = r;
  info.pad_w = pw;
  info.pad_h = ph;
  return info;
}

void OrtYoloSeg::nms(const std::vector<cv::Rect2f>& boxes,
                      const std::vector<float>& scores,
                      float iou_th, std::vector<int>& keep) const {
  keep.clear();
  std::vector<int> idx(boxes.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::sort(idx.begin(), idx.end(), [&](int a, int b) { return scores[a] > scores[b]; });
  std::vector<char> sup(boxes.size(), 0);
  for (size_t i = 0; i < idx.size(); ++i) {
    int ii = idx[i];
    if (sup[ii]) continue;
    keep.push_back(ii);
    for (size_t j = i + 1; j < idx.size(); ++j) {
      int jj = idx[j];
      if (sup[jj]) continue;
      float xx1 = std::max(boxes[ii].x, boxes[jj].x);
      float yy1 = std::max(boxes[ii].y, boxes[jj].y);
      float xx2 = std::min(boxes[ii].x + boxes[ii].width, boxes[jj].x + boxes[jj].width);
      float yy2 = std::min(boxes[ii].y + boxes[ii].height, boxes[jj].y + boxes[jj].height);
      float w = std::max(0.f, xx2 - xx1), h = std::max(0.f, yy2 - yy1);
      float inter = w * h;
      float uni = boxes[ii].width * boxes[ii].height + boxes[jj].width * boxes[jj].height - inter;
      if (inter / (uni + 1e-6f) > iou_th) sup[jj] = 1;
    }
  }
}

bool OrtYoloSeg::infer(const cv::Mat& bgr, float conf_thres, float iou_thres, SegResult* out) {
  if (!out || bgr.empty() || !session_) return false;
  out->boxes.clear(); out->classes.clear(); out->scores.clear(); out->masks.clear();

  // Preprocess
  cv::Mat resized;
  auto info = letterbox(bgr, resized);
  cv::Mat rgb;
  cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
  cv::Mat f32;
  rgb.convertTo(f32, CV_32F, 1.0 / 255.0);

  // CHW blob
  std::vector<float> blob(3 * net_h_ * net_w_);
  std::vector<cv::Mat> channels(3);
  for (int c = 0; c < 3; ++c)
    channels[c] = cv::Mat(net_h_, net_w_, CV_32F, blob.data() + c * net_h_ * net_w_);
  cv::split(f32, channels);

  // Create input tensor
  std::array<int64_t, 4> in_shape = {1, 3, net_h_, net_w_};
  auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value in_tensor = Ort::Value::CreateTensor<float>(
      memory_info, blob.data(), blob.size(), in_shape.data(), in_shape.size());

  // Run inference
  auto outputs = session_->Run(Ort::RunOptions{nullptr},
                               input_names_.data(), &in_tensor, 1,
                               output_names_.data(), output_names_.size());

  // Parse det output [1, 4+nc+ncoef, N]
  const float* det = outputs[0].GetTensorData<float>();
  auto det_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
  int C = static_cast<int>(det_shape[1]);
  int N = static_cast<int>(det_shape[2]);
  (void)C;

  // Parse proto output [1, ncoef, ph, pw]
  const float* proto = outputs[1].GetTensorData<float>();
  int P = num_coeff_, Hm = proto_h_, Wm = proto_w_;
  int ncls = num_classes_;

  // Decode detections
  std::vector<cv::Rect2f> boxes;
  std::vector<float> scores;
  std::vector<int> classes;
  std::vector<std::vector<float>> coeffs;

  for (int i = 0; i < N; ++i) {
    float x = det[0 * N + i], y = det[1 * N + i];
    float w = det[2 * N + i], h = det[3 * N + i];

    int best_cls = 0;
    float best_score = 0.f;
    for (int c = 0; c < ncls; ++c) {
      float s = det[(4 + c) * N + i];
      if (s > best_score) { best_score = s; best_cls = c; }
    }
    if (best_score < conf_thres) continue;

    // Convert from center to corner, undo letterbox
    float x1 = (x - w * 0.5f - info.pad_w) / info.scale;
    float y1 = (y - h * 0.5f - info.pad_h) / info.scale;
    float x2 = (x + w * 0.5f - info.pad_w) / info.scale;
    float y2 = (y + h * 0.5f - info.pad_h) / info.scale;
    x1 = std::clamp(x1, 0.f, static_cast<float>(bgr.cols - 1));
    y1 = std::clamp(y1, 0.f, static_cast<float>(bgr.rows - 1));
    x2 = std::clamp(x2, 0.f, static_cast<float>(bgr.cols - 1));
    y2 = std::clamp(y2, 0.f, static_cast<float>(bgr.rows - 1));

    cv::Rect2f r(x1, y1, std::max(0.f, x2 - x1), std::max(0.f, y2 - y1));
    if (r.width <= 1 || r.height <= 1) continue;

    boxes.push_back(r);
    scores.push_back(best_score);
    classes.push_back(best_cls);

    std::vector<float> cv(P);
    for (int k = 0; k < P; ++k) cv[k] = det[(4 + ncls + k) * N + i];
    coeffs.emplace_back(std::move(cv));
  }

  // NMS
  std::vector<int> keep;
  nms(boxes, scores, iou_thres, keep);
  if (static_cast<int>(keep.size()) > max_det_) {
    std::sort(keep.begin(), keep.end(), [&](int a, int b) { return scores[a] > scores[b]; });
    keep.resize(max_det_);
  }

  // Decode masks
  std::vector<cv::Mat> proto_planes(P);
  for (int p = 0; p < P; ++p)
    proto_planes[p] = cv::Mat(Hm, Wm, CV_32F, const_cast<float*>(proto + p * Hm * Wm));

  for (int idx : keep) {
    const auto& r = boxes[idx];
    const auto& cvf = coeffs[idx];

    // Linear combination of protos
    cv::Mat ms = cv::Mat::zeros(Hm, Wm, CV_32F);
    for (int p = 0; p < P; ++p) ms += proto_planes[p] * cvf[p];

    // Sigmoid
    cv::Mat sig;
    cv::exp(-ms, sig);
    sig = 1.0 / (1.0 + sig);

    // Resize to net size, remove padding
    cv::Mat mnet;
    cv::resize(sig, mnet, cv::Size(net_w_, net_h_), 0, 0, cv::INTER_LINEAR);
    cv::Rect roi(info.pad_w, info.pad_h, net_w_ - 2 * info.pad_w, net_h_ - 2 * info.pad_h);
    roi &= cv::Rect(0, 0, net_w_, net_h_);
    cv::Mat munp = (roi.width > 0 && roi.height > 0) ? mnet(roi).clone() : mnet;

    // Create full-size binary mask within bbox
    cv::Mat mb = cv::Mat::zeros(bgr.rows, bgr.cols, CV_8UC1);
    cv::Rect ri(static_cast<int>(r.x), static_cast<int>(r.y),
                static_cast<int>(r.width), static_cast<int>(r.height));
    ri &= cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (ri.width > 0 && ri.height > 0) {
      float s = (info.scale > 0.f) ? info.scale : 1.f;
      int x1u = static_cast<int>(std::floor(r.x * s));
      int y1u = static_cast<int>(std::floor(r.y * s));
      int x2u = static_cast<int>(std::ceil((r.x + r.width) * s));
      int y2u = static_cast<int>(std::ceil((r.y + r.height) * s));
      cv::Rect ri_u(x1u, y1u, std::max(0, x2u - x1u), std::max(0, y2u - y1u));
      ri_u &= cv::Rect(0, 0, munp.cols, munp.rows);
      if (ri_u.width > 0 && ri_u.height > 0) {
        cv::Mat roi_rs;
        cv::resize(munp(ri_u), roi_rs, ri.size(), 0, 0, cv::INTER_LINEAR);
        cv::Mat m8;
        roi_rs.convertTo(m8, CV_8UC1, 255.0);
        cv::threshold(m8, m8, 128, 255, cv::THRESH_BINARY);
        m8.copyTo(mb(ri));
      }
    }

    out->boxes.emplace_back(static_cast<int>(r.x), static_cast<int>(r.y),
                            static_cast<int>(r.width), static_cast<int>(r.height));
    out->classes.push_back(classes[idx]);
    out->scores.push_back(scores[idx]);
    out->masks.emplace_back(std::move(mb));
  }
  return true;
}

}  // namespace fv_cube_detector
