#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <map>
#include <vector>
#include <algorithm>

namespace fluent_ui {

using Vars = std::map<std::string, std::string>;

class Renderer {
public:
  static Renderer from_params() { return Renderer(); }

  void draw_lines(cv::Mat &img,
                  const std::vector<std::string> &lines,
                  const cv::Rect &anchor_bbox,
                  const std::string &anchor,
                  int padding,
                  const Vars &vars,
                  const cv::Scalar &color = cv::Scalar(255,255,255))
  {
      auto subst = [&](std::string s){
          for (const auto &kv : vars) {
              std::string key = std::string("${") + kv.first + "}";
              size_t pos = 0; while ((pos = s.find(key, pos)) != std::string::npos) {
                  s.replace(pos, key.size(), kv.second); pos += kv.second.size();
              }
          }
          return s;
      };

      int x = anchor_bbox.x;
      int y = anchor_bbox.y + anchor_bbox.height + padding + 20;
      if (anchor == "top_left") { x = padding; y = padding + 24; }
      else if (anchor == "top_right") { x = std::max(0, img.cols - 300); y = padding + 24; }

      int lh = 22;
      for (size_t i=0; i<lines.size(); ++i) {
          std::string text = subst(lines[i]);
          cv::putText(img, text, cv::Point(x, y + static_cast<int>(i)*lh),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
      }
  }
};

} // namespace fluent_ui


