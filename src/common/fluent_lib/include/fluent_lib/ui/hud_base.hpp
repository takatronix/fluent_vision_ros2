#pragma once
#include <opencv2/opencv.hpp>

namespace fluent_ui {

class HUDWidget {
public:
    virtual ~HUDWidget() = default;
    virtual void render(cv::Mat& image) = 0;
};

} // namespace fluent_ui


