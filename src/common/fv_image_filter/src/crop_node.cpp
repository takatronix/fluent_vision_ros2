#include "fv_image_filter/image_filter_base.hpp"

class CropFilter : public ImageFilterBase {
public:
    CropFilter() : ImageFilterBase("fv_crop") {
        x_ = declare_and_get("x", 0);
        y_ = declare_and_get("y", 0);
        w_ = declare_and_get("width", 0);   // 0 = full width
        h_ = declare_and_get("height", 0);  // 0 = full height
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if      (p.get_name() == "x")      x_ = static_cast<int>(p.as_int());
            else if (p.get_name() == "y")      y_ = static_cast<int>(p.as_int());
            else if (p.get_name() == "width")  w_ = static_cast<int>(p.as_int());
            else if (p.get_name() == "height") h_ = static_cast<int>(p.as_int());
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        int cx = std::clamp(x_, 0, input.cols - 1);
        int cy = std::clamp(y_, 0, input.rows - 1);
        int cw = (w_ > 0) ? w_ : (input.cols - cx);
        int ch = (h_ > 0) ? h_ : (input.rows - cy);
        cw = std::min(cw, input.cols - cx);
        ch = std::min(ch, input.rows - cy);
        if (cw <= 0 || ch <= 0) return input.clone();
        return input(cv::Rect(cx, cy, cw, ch)).clone();
    }

private:
    int x_ = 0, y_ = 0, w_ = 0, h_ = 0;
};

FV_FILTER_MAIN(CropFilter)
