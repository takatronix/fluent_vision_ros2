#include "fv_image_filter/image_filter_base.hpp"

/// Set width/height > 0 for absolute resize, or scale_factor > 0 for relative.
/// interpolation: 0=NEAREST 1=LINEAR 2=CUBIC 3=AREA 4=LANCZOS4
class ResizeFilter : public ImageFilterBase {
public:
    ResizeFilter() : ImageFilterBase("fv_resize") {
        width_   = declare_and_get("width", 0);
        height_  = declare_and_get("height", 0);
        scale_   = declare_and_get("scale_factor", 1.0);
        interp_  = declare_and_get("interpolation", 1);  // LINEAR
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if      (p.get_name() == "width")         width_  = static_cast<int>(p.as_int());
            else if (p.get_name() == "height")        height_ = static_cast<int>(p.as_int());
            else if (p.get_name() == "scale_factor")  scale_  = p.as_double();
            else if (p.get_name() == "interpolation") interp_ = static_cast<int>(p.as_int());
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        cv::Mat out;
        if (width_ > 0 && height_ > 0) {
            cv::resize(input, out, {width_, height_}, 0, 0, interp_);
        } else if (scale_ > 0.0 && scale_ != 1.0) {
            cv::resize(input, out, {}, scale_, scale_, interp_);
        } else {
            return input.clone();
        }
        return out;
    }

private:
    int width_ = 0, height_ = 0, interp_ = 1;
    double scale_ = 1.0;
};

FV_FILTER_MAIN(ResizeFilter)
