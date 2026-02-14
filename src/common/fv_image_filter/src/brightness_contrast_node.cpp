#include "fv_image_filter/image_filter_base.hpp"

/// out = input * contrast + brightness
class BrightnessContrastFilter : public ImageFilterBase {
public:
    BrightnessContrastFilter() : ImageFilterBase("fv_brightness_contrast") {
        brightness_ = declare_and_get("brightness", 0.0);   // -255 ~ 255
        contrast_   = declare_and_get("contrast", 1.0);     // 0.0 ~ 3.0
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if      (p.get_name() == "brightness") brightness_ = p.as_double();
            else if (p.get_name() == "contrast")   contrast_   = p.as_double();
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        cv::Mat out;
        input.convertTo(out, -1, contrast_, brightness_);
        return out;
    }

private:
    double brightness_ = 0.0, contrast_ = 1.0;
};

FV_FILTER_MAIN(BrightnessContrastFilter)
