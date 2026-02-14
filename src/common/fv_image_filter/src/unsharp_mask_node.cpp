#include "fv_image_filter/image_filter_base.hpp"

/// Unsharp mask:  out = input + strength * (input - blur(input))
class UnsharpMaskFilter : public ImageFilterBase {
public:
    UnsharpMaskFilter() : ImageFilterBase("fv_unsharp_mask") {
        sigma_    = declare_and_get("sigma", 1.0);
        strength_ = declare_and_get("strength", 1.5);
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if      (p.get_name() == "sigma")    sigma_    = p.as_double();
            else if (p.get_name() == "strength") strength_ = p.as_double();
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        cv::Mat blurred;
        cv::GaussianBlur(input, blurred, {0, 0}, sigma_);
        cv::Mat out;
        cv::addWeighted(input, 1.0 + strength_, blurred, -strength_, 0, out);
        return out;
    }

private:
    double sigma_ = 1.0, strength_ = 1.5;
};

FV_FILTER_MAIN(UnsharpMaskFilter)
