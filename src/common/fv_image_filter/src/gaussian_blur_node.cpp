#include "fv_image_filter/image_filter_base.hpp"

class GaussianBlurFilter : public ImageFilterBase {
public:
    GaussianBlurFilter() : ImageFilterBase("fv_gaussian_blur") {
        ksize_ = declare_and_get("kernel_size", 5);
        sigma_ = declare_and_get("sigma", 1.0);
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if (p.get_name() == "kernel_size") ksize_ = static_cast<int>(p.as_int());
            else if (p.get_name() == "sigma") sigma_ = p.as_double();
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        cv::Mat out;
        int k = ensure_odd(ksize_);
        cv::GaussianBlur(input, out, {k, k}, sigma_);
        return out;
    }

private:
    int ksize_ = 5;
    double sigma_ = 1.0;
};

FV_FILTER_MAIN(GaussianBlurFilter)
