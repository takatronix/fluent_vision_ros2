#include "fv_image_filter/image_filter_base.hpp"

/// method: 0=MEAN_C  1=GAUSSIAN_C
/// type:   0=BINARY  1=BINARY_INV
class AdaptiveThresholdFilter : public ImageFilterBase {
public:
    AdaptiveThresholdFilter() : ImageFilterBase("fv_adaptive_threshold") {
        max_val_    = declare_and_get("max_value", 255.0);
        method_     = declare_and_get("method", 1);   // GAUSSIAN_C
        type_       = declare_and_get("type", 0);     // BINARY
        block_size_ = declare_and_get("block_size", 11);
        c_          = declare_and_get("C", 2.0);
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if      (p.get_name() == "max_value")  max_val_    = p.as_double();
            else if (p.get_name() == "method")     method_     = static_cast<int>(p.as_int());
            else if (p.get_name() == "type")       type_       = static_cast<int>(p.as_int());
            else if (p.get_name() == "block_size") block_size_ = static_cast<int>(p.as_int());
            else if (p.get_name() == "C")          c_          = p.as_double();
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        cv::Mat gray, out;
        to_gray(input, gray);
        int bs = ensure_odd(block_size_);
        if (bs < 3) bs = 3;
        cv::adaptiveThreshold(gray, out, max_val_, method_, type_, bs, c_);
        return out;
    }

private:
    double max_val_ = 255.0, c_ = 2.0;
    int method_ = 1, type_ = 0, block_size_ = 11;
};

FV_FILTER_MAIN(AdaptiveThresholdFilter)
