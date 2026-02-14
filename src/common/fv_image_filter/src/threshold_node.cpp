#include "fv_image_filter/image_filter_base.hpp"

/// Threshold types: 0=BINARY 1=BINARY_INV 2=TRUNC 3=TOZERO 4=TOZERO_INV 8=OTSU
class ThresholdFilter : public ImageFilterBase {
public:
    ThresholdFilter() : ImageFilterBase("fv_threshold") {
        thresh_    = declare_and_get("threshold", 128.0);
        max_val_   = declare_and_get("max_value", 255.0);
        type_      = declare_and_get("type", 0);   // cv::THRESH_BINARY
        use_otsu_  = declare_and_get("otsu", false);
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if      (p.get_name() == "threshold") thresh_   = p.as_double();
            else if (p.get_name() == "max_value") max_val_  = p.as_double();
            else if (p.get_name() == "type")      type_     = static_cast<int>(p.as_int());
            else if (p.get_name() == "otsu")      use_otsu_ = p.as_bool();
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        cv::Mat gray, out;
        to_gray(input, gray);
        int flags = type_;
        if (use_otsu_) flags |= cv::THRESH_OTSU;
        cv::threshold(gray, out, thresh_, max_val_, flags);
        return out;
    }

private:
    double thresh_ = 128.0, max_val_ = 255.0;
    int type_ = 0;
    bool use_otsu_ = false;
};

FV_FILTER_MAIN(ThresholdFilter)
