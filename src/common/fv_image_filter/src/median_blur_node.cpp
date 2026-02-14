#include "fv_image_filter/image_filter_base.hpp"

class MedianBlurFilter : public ImageFilterBase {
public:
    MedianBlurFilter() : ImageFilterBase("fv_median_blur") {
        ksize_ = declare_and_get("kernel_size", 5);
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if (p.get_name() == "kernel_size") ksize_ = static_cast<int>(p.as_int());
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        cv::Mat out;
        cv::medianBlur(input, out, ensure_odd(ksize_));
        return out;
    }

private:
    int ksize_ = 5;
};

FV_FILTER_MAIN(MedianBlurFilter)
