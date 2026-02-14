#include "fv_image_filter/image_filter_base.hpp"

class BoxBlurFilter : public ImageFilterBase {
public:
    BoxBlurFilter() : ImageFilterBase("fv_box_blur") {
        ksize_ = declare_and_get("kernel_size", 5);
        normalize_ = declare_and_get("normalize", true);
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if (p.get_name() == "kernel_size") ksize_ = static_cast<int>(p.as_int());
            else if (p.get_name() == "normalize") normalize_ = p.as_bool();
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        cv::Mat out;
        int k = ensure_odd(ksize_);
        cv::boxFilter(input, out, -1, {k, k}, {-1, -1}, normalize_);
        return out;
    }

private:
    int ksize_ = 5;
    bool normalize_ = true;
};

FV_FILTER_MAIN(BoxBlurFilter)
