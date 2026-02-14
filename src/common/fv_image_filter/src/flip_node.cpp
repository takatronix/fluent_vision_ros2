#include "fv_image_filter/image_filter_base.hpp"

/// flip_code: 0=vertical  1=horizontal  -1=both
class FlipFilter : public ImageFilterBase {
public:
    FlipFilter() : ImageFilterBase("fv_flip") {
        flip_code_ = declare_and_get("flip_code", 1);  // horizontal
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if (p.get_name() == "flip_code") flip_code_ = static_cast<int>(p.as_int());
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        cv::Mat out;
        cv::flip(input, out, flip_code_);
        return out;
    }

private:
    int flip_code_ = 1;
};

FV_FILTER_MAIN(FlipFilter)
