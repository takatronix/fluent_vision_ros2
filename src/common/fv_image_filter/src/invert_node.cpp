#include "fv_image_filter/image_filter_base.hpp"

class InvertFilter : public ImageFilterBase {
public:
    InvertFilter() : ImageFilterBase("fv_invert") {
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>&) override {}

    cv::Mat apply_filter(const cv::Mat& input) override {
        cv::Mat out;
        cv::bitwise_not(input, out);
        return out;
    }
};

FV_FILTER_MAIN(InvertFilter)
