#include "fv_image_filter/image_filter_base.hpp"

class BilateralFilterNode : public ImageFilterBase {
public:
    BilateralFilterNode() : ImageFilterBase("fv_bilateral_filter") {
        d_           = declare_and_get("diameter", 9);
        sigma_color_ = declare_and_get("sigma_color", 75.0);
        sigma_space_ = declare_and_get("sigma_space", 75.0);
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if (p.get_name() == "diameter")     d_ = static_cast<int>(p.as_int());
            else if (p.get_name() == "sigma_color") sigma_color_ = p.as_double();
            else if (p.get_name() == "sigma_space") sigma_space_ = p.as_double();
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        cv::Mat out;
        cv::bilateralFilter(input, out, d_, sigma_color_, sigma_space_);
        return out;
    }

private:
    int d_ = 9;
    double sigma_color_ = 75.0, sigma_space_ = 75.0;
};

FV_FILTER_MAIN(BilateralFilterNode)
