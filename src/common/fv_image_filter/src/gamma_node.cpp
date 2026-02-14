#include "fv_image_filter/image_filter_base.hpp"

class GammaFilter : public ImageFilterBase {
public:
    GammaFilter() : ImageFilterBase("fv_gamma") {
        gamma_ = declare_and_get("gamma", 1.0);  // 0.1 ~ 5.0
        build_lut();
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if (p.get_name() == "gamma") {
                gamma_ = p.as_double();
                build_lut();
            }
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        cv::Mat out;
        cv::LUT(input, lut_, out);
        return out;
    }

private:
    void build_lut() {
        lut_ = cv::Mat(1, 256, CV_8U);
        uchar* p = lut_.ptr();
        double g = (gamma_ > 0.01) ? gamma_ : 0.01;
        double inv = 1.0 / g;
        for (int i = 0; i < 256; ++i) {
            p[i] = cv::saturate_cast<uchar>(std::pow(i / 255.0, inv) * 255.0);
        }
    }

    double gamma_ = 1.0;
    cv::Mat lut_;
};

FV_FILTER_MAIN(GammaFilter)
