#include "fv_image_filter/image_filter_base.hpp"

class LaplacianFilter : public ImageFilterBase {
public:
    LaplacianFilter() : ImageFilterBase("fv_laplacian") {
        ksize_ = declare_and_get("kernel_size", 3);
        scale_ = declare_and_get("scale", 1.0);
        delta_ = declare_and_get("delta", 0.0);
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if      (p.get_name() == "kernel_size") ksize_ = static_cast<int>(p.as_int());
            else if (p.get_name() == "scale")       scale_ = p.as_double();
            else if (p.get_name() == "delta")       delta_ = p.as_double();
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        cv::Mat gray, lap, out;
        to_gray(input, gray);
        cv::Laplacian(gray, lap, CV_16S, ensure_odd(ksize_), scale_, delta_);
        cv::convertScaleAbs(lap, out);
        return out;
    }

private:
    int ksize_ = 3;
    double scale_ = 1.0, delta_ = 0.0;
};

FV_FILTER_MAIN(LaplacianFilter)
