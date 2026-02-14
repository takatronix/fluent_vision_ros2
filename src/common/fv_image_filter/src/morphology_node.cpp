#include "fv_image_filter/image_filter_base.hpp"

/// operation: 0=ERODE 1=DILATE 2=OPEN 3=CLOSE 4=GRADIENT 5=TOPHAT 6=BLACKHAT
/// shape:     0=RECT  1=CROSS  2=ELLIPSE
class MorphologyFilter : public ImageFilterBase {
public:
    MorphologyFilter() : ImageFilterBase("fv_morphology") {
        operation_  = declare_and_get("operation", 0);
        shape_      = declare_and_get("shape", 0);
        ksize_      = declare_and_get("kernel_size", 5);
        iterations_ = declare_and_get("iterations", 1);
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if      (p.get_name() == "operation")   operation_  = static_cast<int>(p.as_int());
            else if (p.get_name() == "shape")       shape_      = static_cast<int>(p.as_int());
            else if (p.get_name() == "kernel_size") ksize_      = static_cast<int>(p.as_int());
            else if (p.get_name() == "iterations")  iterations_ = static_cast<int>(p.as_int());
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        int k = ensure_odd(ksize_);
        auto kernel = cv::getStructuringElement(shape_, {k, k});
        cv::Mat out;
        if (operation_ == 0) {
            cv::erode(input, out, kernel, {-1,-1}, iterations_);
        } else if (operation_ == 1) {
            cv::dilate(input, out, kernel, {-1,-1}, iterations_);
        } else {
            cv::morphologyEx(input, out, operation_, kernel, {-1,-1}, iterations_);
        }
        return out;
    }

private:
    int operation_ = 0, shape_ = 0, ksize_ = 5, iterations_ = 1;
};

FV_FILTER_MAIN(MorphologyFilter)
