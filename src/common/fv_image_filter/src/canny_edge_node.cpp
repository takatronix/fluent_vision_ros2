#include "fv_image_filter/image_filter_base.hpp"

class CannyEdgeFilter : public ImageFilterBase {
public:
    CannyEdgeFilter() : ImageFilterBase("fv_canny_edge") {
        threshold1_  = declare_and_get("threshold1", 50.0);
        threshold2_  = declare_and_get("threshold2", 150.0);
        aperture_    = declare_and_get("aperture_size", 3);
        l2_gradient_ = declare_and_get("l2_gradient", false);
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if      (p.get_name() == "threshold1")    threshold1_  = p.as_double();
            else if (p.get_name() == "threshold2")    threshold2_  = p.as_double();
            else if (p.get_name() == "aperture_size") aperture_    = static_cast<int>(p.as_int());
            else if (p.get_name() == "l2_gradient")   l2_gradient_ = p.as_bool();
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        cv::Mat gray, out;
        to_gray(input, gray);
        cv::Canny(gray, out, threshold1_, threshold2_, ensure_odd(aperture_), l2_gradient_);
        return out;
    }

private:
    double threshold1_ = 50.0, threshold2_ = 150.0;
    int aperture_ = 3;
    bool l2_gradient_ = false;
};

FV_FILTER_MAIN(CannyEdgeFilter)
