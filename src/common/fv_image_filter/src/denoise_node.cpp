#include "fv_image_filter/image_filter_base.hpp"

class DenoiseFilter : public ImageFilterBase {
public:
    DenoiseFilter() : ImageFilterBase("fv_denoise") {
        h_                    = declare_and_get("h", 10.0);
        template_window_size_ = declare_and_get("template_window_size", 7);
        search_window_size_   = declare_and_get("search_window_size", 21);
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if      (p.get_name() == "h")                    h_ = p.as_double();
            else if (p.get_name() == "template_window_size") template_window_size_ = static_cast<int>(p.as_int());
            else if (p.get_name() == "search_window_size")   search_window_size_   = static_cast<int>(p.as_int());
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        cv::Mat out;
        int tw = ensure_odd(template_window_size_);
        int sw = ensure_odd(search_window_size_);
        if (input.channels() == 1) {
            cv::fastNlMeansDenoising(input, out, static_cast<float>(h_), tw, sw);
        } else {
            cv::fastNlMeansDenoisingColored(input, out, static_cast<float>(h_),
                                            static_cast<float>(h_), tw, sw);
        }
        return out;
    }

private:
    double h_ = 10.0;
    int template_window_size_ = 7, search_window_size_ = 21;
};

FV_FILTER_MAIN(DenoiseFilter)
