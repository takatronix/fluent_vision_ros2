#include "fv_image_filter/image_filter_base.hpp"
#include <unordered_map>

/// conversion: "bgr2gray" "bgr2hsv" "bgr2rgb" "bgr2lab" "bgr2yuv"
///             "gray2bgr" "hsv2bgr" "rgb2bgr" "lab2bgr" "yuv2bgr"
class ColorConvertFilter : public ImageFilterBase {
public:
    ColorConvertFilter() : ImageFilterBase("fv_color_convert") {
        conversion_ = declare_and_get<std::string>("conversion", "bgr2gray");
        update_code();
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if (p.get_name() == "conversion") {
                conversion_ = p.as_string();
                update_code();
            }
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        if (code_ < 0) return input.clone();
        cv::Mat out;
        cv::cvtColor(input, out, code_);
        return out;
    }

private:
    void update_code() {
        static const std::unordered_map<std::string, int> table = {
            {"bgr2gray", cv::COLOR_BGR2GRAY}, {"gray2bgr", cv::COLOR_GRAY2BGR},
            {"bgr2hsv",  cv::COLOR_BGR2HSV},  {"hsv2bgr",  cv::COLOR_HSV2BGR},
            {"bgr2rgb",  cv::COLOR_BGR2RGB},  {"rgb2bgr",  cv::COLOR_RGB2BGR},
            {"bgr2lab",  cv::COLOR_BGR2Lab},  {"lab2bgr",  cv::COLOR_Lab2BGR},
            {"bgr2yuv",  cv::COLOR_BGR2YUV},  {"yuv2bgr",  cv::COLOR_YUV2BGR},
            {"bgr2hls",  cv::COLOR_BGR2HLS},  {"hls2bgr",  cv::COLOR_HLS2BGR},
        };
        auto it = table.find(conversion_);
        code_ = (it != table.end()) ? it->second : -1;
        if (code_ < 0) {
            RCLCPP_WARN(get_logger(), "Unknown conversion: %s", conversion_.c_str());
        }
    }

    std::string conversion_;
    int code_ = -1;
};

FV_FILTER_MAIN(ColorConvertFilter)
