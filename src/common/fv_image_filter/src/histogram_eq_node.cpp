#include "fv_image_filter/image_filter_base.hpp"

class HistogramEqFilter : public ImageFilterBase {
public:
    HistogramEqFilter() : ImageFilterBase("fv_histogram_eq") {
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>&) override {}

    cv::Mat apply_filter(const cv::Mat& input) override {
        if (input.channels() == 1) {
            cv::Mat out;
            cv::equalizeHist(input, out);
            return out;
        }
        // For color: convert to YCrCb, equalize Y channel, convert back
        cv::Mat ycrcb, out;
        cv::cvtColor(input, ycrcb, cv::COLOR_BGR2YCrCb);
        std::vector<cv::Mat> ch;
        cv::split(ycrcb, ch);
        cv::equalizeHist(ch[0], ch[0]);
        cv::merge(ch, ycrcb);
        cv::cvtColor(ycrcb, out, cv::COLOR_YCrCb2BGR);
        return out;
    }
};

FV_FILTER_MAIN(HistogramEqFilter)
