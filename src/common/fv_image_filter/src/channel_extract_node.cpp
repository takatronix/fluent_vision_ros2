#include "fv_image_filter/image_filter_base.hpp"

/// channel: 0=Blue/H  1=Green/S  2=Red/V  (depends on input encoding)
class ChannelExtractFilter : public ImageFilterBase {
public:
    ChannelExtractFilter() : ImageFilterBase("fv_channel_extract") {
        channel_ = declare_and_get("channel", 0);
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if (p.get_name() == "channel") channel_ = static_cast<int>(p.as_int());
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        if (input.channels() == 1) return input.clone();
        std::vector<cv::Mat> channels;
        cv::split(input, channels);
        int idx = std::clamp(channel_, 0, static_cast<int>(channels.size()) - 1);
        return channels[idx];
    }

private:
    int channel_ = 0;
};

FV_FILTER_MAIN(ChannelExtractFilter)
