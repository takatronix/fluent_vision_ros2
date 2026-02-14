#include "fv_image_filter/image_filter_base.hpp"

class CLAHEFilter : public ImageFilterBase {
public:
    CLAHEFilter() : ImageFilterBase("fv_clahe") {
        clip_limit_ = declare_and_get("clip_limit", 2.0);
        tile_size_  = declare_and_get("tile_size", 8);
        rebuild_clahe();
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        bool changed = false;
        for (auto& p : params) {
            if      (p.get_name() == "clip_limit") { clip_limit_ = p.as_double(); changed = true; }
            else if (p.get_name() == "tile_size")  { tile_size_ = static_cast<int>(p.as_int()); changed = true; }
        }
        if (changed) rebuild_clahe();
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        if (input.channels() == 1) {
            cv::Mat out;
            clahe_->apply(input, out);
            return out;
        }
        // Color: apply CLAHE to L channel in LAB space
        cv::Mat lab, out;
        cv::cvtColor(input, lab, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> ch;
        cv::split(lab, ch);
        clahe_->apply(ch[0], ch[0]);
        cv::merge(ch, lab);
        cv::cvtColor(lab, out, cv::COLOR_Lab2BGR);
        return out;
    }

private:
    void rebuild_clahe() {
        int ts = (tile_size_ < 1) ? 1 : tile_size_;
        clahe_ = cv::createCLAHE(clip_limit_, {ts, ts});
    }

    double clip_limit_ = 2.0;
    int tile_size_ = 8;
    cv::Ptr<cv::CLAHE> clahe_;
};

FV_FILTER_MAIN(CLAHEFilter)
