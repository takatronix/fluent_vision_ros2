#include "fv_image_filter/image_filter_base.hpp"

/// angle: rotation angle in degrees (positive = counter-clockwise)
/// For exact 90/180/270, uses fast cv::rotate; otherwise uses warpAffine.
class RotateFilter : public ImageFilterBase {
public:
    RotateFilter() : ImageFilterBase("fv_rotate") {
        angle_ = declare_and_get("angle", 0.0);
        init_filter();
    }

protected:
    void on_parameters_changed(const std::vector<rclcpp::Parameter>& params) override {
        for (auto& p : params) {
            if (p.get_name() == "angle") angle_ = p.as_double();
        }
    }

    cv::Mat apply_filter(const cv::Mat& input) override {
        cv::Mat out;
        int a = static_cast<int>(angle_) % 360;
        if (a < 0) a += 360;

        if (a == 90) {
            cv::rotate(input, out, cv::ROTATE_90_COUNTERCLOCKWISE);
        } else if (a == 180) {
            cv::rotate(input, out, cv::ROTATE_180);
        } else if (a == 270) {
            cv::rotate(input, out, cv::ROTATE_90_CLOCKWISE);
        } else if (a == 0 && angle_ == 0.0) {
            return input.clone();
        } else {
            cv::Point2f center(input.cols / 2.0f, input.rows / 2.0f);
            auto M = cv::getRotationMatrix2D(center, angle_, 1.0);
            cv::warpAffine(input, out, M, input.size());
        }
        return out;
    }

private:
    double angle_ = 0.0;
};

FV_FILTER_MAIN(RotateFilter)
