/**
 * fv_depth_bilateral_node -- Bilateral filter for depth images
 *
 * Subscribes to a raw depth image and publishes a filtered version.
 * Bilateral filter preserves edges (object boundaries) while smoothing
 * noise on flat surfaces -- ideal as pre-processing for surface normals
 * and HHA encoding.
 *
 * Pipeline:  raw depth -> bilateral -> filtered depth
 *            (optional) -> hole fill (nearest-neighbour)
 */

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

template <typename T>
static T get_param(rclcpp::Node& node, const std::string& name, const T& def) {
    node.declare_parameter<T>(name, def);
    return node.get_parameter(name).template get_value<T>();
}

class FvDepthBilateralNode : public rclcpp::Node {
public:
    FvDepthBilateralNode() : Node("fv_depth_bilateral_node") {
        depth_topic_    = get_param(*this, "depth_topic",    std::string("~/depth/image_rect_raw"));
        filtered_topic_ = get_param(*this, "filtered_topic", std::string("~/filtered_topic"));
        depth_scale_    = get_param(*this, "depth_scale",    0.001);
        // Bilateral params
        bilateral_d_      = get_param(*this, "bilateral_d",      5);
        sigma_color_      = get_param(*this, "sigma_color",      0.04);  // in metres
        sigma_space_      = get_param(*this, "sigma_space",      4.5);   // in pixels
        // Hole fill
        enable_hole_fill_ = get_param(*this, "enable_hole_fill", true);
        hole_fill_radius_ = get_param(*this, "hole_fill_radius", 2);
        // Pass-through for original encoding
        frame_id_         = get_param(*this, "frame_id",         std::string(""));

        auto qos = rclcpp::SensorDataQoS();
        sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr m) { on_depth(m); });

        pub_filtered_ = create_publisher<sensor_msgs::msg::Image>(filtered_topic_, 10);

        RCLCPP_INFO(get_logger(),
            "FvDepthBilateralNode: in=%s out=%s d=%d sigma_c=%.3f sigma_s=%.1f hole_fill=%s",
            depth_topic_.c_str(), filtered_topic_.c_str(),
            bilateral_d_, sigma_color_, sigma_space_,
            enable_hole_fill_ ? "on" : "off");
    }

private:
    void on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        cv_bridge::CvImageConstPtr cv_depth;
        try {
            cv_depth = cv_bridge::toCvShare(msg);
        } catch (...) { return; }

        // Convert to float metres
        cv::Mat depth_f;
        bool was_16u = false;
        if (cv_depth->image.type() == CV_16UC1 || cv_depth->image.type() == CV_16SC1) {
            cv_depth->image.convertTo(depth_f, CV_32F, depth_scale_);
            was_16u = true;
        } else if (cv_depth->image.type() == CV_32FC1) {
            depth_f = cv_depth->image.clone();
        } else {
            return;
        }

        // Replace 0 / NaN with NaN for consistent handling
        for (int r = 0; r < depth_f.rows; ++r) {
            float* ptr = depth_f.ptr<float>(r);
            for (int c = 0; c < depth_f.cols; ++c) {
                if (ptr[c] <= 0.0f || std::isnan(ptr[c])) {
                    ptr[c] = 0.0f;  // bilateral ignores 0
                }
            }
        }

        // --- Hole fill (pre-bilateral) ---
        if (enable_hole_fill_ && hole_fill_radius_ > 0) {
            hole_fill_nearest(depth_f, hole_fill_radius_);
        }

        // --- Bilateral filter ---
        // bilateralFilter requires 8U or 32F input, single channel
        cv::Mat filtered;
        cv::bilateralFilter(depth_f, filtered, bilateral_d_,
                            sigma_color_, sigma_space_);

        // Publish in original encoding
        if (was_16u) {
            // Convert back to 16UC1
            cv::Mat out_16u;
            filtered.convertTo(out_16u, CV_16UC1, 1.0 / depth_scale_);
            auto out_msg = cv_bridge::CvImage(msg->header, "16UC1", out_16u).toImageMsg();
            if (!frame_id_.empty()) out_msg->header.frame_id = frame_id_;
            pub_filtered_->publish(*out_msg);
        } else {
            auto out_msg = cv_bridge::CvImage(msg->header, "32FC1", filtered).toImageMsg();
            if (!frame_id_.empty()) out_msg->header.frame_id = frame_id_;
            pub_filtered_->publish(*out_msg);
        }
    }

    /**
     * Simple nearest-neighbour hole fill.
     * For each pixel with depth==0, search within radius for closest valid pixel.
     */
    static void hole_fill_nearest(cv::Mat& depth, int radius) {
        const int h = depth.rows;
        const int w = depth.cols;
        cv::Mat mask = (depth <= 0.0f);
        if (cv::countNonZero(mask) == 0) return;

        cv::Mat filled = depth.clone();
        for (int r = 0; r < h; ++r) {
            const uchar* mptr = mask.ptr<uchar>(r);
            float* fptr = filled.ptr<float>(r);
            for (int c = 0; c < w; ++c) {
                if (!mptr[c]) continue;
                // Search neighbourhood
                float best = 0.0f;
                int best_dist = radius * radius + 1;
                int r0 = std::max(0, r - radius), r1 = std::min(h - 1, r + radius);
                int c0 = std::max(0, c - radius), c1 = std::min(w - 1, c + radius);
                for (int rr = r0; rr <= r1; ++rr) {
                    const float* dptr = depth.ptr<float>(rr);
                    for (int cc = c0; cc <= c1; ++cc) {
                        if (dptr[cc] <= 0.0f) continue;
                        int d2 = (rr - r) * (rr - r) + (cc - c) * (cc - c);
                        if (d2 < best_dist) {
                            best_dist = d2;
                            best = dptr[cc];
                        }
                    }
                }
                if (best > 0.0f) fptr[c] = best;
            }
        }
        depth = filled;
    }

    // Parameters
    std::string depth_topic_, filtered_topic_, frame_id_;
    double depth_scale_, sigma_color_, sigma_space_;
    int bilateral_d_, hole_fill_radius_;
    bool enable_hole_fill_;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_filtered_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FvDepthBilateralNode>());
    rclcpp::shutdown();
    return 0;
}
