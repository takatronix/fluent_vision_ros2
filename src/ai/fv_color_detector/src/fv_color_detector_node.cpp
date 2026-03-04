/**
 * fv_color_detector_node -- HSV color-based object detection with depth distance
 *
 * Detects objects by HSV color filtering, computes bounding boxes via contour
 * analysis, and looks up distance from aligned depth image.
 *
 * Pipeline:  color image -> HSV -> inRange -> morphology -> contours -> bbox
 *            depth image -> distance lookup at bbox centre
 *
 * Modes:
 *   color+depth : depth callback drives processing, color callback caches only
 *   color-only  : wall timer drives processing, color callback caches only
 *
 * Outputs:   overlay image (BGR8) with red bounding boxes + distance/size info
 *            detections as JSON on std_msgs/String topic
 */

#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#if __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#elif __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge/cv_bridge.hpp>
#else
#error "cv_bridge header not found"
#endif

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

template <typename T>
static T get_param(rclcpp::Node& node, const std::string& name, const T& def) {
    node.declare_parameter<T>(name, def);
    return node.get_parameter(name).template get_value<T>();
}

class FvColorDetectorNode : public rclcpp::Node {
public:
    FvColorDetectorNode() : Node("fv_color_detector_node") {
        // Topic parameters
        color_topic_  = get_param(*this, "color_topic",  std::string("~/color/image_raw"));
        depth_topic_  = get_param(*this, "depth_topic",  std::string("~/depth/image_rect_raw"));
        overlay_topic_    = get_param(*this, "overlay_topic",    std::string("~/overlay"));
        detections_topic_ = get_param(*this, "detections_topic", std::string("~/detections"));

        // HSV range parameters
        h_min_ = get_param(*this, "h_min", 0);
        h_max_ = get_param(*this, "h_max", 180);
        s_min_ = get_param(*this, "s_min", 80);
        s_max_ = get_param(*this, "s_max", 255);
        v_min_ = get_param(*this, "v_min", 80);
        v_max_ = get_param(*this, "v_max", 255);

        // Detection parameters
        min_area_    = get_param(*this, "min_area", 500);
        depth_scale_ = get_param(*this, "depth_scale", 0.0001);
        depth_roi_radius_ = get_param(*this, "depth_roi_radius", 5);

        // Aspect ratio filter (w/h)
        aspect_min_ = get_param(*this, "aspect_min", 0.5);
        aspect_max_ = get_param(*this, "aspect_max", 2.0);

        // Rate limiting
        double process_fps = get_param(*this, "process_fps", 15.0);
        if (process_fps > 0.0) {
            min_interval_ns_ = static_cast<int64_t>(1.0e9 / process_fps);
        }

        // Morphology kernel size
        morph_size_ = get_param(*this, "morph_size", 5);

        // Depth is optional: empty or "none" = color-only mode
        depth_enabled_ = !depth_topic_.empty() && depth_topic_ != "none";

        // Subscribe (SensorDataQoS for all image topics)
        auto qos = rclcpp::SensorDataQoS();
        sub_color_ = create_subscription<sensor_msgs::msg::Image>(
            color_topic_, qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr m) { on_color(m); });

        if (depth_enabled_) {
            sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
                depth_topic_, qos,
                [this](sensor_msgs::msg::Image::ConstSharedPtr m) { on_depth(m); });
        } else {
            // Color-only: timer drives processing (keeps on_color callback lightweight)
            double fps = (min_interval_ns_ > 0)
                ? 1.0e9 / static_cast<double>(min_interval_ns_) : 15.0;
            auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / fps));
            timer_ = create_wall_timer(period, [this]() { on_timer(); });
        }

        // Publish (SensorDataQoS)
        auto pub_qos = rclcpp::SensorDataQoS();
        pub_overlay_    = create_publisher<sensor_msgs::msg::Image>(overlay_topic_, pub_qos);
        pub_detections_ = create_publisher<std_msgs::msg::String>(detections_topic_, pub_qos);

        RCLCPP_INFO(get_logger(),
            "FvColorDetectorNode: color=%s depth=%s overlay=%s "
            "HSV=[%d-%d, %d-%d, %d-%d] min_area=%d aspect=[%.2f-%.2f] "
            "depth_scale=%.6f fps=%.1f mode=%s",
            color_topic_.c_str(), depth_topic_.c_str(), overlay_topic_.c_str(),
            h_min_, h_max_, s_min_, s_max_, v_min_, v_max_,
            min_area_, aspect_min_, aspect_max_, depth_scale_, process_fps,
            depth_enabled_ ? "color+depth" : "color-only");
    }

private:
    // Color callback: always lightweight (cache only)
    void on_color(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(color_mtx_);
        latest_color_ = msg;
    }

    // Color-only mode: timer drives processing
    void on_timer() {
        sensor_msgs::msg::Image::ConstSharedPtr color_msg;
        {
            std::lock_guard<std::mutex> lk(color_mtx_);
            if (!latest_color_) return;
            color_msg = latest_color_;
        }
        process(color_msg, nullptr);
    }

    // Depth mode: depth callback drives processing
    void on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
        if (min_interval_ns_ > 0 && (now_ns - last_process_ns_) < min_interval_ns_) {
            return;
        }
        last_process_ns_ = now_ns;

        sensor_msgs::msg::Image::ConstSharedPtr color_msg;
        {
            std::lock_guard<std::mutex> lk(color_mtx_);
            if (!latest_color_) return;
            color_msg = latest_color_;
        }
        process(color_msg, msg);
    }

    void process(sensor_msgs::msg::Image::ConstSharedPtr color_msg,
                 sensor_msgs::msg::Image::ConstSharedPtr depth_msg) {
        cv_bridge::CvImageConstPtr cv_color, cv_depth;
        try {
            cv_color = cv_bridge::toCvShare(color_msg, "bgr8");
            if (depth_msg) {
                cv_depth = cv_bridge::toCvShare(depth_msg);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "cv_bridge error: %s", e.what());
            return;
        }

        const cv::Mat& bgr = cv_color->image;
        if (bgr.empty()) return;

        cv::Mat depth;
        if (cv_depth && !cv_depth->image.empty()) {
            depth = cv_depth->image;
        }

        // HSV filtering
        cv::Mat hsv, mask;
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

        if (h_min_ <= h_max_) {
            cv::inRange(hsv,
                cv::Scalar(h_min_, s_min_, v_min_),
                cv::Scalar(h_max_, s_max_, v_max_),
                mask);
        } else {
            // Wrap-around hue (e.g. red: h_min=170, h_max=10)
            cv::Mat mask1, mask2;
            cv::inRange(hsv,
                cv::Scalar(h_min_, s_min_, v_min_),
                cv::Scalar(180, s_max_, v_max_),
                mask1);
            cv::inRange(hsv,
                cv::Scalar(0, s_min_, v_min_),
                cv::Scalar(h_max_, s_max_, v_max_),
                mask2);
            mask = mask1 | mask2;
        }

        // Morphology: open (remove noise) then close (fill gaps)
        if (morph_size_ > 0) {
            cv::Mat kernel = cv::getStructuringElement(
                cv::MORPH_RECT, cv::Size(morph_size_, morph_size_));
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        }

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::sort(contours.begin(), contours.end(),
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return cv::contourArea(a) > cv::contourArea(b);
            });

        // Overlay + detections
        cv::Mat overlay = bgr.clone();
        std::ostringstream json_arr;

        int det_count = 0;
        const cv::Scalar RED(0, 0, 255);
        const cv::Scalar WHITE(255, 255, 255);
        const cv::Scalar BG(0, 0, 160);

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < static_cast<double>(min_area_)) continue;

            cv::Rect bbox = cv::boundingRect(contour);

            double aspect = (bbox.height > 0)
                ? static_cast<double>(bbox.width) / static_cast<double>(bbox.height)
                : 0.0;
            if (aspect < aspect_min_ || aspect > aspect_max_) continue;

            int cx = bbox.x + bbox.width / 2;
            int cy = bbox.y + bbox.height / 2;

            float distance_m = depth.empty() ? 0.0f : lookup_depth(depth, cx, cy);

            // Bounding box + crosshair
            cv::rectangle(overlay, bbox, RED, 2);
            int ch = 6;
            cv::line(overlay, cv::Point(cx - ch, cy), cv::Point(cx + ch, cy), RED, 1);
            cv::line(overlay, cv::Point(cx, cy - ch), cv::Point(cx, cy + ch), RED, 1);

            // Info text
            char line1[96], line2[96];
            if (distance_m > 0.0f) {
                std::snprintf(line1, sizeof(line1), "CUBE  %.3f m", distance_m);
            } else {
                std::snprintf(line1, sizeof(line1), "CUBE  --- m");
            }
            std::snprintf(line2, sizeof(line2), "%dx%d  ar=%.2f  area=%d",
                bbox.width, bbox.height, aspect, static_cast<int>(area));

            draw_label(overlay, line1, bbox.x, bbox.y - 22, WHITE, BG);
            draw_label(overlay, line2, bbox.x, bbox.y - 4, WHITE, BG);

            // JSON
            if (det_count > 0) json_arr << ",";
            json_arr << "{\"id\":" << det_count
                     << ",\"x\":" << cx << ",\"y\":" << cy
                     << ",\"w\":" << bbox.width << ",\"h\":" << bbox.height
                     << ",\"aspect\":" << aspect
                     << ",\"area\":" << static_cast<int>(area)
                     << ",\"distance_m\":" << (distance_m > 0.0f ? distance_m : 0.0f)
                     << "}";
            det_count++;
        }

        // Publish overlay with current timestamp
        auto hdr = color_msg->header;
        hdr.stamp = now();
        auto overlay_msg = cv_bridge::CvImage(hdr, "bgr8", overlay).toImageMsg();
        pub_overlay_->publish(*overlay_msg);

        // Publish detections JSON
        std_msgs::msg::String det_msg;
        det_msg.data = "{\"count\":" + std::to_string(det_count)
                     + ",\"detections\":[" + json_arr.str() + "]}";
        pub_detections_->publish(det_msg);
    }

    static void draw_label(cv::Mat& img, const char* text,
                           int x, int y,
                           const cv::Scalar& fg, const cv::Scalar& bg) {
        int baseline = 0;
        double scale = 0.5;
        int thickness = 1;
        cv::Size sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX,
                                      scale, thickness, &baseline);
        cv::Point org(x, y);
        if (org.y < sz.height + 2) org.y = sz.height + 2;

        cv::rectangle(img,
            cv::Point(org.x - 1, org.y - sz.height - 2),
            cv::Point(org.x + sz.width + 3, org.y + baseline + 2),
            bg, cv::FILLED);
        cv::putText(img, text, org,
            cv::FONT_HERSHEY_SIMPLEX, scale, fg, thickness);
    }

    float lookup_depth(const cv::Mat& depth, int cx, int cy) {
        const int r = depth_roi_radius_;
        int y0 = std::max(0, cy - r);
        int y1 = std::min(depth.rows - 1, cy + r);
        int x0 = std::max(0, cx - r);
        int x1 = std::min(depth.cols - 1, cx + r);

        std::vector<float> values;
        values.reserve(static_cast<size_t>((2 * r + 1) * (2 * r + 1)));

        if (depth.type() == CV_16UC1) {
            for (int y = y0; y <= y1; ++y) {
                const uint16_t* ptr = depth.ptr<uint16_t>(y);
                for (int x = x0; x <= x1; ++x) {
                    float d = static_cast<float>(ptr[x]) * static_cast<float>(depth_scale_);
                    if (d > 0.01f && d < 10.0f) values.push_back(d);
                }
            }
        } else if (depth.type() == CV_32FC1) {
            for (int y = y0; y <= y1; ++y) {
                const float* ptr = depth.ptr<float>(y);
                for (int x = x0; x <= x1; ++x) {
                    if (ptr[x] > 0.01f && ptr[x] < 10.0f) values.push_back(ptr[x]);
                }
            }
        }

        if (values.empty()) return 0.0f;

        size_t mid = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + static_cast<long>(mid), values.end());
        return values[mid];
    }

    // Parameters
    std::string color_topic_, depth_topic_, overlay_topic_, detections_topic_;
    int h_min_, h_max_, s_min_, s_max_, v_min_, v_max_;
    int min_area_;
    double depth_scale_;
    int depth_roi_radius_;
    int morph_size_;
    double aspect_min_, aspect_max_;
    int64_t min_interval_ns_ = 0;
    int64_t last_process_ns_ = 0;

    bool depth_enabled_;

    // Color frame cache (written by on_color, read by on_timer/on_depth)
    std::mutex color_mtx_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_color_;

    // ROS handles
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_overlay_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_detections_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FvColorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
