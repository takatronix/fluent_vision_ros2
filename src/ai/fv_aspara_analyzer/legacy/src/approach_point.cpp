// SPDX-License-Identifier: MIT
#include "fv_aspara_analyzer/approach_point.hpp"
#include "fv_aspara_analyzer/aspara_info.hpp"
#include <algorithm>
#include <cmath>

namespace fv_aspara_analyzer {

static inline float clampf(float v, float lo, float hi) {
	return std::max(lo, std::min(hi, v));
}

cv::Point2f computeApproachPoint2D(
	const cv::Rect& bbox,
	const cv::Size& image_size,
	const ApproachParams& p)
{
	if (bbox.width <= 0 || bbox.height <= 0 || image_size.width <= 0 || image_size.height <= 0) {
		return cv::Point2f(-1.f, -1.f);
	}
    // 中心は常に厳密な半端値で計算（丸めを最後に行う）
    float cx = static_cast<float>(bbox.x) + static_cast<float>(bbox.width) * 0.5f;
    float cy = static_cast<float>(bbox.y) + static_cast<float>(bbox.height) * 0.5f;
	float bottom = static_cast<float>(bbox.y + bbox.height);
	float y = cy;
	switch (p.y_mode) {
		case ApproachYMode::CENTER_TO_BOTTOM_RATIO:
			y = cy + p.y_ratio * (bottom - cy);
			break;
		case ApproachYMode::CENTER:
            y = cy; // センター指定時は余計な補正を一切行わない
			break;
		case ApproachYMode::ROOT_SCANLINE:
			// 既定はcenter_to_bottom_ratioと同様に扱い、のちに差し替え可能
			y = cy + p.y_ratio * (bottom - cy);
			break;
	}
	cx += static_cast<float>(p.x_offset_px);
	y  += static_cast<float>(p.y_offset_px);
	float u = clampf(cx, 0.0f, static_cast<float>(image_size.width  - 1));
	float v = clampf(y,  0.0f, static_cast<float>(image_size.height - 1));
	return cv::Point2f(u, v);
}

static inline bool fetchDepthMeters(
	const cv::Mat& depth,
	int x, int y,
	double depth_scale_m_16u,
	float& out_m)
{
	if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows) return false;
	if (depth.type() == CV_16UC1) {
		uint16_t raw = depth.at<uint16_t>(y, x);
		if (raw == 0) return false;
		out_m = static_cast<float>(raw * depth_scale_m_16u);
		return std::isfinite(out_m) && out_m > 0.0f;
	} else if (depth.type() == CV_32FC1) {
		float m = depth.at<float>(y, x);
		if (!std::isfinite(m) || m <= 0.0f) return false;
		out_m = m;
		return true;
	}
	return false;
}

bool computeApproachPoint3D(
	const cv::Point2f& pixel_color,
	const cv::Mat& depth_image,
	double depth_scale_m_16u,
	const sensor_msgs::msg::CameraInfo& cam,
	const DepthParams& dp,
	geometry_msgs::msg::Point& out_world)
{
	out_world.x = out_world.y = out_world.z = std::numeric_limits<double>::quiet_NaN();
	if (pixel_color.x < 0 || pixel_color.y < 0 || depth_image.empty()) return false;

	// サンプリング窓半径
	int r = std::max(0, dp.window_px);
	std::vector<float> samples;
	samples.reserve((2*r+1)*(2*r+1));
	int cx = static_cast<int>(std::round(pixel_color.x));
	int cy = static_cast<int>(std::round(pixel_color.y));

	auto collect = [&](int radius){
		for (int dv = -radius; dv <= radius; ++dv) {
			for (int du = -radius; du <= radius; ++du) {
				float z;
				if (fetchDepthMeters(depth_image, cx + du, cy + dv, depth_scale_m_16u, z)) {
					if (dp.max_range_m <= 0.0f || z <= dp.max_range_m) samples.push_back(z);
				}
			}
		}
	};

	collect(r);
	if (static_cast<int>(samples.size()) < dp.min_valid && dp.search_expand_px > 0) {
		collect(r + dp.search_expand_px);
	}
	if (static_cast<int>(samples.size()) < std::max(1, dp.min_valid)) return false;

	float z = 0.0f;
	if (dp.strategy == "mean") {
		for (float v : samples) z += v; z = z / static_cast<float>(samples.size());
	} else {
		// median
		size_t m = samples.size()/2;
		std::nth_element(samples.begin(), samples.begin()+m, samples.end());
		z = samples[m];
	}

	// バックプロジェクション（カラー内参前提）
	double fx = cam.k[0], fy = cam.k[4], cx_cam = cam.k[2], cy_cam = cam.k[5];
	if (!(fx > 0.0) || !(fy > 0.0)) return false;
	double x_norm = (static_cast<double>(cx) - cx_cam) / fx;
	double y_norm = (static_cast<double>(cy) - cy_cam) / fy;
	out_world.x = static_cast<double>(z) * x_norm;
	out_world.y = static_cast<double>(z) * y_norm;
	out_world.z = static_cast<double>(z);
	return std::isfinite(out_world.x) && std::isfinite(out_world.y) && std::isfinite(out_world.z);
}

void drawApproachPoint(
	cv::Mat& image,
	const cv::Point2f& pixel,
	const geometry_msgs::msg::Point* world,
	bool world_valid,
	const OverlayParams& op)
{
	if (!op.enable) return;
	if (pixel.x < 0 || pixel.y < 0 || pixel.x >= image.cols || pixel.y >= image.rows) return;
	cv::circle(image, pixel, op.radius, op.color_bgr, op.thickness, cv::LINE_AA);
	if (op.debug_xyz_enable && world && world_valid) {
		char buf[128];
		std::snprintf(buf, sizeof(buf), "x=%.3f y=%.3f z=%.3f", world->x, world->y, world->z);
		int base_x = static_cast<int>(std::round(pixel.x)) + op.radius + 6;
		int base_y = static_cast<int>(std::round(pixel.y)) + 4;
		cv::putText(image, buf, cv::Point(base_x, base_y), cv::FONT_HERSHEY_SIMPLEX, op.debug_xyz_font_scale, op.debug_xyz_color_bgr, op.debug_xyz_thickness, cv::LINE_AA);
	}
}

} // namespace fv_aspara_analyzer


