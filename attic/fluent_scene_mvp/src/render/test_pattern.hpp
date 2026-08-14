#pragma once

// Internal: deterministic synthetic frame inputs shared by the fv_render tool
// and the renderer tests. Input names match the section-11 example scene
// (camera / detections / status_text).

#include <cmath>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "fluent_scene/render/renderer.hpp"

namespace fluent_scene {
namespace render {

struct SyntheticFrame {
    std::vector<uint8_t> camera;
    uint32_t camera_width = 0;
    uint32_t camera_height = 0;
    std::vector<DetectionInstance> detections;
    std::vector<Point2f> path;     // sine sweep, for polyline scenes
    std::vector<Point2f> markers;  // waypoints along the path, for circle scenes
    std::string status_text;

    FrameInputs inputs() const {
        FrameInputs frame;
        frame.images["camera"] = CpuImageView{camera_width, camera_height, camera.data()};
        frame.detections["detections"] = detections;
        frame.points["path"] = path;
        frame.points["markers"] = markers;
        frame.strings["status_text"] = status_text;
        return frame;
    }
};

inline SyntheticFrame makeSyntheticFrame(uint32_t frame_index, uint32_t camera_width,
                                         uint32_t camera_height, uint32_t output_width,
                                         uint32_t output_height) {
    SyntheticFrame frame;
    frame.camera_width = camera_width;
    frame.camera_height = camera_height;
    frame.camera.resize(static_cast<size_t>(camera_width) * camera_height * 4);
    const uint32_t bar_x = (frame_index * 8) % camera_width;
    for (uint32_t y = 0; y < camera_height; ++y) {
        uint8_t* row = frame.camera.data() + static_cast<size_t>(y) * camera_width * 4;
        for (uint32_t x = 0; x < camera_width; ++x) {
            uint8_t* pixel = row + static_cast<size_t>(x) * 4;
            const bool in_bar = x >= bar_x && x < bar_x + 24;
            pixel[0] = in_bar ? 0xff : static_cast<uint8_t>((x * 255) / camera_width);
            pixel[1] = in_bar ? 0xff : static_cast<uint8_t>((y * 255) / camera_height);
            pixel[2] = in_bar ? 0xff : static_cast<uint8_t>((x + y + frame_index * 4) & 0xff);
            pixel[3] = 0xff;
        }
    }
    for (uint32_t i = 0; i < 5; ++i) {
        DetectionInstance detection;
        const float fw = static_cast<float>(output_width);
        const float fh = static_cast<float>(output_height);
        detection.bbox[0] = fw * (0.06f + 0.18f * static_cast<float>(i)) +
                            static_cast<float>((frame_index * 3 + i * 7) % 40);
        detection.bbox[1] = fh * (0.25f + 0.12f * static_cast<float>(i % 3));
        detection.bbox[2] = fw * 0.11f;
        detection.bbox[3] = fh * 0.16f;
        detection.score = 0.55f + 0.08f * static_cast<float>(i);
        detection.label = "aspara";
        frame.detections.push_back(detection);
    }
    // Animated sine path across the lower half plus waypoints every 8th vertex.
    const float fw = static_cast<float>(output_width);
    const float fh = static_cast<float>(output_height);
    const float phase = static_cast<float>(frame_index) * 0.06f;
    for (uint32_t i = 0; i <= 48; ++i) {
        const float t = static_cast<float>(i) / 48.0f;
        Point2f point;
        point.x = fw * (0.04f + 0.92f * t);
        point.y = fh * (0.72f + 0.14f * std::sin(t * 9.0f + phase));
        frame.path.push_back(point);
        if (i % 8 == 0) {
            frame.markers.push_back(point);
        }
    }
    frame.status_text = "映像を待っています — アスパラ検出中";
    return frame;
}

inline bool writePpm(const std::string& path, const std::vector<uint8_t>& rgba, uint32_t width,
                     uint32_t height) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return false;
    }
    out << "P6\n" << width << ' ' << height << "\n255\n";
    std::vector<uint8_t> row(static_cast<size_t>(width) * 3);
    for (uint32_t y = 0; y < height; ++y) {
        const uint8_t* src = rgba.data() + static_cast<size_t>(y) * width * 4;
        for (uint32_t x = 0; x < width; ++x) {
            row[x * 3 + 0] = src[x * 4 + 0];
            row[x * 3 + 1] = src[x * 4 + 1];
            row[x * 3 + 2] = src[x * 4 + 2];
        }
        out.write(reinterpret_cast<const char*>(row.data()), static_cast<std::streamsize>(row.size()));
    }
    return static_cast<bool>(out);
}

inline bool readPpm(const std::string& path, std::vector<uint8_t>& rgb, uint32_t& width,
                    uint32_t& height) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }
    std::string magic;
    uint32_t max_value = 0;
    in >> magic >> width >> height >> max_value;
    if (magic != "P6" || max_value != 255) {
        return false;
    }
    in.get();  // single whitespace after the header
    rgb.resize(static_cast<size_t>(width) * height * 3);
    in.read(reinterpret_cast<char*>(rgb.data()), static_cast<std::streamsize>(rgb.size()));
    return static_cast<bool>(in);
}

// Mean and max absolute difference over RGB between an RGBA image and an RGB
// golden of the same dimensions.
inline void diffRgbaVsRgb(const std::vector<uint8_t>& rgba, const std::vector<uint8_t>& rgb,
                          double& mean_diff, uint32_t& max_diff) {
    const size_t pixel_count = rgb.size() / 3;
    double total = 0.0;
    max_diff = 0;
    for (size_t i = 0; i < pixel_count; ++i) {
        for (int c = 0; c < 3; ++c) {
            const int a = rgba[i * 4 + c];
            const int b = rgb[i * 3 + c];
            const uint32_t diff = static_cast<uint32_t>(a > b ? a - b : b - a);
            total += diff;
            max_diff = std::max(max_diff, diff);
        }
    }
    mean_diff = pixel_count > 0 ? total / (static_cast<double>(pixel_count) * 3.0) : 0.0;
}

}  // namespace render
}  // namespace fluent_scene
