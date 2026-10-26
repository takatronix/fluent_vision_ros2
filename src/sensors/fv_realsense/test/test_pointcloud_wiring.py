#!/usr/bin/env python3
"""Compile the production RGB sampler with pinhole SDK/OpenCV test doubles.

No ROS, SDK, OpenCV or device is needed. This verifies caller wiring and bounds,
not librealsense distortion handling or the full node ABI/build.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / 'src/fv_realsense_node.cpp').read_text()


def function(text, start):
    begin = text.index(start)
    brace = text.index('{', begin)
    level = 1
    end = brace + 1
    while level:
        level += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[begin:end]


class PointcloudWiringTest(unittest.TestCase):
    def test_source_frame_and_time(self):
        cloud = function(SOURCE, 'std::unique_ptr<sensor_msgs::msg::PointCloud2> FVDepthCameraNode::buildPointCloud(')
        self.assertIn('cloud_msg->header.stamp = stamp;', cloud)
        self.assertIn('cloud_msg->header.frame_id = tf_config_.depth_optical_frame;', cloud)
        publish = function(SOURCE, 'void FVDepthCameraNode::queueFramesForPublish(')
        self.assertIn('buildPointCloud(color_frame, depth_frame, stamp)', publish)
        self.assertIn('cloud_msg.header.frame_id = tf_config_.depth_optical_frame;', publish)
        self.assertIn('cloud_msg.width = fv_realsense::sampledExtent(dw, step);', publish)
        self.assertIn('cloud_msg.height = fv_realsense::sampledExtent(dh, step);', publish)
        for body in (cloud, publish):
            self.assertIn('depth_frame.get_profile().get_extrinsics_to(color_frame.get_profile())', body)
            self.assertIn('pointColor(color_image, color_intrinsics_, depth_to_color, point)', body)

    def test_production_rgb_sampler(self):
        preamble = r'''
#include "fv_realsense/pointcloud_geometry.hpp"
#include <cassert>
#include <limits>
#include <vector>
namespace cv {
struct Vec3b {
    unsigned char b, g, r;
    Vec3b(int b_=0, int g_=0, int r_=0): b(b_), g(g_), r(r_) {}
};
struct Mat {
    int rows, cols;
    std::vector<Vec3b> pixels;
    Mat(int h, int w): rows(h), cols(w), pixels(h*w) {}
    template<class T> T at(int v, int u) const {
        assert(v >= 0 && v < rows && u >= 0 && u < cols);
        return pixels.at(v*cols+u);
    }
};
}
struct rs2_intrinsics { float fx, fy, ppx, ppy; };
struct rs2_extrinsics { float translation[3]; };
void rs2_transform_point_to_point(float out[3], const rs2_extrinsics* e, const float in[3]) {
    for (int i=0; i<3; ++i) out[i] = in[i] + e->translation[i];
}
void rs2_project_point_to_pixel(float out[2], const rs2_intrinsics* k, const float in[3]) {
    out[0] = k->fx*in[0]/in[2]+k->ppx;
    out[1] = k->fy*in[1]/in[2]+k->ppy;
}
'''
        checks = r'''
int main() {
    cv::Mat color(4, 6);
    color.pixels[2*6+3] = cv::Vec3b(11,22,33);
    const rs2_intrinsics k{10,10,2,2};
    const rs2_extrinsics e{{0.1f,0,0}};
    const float center[3] = {0,0,1};
    const auto found = pointColor(color,k,e,center);
    assert(found.b == 11 && found.g == 22 && found.r == 33);
    const float cases[][3] = {{1000,600,1}, {-1000,0,1}, {0,0,-1}, {0,0,0},
      {std::numeric_limits<float>::quiet_NaN(),0,1},
      {0,0,std::numeric_limits<float>::infinity()}};
    for (const auto& point : cases) {
        const auto black = pointColor(color,k,e,point);
        assert(black.b == 0 && black.g == 0 && black.r == 0);
    }
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / 'sampler.cpp'
            binary = Path(tmp) / 'sampler'
            source.write_text(preamble + function(SOURCE, 'cv::Vec3b pointColor(') + checks)
            subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', '-O1', '-g',
                            '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                            '-fno-omit-frame-pointer', '-I', str(PACKAGE / 'include'),
                            str(source), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
