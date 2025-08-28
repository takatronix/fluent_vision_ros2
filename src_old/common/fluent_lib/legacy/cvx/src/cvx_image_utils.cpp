#include "cvx/cvx_image_utils.hpp"
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace cvx {
std::vector<float> preprocessToCHW(const cv::Mat& src, int out_ch, int out_h, int out_w) {
    cv::Mat img = src.clone();
    // 色変換
    if (img.channels() == 3 && out_ch == 1) {
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    } else if (img.channels() == 1 && out_ch == 3) {
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    } else if (img.channels() == 3 && out_ch == 3) {
        cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    }
    // リサイズ
    cv::resize(img, img, cv::Size(out_w, out_h));
    img.convertTo(img, CV_32F, 1.0/255.0);

    // 16ch拡張
    if (out_ch == 16) {
        std::vector<cv::Mat> channels(16, img);
        cv::merge(channels, img); // [H,W,16]
    }

    std::vector<float> blob_data(out_ch * out_h * out_w);
    if (out_ch == 1) {
        for (int h = 0; h < out_h; ++h)
            for (int w = 0; w < out_w; ++w)
                blob_data[h * out_w + w] = img.at<float>(h, w);
    } else if (out_ch == 3) {
        for (int c = 0; c < 3; ++c)
            for (int h = 0; h < out_h; ++h)
                for (int w = 0; w < out_w; ++w)
                    blob_data[c * out_h * out_w + h * out_w + w] = img.at<cv::Vec3f>(h, w)[c];
    } else if (out_ch == 16) {
        for (int c = 0; c < 16; ++c)
            for (int h = 0; h < out_h; ++h)
                for (int w = 0; w < out_w; ++w)
                    blob_data[c * out_h * out_w + h * out_w + w] = img.at<cv::Vec<float,16>>(h, w)[c];
    } else {
        throw std::runtime_error("Unsupported output channel count in preprocessToCHW");
    }
    return blob_data;
}
}
