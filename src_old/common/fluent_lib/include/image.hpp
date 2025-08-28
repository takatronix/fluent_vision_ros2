#pragma once

#include <opencv2/opencv.hpp>
#include <string>

/**
 * @brief Beautiful Image processing API
 */
class Image {
public:
    // ファクトリ
    static Image load(const std::string& path);
    static Image from(const cv::Mat& mat);
    static Image create(int width, int height, int type = CV_8UC3);
    
    // 基本操作
    Image& resize(int width, int height);
    Image& resize(double scale);
    Image& crop(int x, int y, int w, int h);
    Image& flip(int code = 1);
    Image& rotate(double angle);
    
    // フィルタ
    Image& blur(int size = 15);
    Image& sharp();
    Image& bright(double alpha = 1.2);
    Image& contrast(double beta = 30);
    Image& gamma(double gamma = 1.5);
    
    // 描画
    Image& text(const std::string& txt, int x = 10, int y = 30);
    Image& circle(int x, int y, int radius, cv::Scalar color = cv::Scalar(0,255,0));
    Image& rect(int x, int y, int w, int h, cv::Scalar color = cv::Scalar(0,255,0));
    Image& line(int x1, int y1, int x2, int y2, cv::Scalar color = cv::Scalar(0,255,0));
    
    // 出力
    void save(const std::string& path);
    void show(const std::string& window = "Image");
    cv::Mat& mat() { return img_; }
    const cv::Mat& mat() const { return img_; }

private:
    cv::Mat img_;
    explicit Image(const cv::Mat& mat) : img_(mat) {}
};