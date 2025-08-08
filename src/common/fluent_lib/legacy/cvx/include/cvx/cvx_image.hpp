#pragma once
#include <opencv2/opencv.hpp>

namespace cvx {
    // 上位互換Matクラス
    class Mat {
        cv::Mat mat_;
    public:
        // コンストラクタ
        // --- 静的生成ユーティリティ ---
        /// @brief 指定サイズ・型の真っ黒画像（全0）を生成
        static Mat createZeros(cv::Size size, int type = CV_8UC3) { return Mat(cv::Mat::zeros(size, type)); }
        /// @brief 指定サイズ・型の空画像（未初期化）を生成
        static Mat createEmpty(cv::Size size, int type = CV_8UC3) { return Mat(cv::Mat(size, type)); }
        /// @brief 指定サイズ・型・色の単色画像を生成
        static Mat createBlank(cv::Size size, cv::Scalar color, int type = CV_8UC3) {
            cv::Mat m(size, type, color);
            return Mat(m);
        }

        Mat();
        Mat(const cv::Mat& m);
        Mat(cv::Mat&& m);
        // 暗黙変換
        operator cv::Mat() const { return mat_; }
        cv::Mat& get() { return mat_; }
        const cv::Mat& get() const { return mat_; }
        // 基本操作
        Mat clone() const;
        cv::Size size() const;
        int type() const;
        int channels() const;
        /// @brief 画像情報を標準出力に表示（デバッグ用）
        void printInfo(const std::string& label = "") const;
        /// @brief OpenCV type()値からCV_8UC3等の型文字列を返す
        static std::string typeStr(int type);
        /// @brief チャンネル数や型からBGR/BGRA/GRAYなどの色空間名を返す
        static std::string colorStr(int type, int channels);
        // 画像処理
        Mat toGray() const;                 // BGR/カラー→グレースケール
        Mat toColor() const;                // グレースケール→BGRカラー
        Mat toBGR() const;                  // 任意→BGR
        Mat toRGB() const;                  // 任意→RGB
        Mat toHSV() const;                  // 任意→HSV
        Mat toLab() const;                  // 任意→Lab
        Mat toBGRA() const;                 // 任意→BGRA
        Mat toRGBA() const;                 // 任意→RGBA
        Mat toYCrCb() const;                // 任意→YCrCb
        Mat toXYZ() const;                  // 任意→XYZ
        Mat toLuv() const;                  // 任意→Luv
        Mat toHLS() const;                  // 任意→HLS
        Mat toGray16() const;               // 16bitグレースケール
        Mat toGray32() const;               // 32bitグレースケール
        Mat toFloat() const;                // float型変換
        Mat toU8() const;                   // 8bit unsigned
        Mat toU16() const;                  // 16bit unsigned
        Mat toS16() const;                  // 16bit signed
        Mat to32F() const;                  // 32bit float
        Mat to64F() const;                  // 64bit float
        /// @brief 画像を平均値フィルタで平滑化
        Mat blur(int ksize=3) const;
        /// @brief ガウシアンフィルタ
        Mat gaussianBlur(int ksize=3, double sigma=1.0) const;
        /// @brief メディアンフィルタ
        Mat medianBlur(int ksize=3) const;
        /// @brief バイラテラルフィルタ
        Mat bilateralFilter(int d=9, double sigmaColor=75.0, double sigmaSpace=75.0) const;
        /// @brief ラプラシアンシャープ
        Mat laplacian(int ksize=3) const;
        /// @brief Sobelエッジ検出
        Mat sobel(int dx, int dy, int ksize=3) const;
        /// @brief Scharrエッジ検出
        Mat scharr(int dx, int dy) const;
        /// @brief Cannyエッジ検出
        Mat canny(double th1, double th2) const;
        /// @brief 2値化
        Mat threshold(double thresh, double maxval, int type) const;
        /// @brief アダプティブ2値化
        Mat adaptiveThreshold(double maxval, int method, int type, int blockSize, double C) const;
        /// @brief 膨張
        Mat dilate(int ksize=3) const;
        /// @brief 収縮
        Mat erode(int ksize=3) const;
        /// @brief オープニング
        Mat morphologyOpen(int ksize=3) const;
        /// @brief クロージング
        Mat morphologyClose(int ksize=3) const;
        /// @brief 幾何変換（アフィン）
        Mat warpAffine(const cv::Mat& M, const cv::Size& dsize) const;
        /// @brief 幾何変換（射影）
        Mat warpPerspective(const cv::Mat& M, const cv::Size& dsize) const;
        /// @brief 画像の量子化（色数削減）
        Mat quantize(int levels) const;
        /// @brief 画像加算
        Mat add(const Mat& other, double alpha=1.0, double beta=1.0, double gamma=0.0) const;
        /// @brief 画像減算
        Mat subtract(const Mat& other) const;
        /// @brief 画像乗算
        Mat multiply(const Mat& other, double scale=1.0) const;
        /// @brief 画像の重み付き合成（ブレンド）
        Mat blend(const Mat& other, double alpha=0.5) const;
        /// @brief AND論理演算
        Mat bitwiseAnd(const Mat& other) const;
        /// @brief OR論理演算
        Mat bitwiseOr(const Mat& other) const;
        /// @brief XOR論理演算
        Mat bitwiseXor(const Mat& other) const;
        /// @brief NOT論理演算
        Mat bitwiseNot() const;
        Mat normalize(double min=0.0, double max=1.0) const;
        Mat mask(const cv::Mat& mask) const;
        /// @brief 画像にマスクを適用し、マスク部分だけを保持する
        /// @param mask_image マスク画像（0または255の2値画像）
        /// @return マスク部分だけを保持した画像
        Mat applyMask(const Mat& mask_image) const;
        /// @brief 画像にマスクを適用し、マスク部分だけを保持する
        /// @param mask_image マスク画像（0または255の2値画像）
        /// @return マスク部分だけを保持した画像
        Mat applyMask(const cv::Mat& mask_image) const;
    };

    // 既存の関数宣言も残す
    cv::Mat toGray(const cv::Mat& src);
    cv::Mat toColor(const cv::Mat& src);
    cv::Mat normalize(const cv::Mat& src, double min=0.0, double max=1.0);
    cv::Mat resizeTo(const cv::Mat& src, const cv::Size& size);
    cv::Mat mask(const cv::Mat& src, const cv::Mat& mask);
}
