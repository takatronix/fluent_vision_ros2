# FluentVision - 流れるような画像処理ライブラリ

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue" alt="ROS2 Humble">
  <img src="https://img.shields.io/badge/C%2B%2B-17-green" alt="C++17">
  <img src="https://img.shields.io/badge/OpenCV-4.x-red" alt="OpenCV 4.x">
</p>

## 🌊 概要

FluentVisionは、**流れるように美しく**画像処理を記述できるC++ライブラリです。

```cpp
// 思考の流れがそのままコードに
auto result = FluentVision::from(image)
    .blur(2.0)
    .edge(0.2)
    .color(1.2, 1.1)
    .drawText("Detected", Point(10, 30), GREEN)
    .toImageMsg();
```

## ✨ 特徴

- **🚀 高速** - C++による最適化、Pythonの100倍速
- **🔗 チェーン可能** - メソッドチェーンで直感的な記述
- **🤖 ROS2対応** - sensor_msgs::msg::Imageとシームレスに連携
- **🎮 Unity/Web対応** - WebSocket/gRPC経由で簡単接続
- **📦 依存最小** - OpenCVとROS2のみ

## 📦 インストール

### 前提条件
- ROS2 Humble
- OpenCV 4.x
- C++17対応コンパイラ

### ビルド方法
```bash
cd ~/fluent_vision_ros2
colcon build --packages-select cvx fluent_vision
source install/setup.bash
```

## 🚀 クイックスタート

### 基本的な使い方

```cpp
#include <fluent_vision/fluent_vision.hpp>

using namespace fluent_vision;

// ROS2ノードでの使用例
void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    auto processed = FluentVision::from(msg)
        .gaussianBlur(2.0)
        .canny(100, 200)
        .toImageMsg();
    
    publisher_->publish(processed);
}
```

### 高度な使い方

```cpp
// 顔検出してぼかし
auto result = FluentVision::from(input)
    .detectFaces()
    .branch([](FluentVision& img) {
        // 顔領域だけぼかし
        return img.blur(5.0);
    })
    .drawText("Privacy Protected", Point(10, 30))
    .toMat();

// GPU使用
auto fast_result = FluentVision::from(input)
    .useGPU()
    .denoise()
    .cartoon()
    .toMat();
```

## 📚 API リファレンス

### ファクトリメソッド
- `FluentVision::from(cv::Mat)` - OpenCVのMatから作成
- `FluentVision::from(sensor_msgs::msg::Image)` - ROS2メッセージから作成

### フィルタ操作
- `blur(radius)` - ぼかし
- `gaussianBlur(sigma)` - ガウシアンぼかし
- `medianBlur(kernel_size)` - メディアンフィルタ
- `bilateralFilter(sigma_color, sigma_space)` - バイラテラルフィルタ

### エッジ検出
- `edge(threshold)` - 簡易エッジ検出
- `canny(low, high)` - Cannyエッジ検出
- `sobel(scale)` - Sobelフィルタ
- `laplacian(scale)` - ラプラシアンフィルタ

### 色調整
- `color(brightness, contrast, saturation)` - 一括調整
- `brightness(factor)` - 明度調整
- `contrast(factor)` - コントラスト調整
- `saturation(factor)` - 彩度調整
- `gamma(value)` - ガンマ補正

### 描画
- `drawRectangle(rect, color, thickness)` - 矩形描画
- `drawLine(pt1, pt2, color, thickness)` - 直線描画
- `drawCircle(center, radius, color, thickness)` - 円描画
- `drawText(text, pos, color, scale, thickness)` - テキスト描画

### 変換
- `resize(size)` / `resize(scale)` - リサイズ
- `rotate(angle)` - 回転
- `flip(horizontal, vertical)` - 反転
- `crop(roi)` - クロップ

### 出力
- `toMat()` - cv::Matとして取得
- `toImageMsg()` - ROS2メッセージとして取得
- `save(filename)` - ファイルに保存
- `show(window_name)` - 表示（デバッグ用）

## 🎯 使用例

### 1. リアルタイムフィルタ
```cpp
class FilterNode : public rclcpp::Node {
    void processImage(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto result = FluentVision::from(msg)
            .gaussianBlur(1.5)
            .color(1.2, 1.1, 0.9)  // 暖色系に
            .toImageMsg();
        
        publisher_->publish(result);
    }
};
```

### 2. 物体検出の前処理
```cpp
auto preprocessed = FluentVision::from(raw_image)
    .resize(640, 480)          // YOLOの入力サイズに
    .histogram_equalization()  // コントラスト改善
    .denoise()                 // ノイズ除去
    .toMat();
```

### 3. AR表示用の処理
```cpp
auto ar_ready = FluentVision::from(camera_feed)
    .detectMarkers()           // ARマーカー検出
    .drawBoundingBoxes()       // 検出結果を描画
    .blend(virtual_object, 0.7) // 仮想オブジェクトを合成
    .toMat();
```

## 🔧 設定

### CMakeLists.txt
```cmake
find_package(fluent_vision REQUIRED)
target_link_libraries(your_node fluent_vision::fluent_vision)
```

### package.xml
```xml
<depend>fluent_vision</depend>
```

## 🌐 Unity/Web連携

### Unity統合
```csharp
// Unity側
var vision = new FluentVisionConnector("ros2://localhost");
vision.ProcessImage(texture)
    .OnResult(result => UpdateAROverlay(result));
```

### Web統合
```javascript
// ブラウザ側
const vision = new FluentVisionWeb('ws://localhost:9090');
vision.from(webcam)
    .detectFaces()
    .blur(2.0)
    .subscribe(result => canvas.draw(result));
```

## 🏗️ アーキテクチャ

```
FluentVision (高レベルAPI)
    ↓
CVX Library (チェーン可能な画像処理)
    ↓
OpenCV (低レベル実装)
```

## 🤝 貢献

プルリクエスト歓迎です！

## 📄 ライセンス

Apache License 2.0

## 🙏 謝辞

- OpenCVチーム
- ROS2コミュニティ
- cvxライブラリの作者

---

**"Make Image Processing Flow Like Water"** 🌊