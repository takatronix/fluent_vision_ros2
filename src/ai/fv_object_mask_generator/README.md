# FV Object Mask Generator

UNetベースのセマンティックセグメンテーションノード

## 概要

OpenVINOを使用したUNetモデルによるリアルタイムセマンティックセグメンテーション

## 主な機能

- UNetモデルによるピクセル単位の分類
- リアルタイム推論（~50ms）
- GPU/CPU推論切り替え対応
- 推論時間・デバイス情報の画像内表示
- モデル名・ファイル名表示

## トピック

### 入力
- `/fv/d415/color/image_raw` - D415カラー画像
- `/fv/d405/color/image_raw` - D405カラー画像

### 出力
- `/fv/d415/segmentation_mask/image` - セグメンテーションマスク（グレースケール）
- `/fv/d415/segmentation_mask/colored` - カラー付きマスク（可視化用）
- `/fv/d405/segmentation_mask/image` - セグメンテーションマスク（グレースケール）
- `/fv/d405/segmentation_mask/colored` - カラー付きマスク（可視化用）

## 設定ファイル

### d415_mask_generator_config.yaml
```yaml
/**:
  ros__parameters:
    input_image_topic: "/fv/d415/color/image_raw"
    output_segmentation_mask_topic: "/fv/d415/segmentation_mask/image"
    output_colored_mask_topic: "/fv/d415/segmentation_mask/colored"
    
    processing_frequency: 10.0  # Hz
    enable_visualization: true
    
    model:
      type: "unet"
      model_path: "/models/unet_asparagus_ch16_256_v1.0_ep20.xml"
      device: "GPU"  # GPU or CPU
      input_width: 256
      input_height: 256
      confidence_threshold: 0.5
    
    class_names: ["Background", "Asparagus"]
    class_colors: [0, 0, 0, 0, 255, 0]  # BGR colors
```

## 使用方法

### 個別起動
```bash
# D415カメラ用
ros2 launch fv_object_mask_generator fv_object_mask_generator_launch.py \
    config_file:=src/ai/fv_object_mask_generator/config/d415_mask_generator_config.yaml

# D405カメラ用  
ros2 launch fv_object_mask_generator fv_object_mask_generator_launch.py \
    config_file:=src/ai/fv_object_mask_generator/config/d405_mask_generator_config.yaml
```

### システム全体起動
```bash
cd launch
./start_fv.sh
```

## 画面表示情報

colored mask画像に以下の情報が表示されます：
- Model: UNet
- File: モデルファイル名
- Device: GPU/CPU
- Inference: 推論時間（ms）

## 技術仕様

- **入力**: RGB画像（任意解像度）
- **前処理**: 256x256リサイズ、正規化、CHWフォーマット変換
- **モデル**: UNet（OpenVINO形式）
- **後処理**: アルファ合成によるカラーマスク生成
- **出力フレームレート**: 10Hz（設定可能）

## 依存関係

- OpenCV 4.x
- OpenVINO Runtime
- ROS2 Humble
- sensor_msgs, cv_bridge