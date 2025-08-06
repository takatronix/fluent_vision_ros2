# FV Topic Relay

FluentVision用のトピック中継ノードです。ROS2トピック間のデータ中継を行います。

## 概要

fv_topic_relayは、ROS2のトピック間でデータを中継するユーティリティノードです。異なるノード間でのデータ転送や、トピック名の変更、データ形式の変換などを行います。

## 機能

- **トピック中継**: 入力トピックから出力トピックへのデータ転送
- **トピック名変更**: トピック名の動的変更
- **データ変換**: メッセージ形式の変換
- **フィルタリング**: 条件に基づくデータフィルタリング
- **設定可能**: YAMLファイルによる柔軟な設定
- **コンポーネント化**: ROS2コンポーネントとして実装

## 入力/出力

### 対応メッセージタイプ

- `sensor_msgs/msg/Image` - 画像データ
- `sensor_msgs/msg/PointCloud2` - 点群データ
- `std_msgs/msg/String` - 文字列データ
- `std_msgs/msg/Bool` - 真偽値データ
- `geometry_msgs/msg/PoseStamped` - 位置姿勢データ

### 設定例

```yaml
fv_topic_relay:
  ros__parameters:
    relays:
      - input_topic: "/camera/color/image_raw"
        output_topic: "/relay/color/image_raw"
        message_type: "sensor_msgs/msg/Image"
        enabled: true
      - input_topic: "/camera/depth/image_raw"
        output_topic: "/relay/depth/image_raw"
        message_type: "sensor_msgs/msg/Image"
        enabled: true
      - input_topic: "/lidar/points"
        output_topic: "/relay/lidar/points"
        message_type: "sensor_msgs/msg/PointCloud2"
        enabled: true
```

## 使用方法

### 1. ビルド

```bash
cd fluent_vision_ros2
colcon build --packages-select fv_topic_relay
```

### 2. 起動

```bash
# 基本的な起動
ros2 launch fv_topic_relay topic_relay.launch.py

# カスタム設定で起動
ros2 launch fv_topic_relay topic_relay.launch.py \
  config_file:=/path/to/custom_config.yaml
```

### 3. コンポーネントとして使用

```bash
# コンポーネントマネージャーで起動
ros2 run rclcpp_components component_container

# 別のターミナルでコンポーネントをロード
ros2 component load /ComponentManager fv_topic_relay fv_topic_relay::TopicRelayNode
```

## 設定

### パラメータ

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `relays` | `[]` | 中継設定の配列 |
| `relays[].input_topic` | - | 入力トピック名 |
| `relays[].output_topic` | - | 出力トピック名 |
| `relays[].message_type` | - | メッセージタイプ |
| `relays[].enabled` | `true` | 中継有効/無効 |
| `relays[].queue_size` | `10` | キューのサイズ |

## 依存関係

- ROS2 Humble
- rclcpp_components
- sensor_msgs
- std_msgs
- geometry_msgs

## 開発状況

⚠️ **開発中**: このノードは現在開発中です。実装は完了していません。

## ライセンス

Apache-2.0 License 