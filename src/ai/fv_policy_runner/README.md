# fv_policy_runner

`FluentVision` のパイプラインから外部 policy runtime を呼び出す薄い ROS 2 ノードです。

## 役割

- 画像 / JointState / prompt を保持する
- trigger topic を監視して推論開始 / 停止する
- `openpi_runtime` の HTTP worker に観測を送る
- 返ってきた action chunk を 1 step ずつ publish する

## 想定構成

```text
fv_camera / fv_realsense / robot state
    -> fv_policy_runner
    -> HTTP POST /infer
    -> openpi_runtime container
    -> Float32MultiArray / JointState action
```

## 主なパラメータ

- `image_topics`: 入力画像 topic 配列
- `image_names`: policy 側に渡すカメラ名配列
- `state_topic`: JointState 入力
- `trigger_topic`: `std_msgs/Bool`
- `stop_topic`: `std_msgs/Bool`
- `policy_select_topic`: `std_msgs/String`
- `prompt_topic`: `std_msgs/String`
- `backend_endpoint`: 例 `http://127.0.0.1:8000/infer`
- `observation_format`: いまは `aloha` を想定
- `publish_hz`: action を消費するレート

## 備考

- `openpi_runtime` 側は `policy_id` を見て policy を切り替えられる想定です
- 現段階の observation 変換は `aloha` 形式を優先しています
