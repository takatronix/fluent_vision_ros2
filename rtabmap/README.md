# RTABMap 設定フォルダ

RTABMap (Real-Time Appearance-Based Mapping) の設定ファイルとスクリプトをまとめたフォルダです。

## フォルダ構成

```
rtabmap/
├── README.md                    # このファイル
├── configs/                     # 設定ファイル群
│   ├── essential_params.yaml   # 必須パラメータ
│   ├── indoor_mapping.yaml     # 屋内マッピング設定
│   ├── outdoor_mapping.yaml    # 屋外マッピング設定
│   ├── localization.yaml       # ローカライゼーション設定
│   └── performance.yaml        # パフォーマンス調整設定
├── launch/                      # Launch ファイル群
│   ├── rtabmap_rgbd.launch.py  # RGB-D カメラ用
│   ├── rtabmap_lidar.launch.py # LiDAR 用
│   └── rtabmap_multi.launch.py # マルチセンサー用
└── scripts/                     # 実行スクリプト群
    ├── start_mapping.sh         # マッピング開始
    ├── start_localization.sh    # ローカライゼーション開始
    ├── switch_mode.sh           # モード切り替え
    └── check_status.sh          # ステータス確認

```

## 設定項目の概要

### 必須設定項目
1. **フレーム設定**: base_link, map, odom フレームの定義
2. **センサー設定**: RGB-D カメラ、LiDAR、IMU の入力設定
3. **モード設定**: マッピング / ローカライゼーション モード選択

### 重要設定項目
1. **最適化設定**: g2o, GTSAM, TORO による最適化戦略
2. **視覚オドメトリ設定**: 特徴点抽出・マッチング設定
3. **ループクロージャ設定**: 閉ループ検出の閾値・精度設定
4. **グリッドマップ設定**: 占有格子地図生成設定
5. **メモリ管理設定**: データベース管理とメモリ制限

## 使用方法

1. 必要な設定ファイルを選択・編集
2. Launch ファイルで RTABMap を起動
3. スクリプトでモード切り替えや状態確認

詳細は各ファイルのコメントを参照してください。