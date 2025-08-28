# =============================================================================
# Fluent Vision - 物体検出ノード CMakeLists.txt
# =============================================================================
# 
# 概要：
#   - YOLOv10を使用したリアルタイム物体検出ノードのビルド設定
#   - OpenVINOによる高速推論対応
#   - ROS2 Humble対応のCMake設定
#   - 依存関係の管理とインストール設定
#
# 作者：Takashi Otsuka
# 作成日：2024年
# バージョン：1.0
# =============================================================================

cmake_minimum_required(VERSION 3.8)
project(fv_object_detector)

# ===== コンパイラ警告設定 =====
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)  # 厳格な警告設定
endif()

# ===== 依存関係パッケージ検索 =====
find_package(ament_cmake REQUIRED)           # ROS2ビルドシステム
find_package(ament_cmake_auto REQUIRED)      # ROS2自動ビルド支援
find_package(rclcpp REQUIRED)                # ROS2 C++クライアントライブラリ
find_package(sensor_msgs REQUIRED)           # センサーメッセージ型
find_package(cv_bridge REQUIRED)             # OpenCV-ROS2橋渡し
find_package(image_transport REQUIRED)       # 画像転送ライブラリ
find_package(std_msgs REQUIRED)              # 標準メッセージ型
find_package(geometry_msgs REQUIRED)         # 幾何学メッセージ型
find_package(visualization_msgs REQUIRED)    # 可視化メッセージ型
find_package(vision_msgs REQUIRED)           # コンピュータビジョンメッセージ
find_package(std_srvs REQUIRED)              # 標準サービス型

# ===== サービスメッセージ生成（一時的に無効化） =====
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/SetDetectionState.srv"
#   "srv/GetDetectionStats.srv"
#   DEPENDENCIES std_srvs
# )

# ===== 外部ライブラリ検索 =====
find_package(OpenCV REQUIRED)                # OpenCVコンピュータビジョン
find_package(OpenVINO REQUIRED)              # Intel OpenVINO推論エンジン
find_package(nlohmann_json REQUIRED)         # JSON処理ライブラリ
find_package(yaml-cpp REQUIRED)              # YAML処理ライブラリ

# ===== C++標準設定 =====
set(CMAKE_CXX_STANDARD 17)                   # C++17標準を使用
set(CMAKE_CXX_STANDARD_REQUIRED ON)          # 標準準拠を必須とする

# ===== インクルードディレクトリ設定 =====
include_directories(include)                 # ヘッダーファイルディレクトリ

# ===== 実行ファイル作成 =====
add_executable(fv_object_detector_node
  src/fv_object_detector_node.cpp            # メインノードソース
  src/object_tracker.cpp                     # オブジェクトトラッカー実装
  src/ai_model.cpp                           # AIモデル基底クラス実装
  src/yolov10_model.cpp                      # YOLOv10モデル実装
)

# ===== ROS2依存関係リンク =====
ament_target_dependencies(fv_object_detector_node
  rclcpp                    # ROS2 C++クライアントライブラリ
  sensor_msgs               # センサーメッセージ型
  cv_bridge                 # OpenCV-ROS2橋渡し
  image_transport           # 画像転送ライブラリ
  std_msgs                  # 標準メッセージ型
  geometry_msgs             # 幾何学メッセージ型
  visualization_msgs        # 可視化メッセージ型
  vision_msgs               # コンピュータビジョンメッセージ
)

# ===== 外部ライブラリリンク =====
target_link_libraries(fv_object_detector_node
  ${OpenCV_LIBS}                            # OpenCVライブラリ
  openvino::runtime                         # OpenVINO推論エンジン
  nlohmann_json::nlohmann_json              # JSON処理ライブラリ
  yaml-cpp                                  # YAML処理ライブラリ
)

# ===== インストール設定 =====
# 実行ファイルのインストール
install(TARGETS
  fv_object_detector_node
  DESTINATION lib/${PROJECT_NAME}            # インストール先：lib/fv_object_detector/
)

# ディレクトリのインストール
install(DIRECTORY
  launch                    # launchディレクトリ
  config                    # configディレクトリ
  DESTINATION share/${PROJECT_NAME}          # インストール先：share/fv_object_detector/
)

# ヘッダーファイルのインストール
install(DIRECTORY include/
  DESTINATION include/                       # インストール先：include/
)

# ===== テスト設定 =====
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)                    # 自動リントツール
  ament_lint_auto_find_test_dependencies()                  # テスト依存関係検索
endif()

# ===== パッケージ設定 =====
ament_package()  # ROS2パッケージ設定の完了 