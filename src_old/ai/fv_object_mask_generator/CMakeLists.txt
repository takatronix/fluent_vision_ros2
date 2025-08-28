# =============================================================================
# Fluent Vision - オブジェクトマスク生成ノード CMakeLists.txt
# =============================================================================
# 
# 概要：
#   - UNetを使用したセグメンテーションマスク生成ノードのビルド設定
#   - OpenVINOによる高速推論対応
#   - ROS2 Humble対応のCMake設定
#   - 依存関係の管理とインストール設定
#
# 作者：Takashi Otsuka
# 作成日：2024年
# バージョン：1.0
# =============================================================================

cmake_minimum_required(VERSION 3.8)
project(fv_object_mask_generator)

# ===== コンパイラ警告設定 =====
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)  # 厳格な警告設定
endif()

# ===== 依存関係パッケージ検索 =====
find_package(ament_cmake REQUIRED)           # ROS2ビルドシステム
find_package(rclcpp REQUIRED)                # ROS2 C++クライアントライブラリ
find_package(rclcpp_components REQUIRED)     # ROS2コンポーネント
find_package(sensor_msgs REQUIRED)           # センサーメッセージ型
find_package(std_msgs REQUIRED)              # 標準メッセージ型
find_package(cv_bridge REQUIRED)             # OpenCV-ROS2橋渡し
find_package(tf2 REQUIRED)                   # TF2座標変換ライブラリ
find_package(tf2_ros REQUIRED)               # TF2 ROS2統合
find_package(vision_msgs REQUIRED)           # コンピュータビジョンメッセージ
find_package(OpenCV REQUIRED)                # OpenCVコンピュータビジョン
find_package(OpenVINO REQUIRED)              # Intel OpenVINO推論エンジン
find_package(PkgConfig REQUIRED)             # pkg-config
pkg_check_modules(JSON REQUIRED nlohmann_json)  # JSON処理ライブラリ

# ===== インクルードディレクトリ設定 =====
include_directories(
  include                    # ヘッダーファイルディレクトリ
  ${OpenCV_INCLUDE_DIRS}     # OpenCVインクルードディレクトリ
  ${JSON_INCLUDE_DIRS}       # JSONライブラリインクルードディレクトリ
)

# ===== 実行ファイル作成 =====
add_executable(fv_object_mask_generator_node
  src/fv_object_mask_generator_node.cpp  # メインノードソース
)

# ===== ROS2依存関係リンク =====
ament_target_dependencies(fv_object_mask_generator_node
  rclcpp                    # ROS2 C++クライアントライブラリ
  rclcpp_components         # ROS2コンポーネント
  sensor_msgs               # センサーメッセージ型
  std_msgs                  # 標準メッセージ型
  cv_bridge                 # OpenCV-ROS2橋渡し
  tf2                       # TF2座標変換ライブラリ
  tf2_ros                   # TF2 ROS2統合
  vision_msgs               # コンピュータビジョンメッセージ
  OpenCV                    # OpenCVライブラリ
)

# ===== 外部ライブラリリンク =====
target_link_libraries(fv_object_mask_generator_node
  openvino::runtime         # OpenVINO推論エンジン
  ${JSON_LIBRARIES}         # JSON処理ライブラリ
)

# ===== インストール設定 =====
# 実行ファイルのインストール
install(TARGETS
  fv_object_mask_generator_node
  DESTINATION lib/${PROJECT_NAME}  # インストール先：lib/fv_object_mask_generator/
)

# ディレクトリのインストール
install(DIRECTORY
  launch                    # launchディレクトリ
  DESTINATION share/${PROJECT_NAME}  # インストール先：share/fv_object_mask_generator/
)

# 設定ファイルのインストール
install(DIRECTORY
  config                    # configディレクトリ
  DESTINATION share/${PROJECT_NAME}  # インストール先：share/fv_object_mask_generator/
)

# ヘッダーファイルのインストール
install(DIRECTORY
  include/                  # ヘッダーファイルディレクトリ
  DESTINATION include       # インストール先：include/
)

# ===== テスト設定 =====
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)                    # 自動リントツール
  ament_lint_auto_find_test_dependencies()                  # テスト依存関係検索
endif()

# ===== パッケージ設定 =====
ament_package()  # ROS2パッケージ設定の完了