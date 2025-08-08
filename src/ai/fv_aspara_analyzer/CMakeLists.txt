# =============================================================================
# Fluent Vision - アスパラガス解析ノード CMakeLists.txt
# =============================================================================
# 
# 概要：
#   - アスパラガス3D点群解析ノードのビルド設定
#   - ROS2 Humble対応のCMake設定
#   - 依存関係の管理とインストール設定
#
# 作者：Takashi Otsuka
# 作成日：2024年
# バージョン：1.0
# =============================================================================

cmake_minimum_required(VERSION 3.8)
project(fv_aspara_analyzer)

# ===== コンパイラ警告設定 =====
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)  # 厳格な警告設定
endif()

# ===== 依存関係パッケージ検索 =====
find_package(ament_cmake REQUIRED)           # ROS2ビルドシステム
find_package(rclcpp REQUIRED)                # ROS2 C++クライアントライブラリ
find_package(std_msgs REQUIRED)              # 標準メッセージ型
find_package(sensor_msgs REQUIRED)           # センサーメッセージ型
find_package(geometry_msgs REQUIRED)         # 幾何学メッセージ型
find_package(cv_bridge REQUIRED)             # OpenCV-ROS2橋渡し
find_package(image_transport REQUIRED)       # 画像転送ライブラリ
find_package(tf2 REQUIRED)                   # TF2座標変換ライブラリ
find_package(tf2_ros REQUIRED)               # TF2 ROS2統合
find_package(tf2_geometry_msgs REQUIRED)     # TF2幾何学メッセージ
find_package(pcl_conversions REQUIRED)       # PCL-ROS2変換
find_package(pcl_ros REQUIRED)               # PCL ROS2統合
find_package(OpenCV REQUIRED)                # OpenCVコンピュータビジョン
find_package(vision_msgs REQUIRED)           # コンピュータビジョンメッセージ
find_package(std_srvs REQUIRED)              # 標準サービスメッセージ
find_package(fluent_lib REQUIRED)

# ===== インクルードディレクトリ設定 =====
include_directories(include)                 # ヘッダーファイルディレクトリ
include_directories(${OpenCV_INCLUDE_DIRS})  # OpenCVインクルードディレクトリ
include_directories(/home/aspara/seedbox-r1/fluent_vision_ros2/src/common/fluent_lib/include)  # fluent_lib

# ===== 依存関係リスト =====
set(dependencies
  rclcpp                    # ROS2 C++クライアントライブラリ
  std_msgs                  # 標準メッセージ型
  sensor_msgs               # センサーメッセージ型
  geometry_msgs             # 幾何学メッセージ型
  cv_bridge                 # OpenCV-ROS2橋渡し
  image_transport           # 画像転送ライブラリ
  tf2                       # TF2座標変換ライブラリ
  tf2_ros                   # TF2 ROS2統合
  tf2_geometry_msgs         # TF2幾何学メッセージ
  pcl_conversions           # PCL-ROS2変換
  pcl_ros                   # PCL ROS2統合
  vision_msgs               # コンピュータビジョンメッセージ
  std_srvs                  # 標準サービスメッセージ
  fluent_lib                # Fluent統合ライブラリ
)

# ===== 実行ファイル作成 =====
add_executable(fv_aspara_analyzer_node
  src/fv_aspara_analyzer_node.cpp  # メインソースファイル（統合管理）
  src/aspara_selection.cpp         # アスパラ選択管理クラス
)


# ===== 依存関係とライブラリリンク =====
ament_target_dependencies(fv_aspara_analyzer_node ${dependencies})  # ROS2依存関係
target_link_libraries(fv_aspara_analyzer_node ${OpenCV_LIBRARIES})  # ライブラリリンク


# ===== インストール設定 =====
# 実行ファイルのインストール
install(TARGETS
  fv_aspara_analyzer_node
  DESTINATION lib/${PROJECT_NAME}  # インストール先：lib/fv_aspara_analyzer/
)

# ディレクトリのインストール
install(DIRECTORY
  launch                    # launchディレクトリ
  DESTINATION share/${PROJECT_NAME}/  # インストール先：share/fv_aspara_analyzer/
)

# ===== テスト設定 =====
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)                    # 自動リントツール
  ament_lint_auto_find_test_dependencies()                  # テスト依存関係検索
endif()

# ===== パッケージ設定 =====
ament_package()  # ROS2パッケージ設定の完了