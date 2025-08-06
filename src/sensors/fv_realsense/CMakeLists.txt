# =============================================================================
# Fluent Vision - RealSenseカメラノード CMakeLists.txt
# =============================================================================
# 
# 概要：
#   - Intel RealSenseカメラ（D415/D405）のROS2ノードのビルド設定
#   - カラー、深度、点群データの取得と配信
#   - TF2座標変換とサービス提供
#   - ROS2 Humble対応のCMake設定
#
# 作者：Takashi Otsuka
# 作成日：2024年
# バージョン：1.0
# =============================================================================

cmake_minimum_required(VERSION 3.8)
project(fv_realsense)

# ===== コンパイラ警告設定 =====
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)  # 厳格な警告設定
endif()

# ===== 依存関係パッケージ検索 =====
find_package(ament_cmake REQUIRED)                    # ROS2ビルドシステム
find_package(rclcpp REQUIRED)                         # ROS2 C++クライアントライブラリ
find_package(sensor_msgs REQUIRED)                    # センサーメッセージ型
find_package(cv_bridge REQUIRED)                      # OpenCV-ROS2橋渡し
find_package(tf2_ros REQUIRED)                        # TF2 ROS2統合
find_package(PkgConfig REQUIRED)                      # pkg-config
pkg_check_modules(REALSENSE2 REQUIRED realsense2)     # Intel RealSense SDK
find_package(pcl_conversions REQUIRED)                # PCL-ROS2変換
find_package(pcl_ros REQUIRED)                        # PCL ROS2統合
find_package(image_transport REQUIRED)                # 画像転送ライブラリ
find_package(compressed_image_transport REQUIRED)     # 圧縮画像転送
find_package(geometry_msgs REQUIRED)                  # 幾何学メッセージ型
find_package(std_srvs REQUIRED)                       # 標準サービス型
find_package(OpenCV REQUIRED)                         # OpenCVコンピュータビジョン
find_package(rosidl_default_generators REQUIRED)      # ROS2インターフェース生成

# ===== サービスメッセージ生成 =====
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDistance.srv"      # 距離取得サービス
  "srv/GetCameraInfo.srv"    # カメラ情報取得サービス
  "srv/SetMode.srv"          # モード設定サービス
  DEPENDENCIES geometry_msgs std_srvs
)

# ===== インクルードディレクトリ設定 =====
include_directories(
  include                    # ヘッダーファイルディレクトリ
  ${OpenCV_INCLUDE_DIRS}     # OpenCVインクルードディレクトリ
  ${REALSENSE2_INCLUDE_DIRS} # RealSense SDKインクルードディレクトリ
)

# ===== 実行ファイル作成 =====
add_executable(fv_realsense_node
  src/fv_realsense_node.cpp  # メインRealSenseノード
)

add_executable(fv_realsense_simple_node
  src/fv_realsense_simple_node.cpp  # シンプルRealSenseノード
)

add_executable(image_saver_node
  src/image_saver_node.cpp   # 画像保存ノード
)

# ===== メインノードの依存関係リンク =====
ament_target_dependencies(fv_realsense_node
  rclcpp                    # ROS2 C++クライアントライブラリ
  sensor_msgs               # センサーメッセージ型
  cv_bridge                 # OpenCV-ROS2橋渡し
  tf2_ros                   # TF2 ROS2統合
  pcl_conversions           # PCL-ROS2変換
  pcl_ros                   # PCL ROS2統合
  image_transport           # 画像転送ライブラリ
  compressed_image_transport # 圧縮画像転送
  geometry_msgs             # 幾何学メッセージ型
  std_srvs                  # 標準サービス型
)

target_link_libraries(fv_realsense_node
  ${OpenCV_LIBS}            # OpenCVライブラリ
  ${REALSENSE2_LIBRARIES}   # RealSense SDKライブラリ
)

# ===== シンプルノードの依存関係リンク（最小限） =====
ament_target_dependencies(fv_realsense_simple_node
  rclcpp                    # ROS2 C++クライアントライブラリ
  sensor_msgs               # センサーメッセージ型
  cv_bridge                 # OpenCV-ROS2橋渡し
)

target_link_libraries(fv_realsense_simple_node
  ${OpenCV_LIBS}            # OpenCVライブラリ
)

# ===== 画像保存ノードの依存関係リンク =====
ament_target_dependencies(image_saver_node
  rclcpp                    # ROS2 C++クライアントライブラリ
  sensor_msgs               # センサーメッセージ型
  cv_bridge                 # OpenCV-ROS2橋渡し
)

target_link_libraries(image_saver_node
  ${OpenCV_LIBS}            # OpenCVライブラリ
)

# ===== 生成されたインターフェースのリンク =====
rosidl_target_interfaces(fv_realsense_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ===== インストール設定 =====
# 実行ファイルのインストール
install(TARGETS
  fv_realsense_node         # メインRealSenseノード
  fv_realsense_simple_node  # シンプルRealSenseノード
  image_saver_node          # 画像保存ノード
  DESTINATION lib/${PROJECT_NAME}  # インストール先：lib/fv_realsense/
)

# ディレクトリのインストール
install(DIRECTORY
  launch                    # launchディレクトリ
  config                    # configディレクトリ
  scripts                   # scriptsディレクトリ
  DESTINATION share/${PROJECT_NAME}  # インストール先：share/fv_realsense/
)

# ヘッダーファイルのインストール
install(DIRECTORY include/
  DESTINATION include/       # インストール先：include/
)

# ===== テスト設定 =====
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)                    # 自動リントツール
  ament_lint_auto_find_test_dependencies()                  # テスト依存関係検索
endif()

# ===== パッケージ設定 =====
ament_package()  # ROS2パッケージ設定の完了 