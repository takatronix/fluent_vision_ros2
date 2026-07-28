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

# RealSense SDKの検索（複数の方法を試す）
set(REALSENSE2_FOUND FALSE)
set(REALSENSE2_INCLUDE_DIRS "")
set(REALSENSE2_LIBRARIES "")

# 方法0: CMake config (正規)。ros-jazzy-librealsense2 は
# /opt/ros/<distro>/lib/<arch>/cmake/realsense2/ に config を置く
# (lib 本体も <arch> サブディレクトリ側 — 方法2 の固定パスには載らない)。
# 同じ版のSDKが複数ある場合は、実行時とのABI不一致を避けるためシステム側を
# 優先する。明示された realsense2_DIR は尊重する。
if(NOT realsense2_DIR)
  foreach(_rs2_config_dir
      "/opt/ros/$ENV{ROS_DISTRO}/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/realsense2"
      "/usr/local/lib/cmake/realsense2")
    if(EXISTS "${_rs2_config_dir}/realsense2Config.cmake")
      set(realsense2_DIR "${_rs2_config_dir}" CACHE PATH
          "Preferred RealSense SDK CMake config")
      break()
    endif()
  endforeach()
endif()
find_package(realsense2 CONFIG QUIET)
if(realsense2_FOUND)
  set(REALSENSE2_FOUND TRUE)
  set(REALSENSE2_LIBRARIES realsense2::realsense2)
  # 本CMakeLists は include_directories(${REALSENSE2_INCLUDE_DIRS}) で
  # ヘッダを渡す作りなので、imported target からも取り出しておく
  get_target_property(_rs2_inc realsense2::realsense2 INTERFACE_INCLUDE_DIRECTORIES)
  if(_rs2_inc)
    set(REALSENSE2_INCLUDE_DIRS ${_rs2_inc})
  endif()
  message(STATUS "Found RealSense SDK via CMake config (${realsense2_DIR})")
endif()

# 方法1: pkg-configで検索
if(NOT REALSENSE2_FOUND)
  pkg_check_modules(REALSENSE2_PKG QUIET realsense2)
endif()
if(NOT REALSENSE2_FOUND AND REALSENSE2_PKG_FOUND)
  set(REALSENSE2_FOUND TRUE)
  set(REALSENSE2_INCLUDE_DIRS ${REALSENSE2_PKG_INCLUDE_DIRS})
  set(REALSENSE2_LIBRARIES ${REALSENSE2_PKG_LIBRARIES})
  message(STATUS "Found RealSense SDK via pkg-config")
elseif(NOT REALSENSE2_FOUND)
  # 方法2: 直接ライブラリを検索
  find_library(REALSENSE2_LIB
    NAMES realsense2 librealsense2
    PATHS
      /usr/lib
      /usr/local/lib
      /usr/lib/aarch64-linux-gnu
      /opt/ros/jazzy/lib
    NO_DEFAULT_PATH
  )
  
  find_path(REALSENSE2_INCLUDE
    NAMES librealsense/rs.hpp
    PATHS
      /usr/include
      /usr/local/include
      /opt/ros/jazzy/include
    NO_DEFAULT_PATH
  )
  
  if(REALSENSE2_LIB AND REALSENSE2_INCLUDE)
    set(REALSENSE2_FOUND TRUE)
    set(REALSENSE2_INCLUDE_DIRS ${REALSENSE2_INCLUDE})
    set(REALSENSE2_LIBRARIES ${REALSENSE2_LIB})
    message(STATUS "Found RealSense SDK: ${REALSENSE2_LIB}")
  else()
    message(WARNING "RealSense SDK not found. Install it from: https://github.com/IntelRealSense/librealsense")
    message(WARNING "Or build without RealSense support by setting ENABLE_REALSENSE=OFF")
  endif()
endif()

# RealSense SDKサポートの有効/無効を設定
option(ENABLE_REALSENSE "Enable RealSense SDK support" ${REALSENSE2_FOUND})
if(ENABLE_REALSENSE AND NOT REALSENSE2_FOUND)
    message(WARNING "RealSense SDK not found. Disabling RealSense support.")
    message(WARNING "To install: https://github.com/IntelRealSense/librealsense")
    set(ENABLE_REALSENSE OFF)
endif()

# RealSense SDKサポートの有効/無効を設定
option(ENABLE_REALSENSE "Enable RealSense SDK support" ${REALSENSE2_FOUND})
if(ENABLE_REALSENSE AND NOT REALSENSE2_FOUND)
    message(WARNING "RealSense SDK not found. Disabling RealSense support.")
    message(WARNING "To install: https://github.com/IntelRealSense/librealsense")
    set(ENABLE_REALSENSE OFF)
endif()

if(NOT ENABLE_REALSENSE)
  message(STATUS "RealSense support disabled. Skipping fv_realsense package build.")
  message(STATUS "To enable: Install RealSense SDK and rebuild, or set -DENABLE_REALSENSE=ON")
  # パッケージをスキップするために空のターゲットを作成
  add_custom_target(fv_realsense_node ALL
    COMMAND ${CMAKE_COMMAND} -E echo "fv_realsense_node skipped: RealSense SDK not available"
  )
  ament_package()
  return()
endif()

# RealSense SDKが見つかった場合のみ続行
message(STATUS "RealSense SDK found. Building fv_realsense package.")
find_package(fluent_lib REQUIRED)                     # Fluent Lib (cv_bridge_compat)
find_package(pcl_conversions REQUIRED)                # PCL-ROS2変換
find_package(pcl_ros REQUIRED)                        # PCL ROS2統合
find_package(image_transport REQUIRED)                # 画像転送ライブラリ
find_package(compressed_image_transport REQUIRED)     # 圧縮画像転送
find_package(geometry_msgs REQUIRED)                  # 幾何学メッセージ型
find_package(std_srvs REQUIRED)                       # 標準サービス型
find_package(OpenCV REQUIRED)                         # OpenCVコンピュータビジョン
find_package(TIFF REQUIRED)
find_package(CURL REQUIRED)
find_package(GDAL QUIET)
find_package(rosidl_default_generators REQUIRED)      # ROS2インターフェース生成

# ===== サービスメッセージ生成 =====
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDistance.srv"      # 距離取得サービス
  "srv/GetCameraInfo.srv"    # カメラ情報取得サービス
  "srv/SetMode.srv"          # モード設定サービス
  "srv/GeneratePointCloud.srv"  # 点群生成サービス
  DEPENDENCIES geometry_msgs std_srvs sensor_msgs
)

# ===== インクルードディレクトリ設定 =====
include_directories(
  include                    # ヘッダーファイルディレクトリ
  ${OpenCV_INCLUDE_DIRS}     # OpenCVインクルードディレクトリ
)
if(ENABLE_REALSENSE)
  include_directories(${REALSENSE2_INCLUDE_DIRS}) # RealSense SDKインクルードディレクトリ
  link_directories(${REALSENSE2_PKG_LIBRARY_DIRS}) # RealSense SDKライブラリディレクトリ
endif()

# ===== 実行ファイル作成 =====
add_executable(fv_realsense_node
  src/fv_realsense_node.cpp  # メインRealSenseノード
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
  fluent_lib                # Fluent Lib
)

target_link_libraries(fv_realsense_node
  ${OpenCV_LIBS}            # OpenCVライブラリ
  TIFF::TIFF
  CURL::libcurl
)
if(ENABLE_REALSENSE)
  # RealSense SDKライブラリのリンク (REALSENSE2_LIBRARIES は 方法0/1/2 の
  # どの探索経路でも設定される共通変数。旧 REALSENSE2_PKG_LIBRARIES は
  # pkg-config 経路以外では空でリンク漏れになるバグだった)
  target_link_libraries(fv_realsense_node ${REALSENSE2_LIBRARIES})
  target_compile_definitions(fv_realsense_node PRIVATE ENABLE_REALSENSE=1)
else()
  target_compile_definitions(fv_realsense_node PRIVATE ENABLE_REALSENSE=0)
endif()
if(GDAL_FOUND)
  target_link_libraries(fv_realsense_node GDAL::GDAL)
endif()

# ===== シンプルノードの依存関係リンク（最小限） =====

# ===== 画像保存ノードの依存関係リンク =====

# ===== 生成されたインターフェースのリンク =====
rosidl_target_interfaces(fv_realsense_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ===== インストール設定 =====
# 実行ファイルのインストール
install(TARGETS
  fv_realsense_node         # メインRealSenseノード
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

# ===== パッケージエクスポート設定 =====
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  rosidl_default_runtime
)
ament_export_include_directories(include)

# ===== パッケージ設定 =====
ament_package()  # ROS2パッケージ設定の完了 
