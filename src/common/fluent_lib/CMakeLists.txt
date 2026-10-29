cmake_minimum_required(VERSION 3.8)
project(fluent_lib)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find system dependencies
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
# freetype テキスト描画 (fluent_text) はオプション: opencv2/freetype.hpp
# (OpenCV contrib) が無い環境 (native Jetson/Jazzy 等) ではテキスト描画抜きで
# ビルドする。コンテナ (contrib あり) では従来どおり有効。
find_package(Freetype QUIET)
find_path(OPENCV_FREETYPE_INCLUDE opencv2/freetype.hpp
  PATHS ${OpenCV_INCLUDE_DIRS} NO_DEFAULT_PATH)
if(FREETYPE_FOUND AND OPENCV_FREETYPE_INCLUDE)
  set(FLUENT_HAS_FREETYPE TRUE)
else()
  set(FLUENT_HAS_FREETYPE FALSE)
  message(WARNING "fluent_lib: opencv2/freetype.hpp not found - "
                  "building WITHOUT fluent_text (text drawing disabled)")
endif()
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

# Source files
set(FLUENT_LIB_SOURCES
  src/fluent_image.cpp
  src/fluent_cloud_impl.cpp
  src/fluent_utils.cpp
)
if(FLUENT_HAS_FREETYPE)
  list(APPEND FLUENT_LIB_SOURCES src/fluent_text.cpp)
endif()

# Create library
add_library(${PROJECT_NAME} SHARED ${FLUENT_LIB_SOURCES})

# Target dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)
if(FLUENT_HAS_FREETYPE)
  target_link_libraries(${PROJECT_NAME} ${FREETYPE_LIBRARIES})
endif()

# Install headers
# JPEG の専用回路 (hw_jpeg.hpp) の確かめ道具。GStreamer があれば専用回路と OpenCV を比べ、無ければ OpenCV だけ
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(FL_GST IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
endif()
add_executable(hw_jpeg_check tools/hw_jpeg_check.cpp)
target_include_directories(hw_jpeg_check PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(hw_jpeg_check ${OpenCV_LIBS})
if(FL_GST_FOUND)
  target_compile_definitions(hw_jpeg_check PRIVATE FLUENT_HW_JPEG_GST=1)
  target_link_libraries(hw_jpeg_check PkgConfig::FL_GST ${CMAKE_DL_LIBS})
endif()
# 道具は使う側と同じく画素ループを最適化して測る (機体の colcon build は build type を付けず、最適化なしだと専用回路の方が重く見える)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set_source_files_properties(tools/hw_jpeg_check.cpp PROPERTIES COMPILE_OPTIONS "-O3")
endif()
install(TARGETS hw_jpeg_check DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

# Expose include directory to downstream packages
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_geometry_msgs
  yaml-cpp
)
if(FLUENT_HAS_FREETYPE)
  ament_export_dependencies(Freetype)
endif()

# Lint and test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()