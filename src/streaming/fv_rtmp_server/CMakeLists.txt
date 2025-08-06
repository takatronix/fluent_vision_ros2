cmake_minimum_required(VERSION 3.8)
project(fv_rtmp_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Find FFmpeg libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_check_modules(SWSCALE REQUIRED libswscale)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)
include_directories(${AVCODEC_INCLUDE_DIRS})
include_directories(${AVFORMAT_INCLUDE_DIRS})
include_directories(${AVUTIL_INCLUDE_DIRS})
include_directories(${SWSCALE_INCLUDE_DIRS})

# Create executable
add_executable(fv_rtmp_server_node
  src/rtmp_bridge_node.cpp
  src/rtmp_server.cpp
  src/video_processor.cpp
)

# Link dependencies
ament_target_dependencies(fv_rtmp_server_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  ament_index_cpp
)

# Link FFmpeg libraries
target_link_libraries(fv_rtmp_server_node
  ${AVCODEC_LIBRARIES}
  ${AVFORMAT_LIBRARIES}
  ${AVUTIL_LIBRARIES}
  ${SWSCALE_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
)

# Add compiler flags
target_compile_options(fv_rtmp_server_node PRIVATE
  ${AVCODEC_CFLAGS_OTHER}
  ${AVFORMAT_CFLAGS_OTHER}
  ${AVUTIL_CFLAGS_OTHER}
  ${SWSCALE_CFLAGS_OTHER}
)

# Install targets
install(TARGETS
  fv_rtmp_server_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 