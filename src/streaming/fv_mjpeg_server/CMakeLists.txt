cmake_minimum_required(VERSION 3.8)
project(fv_mjpeg_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# Create executable
add_executable(fv_mjpeg_server_node
  src/fv_mjpeg_server_node.cpp
)

# Link dependencies
ament_target_dependencies(fv_mjpeg_server_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

# Link OpenCV
target_link_libraries(fv_mjpeg_server_node
  ${OpenCV_LIBS}
)

# Install targets
install(TARGETS
  fv_mjpeg_server_node
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
