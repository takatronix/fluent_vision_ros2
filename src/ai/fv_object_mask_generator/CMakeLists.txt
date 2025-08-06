cmake_minimum_required(VERSION 3.8)
project(fv_object_mask_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSON REQUIRED nlohmann_json)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${JSON_INCLUDE_DIRS}
)

# Add executable
add_executable(fv_object_mask_generator_node
  src/fv_object_mask_generator_node.cpp
)

ament_target_dependencies(fv_object_mask_generator_node
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
  tf2
  tf2_ros
  vision_msgs
  OpenCV
)

target_link_libraries(fv_object_mask_generator_node
  openvino::runtime
  ${JSON_LIBRARIES}
)

# Install executable
install(TARGETS
  fv_object_mask_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()