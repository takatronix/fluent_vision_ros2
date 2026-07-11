cmake_minimum_required(VERSION 3.8)
project(fv_insta360_x3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(fv_insta360_x3_node
  src/fv_insta360_x3_node.cpp
)

add_executable(fv_insta360_x3_stitcher_node
  src/fv_insta360_x3_stitcher_node.cpp
)

ament_target_dependencies(fv_insta360_x3_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(fv_insta360_x3_node
  ${OpenCV_LIBS}
)

ament_target_dependencies(fv_insta360_x3_stitcher_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(fv_insta360_x3_stitcher_node
  ${OpenCV_LIBS}
)

install(TARGETS
  fv_insta360_x3_node
  fv_insta360_x3_stitcher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
