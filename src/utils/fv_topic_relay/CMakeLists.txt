cmake_minimum_required(VERSION 3.8)
project(fv_topic_relay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Composable node library
add_library(fv_topic_relay_component SHARED
  src/fv_topic_relay_node.cpp
)
target_include_directories(fv_topic_relay_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(fv_topic_relay_component PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  fv_topic_relay_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)
rclcpp_components_register_node(
  fv_topic_relay_component
  PLUGIN "fv_topic_relay::FVTopicRelayNode"
  EXECUTABLE fv_topic_relay_node
)

install(TARGETS fv_topic_relay_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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