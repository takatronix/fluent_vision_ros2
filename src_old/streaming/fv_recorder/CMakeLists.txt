cmake_minimum_required(VERSION 3.10)
project(fv_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(nlohmann_json REQUIRED)

# Generate services and messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartRecording.srv"
  "srv/StopRecording.srv"
  "srv/StartPlayback.srv"
  "srv/StopPlayback.srv"
  "msg/RecordingStatus.msg"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(include)

# Create recorder node
add_executable(fv_recorder_node src/fv_recorder_node.cpp src/format_writers.cpp)
ament_target_dependencies(fv_recorder_node
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  cv_bridge
  OpenCV
  image_transport
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  nlohmann_json
)
rosidl_target_interfaces(fv_recorder_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Create player node
add_executable(fv_player_node src/fv_player_node.cpp)
ament_target_dependencies(fv_player_node
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  cv_bridge
  OpenCV
  image_transport
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  nlohmann_json
)
rosidl_target_interfaces(fv_player_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install executables
install(TARGETS
  fv_recorder_node
  fv_player_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
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