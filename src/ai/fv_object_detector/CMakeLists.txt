cmake_minimum_required(VERSION 3.8)
project(fv_object_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Generate service messages
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/SetDetectionState.srv"
#   "srv/GetDetectionStats.srv"
#   DEPENDENCIES std_srvs
# )

find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(include)

# Create the object detector node
add_executable(fv_object_detector_node
  src/fv_object_detector_node.cpp
  src/object_tracker.cpp
  src/ai_model.cpp
  src/yolov10_model.cpp
)

# Link libraries
ament_target_dependencies(fv_object_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
  geometry_msgs
  visualization_msgs
  vision_msgs
)

target_link_libraries(fv_object_detector_node
  ${OpenCV_LIBS}
  openvino::runtime
  nlohmann_json::nlohmann_json
  yaml-cpp
)

# Install
install(TARGETS
  fv_object_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 