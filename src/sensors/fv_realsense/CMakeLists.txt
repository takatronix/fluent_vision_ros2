cmake_minimum_required(VERSION 3.8)
project(fv_realsense)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
# Find librealsense2
find_package(PkgConfig REQUIRED)
pkg_check_modules(REALSENSE2 REQUIRED realsense2)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(compressed_image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDistance.srv"
  "srv/GetCameraInfo.srv"
  DEPENDENCIES geometry_msgs std_srvs
)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${REALSENSE2_INCLUDE_DIRS}
)

# Create executables
add_executable(fv_realsense_node
  src/fv_realsense_node.cpp
)

add_executable(fv_realsense_simple_node
  src/fv_realsense_simple_node.cpp
)

add_executable(image_saver_node
  src/image_saver_node.cpp
)

# Link libraries for main node
ament_target_dependencies(fv_realsense_node
  rclcpp
  sensor_msgs
  cv_bridge
  tf2_ros
  pcl_conversions
  pcl_ros
  image_transport
  compressed_image_transport
  geometry_msgs
  std_srvs
)

target_link_libraries(fv_realsense_node
  ${OpenCV_LIBS}
  ${REALSENSE2_LIBRARIES}
)

# Link libraries for simple node (minimal dependencies)
ament_target_dependencies(fv_realsense_simple_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(fv_realsense_simple_node
  ${OpenCV_LIBS}
)

# Link libraries for image saver node
ament_target_dependencies(image_saver_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(image_saver_node
  ${OpenCV_LIBS}
)

# Link generated interfaces
rosidl_target_interfaces(fv_realsense_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install
install(TARGETS
  fv_realsense_node
  fv_realsense_simple_node
  image_saver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 