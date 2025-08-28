cmake_minimum_required(VERSION 3.8)
project(fv_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(compressed_image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetCameraInfo.srv"
  "srv/SetCameraSettings.srv"
  DEPENDENCIES geometry_msgs std_srvs
)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Create executables
add_executable(fv_camera_node
  src/fv_camera_node.cpp
)

add_executable(list_cameras_node
  src/list_cameras_node.cpp
)

# Link libraries for main node
ament_target_dependencies(fv_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  tf2_ros
  image_transport
  compressed_image_transport
  geometry_msgs
  std_srvs
)

target_link_libraries(fv_camera_node
  ${OpenCV_LIBS}
)



# Link libraries for list cameras node
ament_target_dependencies(list_cameras_node
  rclcpp
  std_msgs
)

target_link_libraries(list_cameras_node
  ${OpenCV_LIBS}
)

# Link generated interfaces
rosidl_target_interfaces(fv_camera_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install
install(TARGETS
  fv_camera_node
  list_cameras_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
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