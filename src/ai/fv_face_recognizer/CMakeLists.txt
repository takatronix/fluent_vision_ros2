cmake_minimum_required(VERSION 3.16)
project(fv_face_recognizer)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Placeholder for future face recognition node
# add_executable(fv_face_recognizer_node
#   src/fv_face_recognizer_node.cpp
# )
# 
# ament_target_dependencies(fv_face_recognizer_node
#   rclcpp
#   sensor_msgs
#   cv_bridge
#   image_transport
#   std_msgs
#   geometry_msgs
#   visualization_msgs
# )
# 
# install(TARGETS
#   fv_face_recognizer_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()