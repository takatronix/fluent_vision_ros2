cmake_minimum_required(VERSION 3.8)
project(fluent_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
# find_package(cvx REQUIRED)  # Temporarily disabled

# FluentVision library
add_library(fluent_vision SHARED
  src/fluent_vision.cpp
  src/fluent_animation.cpp
)

target_include_directories(fluent_vision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(fluent_vision PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(fluent_vision
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  # cvx  # Temporarily disabled
)

target_link_libraries(fluent_vision
  ${OpenCV_LIBS}
)

# Export library
ament_export_targets(fluent_vision_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS fluent_vision
  EXPORT fluent_vision_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()