cmake_minimum_required(VERSION 3.8)
project(fluent_cloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# FluentCloud library
add_library(fluent_cloud SHARED
  src/fluent_cloud.cpp
)

target_include_directories(fluent_cloud PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(fluent_cloud PUBLIC c_std_99 cxx_std_17)

# PCL definitions
target_compile_definitions(fluent_cloud PUBLIC ${PCL_DEFINITIONS})

ament_target_dependencies(fluent_cloud
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(fluent_cloud
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

# Export library
ament_export_targets(fluent_cloud_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  PCL
  pcl_conversions
  OpenCV
  Eigen3
)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS fluent_cloud
  EXPORT fluent_cloud_targets
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