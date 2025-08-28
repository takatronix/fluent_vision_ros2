cmake_minimum_required(VERSION 3.8)
project(fluent_lib)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find system dependencies
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

# Include directories
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(legacy/cvx/include)

# Source files
set(FLUENT_LIB_SOURCES
  src/fluent_image.cpp
  src/fluent_cloud_impl.cpp
  src/fluent_utils.cpp
  src/fluent_text.cpp
  src/cvx_text.cpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${FLUENT_LIB_SOURCES})

# Target dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link libraries
target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Expose include directory to downstream packages
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Lint and test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()