cmake_minimum_required(VERSION 3.8)
project(fv_3d_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)
find_package(fv_msgs REQUIRED)
find_package(fluent_lib QUIET)
# tf2 for stamped lookup so we can re-publish detection points in
# whichever frame downstream consumers want (typically base_link).
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(fv_3d_detector_node
  src/fv_3d_detector_node.cpp
)

ament_target_dependencies(fv_3d_detector_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  fv_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# fluent_cloud/pipeline.hpp is header-only (PCL + Eigen only)
# Use ament install path first, fall back to relative source path
if(fluent_lib_FOUND)
  set(FLUENT_LIB_INCLUDE "${fluent_lib_INCLUDE_DIRS}")
else()
  set(FLUENT_LIB_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/../../common/fluent_lib/include")
endif()
target_include_directories(fv_3d_detector_node
  PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${FLUENT_LIB_INCLUDE}
)

target_link_libraries(fv_3d_detector_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_compile_features(fv_3d_detector_node PUBLIC cxx_std_17)

install(TARGETS fv_3d_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
