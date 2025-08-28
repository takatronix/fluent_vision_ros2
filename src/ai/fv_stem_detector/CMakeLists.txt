cmake_minimum_required(VERSION 3.8)
project(fv_stem_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(fluent_lib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StemDetection.msg"
  "msg/StemDetectionArray.msg"
  "msg/StemVector.msg"
  "msg/StemVectorArray.msg"
  "msg/RegionCloud.msg"
  "msg/RegionCloudArray.msg"
  "srv/SetSourceMode.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}_lib STATIC
  src/stem_detector.cpp
  src/aspara_3d_pipeline.cpp
  src/overlay_renderer.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp sensor_msgs std_msgs image_transport cv_bridge message_filters vision_msgs geometry_msgs std_srvs OpenCV pcl_conversions fluent_lib
)
rosidl_target_interfaces(${PROJECT_NAME}_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib Eigen3::Eigen ${PCL_LIBRARIES})

add_executable(stem_detector_node src/stem_detector_node.cpp)
target_link_libraries(stem_detector_node ${PROJECT_NAME}_lib)
ament_target_dependencies(stem_detector_node
  rclcpp sensor_msgs std_msgs image_transport cv_bridge message_filters vision_msgs geometry_msgs std_srvs OpenCV pcl_conversions fluent_lib
)
rosidl_target_interfaces(stem_detector_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(stem_detector_node ${PCL_LIBRARIES} Eigen3::Eigen)

install(TARGETS
  stem_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config)
endif()

install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()


