cmake_minimum_required(VERSION 3.8)
project(fv_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection2D.msg"
  "msg/DetectionArray.msg"
  "msg/StalkMetrics.msg"
  "msg/StalkMetricsArray.msg"
  "msg/DetectionCloudIndices.msg"
  "msg/Object3D.msg"
  "msg/Object3DArray.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
