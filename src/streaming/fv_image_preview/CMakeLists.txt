cmake_minimum_required(VERSION 3.8)
project(fv_image_preview)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(fv_image_preview_node src/fv_image_preview_node.cpp)
target_include_directories(fv_image_preview_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(fv_image_preview_node ${OpenCV_LIBRARIES})
ament_target_dependencies(fv_image_preview_node rclcpp sensor_msgs)

install(TARGETS fv_image_preview_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
