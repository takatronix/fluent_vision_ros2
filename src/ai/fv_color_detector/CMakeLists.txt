cmake_minimum_required(VERSION 3.8)
project(fv_color_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

set(FV_OPENCV_LIBS)
if(TARGET opencv_core)
  list(APPEND FV_OPENCV_LIBS opencv_core opencv_imgproc)
else()
  list(APPEND FV_OPENCV_LIBS ${OpenCV_LIBS})
endif()

set(FV_DEPS rclcpp sensor_msgs std_msgs cv_bridge)

add_executable(fv_color_detector_node src/fv_color_detector_node.cpp)
ament_target_dependencies(fv_color_detector_node ${FV_DEPS})
target_include_directories(fv_color_detector_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(fv_color_detector_node ${FV_OPENCV_LIBS})
target_compile_features(fv_color_detector_node PUBLIC cxx_std_17)

install(TARGETS fv_color_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
