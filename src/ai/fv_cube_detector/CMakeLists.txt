cmake_minimum_required(VERSION 3.8)
project(fv_cube_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(fv_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(fluent_lib REQUIRED)

# ONNX Runtime - search user install and system paths
set(ONNXRUNTIME_ROOT "$ENV{HOME}/onnxruntime" CACHE PATH "ONNX Runtime install root")

find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  PATHS ${ONNXRUNTIME_ROOT}/include
        /usr/local/include
        /usr/include
  NO_DEFAULT_PATH
)

find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime
  PATHS ${ONNXRUNTIME_ROOT}/lib
        /usr/local/lib
        /usr/lib
  NO_DEFAULT_PATH
)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB)
  message(WARNING "ONNX Runtime not found. Skipping fv_cube_detector build.")
  ament_package()
  return()
endif()

message(STATUS "ONNX Runtime include: ${ONNXRUNTIME_INCLUDE_DIR}")
message(STATUS "ONNX Runtime lib: ${ONNXRUNTIME_LIB}")

include_directories(include)
include_directories(${ONNXRUNTIME_INCLUDE_DIR})
include_directories(${fluent_lib_INCLUDE_DIRS})

# Prefer linking only the required OpenCV modules
set(FV_OPENCV_LIBS)
if(TARGET opencv_core)
  list(APPEND FV_OPENCV_LIBS opencv_core opencv_imgproc)
else()
  list(APPEND FV_OPENCV_LIBS ${OpenCV_LIBS})
endif()

add_executable(fv_cube_detector_node
  src/ort_yolo_seg.cpp
  src/fv_cube_detector_node.cpp
)

target_link_libraries(fv_cube_detector_node
  ${ONNXRUNTIME_LIB}
  ${FV_OPENCV_LIBS}
  fluent_lib::fluent_lib
)

ament_target_dependencies(fv_cube_detector_node
  rclcpp sensor_msgs fv_msgs cv_bridge fluent_lib
)

install(TARGETS fv_cube_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
