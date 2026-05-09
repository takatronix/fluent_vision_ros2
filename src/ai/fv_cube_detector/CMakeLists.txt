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

# TensorRT (optional GPU acceleration)
find_path(TENSORRT_INCLUDE_DIR
  NAMES NvInfer.h
  PATHS /usr/include /usr/local/include
        /usr/include/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu
        /usr/include/aarch64-linux-gnu
        /usr/include/x86_64-linux-gnu
)
# nvinfer ships in the multiarch lib dir on aarch64 (Thor: TRT 10 lives in
# /lib/aarch64-linux-gnu). Some installs only ship versioned soname, so
# accept libnvinfer.so.10 as well.
find_library(NVINFER_LIB
  NAMES nvinfer libnvinfer.so.10
  PATHS /usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu
        /lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu
        /usr/lib/aarch64-linux-gnu
        /lib/aarch64-linux-gnu
        /usr/lib/x86_64-linux-gnu
)
# cudart on aarch64 sbsa (Thor) lives under targets/sbsa-linux/lib, on
# Jetson L4T under targets/aarch64-linux/lib, x86_64 under lib64.
find_library(CUDART_LIB
  NAMES cudart
  PATHS /usr/local/cuda/targets/sbsa-linux/lib
        /usr/local/cuda/targets/aarch64-linux/lib
        /usr/local/cuda/lib64
        /usr/local/cuda/lib
        /usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu
)

set(FV_HAS_TENSORRT OFF)
if(TENSORRT_INCLUDE_DIR AND NVINFER_LIB AND CUDART_LIB)
  set(FV_HAS_TENSORRT ON)
  message(STATUS "TensorRT found: ${NVINFER_LIB}")
else()
  message(STATUS "TensorRT not found. TRT backend disabled (ORT-only).")
endif()

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

set(FV_CUBE_SOURCES
  src/ort_yolo_seg.cpp
  src/fv_cube_detector_node.cpp
)
set(FV_CUBE_LIBS
  ${ONNXRUNTIME_LIB}
  ${FV_OPENCV_LIBS}
  fluent_lib::fluent_lib
)

if(FV_HAS_TENSORRT)
  list(APPEND FV_CUBE_SOURCES src/trt_yolo_seg.cpp)
  list(APPEND FV_CUBE_LIBS ${NVINFER_LIB} ${CUDART_LIB})
  include_directories(${TENSORRT_INCLUDE_DIR})
  if(CUDART_LIB)
    get_filename_component(_cuda_lib_dir ${CUDART_LIB} DIRECTORY)
    include_directories(${_cuda_lib_dir}/../include)
  endif()
endif()

add_executable(fv_cube_detector_node ${FV_CUBE_SOURCES})

if(FV_HAS_TENSORRT)
  target_compile_definitions(fv_cube_detector_node PRIVATE FV_HAS_TENSORRT)
endif()

target_link_libraries(fv_cube_detector_node ${FV_CUBE_LIBS})

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
