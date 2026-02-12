cmake_minimum_required(VERSION 3.8)
project(fv_instance_seg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(fv_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(OpenVINO QUIET)
find_package(CUDAToolkit QUIET)
find_package(fluent_lib REQUIRED)

set(FV_HAVE_TENSORRT FALSE)
find_path(TENSORRT_INCLUDE_DIR
  NvInfer.h
  PATHS
    /usr/include
    /usr/include/x86_64-linux-gnu
    /usr/include/aarch64-linux-gnu
    /usr/local/include
)
find_library(TENSORRT_NVINFER_LIB
  nvinfer
  PATHS
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    /usr/lib/aarch64-linux-gnu
    /usr/local/lib
)

if(TENSORRT_INCLUDE_DIR AND TENSORRT_NVINFER_LIB AND CUDAToolkit_FOUND)
  set(FV_HAVE_TENSORRT TRUE)
endif()

include_directories(include)
include_directories(${fluent_lib_INCLUDE_DIRS})

# Prefer linking only the required OpenCV modules to avoid pulling
# in optional dependencies (e.g., GDAL, libtiff, libcurl) via a
# broad OpenCV_LIBS list present in some environments.
set(FV_OPENCV_LIBS)
if(TARGET opencv_core)
  list(APPEND FV_OPENCV_LIBS opencv_core opencv_imgproc)
else()
  # Fallback for environments without imported OpenCV targets
  list(APPEND FV_OPENCV_LIBS ${OpenCV_LIBS})
endif()

add_library(fv_instance_seg_backends
  src/backends/noop_inferencer.cpp
)

target_link_libraries(fv_instance_seg_backends ${FV_OPENCV_LIBS})

if(OpenVINO_FOUND)
  target_sources(fv_instance_seg_backends PRIVATE
    src/backends/ov_yolo_seg.cpp
  )
  target_link_libraries(fv_instance_seg_backends openvino::runtime)
  target_compile_definitions(fv_instance_seg_backends PRIVATE
    FV_INSTANCE_SEG_HAVE_OPENVINO=1
  )
endif()

set(FV_ENABLE_TENSORRT_BACKEND FALSE)
if(FV_HAVE_TENSORRT)
  set(FV_TRT_LIBS ${TENSORRT_NVINFER_LIB})
  set(FV_HAVE_CUDART FALSE)
  if(TARGET CUDA::cudart)
    list(APPEND FV_TRT_LIBS CUDA::cudart)
    set(FV_HAVE_CUDART TRUE)
  else()
    find_library(CUDART_LIB cudart)
    if(CUDART_LIB)
      list(APPEND FV_TRT_LIBS ${CUDART_LIB})
      set(FV_HAVE_CUDART TRUE)
    else()
      message(WARNING "TensorRT found but CUDA runtime (cudart) not found. TensorRT backend disabled.")
    endif()
  endif()
  if(FV_HAVE_CUDART)
    set(FV_ENABLE_TENSORRT_BACKEND TRUE)
  endif()
endif()

if(FV_ENABLE_TENSORRT_BACKEND)
  target_sources(fv_instance_seg_backends PRIVATE
    src/backends/trt_yolo_seg.cpp
  )
  target_include_directories(fv_instance_seg_backends PRIVATE ${TENSORRT_INCLUDE_DIR})
  target_link_libraries(fv_instance_seg_backends
    ${FV_TRT_LIBS}
  )
  target_compile_definitions(fv_instance_seg_backends PRIVATE
    FV_INSTANCE_SEG_HAVE_TENSORRT=1
  )
endif()

if(NOT OpenVINO_FOUND)
  message(WARNING "OpenVINO not found. fv_instance_seg OpenVINO backend disabled.")
endif()

if(NOT FV_HAVE_TENSORRT)
  message(WARNING "TensorRT not found. fv_instance_seg TensorRT backend disabled.")
elseif(NOT FV_ENABLE_TENSORRT_BACKEND)
  message(WARNING "TensorRT backend prerequisites incomplete. fv_instance_seg TensorRT backend disabled.")
endif()

ament_target_dependencies(fv_instance_seg_backends
  rclcpp sensor_msgs fv_msgs cv_bridge
)

add_executable(fv_instance_seg_node
  src/fv_instance_seg_node.cpp
)
target_link_libraries(fv_instance_seg_node
  fv_instance_seg_backends
  ${FV_OPENCV_LIBS}
  tiff
  curl
  fluent_lib::fluent_lib
)
ament_target_dependencies(fv_instance_seg_node
  rclcpp sensor_msgs fv_msgs cv_bridge fluent_lib
)

install(TARGETS fv_instance_seg_backends fv_instance_seg_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
