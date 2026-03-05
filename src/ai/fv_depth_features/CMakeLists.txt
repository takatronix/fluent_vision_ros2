cmake_minimum_required(VERSION 3.8)
project(fv_depth_features)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

set(FV_OPENCV_LIBS)
if(TARGET opencv_core)
  list(APPEND FV_OPENCV_LIBS opencv_core opencv_imgproc opencv_imgcodecs)
else()
  list(APPEND FV_OPENCV_LIBS ${OpenCV_LIBS})
endif()

set(FV_DEPS rclcpp sensor_msgs std_msgs cv_bridge)

# --- HHA encoder node ---
add_executable(fv_hha_node src/fv_hha_node.cpp)
ament_target_dependencies(fv_hha_node ${FV_DEPS})
target_include_directories(fv_hha_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(fv_hha_node ${FV_OPENCV_LIBS})
target_compile_features(fv_hha_node PUBLIC cxx_std_17)

# --- Depth bilateral filter node ---
add_executable(fv_depth_bilateral_node src/fv_depth_bilateral_node.cpp)
ament_target_dependencies(fv_depth_bilateral_node ${FV_DEPS})
target_include_directories(fv_depth_bilateral_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(fv_depth_bilateral_node ${FV_OPENCV_LIBS})
target_compile_features(fv_depth_bilateral_node PUBLIC cxx_std_17)

# --- Surface normals + RGBD node ---
add_executable(fv_depth_normals_node src/fv_depth_normals_node.cpp)
ament_target_dependencies(fv_depth_normals_node ${FV_DEPS})
target_include_directories(fv_depth_normals_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(fv_depth_normals_node ${FV_OPENCV_LIBS})
target_compile_features(fv_depth_normals_node PUBLIC cxx_std_17)

# --- Depth to PointCloud2 node ---
add_executable(fv_pointcloud_node src/fv_pointcloud_node.cpp)
ament_target_dependencies(fv_pointcloud_node ${FV_DEPS})
target_include_directories(fv_pointcloud_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(fv_pointcloud_node ${FV_OPENCV_LIBS})
target_compile_features(fv_pointcloud_node PUBLIC cxx_std_17)

install(TARGETS fv_depth_bilateral_node fv_hha_node fv_depth_normals_node fv_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
