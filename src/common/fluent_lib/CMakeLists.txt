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
find_package(Threads REQUIRED)
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
# freetype テキスト描画 (fluent_text) はオプション: opencv2/freetype.hpp
# (OpenCV contrib) が無い環境 (native Jetson/Jazzy 等) ではテキスト描画抜きで
# ビルドする。コンテナ (contrib あり) では従来どおり有効。
find_package(Freetype QUIET)
find_path(OPENCV_FREETYPE_INCLUDE opencv2/freetype.hpp
  PATHS ${OpenCV_INCLUDE_DIRS} NO_DEFAULT_PATH)
if(FREETYPE_FOUND AND OPENCV_FREETYPE_INCLUDE)
  set(FLUENT_HAS_FREETYPE TRUE)
else()
  set(FLUENT_HAS_FREETYPE FALSE)
  message(WARNING "fluent_lib: opencv2/freetype.hpp not found - "
                  "using cv::putText fallback (Japanese glyphs unavailable)")
endif()
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

# Source files
set(FLUENT_LIB_SOURCES
  src/fluent_image.cpp
  src/fluent_cloud_impl.cpp
  src/fluent_utils.cpp
  src/ros_io_thread.cpp
  # Always provide fluent::text symbols.  Without opencv_contrib/freetype the
  # implementation falls back to cv::putText; omitting this source made every
  # downstream overlay node fail at link time.
  src/fluent_text.cpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${FLUENT_LIB_SOURCES})
if(FLUENT_HAS_FREETYPE)
  target_compile_definitions(${PROJECT_NAME} PRIVATE FLUENT_HAS_FREETYPE=1)
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE FLUENT_HAS_FREETYPE=0)
endif()

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
  Threads::Threads
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)
if(FLUENT_HAS_FREETYPE)
  target_link_libraries(${PROJECT_NAME} ${FREETYPE_LIBRARIES})
endif()

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
install(
  PROGRAMS
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/setup_dds_receive_buffer.sh
    ${CMAKE_CURRENT_SOURCE_DIR}/scripts/setup_dds_send_buffer.sh
  DESTINATION bin
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
  yaml-cpp
)
if(FLUENT_HAS_FREETYPE)
  ament_export_dependencies(Freetype)
endif()

# Lint and test
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_ros_io_thread
    test/test_ros_io_thread.cpp
  )
  target_include_directories(test_ros_io_thread PRIVATE src)
  target_link_libraries(test_ros_io_thread ${PROJECT_NAME} Threads::Threads)

  add_test(
    NAME test_setup_dds_receive_buffer
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test/test_setup_dds_receive_buffer.sh
  )
  add_test(
    NAME test_setup_dds_send_buffer
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test/test_setup_dds_send_buffer.sh
  )
endif()

ament_package()
