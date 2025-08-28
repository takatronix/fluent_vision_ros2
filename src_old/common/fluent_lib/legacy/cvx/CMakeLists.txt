cmake_minimum_required(VERSION 3.8)
project(cvx)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

# cvx library
add_library(cvx SHARED
  src/cvx_draw.cpp
  src/cvx_image.cpp
  src/cvx_image_filter.cpp
  src/cvx_image_utils.cpp
  src/cvx_text.cpp
)

target_include_directories(cvx PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_features(cvx PUBLIC c_std_99 cxx_std_17)

target_link_libraries(cvx
  ${OpenCV_LIBS}
)

# Export library
ament_export_targets(cvx_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS cvx
  EXPORT cvx_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()