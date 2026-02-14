cmake_minimum_required(VERSION 3.8)
project(fv_image_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(fluent_lib REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc photo)

# --- OpenCV target-based linking (works on Humble & Jazzy) ---
set(FV_OPENCV_LIBS)
if(TARGET opencv_core)
  list(APPEND FV_OPENCV_LIBS opencv_core opencv_imgproc opencv_photo)
else()
  list(APPEND FV_OPENCV_LIBS ${OpenCV_LIBS})
endif()

set(FV_DEPS rclcpp sensor_msgs cv_bridge fluent_lib)

# --- Helper: add a filter node target ---
macro(add_filter_node TARGET_NAME SOURCE_FILE)
  add_executable(${TARGET_NAME} src/${SOURCE_FILE})
  ament_target_dependencies(${TARGET_NAME} ${FV_DEPS})
  target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(${TARGET_NAME} ${FV_OPENCV_LIBS})
  target_compile_features(${TARGET_NAME} PUBLIC cxx_std_17)
endmacro()

# ── Blur ──
add_filter_node(fv_gaussian_blur_node     gaussian_blur_node.cpp)
add_filter_node(fv_median_blur_node       median_blur_node.cpp)
add_filter_node(fv_box_blur_node          box_blur_node.cpp)
add_filter_node(fv_bilateral_filter_node  bilateral_filter_node.cpp)

# ── Edge Detection ──
add_filter_node(fv_canny_edge_node        canny_edge_node.cpp)
add_filter_node(fv_sobel_edge_node        sobel_edge_node.cpp)
add_filter_node(fv_laplacian_node         laplacian_node.cpp)

# ── Threshold ──
add_filter_node(fv_threshold_node         threshold_node.cpp)
add_filter_node(fv_adaptive_threshold_node adaptive_threshold_node.cpp)

# ── Morphology ──
add_filter_node(fv_morphology_node        morphology_node.cpp)

# ── Color ──
add_filter_node(fv_color_convert_node     color_convert_node.cpp)
add_filter_node(fv_channel_extract_node   channel_extract_node.cpp)
add_filter_node(fv_invert_node            invert_node.cpp)

# ── Adjustment ──
add_filter_node(fv_brightness_contrast_node brightness_contrast_node.cpp)
add_filter_node(fv_gamma_node             gamma_node.cpp)
add_filter_node(fv_histogram_eq_node      histogram_eq_node.cpp)
add_filter_node(fv_clahe_node             clahe_node.cpp)

# ── Transform ──
add_filter_node(fv_resize_node            resize_node.cpp)
add_filter_node(fv_rotate_node            rotate_node.cpp)
add_filter_node(fv_flip_node              flip_node.cpp)
add_filter_node(fv_crop_node              crop_node.cpp)

# ── Effects ──
add_filter_node(fv_unsharp_mask_node      unsharp_mask_node.cpp)
add_filter_node(fv_denoise_node           denoise_node.cpp)

# ── Install ──
install(TARGETS
  fv_gaussian_blur_node fv_median_blur_node fv_box_blur_node fv_bilateral_filter_node
  fv_canny_edge_node fv_sobel_edge_node fv_laplacian_node
  fv_threshold_node fv_adaptive_threshold_node
  fv_morphology_node
  fv_color_convert_node fv_channel_extract_node fv_invert_node
  fv_brightness_contrast_node fv_gamma_node fv_histogram_eq_node fv_clahe_node
  fv_resize_node fv_rotate_node fv_flip_node fv_crop_node
  fv_unsharp_mask_node fv_denoise_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
