# fluent_stage — runtime 2D layer tree + reference CPU renderer (Phase L0).
# Standalone CMake; not a ROS package (COLCON_IGNORE keeps colcon out).
#
#   cmake -S . -B build && cmake --build build -j && ctest --test-dir build

cmake_minimum_required(VERSION 3.16)
project(fluent_stage VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(FS_TEXT REQUIRED IMPORTED_TARGET freetype2 harfbuzz)

add_library(fluent_stage
  src/stage.cpp
  src/text_atlas.cpp
  src/cpu_renderer.cpp
  src/ui.cpp
  src/effects.cpp
  src/scene/diagnostics.cpp
  src/scene/sha256.cpp
  src/scene/yaml_parser.cpp
  src/scene/schema.cpp
  src/scene/parser.cpp
  src/scene/writer.cpp
  src/scene/compiler.cpp
  src/scene/describe.cpp
  src/scene/linter.cpp
  src/scene/inspector.cpp
  src/scene/binding.cpp
)
target_include_directories(fluent_stage PUBLIC include)
target_link_libraries(fluent_stage PRIVATE PkgConfig::FS_TEXT)
target_compile_options(fluent_stage PRIVATE -Wall -Wextra)

# ---- Vulkan backend (L1): shaders are compiled to SPIR-V at build time ------
# and embedded — zero shader compilation at runtime (§11). Optional: without
# Vulkan/glslc the library is CPU-only and FS_HAVE_VULKAN is undefined.

find_package(Vulkan QUIET)
find_program(FS_GLSLC glslc)
if(Vulkan_FOUND AND FS_GLSLC)
  set(FS_SHADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/backend/vulkan/shaders)
  set(FS_SHADER_GEN ${CMAKE_CURRENT_BINARY_DIR}/shaders_gen)
  file(MAKE_DIRECTORY ${FS_SHADER_GEN})
  set(FS_SHADER_INCS "")
  foreach(FS_SHADER quad.vert fullscreen.vert shape_mask.frag glyph_mask.frag
          mask_composite.frag shape_color.frag image.frag filter.frag blur.frag
          composite.frag unpremul.frag)
    set(FS_SHADER_INC ${FS_SHADER_GEN}/${FS_SHADER}.inc)
    add_custom_command(
      OUTPUT ${FS_SHADER_INC}
      COMMAND ${FS_GLSLC} -O -mfmt=c
              -I ${CMAKE_CURRENT_SOURCE_DIR}/include -I ${FS_SHADER_DIR}
              -o ${FS_SHADER_INC} ${FS_SHADER_DIR}/${FS_SHADER}
      DEPENDS ${FS_SHADER_DIR}/${FS_SHADER}
              ${FS_SHADER_DIR}/push_common.glsl
              ${FS_SHADER_DIR}/shape_modes.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/fluent_stage/shared/shapes_shared.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/fluent_stage/shared/filters_shared.h
      COMMENT "glslc ${FS_SHADER}")
    list(APPEND FS_SHADER_INCS ${FS_SHADER_INC})
  endforeach()
  add_custom_target(fs_shaders DEPENDS ${FS_SHADER_INCS})

  target_sources(fluent_stage PRIVATE src/vulkan_renderer.cpp)
  # Vulkan C structs are idiomatically brace-initialized from their sType;
  # the aggregate zero-fills the rest — the warning is pure noise here.
  set_source_files_properties(src/vulkan_renderer.cpp
    PROPERTIES COMPILE_OPTIONS "-Wno-missing-field-initializers")
  add_dependencies(fluent_stage fs_shaders)
  target_include_directories(fluent_stage PRIVATE ${FS_SHADER_GEN} ${FS_SHADER_DIR})
  target_link_libraries(fluent_stage PRIVATE Vulkan::Vulkan)
  target_compile_definitions(fluent_stage PUBLIC FS_HAVE_VULKAN=1)
else()
  message(STATUS "fluent_stage: Vulkan backend disabled (needs Vulkan headers + glslc)")
endif()

# ---- examples (all buildable and runnable; §12-3) ---------------------------

add_executable(hud_basic examples/hud_basic.cpp)
target_link_libraries(hud_basic PRIVATE fluent_stage)

add_executable(transaction_fade examples/transaction_fade.cpp)
target_link_libraries(transaction_fade PRIVATE fluent_stage)

add_executable(filters_tour examples/filters_tour.cpp)
target_link_libraries(filters_tour PRIVATE fluent_stage)

add_executable(ui_demo examples/ui_demo.cpp)
target_link_libraries(ui_demo PRIVATE fluent_stage)

add_executable(ui_catalog examples/ui_catalog.cpp)
target_link_libraries(ui_catalog PRIVATE fluent_stage)

add_executable(ripple_demo examples/ripple_demo.cpp)
target_link_libraries(ripple_demo PRIVATE fluent_stage)

add_executable(bench examples/bench.cpp)
target_link_libraries(bench PRIVATE fluent_stage)

# ---- fvsc: the Scene compiler CLI (§13) -------------------------------------

add_executable(fvsc tools/fvsc.cpp)
target_link_libraries(fvsc PRIVATE fluent_stage)

# ---- stage_web: live camera effects in a browser ----------------------------
# H.264(NVENC)/WebSocket out, pointer events in. ROS 2 input is optional:
# configure with the ROS environment sourced to get the live camera topic.
find_package(JPEG QUIET)
if(JPEG_FOUND)
  add_executable(stage_web tools/stage_web.cpp)
  target_link_libraries(stage_web PRIVATE fluent_stage JPEG::JPEG)
  # scene_web: live .fvs preview server (edit → save → atomic swap, §2/L2)
  add_executable(scene_web tools/scene_web.cpp)
  target_link_libraries(scene_web PRIVATE fluent_stage JPEG::JPEG)
  find_package(Threads REQUIRED)
  target_link_libraries(stage_web PRIVATE Threads::Threads)
  target_link_libraries(scene_web PRIVATE Threads::Threads)
  find_package(rclcpp QUIET)
  find_package(sensor_msgs QUIET)
  if(rclcpp_FOUND AND sensor_msgs_FOUND)
    foreach(FS_WEB_TOOL stage_web scene_web)
      target_link_libraries(${FS_WEB_TOOL} PRIVATE rclcpp::rclcpp ${sensor_msgs_TARGETS})
      target_compile_definitions(${FS_WEB_TOOL} PRIVATE FS_HAVE_ROS=1)
    endforeach()
    message(STATUS "fluent_stage: stage_web/scene_web with ROS 2 camera input")

    # scene_node: the production ROS 2 adapter (Phase L3) — scene.fvs +
    # binding.fvb → subscriptions, render, published Surface. Needs ROS.
    add_executable(scene_node tools/scene_node.cpp)
    target_link_libraries(scene_node PRIVATE fluent_stage JPEG::JPEG Threads::Threads
                          rclcpp::rclcpp ${sensor_msgs_TARGETS})
    find_package(vision_msgs QUIET)
    find_package(geometry_msgs QUIET)
    find_package(std_msgs QUIET)
    if(geometry_msgs_FOUND AND std_msgs_FOUND)
      target_link_libraries(scene_node PRIVATE ${geometry_msgs_TARGETS} ${std_msgs_TARGETS})
    endif()
    if(vision_msgs_FOUND)
      target_link_libraries(scene_node PRIVATE ${vision_msgs_TARGETS})
      target_compile_definitions(scene_node PRIVATE FS_HAVE_VISION_MSGS=1)
      message(STATUS "fluent_stage: scene_node with vision_msgs detections")
    endif()
  else()
    message(STATUS "fluent_stage: stage_web/scene_web without ROS (source ROS env to enable)")
  endif()
else()
  message(STATUS "fluent_stage: stage_web/scene_web disabled (needs libjpeg-dev)")
endif()

# ---- tests ------------------------------------------------------------------

enable_testing()
add_executable(stage_tests tests/stage_tests.cpp)
target_link_libraries(stage_tests PRIVATE fluent_stage)
add_test(NAME stage_tests COMMAND stage_tests)

add_executable(ui_tests tests/ui_tests.cpp)
target_link_libraries(ui_tests PRIVATE fluent_stage)
add_test(NAME ui_tests COMMAND ui_tests)

# L2 Scene layer: validation, digest, fmt, YAML↔C++ parity, linter (§14 L2).
add_executable(scene_tests tests/scene_tests.cpp)
target_link_libraries(scene_tests PRIVATE fluent_stage)
add_test(NAME scene_tests COMMAND scene_tests)

# Image-level golden tests. `./golden_tests --update` regenerates the
# references in tests/golden/ from the current build.
add_executable(golden_tests tests/golden_tests.cpp)
target_link_libraries(golden_tests PRIVATE fluent_stage)
target_compile_definitions(golden_tests
  PRIVATE GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/golden")
add_test(NAME golden_tests COMMAND golden_tests)

# The §11 cross-backend guarantee: the GPU renders the same goldens the CPU
# reference defined. Skips (77) on machines without a Vulkan device.
add_test(NAME golden_tests_vulkan COMMAND golden_tests --renderer=vulkan)
set_tests_properties(golden_tests_vulkan PROPERTIES SKIP_RETURN_CODE 77)

# Examples run in CI too (§12-3: every example builds and runs).
add_test(NAME example_hud_basic COMMAND hud_basic)
add_test(NAME example_transaction_fade COMMAND transaction_fade)
add_test(NAME example_filters_tour COMMAND filters_tour)
add_test(NAME example_ui_demo COMMAND ui_demo)
add_test(NAME example_ui_catalog COMMAND ui_catalog)
add_test(NAME example_ripple_demo COMMAND ripple_demo)
