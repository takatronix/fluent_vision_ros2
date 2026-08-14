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

# ---- stage_web: the Stage in a browser (MJPEG out, pointer events in) -------
find_package(JPEG QUIET)
if(JPEG_FOUND)
  add_executable(stage_web tools/stage_web.cpp)
  target_link_libraries(stage_web PRIVATE fluent_stage JPEG::JPEG)
  find_package(Threads REQUIRED)
  target_link_libraries(stage_web PRIVATE Threads::Threads)
else()
  message(STATUS "fluent_stage: stage_web disabled (needs libjpeg-dev)")
endif()

# ---- tests ------------------------------------------------------------------

enable_testing()
add_executable(stage_tests tests/stage_tests.cpp)
target_link_libraries(stage_tests PRIVATE fluent_stage)
add_test(NAME stage_tests COMMAND stage_tests)

add_executable(ui_tests tests/ui_tests.cpp)
target_link_libraries(ui_tests PRIVATE fluent_stage)
add_test(NAME ui_tests COMMAND ui_tests)

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
