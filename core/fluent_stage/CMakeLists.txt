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
)
target_include_directories(fluent_stage PUBLIC include)
target_link_libraries(fluent_stage PRIVATE PkgConfig::FS_TEXT)
target_compile_options(fluent_stage PRIVATE -Wall -Wextra)

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

# Examples run in CI too (§12-3: every example builds and runs).
add_test(NAME example_hud_basic COMMAND hud_basic)
add_test(NAME example_transaction_fade COMMAND transaction_fade)
add_test(NAME example_filters_tour COMMAND filters_tour)
add_test(NAME example_ui_demo COMMAND ui_demo)
add_test(NAME example_ui_catalog COMMAND ui_catalog)
