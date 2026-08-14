# Fluent Scene core — ROS-free .fvs parser, validator, canonical IR, and CLI.
# This target set intentionally has no ROS 2, learning-framework, or GPU
# dependency (spec section 14: the MVP slice must build and test without ROS 2).
cmake_minimum_required(VERSION 3.16)
project(fluent_scene VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_library(fluent_scene STATIC
    src/binding.cpp
    src/binding_config.cpp
    src/diagnostics.cpp
    src/json.cpp
    src/planner.cpp
    src/registry.cpp
    src/sha256.cpp
    src/types.cpp
    src/validator.cpp
    src/yaml_parser.cpp
)
target_include_directories(fluent_scene PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(fluent_scene PRIVATE -Wall -Wextra)

add_executable(fvsc tools/fvsc.cpp)
target_link_libraries(fvsc PRIVATE fluent_scene)
target_compile_options(fvsc PRIVATE -Wall -Wextra)

add_executable(fvs_bench tools/fvs_bench.cpp)
target_link_libraries(fvs_bench PRIVATE fluent_scene)
target_compile_options(fvs_bench PRIVATE -Wall -Wextra)

enable_testing()
add_executable(fvs_tests tests/fvs_tests.cpp)
target_link_libraries(fvs_tests PRIVATE fluent_scene)
target_compile_options(fvs_tests PRIVATE -Wall -Wextra)
add_test(NAME fvs_tests COMMAND fvs_tests ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(fvs_binding_tests tests/binding_tests.cpp)
target_link_libraries(fvs_binding_tests PRIVATE fluent_scene)
target_compile_options(fvs_binding_tests PRIVATE -Wall -Wextra)
add_test(NAME fvs_binding_tests COMMAND fvs_binding_tests ${CMAKE_CURRENT_SOURCE_DIR})

# ---------------------------------------------------------------------------
# Stage 2: retained renderer backends (Vulkan + CPU reference).
# Optional — the ROS-free core above always builds; the renderer needs
# Vulkan headers, freetype2 + harfbuzz, and glslc. The core library itself
# never links these (spec decision 1: the reusable core stays dependency-free;
# backend-specific code sits beneath a narrow GPU interface).
# ---------------------------------------------------------------------------
option(FLUENT_SCENE_RENDERER "Build the retained renderer backends" ON)
if(FLUENT_SCENE_RENDERER)
    find_package(Vulkan QUIET)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(FVS_TEXT IMPORTED_TARGET freetype2 harfbuzz)
    endif()
    find_program(FVS_GLSLC glslc)
    if(Vulkan_FOUND AND FVS_TEXT_FOUND AND FVS_GLSLC)
        set(FVS_SHADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/backend/vulkan/shaders)
        set(FVS_SHADER_GEN ${CMAKE_CURRENT_BINARY_DIR}/shaders_gen)
        file(MAKE_DIRECTORY ${FVS_SHADER_GEN})
        set(FVS_SHADER_INCS "")
        foreach(FVS_SHADER image.vert image.frag boxes.vert boxes.frag text.vert text.frag
                circles.vert circles.frag polyline.vert polyline.frag)
            set(FVS_SHADER_INC ${FVS_SHADER_GEN}/${FVS_SHADER}.inc)
            add_custom_command(
                OUTPUT ${FVS_SHADER_INC}
                COMMAND ${FVS_GLSLC} -O -mfmt=c -o ${FVS_SHADER_INC} ${FVS_SHADER_DIR}/${FVS_SHADER}
                DEPENDS ${FVS_SHADER_DIR}/${FVS_SHADER}
                COMMENT "glslc ${FVS_SHADER}")
            list(APPEND FVS_SHADER_INCS ${FVS_SHADER_INC})
        endforeach()
        add_custom_target(fvs_shaders DEPENDS ${FVS_SHADER_INCS})

        add_library(fluent_scene_render STATIC
            src/render/scene_model.cpp
            src/render/text_atlas.cpp
            backend/cpu/cpu_renderer.cpp
            backend/vulkan/vulkan_renderer.cpp
        )
        add_dependencies(fluent_scene_render fvs_shaders)
        target_include_directories(fluent_scene_render PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src ${FVS_SHADER_GEN})
        target_link_libraries(fluent_scene_render
            PUBLIC fluent_scene
            PRIVATE Vulkan::Vulkan PkgConfig::FVS_TEXT)
        target_compile_options(fluent_scene_render PRIVATE -Wall -Wextra)

        add_executable(fv_render tools/fv_render.cpp)
        target_include_directories(fv_render PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_link_libraries(fv_render PRIVATE fluent_scene_render)
        target_compile_options(fv_render PRIVATE -Wall -Wextra)

        add_executable(fvs_render_tests tests/render_tests.cpp)
        target_include_directories(fvs_render_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_link_libraries(fvs_render_tests PRIVATE fluent_scene_render)
        target_compile_options(fvs_render_tests PRIVATE -Wall -Wextra)
        add_test(NAME fvs_render_tests COMMAND fvs_render_tests ${CMAKE_CURRENT_SOURCE_DIR})
        set_tests_properties(fvs_render_tests PROPERTIES SKIP_RETURN_CODE 77)
    else()
        message(STATUS
            "fluent_scene renderer disabled (requires Vulkan headers, freetype2+harfbuzz, glslc)")
    endif()
endif()

# ---------------------------------------------------------------------------
# Stage 4: ROS 2 adapter node (optional; needs an rclcpp environment, e.g.
# -DCMAKE_PREFIX_PATH=/opt/ros/jazzy). The adapter owns transport only; the
# core and renderer targets above never link ROS.
# ---------------------------------------------------------------------------
option(FLUENT_SCENE_ROS2 "Build the ROS 2 adapter node" ON)
if(FLUENT_SCENE_ROS2 AND TARGET fluent_scene_render)
    find_package(rclcpp QUIET)
    find_package(sensor_msgs QUIET)
    find_package(vision_msgs QUIET)
    find_package(std_msgs QUIET)
    if(rclcpp_FOUND AND sensor_msgs_FOUND AND vision_msgs_FOUND AND std_msgs_FOUND)
        add_executable(fv_scene_node adapters/ros2/fv_scene_node.cpp)
        target_include_directories(fv_scene_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_link_libraries(fv_scene_node PRIVATE fluent_scene_render rclcpp::rclcpp
            ${sensor_msgs_TARGETS} ${vision_msgs_TARGETS} ${std_msgs_TARGETS})
        target_compile_options(fv_scene_node PRIVATE -Wall -Wextra)
    else()
        message(STATUS "fluent_scene ROS 2 adapter disabled (rclcpp/sensor_msgs/vision_msgs not found)")
    endif()
endif()
