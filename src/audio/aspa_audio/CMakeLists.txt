cmake_minimum_required(VERSION 3.16)
project(aspa_audio)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(fluent_dialogue_dora_interfaces REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
if(EXISTS "$ENV{HOME}/.cargo/bin/cargo")
  set(CARGO_EXECUTABLE "$ENV{HOME}/.cargo/bin/cargo")
else()
  find_program(CARGO_EXECUTABLE cargo REQUIRED)
endif()
execute_process(
  COMMAND ${CARGO_EXECUTABLE} --version
  OUTPUT_VARIABLE CARGO_VERSION_OUTPUT
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" CARGO_VERSION "${CARGO_VERSION_OUTPUT}")
if(NOT CARGO_VERSION OR CARGO_VERSION VERSION_LESS "1.85.0")
  message(FATAL_ERROR "aspa_audio requires Rust/Cargo >= 1.85; found ${CARGO_VERSION_OUTPUT}")
endif()

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/playback_controller DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

set(CARGO_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo-target")
add_custom_target(aspa_audio_rust ALL
  COMMAND ${CMAKE_COMMAND} -E env
    CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
    ${CARGO_EXECUTABLE} build --release --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/rust/Cargo.toml
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rust
  COMMENT "Building Rust/CPAL ROS 2 audio nodes"
  VERBATIM
)
install(PROGRAMS
  ${CARGO_TARGET_DIR}/release/audio_capture
  ${CARGO_TARGET_DIR}/release/audio_output
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_playback test/test_playback.py)
endif()

ament_package()
