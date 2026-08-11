cmake_minimum_required(VERSION 3.16)
project(fv_audio)

find_package(ament_cmake REQUIRED)
find_package(fv_audio_interfaces REQUIRED)
find_package(fv_speech_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
if(EXISTS "$ENV{HOME}/.cargo/bin/cargo")
  set(CARGO_EXECUTABLE "$ENV{HOME}/.cargo/bin/cargo")
else()
  find_program(CARGO_EXECUTABLE cargo REQUIRED)
endif()
set(FV_AUDIO_RUST_TOOLCHAIN "1.92.0")
execute_process(
  COMMAND ${CMAKE_COMMAND} -E env RUSTUP_TOOLCHAIN=${FV_AUDIO_RUST_TOOLCHAIN}
    ${CARGO_EXECUTABLE} --version
  OUTPUT_VARIABLE CARGO_VERSION_OUTPUT
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" CARGO_VERSION "${CARGO_VERSION_OUTPUT}")
if(NOT CARGO_VERSION OR CARGO_VERSION VERSION_LESS "1.92.0")
  message(FATAL_ERROR "fv_audio requires Rust/Cargo >= 1.92; found ${CARGO_VERSION_OUTPUT}")
endif()

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

set(CARGO_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo-target")
add_custom_target(fv_audio_rust ALL
  COMMAND ${CMAKE_COMMAND} -E env
    RUSTUP_TOOLCHAIN=${FV_AUDIO_RUST_TOOLCHAIN}
    CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
    IDL_PACKAGE_FILTER=fv_audio_interfaces\;fv_speech_interfaces\;std_msgs
    ${CARGO_EXECUTABLE} build --locked --release
      --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/rust/Cargo.toml
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rust
  COMMENT "Building Rust ROS 2 audio and playback nodes"
  VERBATIM
)
install(PROGRAMS
  ${CARGO_TARGET_DIR}/release/audio_capture
  ${CARGO_TARGET_DIR}/release/audio_device_registry
  ${CARGO_TARGET_DIR}/release/audio_aec
  ${CARGO_TARGET_DIR}/release/audio_output
  ${CARGO_TARGET_DIR}/release/playback_controller
  ${CARGO_TARGET_DIR}/release/tts_timing_collector
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_fv_audio_launch_contract
    test/test_launch_contract.py
  )
  add_test(
    NAME fv_audio_rust_tests
    COMMAND ${CMAKE_COMMAND} -E env
      RUSTUP_TOOLCHAIN=${FV_AUDIO_RUST_TOOLCHAIN}
      CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
      IDL_PACKAGE_FILTER=fv_audio_interfaces\;fv_speech_interfaces\;std_msgs
      ${CARGO_EXECUTABLE} test --locked
        --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/rust/Cargo.toml
  )
  set_tests_properties(fv_audio_rust_tests PROPERTIES TIMEOUT 600)
endif()

ament_package()
