cmake_minimum_required(VERSION 3.16)
project(fv_speech)

find_package(ament_cmake REQUIRED)
find_package(fv_speech_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

if(EXISTS "$ENV{HOME}/.cargo/bin/cargo")
  set(CARGO_EXECUTABLE "$ENV{HOME}/.cargo/bin/cargo")
else()
  find_program(CARGO_EXECUTABLE cargo REQUIRED)
endif()

set(FV_SPEECH_RUST_TOOLCHAIN "1.88.0")
execute_process(
  COMMAND ${CMAKE_COMMAND} -E env RUSTUP_TOOLCHAIN=${FV_SPEECH_RUST_TOOLCHAIN}
    ${CARGO_EXECUTABLE} --version
  OUTPUT_VARIABLE CARGO_VERSION_OUTPUT
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" CARGO_VERSION "${CARGO_VERSION_OUTPUT}")
if(NOT CARGO_VERSION OR CARGO_VERSION VERSION_LESS "1.88.0")
  message(FATAL_ERROR "fv_speech requires Rust/Cargo >= 1.88; found ${CARGO_VERSION_OUTPUT}")
endif()

install(DIRECTORY launch config models DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES
  README.md
  LICENSE.SILERO-VAD
  THIRD_PARTY_NOTICES.md
  DESTINATION share/${PROJECT_NAME}
)

set(CARGO_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo-target")
add_custom_target(fv_speech_rust ALL
  COMMAND ${CMAKE_COMMAND} -E env
    RUSTUP_TOOLCHAIN=${FV_SPEECH_RUST_TOOLCHAIN}
    CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
    ${CARGO_EXECUTABLE} build --locked --release --features ros2
      --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/rust/Cargo.toml
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rust
  COMMENT "Building FluentVision Rust ROS 2 speech nodes"
  VERBATIM
)
install(PROGRAMS
  ${CARGO_TARGET_DIR}/release/fv_speech_runtime_exec
  ${CARGO_TARGET_DIR}/release/silero_vad
  ${CARGO_TARGET_DIR}/release/turn_detector
  ${CARGO_TARGET_DIR}/release/parakeet_asr
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fv_speech_contract test/test_package_contract.py)
  add_test(
    NAME fv_speech_rust_tests
    COMMAND ${CMAKE_COMMAND} -E env
      RUSTUP_TOOLCHAIN=${FV_SPEECH_RUST_TOOLCHAIN}
      CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
      ${CARGO_EXECUTABLE} test --locked
        --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/rust/Cargo.toml
  )
  set_tests_properties(fv_speech_rust_tests PROPERTIES TIMEOUT 1200)
endif()

ament_package()
