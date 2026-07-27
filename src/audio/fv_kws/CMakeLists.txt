cmake_minimum_required(VERSION 3.16)
project(fv_kws)

find_package(ament_cmake REQUIRED)
find_package(fv_speech_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

if(EXISTS "$ENV{HOME}/.cargo/bin/cargo")
  set(CARGO_EXECUTABLE "$ENV{HOME}/.cargo/bin/cargo")
else()
  find_program(CARGO_EXECUTABLE cargo REQUIRED)
endif()

set(FV_KWS_RUST_TOOLCHAIN "1.88.0")
execute_process(
  COMMAND ${CMAKE_COMMAND} -E env RUSTUP_TOOLCHAIN=${FV_KWS_RUST_TOOLCHAIN}
    ${CARGO_EXECUTABLE} --version
  OUTPUT_VARIABLE CARGO_VERSION_OUTPUT
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" CARGO_VERSION "${CARGO_VERSION_OUTPUT}")
if(NOT CARGO_VERSION OR CARGO_VERSION VERSION_LESS "1.88.0")
  message(FATAL_ERROR "fv_kws requires Rust/Cargo >= 1.88; found ${CARGO_VERSION_OUTPUT}")
endif()

if(DEFINED ENV{ASPA_VOSK_KWS_ROOT} AND IS_ABSOLUTE "$ENV{ASPA_VOSK_KWS_ROOT}")
  set(FV_KWS_RUNTIME_ROOT "$ENV{ASPA_VOSK_KWS_ROOT}")
elseif(DEFINED ENV{XDG_DATA_HOME} AND IS_ABSOLUTE "$ENV{XDG_DATA_HOME}")
  set(FV_KWS_DATA_HOME "$ENV{XDG_DATA_HOME}")
  set(FV_KWS_RUNTIME_ROOT "${FV_KWS_DATA_HOME}/aspa-navigation/vosk-kws")
else()
  set(FV_KWS_DATA_HOME "$ENV{HOME}/.local/share")
  set(FV_KWS_RUNTIME_ROOT "${FV_KWS_DATA_HOME}/aspa-navigation/vosk-kws")
endif()
set(FV_KWS_RUNTIME_MANIFEST "${FV_KWS_RUNTIME_ROOT}/runtime.json")
set(FV_KWS_LIBRARY_DIR "${FV_KWS_RUNTIME_ROOT}/lib")
set(FV_KWS_LIBRARY "${FV_KWS_LIBRARY_DIR}/libvosk.so")
if(NOT EXISTS "${FV_KWS_LIBRARY}")
  message(FATAL_ERROR
    "Verified Vosk library is missing: ${FV_KWS_LIBRARY}. "
    "Run scripts/setup-vosk-kws --install first."
  )
endif()
if(NOT EXISTS "${FV_KWS_RUNTIME_MANIFEST}")
  message(FATAL_ERROR
    "Verified KWS runtime manifest is missing: ${FV_KWS_RUNTIME_MANIFEST}. "
    "Run scripts/setup-vosk-kws --install first."
  )
endif()
file(SHA256 "${FV_KWS_LIBRARY}" FV_KWS_LIBRARY_SHA256)
if(NOT FV_KWS_LIBRARY_SHA256 STREQUAL
    "0e9df29f060a93cf3df3263a4d3635e1b75688a5fd84e86ade1599372e3c9597")
  message(FATAL_ERROR "libvosk SHA-256 mismatch")
endif()
set(FV_KWS_RUSTFLAGS "-Lnative=${FV_KWS_LIBRARY_DIR}")
set(FV_KWS_LD_LIBRARY_PATH "${FV_KWS_LIBRARY_DIR}")
if(DEFINED ENV{LD_LIBRARY_PATH} AND NOT "$ENV{LD_LIBRARY_PATH}" STREQUAL "")
  string(APPEND FV_KWS_LD_LIBRARY_PATH ":$ENV{LD_LIBRARY_PATH}")
endif()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES README.md THIRD_PARTY_NOTICES.md DESTINATION share/${PROJECT_NAME})

set(CARGO_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo-target")
add_custom_target(fv_kws_rust ALL
  COMMAND ${CMAKE_COMMAND} -E env
    RUSTUP_TOOLCHAIN=${FV_KWS_RUST_TOOLCHAIN}
    CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
    RUSTFLAGS=${FV_KWS_RUSTFLAGS}
    ${CARGO_EXECUTABLE} build --locked --release --features ros2
      --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/rust/Cargo.toml
  COMMAND ${CMAKE_COMMAND} -E env
    LD_LIBRARY_PATH=${FV_KWS_LD_LIBRARY_PATH}
    ${CARGO_TARGET_DIR}/release/fv_kws_model_check
    ${FV_KWS_RUNTIME_MANIFEST}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rust
  COMMENT "Building FluentVision Vosk keyword spotter"
  VERBATIM
)
install(PROGRAMS
  ${CARGO_TARGET_DIR}/release/fv_kws
  ${CARGO_TARGET_DIR}/release/fv_kws_benchmark
  ${CARGO_TARGET_DIR}/release/fv_kws_model_check
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fv_kws_contract test/test_package_contract.py)
  add_test(
    NAME fv_kws_rust_tests
    COMMAND ${CMAKE_COMMAND} -E env
      RUSTUP_TOOLCHAIN=${FV_KWS_RUST_TOOLCHAIN}
      CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
      RUSTFLAGS=${FV_KWS_RUSTFLAGS}
      LD_LIBRARY_PATH=${FV_KWS_LD_LIBRARY_PATH}
      ${CARGO_EXECUTABLE} test --locked
        --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/rust/Cargo.toml
  )
  set_tests_properties(fv_kws_rust_tests PROPERTIES TIMEOUT 1200)
endif()

ament_package()
