cmake_minimum_required(VERSION 3.16)
project(fv_video_event_describer)

find_package(ament_cmake REQUIRED)
find_package(fv_episode_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

if(EXISTS "$ENV{HOME}/.cargo/bin/cargo")
  set(CARGO_EXECUTABLE "$ENV{HOME}/.cargo/bin/cargo")
else()
  find_program(CARGO_EXECUTABLE cargo REQUIRED)
endif()

set(CARGO_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo-target")
add_custom_target(fv_video_event_describer_rust ALL
  COMMAND ${CMAKE_COMMAND} -E env
    RUSTUP_TOOLCHAIN=1.88.0
    CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
    ${CARGO_EXECUTABLE} build --locked --release --features ros2
      --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/rust/Cargo.toml
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rust
  COMMENT "Building Rust video event describer"
  VERBATIM
)

install(PROGRAMS
  ${CARGO_TARGET_DIR}/release/video_event_describer
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_test(
    NAME fv_video_event_describer_rust_tests
    COMMAND ${CMAKE_COMMAND} -E env
      RUSTUP_TOOLCHAIN=1.88.0
      CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
      ${CARGO_EXECUTABLE} test --locked
        --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/rust/Cargo.toml
  )
endif()

ament_package()
