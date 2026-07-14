cmake_minimum_required(VERSION 3.8)
project(fv_audio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ALSA REQUIRED alsa)

include_directories(
  include
  ${ALSA_INCLUDE_DIRS}
)

set(service_files
  "srv/ListDevices.srv"
  "srv/SwitchDevice.srv"
  "srv/Record.srv"
)

set(message_files
  "msg/AudioFrame.msg"
  "msg/PlaybackDone.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${service_files}
  ${message_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(fv_audio_node
  src/fv_audio_node.cpp
)

ament_target_dependencies(fv_audio_node
  rclcpp
  std_msgs
  std_srvs
  diagnostic_msgs
)

target_include_directories(fv_audio_node PRIVATE ${ALSA_INCLUDE_DIRS})
target_link_libraries(fv_audio_node ${ALSA_LIBRARIES})

rosidl_target_interfaces(fv_audio_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(sine_wave_generator
  src/sine_wave_generator.cpp
)

ament_target_dependencies(sine_wave_generator
  rclcpp
  std_msgs
)

rosidl_target_interfaces(sine_wave_generator
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS fv_audio_node sine_wave_generator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(PROGRAMS
  scripts/radio_streamer.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
