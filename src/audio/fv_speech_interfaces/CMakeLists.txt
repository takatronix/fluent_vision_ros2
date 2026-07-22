cmake_minimum_required(VERSION 3.8)
project(fv_speech_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(message_files
  "msg/AsrControl.msg"
  "msg/AudioFrame.msg"
  "msg/Transcript.msg"
  "msg/TurnEvent.msg"
  "msg/VoiceActivity.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${message_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fv_speech_interfaces test/test_interfaces.py)
endif()

ament_package()
